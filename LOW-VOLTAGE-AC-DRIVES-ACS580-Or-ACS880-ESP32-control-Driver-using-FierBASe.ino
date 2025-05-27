// This sketch enables remote control and monitoring of ABB ACS580/ACS880 low-voltage AC drives
// using an Espressif ESP32 microcontroller and Google's Firebase Realtime Database.
// Key functionalities include:
// - Wi-Fi connectivity for Firebase communication.
// - Reading drive status inputs (Ready, Default, Thermistors, In Use).
// - Reading sensor data (Ultrasonic distance, Potentiometer for frequency).
// - Sending status and sensor data to Firebase.
// - Receiving control commands from Firebase (Start/Stop, Frequency, Relay control).
// - Controlling a servo motor to simulate drive frequency input.
// - Controlling digital outputs (relays/lamps).

//Required HTTPClientESP32Ex library to be installed  https://github.com/mobizt/HTTPClientESP32Ex

#include <WiFi.h>             // Library for Wi-Fi connectivity
#include "FirebaseESP32.h"    // Library for Firebase Realtime Database interaction
#include <HCSR04.h>           // Library for HC-SR04 ultrasonic distance sensor
#include <ESP32Servo.h>       // Library for ESP32 specific servo control

// Firebase Project Configuration
#define FIREBASE_HOST "sudpotassep60.firebaseio.com" // Firebase Realtime Database URL
#define FIREBASE_AUTH "xxxxxxxxxxxxxxxxxxxxxxxxxxxxx"  // Firebase Authentication Token (replace with your actual token)

// Wi-Fi Network Credentials
#define WIFI_SSID "WIFI_SSID"         // Your Wi-Fi network name (SSID)
#define WIFI_PASSWORD "WIFI_PASSWORD" // Your Wi-Fi network password

// Firebase Realtime Database objects
FirebaseData firebaseData; // FirebaseESP32 data object for storing response data

// Peripheral objects
Servo myservo;  // Servo object to control the servo motor
HCSR04 hc(27,14); // HCSR04 sensor object: trigPin=GPIO27, echoPin=GPIO14

// Timing variables
unsigned long sendDataPrevMillis = 0; // Stores the last time data was sent to Firebase

// Firebase Database Paths (Strings for flexibility)
String path = "/"; // Root path for Firebase stream (can be more specific if needed)
// Input paths: Data read from sensors/drive and sent TO Firebase
String distancePath = "/input/Distance";     // Path for ultrasonic sensor distance reading
String courantPath = "/input/courant";       // Path for current reading (placeholder, not implemented in this code)
String vitessePath = "/input/vitesse";       // Path for speed reading (placeholder, not implemented in this code)
String frequencePath = "/input/frequance";   // Path for frequency reading (from potentiometer)
String ReadyPath = "/input/Ready";         // Path for drive ready status
String defautPath = "/input/defaut";        // Path for drive fault status
String thermistorstPath = "/input/thermistors"; // Path for drive thermistor status
String inusePath = "/input/in use";        // Path for drive in-use status
// Output paths: Data received FROM Firebase to control ESP32/drive
String MarchePath = "/output/marche";      // Path for drive start/stop command

// Global variables for sensor readings and control logic
int niveau;                     // Stores distance from HC-SR04 sensor in cm
float MfrequanceF = 14;         // Target frequency for servo (float, potentially for finer control - currently not primary)
int MfrequanceI = 14;           // Target frequency for servo (integer, used for mapping to servo angle)
int servoPin = 15;              // ESP32 pin connected to the servo motor's signal line
bool moniteur = true ;          // Flag to enable/disable sending data to Firebase (monitoring)
bool start = false ;            // Flag to indicate if the main process/drive should be started
int times = 15000 ;             // Interval in milliseconds for sending data to Firebase (default 15 seconds)
int frequenceval;               // Stores raw analog value from frequency potentiometer
int LED_BUILTIN = 2;            // ESP32 built-in LED pin (often GPIO2)

// Pin Definitions for Digital I/O
// Output pins (controlled by ESP32)
const int marche = 26;        // Pin to control the main drive start/stop relay (active LOW)
const int lampe = 25;         // Pin to control an external lamp/indicator (active LOW)
const int S3 = 33;            // Pin for general purpose digital output S3 (active LOW)
const int S4 = 32;            // Pin for general purpose digital output S4 (active LOW)

// Input pins (reading status from drive or sensors)
const int Ready = 21;         // Pin to read drive ready status (INPUT_PULLUP, HIGH = Not Ready, LOW = Ready)
const int defaut = 19;        // Pin to read drive fault status (INPUT_PULLUP, HIGH = OK, LOW = Fault)
const int thermistors = 18;   // Pin to read drive thermistor status (INPUT_PULLUP, HIGH = OK, LOW = Problem)
const int inuse = 5;          // Pin to read drive in-use/running status (INPUT_PULLUP, HIGH = Stopped, LOW = Running)
int frequencepot = 34;        // ESP32 ADC pin for reading potentiometer value for frequency setting (Analog Input)
int ADC_Max = 4096;           // Maximum ADC value for ESP32 (12-bit ADC)

uint16_t count = 0; // General purpose counter (currently unused)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300); // Wait for connection
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP()); // Print local IP address
  Serial.println();

  // Initialize Firebase connection
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true); // Enable auto-reconnect to Wi-Fi if connection is lost

  // Attempt to begin streaming data from the root path of Firebase
  // This allows real-time updates if data changes on Firebase at this path or its children
  if (!Firebase.beginStream(firebaseData, path)) {
    Serial.println("------Can't begin stream connection------");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println();
  }

  // Servo motor setup
  myservo.setPeriodHertz(50); // Set servo PWM frequency to 50Hz (standard for SG90)
  myservo.attach(servoPin, 500, 2400); // Attach servo to 'servoPin', define min/max pulse width in microseconds

  // Initialize digital output pins to HIGH (typically off for active-LOW relays)
  digitalWrite(marche, HIGH); // Set 'marche' relay pin to HIGH (OFF)
  digitalWrite(lampe, HIGH);  // Set 'lampe' relay pin to HIGH (OFF)
  digitalWrite(S3, HIGH);     // Set 'S3' relay pin to HIGH (OFF)
  digitalWrite(S4, HIGH);     // Set 'S4' (GPIO32) relay pin to HIGH (OFF) - Note: Corrected from 'digitalWrite(32, HIGH)'

  // Configure pin modes
  pinMode(marche, OUTPUT);      // Set 'marche' pin as an output
  pinMode(lampe, OUTPUT);       // Set 'lampe' pin as an output
  pinMode(S3, OUTPUT);          // Set 'S3' pin as an output
  pinMode(S4, OUTPUT);          // Set 'S4' pin as an output
  pinMode(LED_BUILTIN, OUTPUT); // Set built-in LED pin as an output

  // Configure input pins with internal pull-up resistors
  // This means the pin will read HIGH if nothing is connected or if the connected sensor pulls it LOW
  pinMode(Ready, INPUT_PULLUP);       // Drive ready status input
  pinMode(defaut, INPUT_PULLUP);      // Drive fault status input
  pinMode(thermistors, INPUT_PULLUP); // Drive thermistor status input
  pinMode(inuse, INPUT_PULLUP);       // Drive in-use status input
  // frequencepot (GPIO34) is an analog input, pinMode is not strictly required for analogRead on ESP32
}

void loop() {
  // Read status from digital input pins connected to the drive
  int ReadyVal = digitalRead(Ready);           // Read 'Ready' status (LOW = Ready)
  int defautVal = digitalRead(defaut);         // Read 'Defaut' (Fault) status (LOW = Problem)
  int thermistorsVal = digitalRead(thermistors); // Read 'Thermistors' status (LOW = Problem)
  int inuseVal = digitalRead(inuse);           // Read 'In Use' status (LOW = On Marche/Running)

  // Read sensor values
  niveau = int(hc.dist()); // Read distance from ultrasonic sensor (cm)
  frequenceval = analogRead(frequencepot); // Read raw value from frequency potentiometer

  // Print raw and mapped frequency potentiometer value for debugging
  Serial.println(frequenceval);
  frequenceval = map(frequenceval, 0, ADC_Max, 0, 50); // Map ADC value (0-4095) to frequency range (0-50 Hz)
  Serial.println(frequenceval);
 
  // Update Firebase with drive status based on digital input readings
  if (ReadyVal == HIGH) { // If Ready pin is HIGH, drive is not ready
    Firebase.setString(firebaseData, ReadyPath, "Pas Pret"); // "Not Ready"
  } else { // Else (LOW), drive is ready
    Firebase.setString(firebaseData, ReadyPath, "Pret");     // "Ready"
  }

  if (defautVal == HIGH) { // If Defaut pin is HIGH, no fault
    Firebase.setString(firebaseData, defautPath, "Bien");    // "Good"
  } else { // Else (LOW), there is a fault
   Firebase.setString(firebaseData, defautPath, "Problem"); // "Problem"
  }

  if (thermistorsVal == HIGH) { // If Thermistors pin is HIGH, thermistors are OK
   Firebase.setString(firebaseData, thermistorstPath, "Bien");   // "Good"
  } else { // Else (LOW), thermistor problem
    Firebase.setString(firebaseData, thermistorstPath, "Problem"); // "Problem"
    // If thermistor problem, automatically turn off the drive ('marche') as a safety measure
    Firebase.setString(firebaseData, MarchePath, "OFF");
    digitalWrite(marche, HIGH); // Set 'marche' relay to OFF
  }

  if (inuseVal == HIGH) { // If In Use pin is HIGH, drive is stopped
    Firebase.setString(firebaseData, inusePath, "Arrêté"); // "Stopped"
  } else { // Else (LOW), drive is running
    Firebase.setString(firebaseData, inusePath, "On Marche"); // "Running"
  }

  // Check for Firebase commands for 'start'
  if (Firebase.getString(firebaseData, "/output/start")) { // Attempt to get string from "/output/start" path
    if (firebaseData.stringData().equals("ON")) { // If command is "ON"
      start = true; // Set start flag
      digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED
    } else if (firebaseData.stringData().equals("OFF")) { // If command is "OFF"
      start = false; // Clear start flag
      digitalWrite(LED_BUILTIN, LOW);  // Turn off built-in LED
    }
  }

  // Check for Firebase command for 'time' (data sending interval)
  if (Firebase.getInt(firebaseData, "/output/time")) { // Attempt to get integer from "/output/time" path
    if (firebaseData.dataType() == "int") { // Check if data type is integer
      times = (firebaseData.intData() * 1000); // Update 'times' interval (value from Firebase is in seconds)
    }
  }

  // Check for Firebase command for 'moniteur' (enable/disable Firebase data sending)
  if (Firebase.getString(firebaseData, "/output/moniteur")) {
    if (firebaseData.stringData().equals("ON")) {
      moniteur = true;  // Enable monitoring
    } else if (firebaseData.stringData().equals("OFF")) {
      moniteur = false; // Disable monitoring
    }
  }

  // Send data to Firebase if monitoring is enabled and interval has passed
  if (moniteur) {
    if (millis() - sendDataPrevMillis > times) { // Check if 'times' interval has elapsed
      sendDataPrevMillis = millis(); // Update the last send time

      // Send current frequency value (from potentiometer) to Firebase
      if (Firebase.setInt(firebaseData, frequencePath, frequenceval)) {
        // Optional: Serial print for confirmation (currently commented out)
        //Serial.print("Frequence : ");
        //Serial.print(firebaseData.intData());
        //Serial.println(" HZ ");
      }
      // Send current distance value (from ultrasonic sensor) to Firebase
      if (Firebase.setInt(firebaseData, distancePath, niveau)) {
        // Optional: Serial print for confirmation (currently commented out)
        //Serial.print("Distance : ");
        //Serial.print(firebaseData.intData());
        //Serial.println(" Cm ");
    }
  }
  }

  // Control digital output S4 based on Firebase command
  if (Firebase.getString(firebaseData, "/output/S4")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(S4, LOW); // Turn S4 ON (active LOW)
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(S4, HIGH); // Turn S4 OFF
    }
  }

  // Control digital output S3 based on Firebase command
  if (Firebase.getString(firebaseData, "/output/S3")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(S3, LOW); // Turn S3 ON (active LOW)
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(S3, HIGH); // Turn S3 OFF
    }
  }

  // Control external lamp based on Firebase command
  if (Firebase.getString(firebaseData, "/output/extlamp")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(lampe, LOW); // Turn lamp ON (active LOW)
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(lampe, HIGH); // Turn lamp OFF
    }
  }

  // Control main drive 'marche' relay based on Firebase command
  if (Firebase.getString(firebaseData, MarchePath)) { // MarchePath is "/output/marche"
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(marche, LOW); // Turn drive ON (active LOW)
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(marche, HIGH); // Turn drive OFF
    }
  }

  // Control servo motor based on Firebase command for 'Mfrequance' (manual frequency)
  if (Firebase.getInt(firebaseData, "/output/Mfrequance")) { // Attempt to get integer for manual frequency
    if (firebaseData.dataType() == "int") {
      MfrequanceI = firebaseData.intData(); // Get integer value (expected 0-50 Hz or similar range)
      // Map the received frequency value (e.g., 14-50 Hz) to servo angle (0-180 degrees)
      // Note: The original mapping seems to be from 14-50. Adjust if the Firebase input range is different.
      MfrequanceI = map(MfrequanceI, 14, 50, 0, 180);
      myservo.write(MfrequanceI); // Write mapped angle to servo
    }
  }
}
// End of loop function
// No custom functions defined in this sketch.
