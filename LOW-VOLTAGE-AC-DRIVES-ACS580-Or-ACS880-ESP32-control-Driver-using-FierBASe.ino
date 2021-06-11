
//Required HTTPClientESP32Ex library to be installed  https://github.com/mobizt/HTTPClientESP32Ex

#include <WiFi.h>
#include "FirebaseESP32.h"
#include <HCSR04.h>
#include <ESP32Servo.h>



#define FIREBASE_HOST "sudpotassep60.firebaseio.com"
#define FIREBASE_AUTH "xxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"


FirebaseData firebaseData; //Define FirebaseESP32 data object
Servo myservo;  // create servo object to control a servo
HCSR04 hc(27,14); //initialisation class HCSR04 (trig pin , echo pin)

unsigned long sendDataPrevMillis = 0;

String path = "/";
String distancePath = "/input/Distance";
String courantPath = "/input/courant";
String vitessePath = "/input/vitesse";
String frequencePath = "/input/frequance";
String ReadyPath = "/input/Ready";
String defautPath = "/input/defaut";
String thermistorstPath = "/input/thermistors";
String inusePath = "/input/in use";
String MarchePath = "/output/marche";
int niveau; 
float MfrequanceF = 14;
int MfrequanceI = 14;
int servoPin = 15;
bool moniteur = true ;
bool start = false ;
int times = 15000 ;
int frequenceval;
int LED_BUILTIN = 2;
const int marche = 26;
const int lampe = 25;
const int S3 = 33;
const int S4 = 32;
const int Ready = 21;
const int defaut = 19;
const int thermistors = 18;
const int inuse = 5;
int frequencepot = 34; 
int ADC_Max = 4096;



uint16_t count = 0;

void setup() {

  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  if (!Firebase.beginStream(firebaseData, path)) {
    Serial.println("------Can't begin stream connection------");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println();
  }
  myservo.setPeriodHertz(50); // Standard 50hz servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using SG90 servo min/max of 500us and 2400us
  // for MG995 large servo, use 1000us and 2000us,
  // which are the defaults, so this line could be

  digitalWrite(marche, HIGH); // "myservo.attach(servoPin);"
  digitalWrite(lampe, HIGH);
  digitalWrite(S3, HIGH);
  digitalWrite(32, HIGH);

  pinMode(marche, OUTPUT);
  pinMode(lampe, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Ready, INPUT_PULLUP);
  pinMode(defaut, INPUT_PULLUP);
  pinMode(thermistors, INPUT_PULLUP);
  pinMode(inuse, INPUT_PULLUP);

}

void loop() {
  int ReadyVal = digitalRead(Ready);
  int defautVal = digitalRead(defaut);
  int thermistorsVal = digitalRead(thermistors);
  int inuseVal = digitalRead(inuse);
  niveau = int(hc.dist());
  frequenceval = analogRead(frequencepot);
  Serial.println(frequenceval);
  frequenceval = map(frequenceval, 0, ADC_Max, 0, 50);
  Serial.println(frequenceval);
 
  

    if (ReadyVal == HIGH) {
    Firebase.setString(firebaseData, ReadyPath, "Pas Pret");
  } else {
    Firebase.setString(firebaseData, ReadyPath, "Pret");
  }
      if (defautVal == HIGH) {
    Firebase.setString(firebaseData, defautPath, "Bien");
      }
 else {
   Firebase.setString(firebaseData, defautPath, "Problem");
      }
      if (thermistorsVal == HIGH) {
   Firebase.setString(firebaseData, thermistorstPath, "Bien");   
  } else {
    Firebase.setString(firebaseData, thermistorstPath, "Problem");
    Firebase.setString(firebaseData, MarchePath, "OFF");
    digitalWrite(marche, HIGH);
      }
      if (inuseVal == HIGH) {
    Firebase.setString(firebaseData, inusePath, "Arrêté");
  } else {
    Firebase.setString(firebaseData, inusePath, "On Marche");
  }

  if (Firebase.getString(firebaseData, "/output/start")) {
    if (firebaseData.stringData().equals("ON")) {
      start = true;
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (firebaseData.stringData().equals("OFF")) {
      start = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  if (Firebase.getInt(firebaseData, "/output/time")) {
    if (firebaseData.dataType() == "int") {
      times = (firebaseData.intData() * 1000);
    }
  }

  if (Firebase.getString(firebaseData, "/output/moniteur")) {
    if (firebaseData.stringData().equals("ON")) {
      moniteur = true;
    } else if (firebaseData.stringData().equals("OFF")) {
      moniteur = false;
    }
  }

  if (moniteur) {
    if (millis() - sendDataPrevMillis > times) {
      sendDataPrevMillis = millis();


      if (Firebase.setInt(firebaseData, frequencePath, frequenceval)) {
        //Serial.print("Frequence : ");
        //Serial.print(firebaseData.intData());
        //Serial.println(" HZ ");
      }
      if (Firebase.setInt(firebaseData, distancePath, niveau)) {
        //Serial.print("Distance : ");
        //Serial.print(firebaseData.intData());
        //Serial.println(" Cm ");
    }
  }
  }

  if (Firebase.getString(firebaseData, "/output/S4")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(S4, LOW);;
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(S4, HIGH);
    }
  }

  if (Firebase.getString(firebaseData, "/output/S3")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(S3, LOW);
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(S3, HIGH);
    }
  }

  if (Firebase.getString(firebaseData, "/output/extlamp")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(lampe, LOW);
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(lampe, HIGH);
    }
  }

  if (Firebase.getString(firebaseData, "/output/marche")) {
    if (firebaseData.stringData().equals("ON")) {
      digitalWrite(marche, LOW);
    } else if (firebaseData.stringData().equals("OFF")) {
      digitalWrite(marche, HIGH);
    }
  }
  if (Firebase.getInt(firebaseData, "/output/Mfrequance")) {
    if (firebaseData.dataType() == "int") {
      MfrequanceI = firebaseData.intData();
      MfrequanceI = map(MfrequanceI, 14, 50, 0, 180);
      myservo.write(MfrequanceI);
    }
  }

}
