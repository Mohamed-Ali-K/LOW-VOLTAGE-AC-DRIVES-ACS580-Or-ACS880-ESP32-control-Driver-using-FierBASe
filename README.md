# ESP32-based Remote Control and Monitoring of ABB AC Drives (ACS580/ACS880) via Firebase

## Overview
This project enables remote control and monitoring of ABB ACS580/ACS880 low-voltage AC drives using an Espressif ESP32 microcontroller. It leverages Google's Firebase Realtime Database for seamless data synchronization and user interaction. The system incorporates a servo motor to simulate analog control inputs to the drive and an ultrasonic sensor for potential feedback mechanisms, such as level sensing or safety interlocks. This setup allows users to monitor and adjust drive parameters remotely over a Wi-Fi connection.

## Features
*   **Remote Operation:** Control AC drive functions (e.g., start, stop, frequency adjustment) from a distance.
*   **Real-time Monitoring:** Synchronize and monitor drive status and sensor data via Firebase.
*   **Distance Sensing:** Utilize an HC-SR04 ultrasonic sensor for applications like level control or obstacle detection.
*   **Auxiliary Control:** Manage auxiliary components using a 4-channel relay module.
*   **Status Indication:** Employ LED indicators for visual feedback on system status.

## Hardware Requirements
*   ABB General Purpose Drive: ACS580 or ACS880 (0.75 to 500 kW)
*   Microcontroller: Espressif ESP32 Development Board
*   Servo Motor: SG-90
*   Ultrasonic Sensor: HC-SR04 Distance Sensor
*   Relay Module: 5 Volt, 4-Channel Arduino Relay Module
*   Connectivity: Stable Wi-Fi Internet Connection (for ESP32 and Firebase)

## Software Requirements & Dependencies
*   **Development Environment:** Arduino IDE or a compatible ESP32 development environment (e.g., PlatformIO).
*   **Required Libraries:**
    *   HTTPClientESP32Ex by mobizt: [https://github.com/mobizt/HTTPClientESP32Ex](https://github.com/mobizt/HTTPClientESP32Ex)
    *   Firebase Realtime Database Arduino Library for ESP32 by mobizt: [https://github.com/mobizt/Firebase-ESP32](https://github.com/mobizt/Firebase-ESP32)
    *   HC-SR04 Ultrasonic Sensor Library by d03n3rfr1tz3: [https://github.com/d03n3rfr1tz3/HC-SR04](https://github.com/d03n3rfr1tz3/HC-SR04)
    *   ESP32Servo Library by jkb-git: [https://github.com/jkb-git/ESP32Servo](https://github.com/jkb-git/ESP32Servo)

## Installation & Setup
*(Details to be added: This section will include step-by-step instructions for wiring the ESP32 to the AC drive's interface, servo motor, ultrasonic sensor, and relay module. It will also cover Firebase project creation and configuration, and the process for compiling and flashing the ESP32 firmware. Ensure correct pin assignments as per the `.ino` sketch are documented here.)*

**Key configuration parameters in the `.ino` sketch:**
*   `FIREBASE_HOST`: Your Firebase project URL (e.g., `your-project-id.firebaseio.com`).
*   `FIREBASE_AUTH`: Your Firebase project secret or an appropriate authentication token.
*   `WIFI_SSID`: Your Wi-Fi network name.
*   `WIFI_PASSWORD`: Your Wi-Fi network password.
*   Pin definitions: Review and document the ESP32 pins used for the servo, ultrasonic sensor, relays, and status LEDs.

## Usage
*(Details to be added: This section will explain how to interact with the system. This includes defining the Firebase Realtime Database paths for sending commands (e.g., start/stop, frequency setpoint) and for reading status information (e.g., drive speed, sensor readings, operational status). The `.ino` sketch references paths like `/output/marche` for control and `/input/vitesse` for feedback.)*

## How it Works (Optional)
The ESP32 microcontroller establishes a connection to the configured Wi-Fi network and connects to the Firebase Realtime Database. It actively listens for commands written to predefined database paths (e.g., to initiate or halt the drive, or to adjust frequency using the servo). Concurrently, the ESP32 reads data from the connected ultrasonic sensor and monitors the drive's status pins (Ready, Defaut, Thermistors, In Use). This information is then published to corresponding paths in Firebase, providing real-time updates. The servo motor translates digital commands into a simulated analog signal for the drive's frequency control. The relay module can be used to manage external components such as main contactors or indicator lamps.

## Wiring Diagram (Optional)
*(A detailed wiring diagram or a reference to one will be added here to illustrate the connections between the ESP32, AC drive, servo, sensors, and relays.)*

## Contributing
Contributions to this project are welcome. Please open an issue on the project repository to discuss proposed changes or submit a pull request with your improvements.

## License
MIT License
*(Note: Please verify if the original project from which this code was derived specified a different license. If not, the MIT License is a common default for open-source hardware and software projects.)*
