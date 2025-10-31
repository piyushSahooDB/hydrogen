// #include <Arduino.h>
// #include <ArduinoOTA.h>
// #include <WiFi.h>
// #include <Servo.h>

// #define DE 4
// #define RE 2
// #define TXD 17
// #define RXD 16

// Servo servo1;
// Servo servo2;
// Servo servo3;


// void setup() {
//   WiFi.begin("Katam raju", "123321123");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//   }
//   ArduinoOTA.begin();
//   pinMode(2, OUTPUT);

//   pinMode(DE, OUTPUT);
//   pinMode(RE, OUTPUT);
//   digitalWrite(DE, LOW);     // stay in receive mode
//   digitalWrite(RE, LOW);
//   Serial.begin(9600);
//   Serial2.begin(9600, SERIAL_8N1, RXD, TXD);
//   Serial.println("ESP32-B waiting for RS485 data...");

//   servo1.attach(13);
//   servo2.attach(14);
//   servo3.attach(15);

//   servo1.writeMicroseconds(1500);
//   servo2.writeMicroseconds(1500);
//   servo3.writeMicroseconds(1500);

//   delay(1000);
// }

// void loop() {
//   ArduinoOTA.handle();
//   if (Serial2.available()) {
//     int s1 = Serial2.parseInt();
//     int s2 = Serial2.parseInt();
//     int s3 = Serial2.parseInt();

//     servo1.writeMicroseconds(s1);
//     servo2.writeMicroseconds(s2);
//     servo3.writeMicroseconds(s3);
//   }
// }

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <ESP32Servo.h>   // ✅ Use ESP32Servo instead of Servo.h

#define DE 4
#define RE 2
#define TXD 17
#define RXD 16

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // Connect to Wi-Fi
  WiFi.begin("Katam raju", "123321123");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // OTA setup
  ArduinoOTA.begin();

  // RS485 direction control pins
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);  // stay in receive mode
  digitalWrite(RE, LOW);

  // Serial setup
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);
  Serial.println("ESP32-B waiting for RS485 data...");

  // ✅ ESP32-specific: assign PWM timers before attaching servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  servo1.setPeriodHertz(50);  // standard 50 Hz for servos
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);

  // Attach servos to pins
  servo1.attach(13, 500, 2500);  // pin, min, max microseconds
  servo2.attach(14, 500, 2500);
  servo3.attach(15, 500, 2500);

  // Initialize all servos to neutral position
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);

  delay(480000);
}

void loop() {
  ArduinoOTA.handle();

  // if (Serial2.available()) {
  //   int s1 = Serial2.parseInt();
  //   int s2 = Serial2.parseInt();
  //   int s3 = Serial2.parseInt();

    // Write servo positions
  servo1.writeMicroseconds(1400);
  servo2.writeMicroseconds(1400);
  servo3.writeMicroseconds(1400);

  //   Serial.printf("Received: %d %d %d\n", s1, s2, s3);
  // }
  //test
}
o