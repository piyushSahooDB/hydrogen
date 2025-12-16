// #include <Arduino.h>
// #include <ArduinoRS485.h>

// #define DE_RE 4  // Pin controlling DE and RE̅

// void setup() {
//   pinMode(DE_RE, OUTPUT);
//   digitalWrite(DE_RE, HIGH); // Enable transmit
//   Serial.begin(9600);
//   RS485.begin(9600);
// }

// void loop() {
//   digitalWrite(DE_RE, HIGH); // Transmit mode
//   RS485.beginTransmission();
//   RS485.write(123);  // Send integer (example)
//   RS485.endTransmission();
//   delay(1000);
// }
