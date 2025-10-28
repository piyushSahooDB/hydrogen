#include <Arduino.h>
#define DE 4
#define RE 2
#define TXD 17
#define RXD 16

void setup() {
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);     // stay in receive mode
  digitalWrite(RE, LOW);
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD, TXD);
  Serial.println("ESP32-B waiting for RS485 data...");
}

void loop() {
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.print("Received via RS485: ");
    Serial.println(msg);
  }
}
