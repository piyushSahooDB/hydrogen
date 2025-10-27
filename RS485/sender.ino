#include <SoftwareSerial.h>

// MAX485 pins
#define DE 3
#define RE 2

// RX, TX for SoftwareSerial (connect to MAX485 RO, DI)
SoftwareSerial RS485Serial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);         // Serial Monitor
  RS485Serial.begin(9600);    // RS485 UART

  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  // Enable transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);

  Serial.println("Arduino RS485 Sender ready...");
}

void loop() {
  int data = random(0, 100);  // random integer 0–99

  RS485Serial.println(data);  // send as text with newline
  Serial.print("Data sent: ");
  Serial.println(data);

  delay(2000); // send every 2 seconds
}
