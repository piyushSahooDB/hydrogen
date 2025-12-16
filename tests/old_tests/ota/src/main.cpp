#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

void setup() {
  WiFi.begin("Katam raju", "123321123");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  ArduinoOTA.begin();
  pinMode(2, OUTPUT);
}

void loop() {
  ArduinoOTA.handle();

  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
}