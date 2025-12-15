
#include <Arduino.h>
#include <string.h>

String receivedMessage = "";

void setup() {
  Serial.begin(115200);
  
  Serial.println("ESP32 is ready. Please enter a message:");
}

void loop() {
  while (Serial.available()) {
    char incomingChar = Serial.read();
    
    if (incomingChar == '\n') { 
      Serial.print("You sent: ");
      Serial.println(receivedMessage);
      
      receivedMessage = "";
    } else {
      receivedMessage += incomingChar;
    }
  }
}
