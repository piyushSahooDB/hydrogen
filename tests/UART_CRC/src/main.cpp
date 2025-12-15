
#include <Arduino.h>
#include <string.h>

/*
    Sending frame structure:
    - 8 bits sof
    - 16 bits BMS Current
    - 16 bits BMS output voltage
    - 16 bits Total Battery Voltage
    - 16 bits Temp
    - 8 bits ERROR (say bms turned off or smth)
    - 8 bits CRC
    SOF is AA

    Receiving frame structure:
    COMMAND : TURN OFF THRUSTER POWER (0x11)
    COMMAND : TURN ON THRUSTER POWER  (0x22)
    COMMAND : 

*/

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
