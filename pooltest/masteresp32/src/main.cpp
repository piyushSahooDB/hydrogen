#include <Arduino.h>

uint16_t escframegen(uint16_t throttle) {
  throttle &= 0x7FF;
  uint16_t packet = (throttle << 1) | 0;
  unsigned int crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
  uint16_t escframe = (packet << 4) | crc;
  return escframe;
}

#define TXPIN   17
#define RXPIN   16
#define BAUDRATE  115200

HardwareSerial picolink(2);

void setup() {
  Serial.begin(115200);
  picolink.begin(BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);
}

void send_escframe(uint16_t escframe) {
  uint8_t dom = (escframe >> 8) & 0xFF;
  uint8_t sub = escframe & 0xFF;
  picolink.write(dom);
  picolink.write(sub);
}

void loop() {
  uint16_t throttle = 100;
  send_escframe(escframegen(throttle));
  delay(1000);
}