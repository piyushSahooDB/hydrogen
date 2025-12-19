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
#define UARTID  2

HardwareSerial picolink(UARTID);

void setup() {
  Serial.begin(115200);
  picolink.begin(BAUDRATE, SERIAL_8N1, RXPIN, TXPIN);
  picolink.write(0b10010000);
}

void send_escframe(uint16_t escframe) {
  uint8_t dom = (escframe >> 8) & 0xFF;
  uint8_t sub = escframe & 0xFF;
  picolink.write(dom);
  picolink.write(sub);
}

void loop() {
  uint16_t throttle[5] = { 1347,1347,1347,1347,1347 };

  // for (int i = 0;i < 2000;i++) {
  //   for (int j = 0;j < 5;j++) {
  //     picolink.write(0b00010000 | i);
  //     delayMicroseconds(100);
  //     send_escframe(0x00000000);
  //     delayMicroseconds(800);
  //   }
  // }
  for (int i = 0;i < 5;i++) {
    picolink.write(0b00010000 | i);
    delayMicroseconds(100);
    send_escframe(escframegen(throttle[i]));
    delayMicroseconds(800);
  }
  // for (int i = 0;i < 2000;i++) {
  //   for (int i = 0;i < 5;i++) {
  //     picolink.write(0b00010000 | i);
  //     delayMicroseconds(100);
  //     send_escframe(escframegen(547));
  //     delayMicroseconds(800);
  //   }
  // }
  // for (int i = 0;i < 2000;i++) {
  //   for (int i = 0;i < 5;i++) {
  //     picolink.write(0b00010000 | i);
  //     delayMicroseconds(100);
  //     send_escframe(escframegen(1547));
  //     delayMicroseconds(800);
  //   }
  // }

  // for (int i = 0;i < 5000;i++) {
  //   picolink.write(0b00010000 | 0);
  //   delayMicroseconds(100);
  //   send_escframe(escframegen(147));
  //   delayMicroseconds(800);
  // }
  // for (int i = 0;i < 5000;i++) {
  //   picolink.write(0b00010000 | 0);
  //   delayMicroseconds(100);
  //   send_escframe(escframegen(1147));
  //   delayMicroseconds(800);
  // }

}