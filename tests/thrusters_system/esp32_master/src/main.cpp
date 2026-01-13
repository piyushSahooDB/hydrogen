#include <Arduino.h>
#include "pico_esc.hpp"

void setup() {
  pico_esc::pico_uart_init();
  pico_esc::arm_thrusters();
}

void loop() {
  uint16_t throttle[5] = { 1347,1347,1347,1347,1347 };
  for (int i = 0;i < 5;i++) {
    Serial1.write(0b00010000 | i);      //sending address
    pico_esc::send_escframe(throttle[i]);
  }
  delayMicroseconds(800);
}