#include <Arduino.h>
#pragma once

class pico_esc {
public:
    static void pico_uart_init();

    static void arm_thrusters();

    static void send_escframe(uint16_t escframe);
};