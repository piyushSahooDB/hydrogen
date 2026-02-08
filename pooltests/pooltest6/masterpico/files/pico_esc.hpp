#include <Arduino.h>
#pragma once

class pico_esc {
public:
    static void init();

    static void arm_thrusters();

    static void send_escframe(uint16_t escframe);
};