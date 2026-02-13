#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <inttypes.h>

#include "dshot.pio.h"

#include "structs.hpp"
#include "imu.hpp"
#include "control.hpp"

State state;
Throttle throttle = { 0, 0, 0, 0, 0, 0 };

PIO pio[5];
uint sm[5];
uint offset[5];
static const uint thruster[5] = { 5, 6, 7, 8, 9 };

void allthrusters_init() {
    for (int i = 0;i < 5;i++) {
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_program, &pio[i],
            &sm[i], &offset[i], thruster[i], 1, true);
        hard_assert(success);
        dshot_program_init(pio[i], sm[i], offset[i], thruster[i]);
    }
}

void arm_thrusters() {
    for (int i = 0;i < 500;i++) {
        for (int j = 0;j < 5;j++) {
            pio_sm_put_blocking(pio[j], sm[j], 0x00000000 << 16);
            sleep_us(700);
        }
    }
    for (int i = 0;i < 10;i++) {
        for (int j = 0;j < 5;j++) {
            pio_sm_put_blocking(pio[j], sm[j], (uint32_t)0x0145 << 16);
            sleep_us(700);
        }
    }
}

int main(void) {

    stdio_init_all();

    sleep_ms(5000);
    printf("program initiating\n");

    imu::init();
    allthrusters_init();
    arm_thrusters();

    printf("program initialised\n");

    while (1) {

        imu::update();
        control::update();

        printf("%d      %d      %d\n", throttle.VB, throttle.VR, throttle.VL);

        uint16_t throttleesc[5] = { throttle.VB,throttle.VR,throttle.VL,throttle.HR,throttle.HL };
        for (int j = 0;j < 5;j++) {
            throttleesc[j] &= 0x7FF;
            uint16_t packet = (throttleesc[j] << 1) | 0;
            unsigned int crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;         //calulating 4bit CRC
            uint16_t escframe = (packet << 4) | crc;        //final 16bit frame that needs to be sent
            pio_sm_put_blocking(pio[j], sm[j], escframe);
        }
        sleep_us(700);
    }



}