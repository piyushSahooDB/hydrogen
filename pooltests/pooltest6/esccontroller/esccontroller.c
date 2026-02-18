#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

#include "dshot.pio.h"

#define TXPIN   29
#define RXPIN   28
#define BAUDRATE  115200
#define UARTID  uart0

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

int main() {
    stdio_init_all();
    uart_init(UARTID, BAUDRATE);
    gpio_set_function(TXPIN, GPIO_FUNC_UART);
    gpio_set_function(RXPIN, GPIO_FUNC_UART);

    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 0);

    allthrusters_init();
    arm_thrusters();

    while (true) {

        uint16_t throttleesc[5] = { 80,80,80,80,80 };
        for (int j = 0;j < 5;j++) {
            throttleesc[j] &= 0x7FF;
            printf("%d      ", throttleesc[j]);
            uint16_t packet = (throttleesc[j] << 1) | 0;
            uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;         //calulating 4bit CRC
            uint16_t escframe = (packet << 4) | crc;        //final 16bit frame that needs to be sent
            pio_sm_put_blocking(pio[j], sm[j], (uint32_t)escframe << 16);
        }

        sleep_ms(1);
    }
}
