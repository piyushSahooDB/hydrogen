#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdint.h>

#include "dshot.pio.h"

#define TXPIN   17
#define RXPIN   16
#define BAUDRATE  115200
#define UARTID  uart0

PIO pio[5];
uint sm[5];
uint offset[5];
static const uint thruster[5] = { 0, 1, 2, 3, 4 };


void allthrusters_init() {
    for (int i = 0;i < 5;i++) {
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_program, &pio[i],
            &sm[i], &offset[i], thruster[i], 1, true);
        hard_assert(success);
        dshot_program_init(pio[i], sm[i], offset[i], thruster[i]);
    }
}

bool crc_check(uint16_t throttle) {
    uint16_t rx_crc = throttle & 0x0F;
    uint16_t packet = throttle >> 4;
    unsigned int calc_crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (rx_crc == calc_crc);
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

        uint8_t address, dom, sub;

        address = uart_getc(UARTID);

        if (address == (0b10010000))
            arm_thrusters();
        else if (address == (0x10)) {
            uint8_t hi = uart_getc(UARTID);
            uint8_t lo = uart_getc(UARTID);
            uint16_t throttle = ((uint16_t)hi << 8) | lo;
            if (crc_check(throttle))
                pio_sm_put_blocking(pio[0], sm[0], (uint32_t)throttle << 16);
        }
        else if (address == (0x11)) {
            uint8_t hi = uart_getc(UARTID);
            uint8_t lo = uart_getc(UARTID);
            uint16_t throttle = ((uint16_t)hi << 8) | lo;
            if (crc_check(throttle))
                pio_sm_put_blocking(pio[1], sm[1], (uint32_t)throttle << 16);
        }
        else if (address == (0x12)) {
            uint8_t hi = uart_getc(UARTID);
            uint8_t lo = uart_getc(UARTID);
            uint16_t throttle = ((uint16_t)hi << 8) | lo;
            if (crc_check(throttle))
                pio_sm_put_blocking(pio[2], sm[2], (uint32_t)throttle << 16);
        }
        else if (address == (0x13)) {
            uint8_t hi = uart_getc(UARTID);
            uint8_t lo = uart_getc(UARTID);
            uint16_t throttle = ((uint16_t)hi << 8) | lo;
            if (crc_check(throttle))
                pio_sm_put_blocking(pio[3], sm[3], (uint32_t)throttle << 16);
        }
        else if (address == (0x14)) {
            uint8_t hi = uart_getc(UARTID);
            uint8_t lo = uart_getc(UARTID);
            uint16_t throttle = ((uint16_t)hi << 8) | lo;
            if (crc_check(throttle))
                pio_sm_put_blocking(pio[4], sm[4], (uint32_t)throttle << 16);
        }
    }
}
