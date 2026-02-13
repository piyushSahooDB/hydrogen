#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

int main(void) {
    stdio_init_all();
    imu::init();
}