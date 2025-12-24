#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "dshot.pio.h"

int main() {
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 0);

    PIO pio;
    uint sm;
    uint offset;
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_program, &pio,
        &sm, &offset, 0, 1, true);
    hard_assert(success);

    dshot_program_init(pio, sm, offset, 0);

    for (int i = 0;i < 4000;i++) {
        pio_sm_put_blocking(pio, sm, 0x000F << 16);         //arming sequence
        sleep_us(700);
    }
    for (int i = 0;i < 10;i++) {
        pio_sm_put_blocking(pio, sm, (uint32_t)0xEFF1 << 16);       //3d mode
        sleep_us(1);
    }
    while (true) {
        pio_sm_put_blocking(pio, sm, (uint32_t)0x126A << 16);       //throttle
        sleep_us(700);
    }
}
