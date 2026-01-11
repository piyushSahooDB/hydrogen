#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "dshot.pio.h"

int main() {

    stdio_init_all();

    // gpio_init(15);
    // gpio_set_dir(15, GPIO_OUT);
    // gpio_put(15, 0);

    PIO pio;
    uint sm;
    uint offset;
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_program, &pio,
        &sm, &offset, 8, 1, true);
    hard_assert(success);

    dshot_program_init(pio, sm, offset, 8);

    for (int i = 0;i < 4000;i++) {
        pio_sm_put_blocking(pio, sm, 0x000F << 16);
        sleep_us(700);
    }
    for (int i = 0;i < 10;i++) {
        pio_sm_put_blocking(pio, sm, (uint32_t)0xEFF1 << 16);
        sleep_us(700);
    }
    while (true) {
        pio_sm_put_blocking(pio, sm, (uint32_t)0x126A << 16);
        uint32_t v = pio_sm_get_blocking(pio, sm);
        for (int i = 31; i >= 0; --i) {
            printf("%"PRIu32, v >> i & 1);
        }
        printf("\n");
        sleep_us(700);
    }
}
