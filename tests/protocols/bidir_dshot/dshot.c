#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "dshot.pio.h"

uint8_t gcr_decode(uint8_t code5)
{
    switch (code5) {
    case 0x19: return 0x0;
    case 0x1B: return 0x1;
    case 0x12: return 0x2;
    case 0x13: return 0x3;
    case 0x1D: return 0x4;
    case 0x15: return 0x5;
    case 0x16: return 0x6;
    case 0x17: return 0x7;
    case 0x1A: return 0x8;
    case 0x09: return 0x9;
    case 0x0A: return 0xA;
    case 0x0B: return 0xB;
    case 0x1E: return 0xC;
    case 0x0D: return 0xD;
    case 0x0E: return 0xE;
    case 0x0F: return 0xF;
    default:   return 0xFF;
    }
}
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
        uint32_t value = pio_sm_get_blocking(pio, sm);
        for (int i = 31; i >= 0; --i) {
            printf("%"PRIu32, value >> i & 1);
        }
        printf("                ");
        uint16_t n[3];
        uint16_t p;
        if (value != 0xFFFFF) {
            uint32_t gcr = (value ^ (value >> 1));
            n[0] = gcr_decode(gcr & 0x1F);
            n[1] = gcr_decode((gcr >> 5) & 0x1F);
            n[2] = gcr_decode((gcr >> 10) & 0x1F);
            p = gcr_decode((gcr >> 15) & 0x1F);
            if ((p & 0x1) == 1) {
                uint16_t rpm = (n[0] | (n[1] << 4) | (n[2] << 8) | ((p & 0x1) << 12));
                // for (int i = 15; i >= 0; --i) {
                //     printf("%"PRIu16, rpm >> i & 1);
                // }
                printf("%d", rpm);
            }
        }
        printf("\n");
        sleep_us(700);
    }
}
