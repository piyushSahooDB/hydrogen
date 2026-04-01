#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "dshot.pio.h"

uint8_t gcr_decode(uint8_t code)
{
    switch (code) {
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

bool crc_check(uint16_t message, uint8_t crc) {
    unsigned int calc_crc = (message ^ (message >> 4) ^ (message >> 8)) & 0x0F;
    return (crc == calc_crc);
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

        for (int i = 20; i >= 0; --i) {
            printf("%"PRIu32, value >> i & 1);
        }
        printf("        ");

        uint8_t crc;
        uint8_t n[2];
        uint8_t p;

        if (value != 0xFFFFF) {

            uint32_t gcr = (value ^ (value >> 1));

            for (int i = 20; i >= 0; --i) {
                printf("%"PRIu32, gcr >> i & 1);
            }
            printf("        ");

            crc = gcr_decode(gcr & 0x1F);
            n[0] = gcr_decode((gcr >> 5) & 0x1F);
            n[1] = gcr_decode((gcr >> 10) & 0x1F);
            p = gcr_decode((gcr >> 15) & 0x1F);

            if (((p & 0x1) == 1)) {

                uint16_t message = ((n[0]) | (n[1] << 4) | (p << 8));

                for (int i = 12; i >= 0; --i) {
                    printf("%"PRIu32, message >> i & 1);
                }
                printf("        ");


                printf("%X ", ((gcr >> 15) & 0x1F));
                printf("%X ", ((gcr >> 10) & 0x1F));
                printf("%X ", ((gcr >> 5) & 0x1F));
                printf("%X      ", (gcr & 0x1F));


                printf("%X ", gcr_decode(((gcr >> 15) & 0x1F)));
                printf("%X ", gcr_decode(((gcr >> 10) & 0x1F)));
                printf("%X ", gcr_decode(((gcr >> 5) & 0x1F)));
                printf("%X      ", gcr_decode((gcr & 0x1F)));


                printf("%d", crc_check(message, crc));

                // if (crc_check(message, crc)) {
                //     uint16_t erpm = ((n[0]) | (n[1] << 4) | ((p & 0x1) << 8));
                //     erpm = (erpm << (p >> 1));
                //     printf("%d", erpm);
                //     printf("        ");
                //     printf("%d", erpm / 7);
                // }

            }
        }
        printf("\n");
        sleep_us(700);
    }
}
