#include <stdint.h>

struct Pins {
    struct BMS {
        uint8_t tx;
        uint8_t rx;
        uint8_t errorInt;
    } BMS;
};