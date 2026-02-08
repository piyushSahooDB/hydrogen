#include <stdint.h>
#pragma once

typedef struct {
    float roll, pitch, heading;
    float wx, wy, wz;
} State;

typedef struct {
    uint16_t VB, VR, VL, HR, HL;
    int zoffset;
} Throttle;