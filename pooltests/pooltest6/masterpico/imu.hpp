#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#pragma once
#include "structs.hpp"

class imu {
public:
    static void init();

    static void update();
};