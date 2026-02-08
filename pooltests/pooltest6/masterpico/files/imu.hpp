#include <Arduino.h>
#pragma once
#include <Wire.h>
#include <7Semi_BNO055.h>
#include "structs.hpp"

extern BNO055_7Semi bno055;

class imu {
public:

    static void init();

    static void update();
};