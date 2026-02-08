#include "config.hpp"
#include "imu.hpp"
#include "structs.hpp"

BNO055_7Semi bno055;

extern State state;


static float heading0 = 0.0f;
static float roll0 = 0.0f;
static float pitch0 = 0.0f;

static int16_t gx = 0;
static int16_t gy = 0;
static int16_t gz = 0;

static const float alpha = 0.1;
static float wx_filt_last = 0;
static float wy_filt_last = 0;
static float wz_filt_last = 0;


float wrapAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void imu::init() {
    Wire.begin(BNO055_SDA, BNO055_SCL, BNO055_BAUDRATE);
    if (!bno055.begin(Wire, 21, 22, false)) {
        Serial.println("BNO055 NOT DETECTED");
        while (1);
    }
    bno055.setMode(Mode::CONFIG);
    delay(50);
    bno055.setMode(Mode::IMUPLUS);
    delay(50);

    bno055.readEuler(heading0, roll0, pitch0);  if (!bno055.readEuler(heading0, roll0, pitch0)) return;
    Serial.println("Roll & Pitch locked");
}

void imu::update() {

    if (!bno055.readEuler(state.heading, state.roll, state.pitch)) return;
    state.roll = wrapAngle(state.roll - roll0);
    state.pitch = wrapAngle(state.pitch - pitch0);
    // Deadband for small angles
    if (abs(state.roll) < 0.02) state.roll = 0;
    if (abs(state.pitch) < 0.02) state.pitch = 0;


    bno055.readGyro(gx, gy, gz);
    // Convert gyro to rad/s
    state.wx = gx * DEG_TO_RAD;
    state.wy = gy * DEG_TO_RAD;
    state.wz = gz * DEG_TO_RAD;
    //low pass filter
    state.wx = alpha * state.wx + (1 - alpha) * wx_filt_last;
    state.wy = alpha * state.wy + (1 - alpha) * wy_filt_last;
    state.wz = alpha * state.wz + (1 - alpha) * wz_filt_last;
    wx_filt_last = state.wx;
    wy_filt_last = state.wy;
    wz_filt_last = state.wz;
}