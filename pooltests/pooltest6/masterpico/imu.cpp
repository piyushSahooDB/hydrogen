#include "config.hpp"
#include "imu.hpp"
#include "structs.hpp"

#define I2C_PORT i2c1
static int addr = 0x28;

extern State state;

static float roll0 = 0.0f;
static float pitch0 = 0.0f;
static float yaw0 = 0.0f;

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

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_pull_up(27);
    gpio_pull_up(26);

    uint8_t buffer[6];

    uint8_t reg = 0x00;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, chipID, 1, false);

    while (1) {
        if (chipID[0] != 0xA0) {
            printf("Chip ID Not Correct\n");
            sleep_ms(100);
        }
    }

    printf("BNO055 connected\n");
    sleep_ms(1000);

    // Use internal oscillator
    uint8_t data[2];
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Set operation to NDOF (Absolute Orientation) to enable all sensors + fusion
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(100);

    // read roll and pitch for lock values
    uint8_t reg_euler = 0x1A;
    i2c_write_blocking(I2C_PORT, addr, &reg_euler, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    yaw0 = ((buffer[1] << 8) | buffer[0]) / 0.27925;  // rad/s
    roll0 = ((buffer[3] << 8) | buffer[2]) / 0.27925;
    pitch0 = ((buffer[5] << 8) | buffer[4]) / 0.27925;

    printf("Roll, Pitch, Yaw locked\n")
}

void imu::update() {

    uint8_t reg_euler = 0x1A;
    i2c_write_blocking(I2C_PORT, addr, &reg_euler, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    state.yaw = ((buffer[1] << 8) | buffer[0]);
    state.roll = ((buffer[3] << 8) | buffer[2]);
    state.pitch = ((buffer[5] << 8) | buffer[4]);
    state.roll = wrapAngle((state.roll / 0.27925) - roll0);  // rad/s
    state.pitch = wrapAngle((state.pitch / 0.27925) - pitch0);
    // Deadband for small angles
    if (abs(state.roll) < 0.02) state.roll = 0;
    if (abs(state.pitch) < 0.02) state.pitch = 0;

    uint8_t reg_gyro = 0x14;
    i2c_write_blocking(I2C_PORT, addr, &reg_gyro, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
    state.wx = ((buffer[1] << 8) | buffer[0]) / 0.27925;  // rad/s
    state.wy = ((buffer[3] << 8) | buffer[2]) / 0.27925;
    state.wz = ((buffer[5] << 8) | buffer[4]) / 0.27925;
    //low pass filter
    state.wx = alpha * state.wx + (1 - alpha) * wx_filt_last;
    state.wy = alpha * state.wy + (1 - alpha) * wy_filt_last;
    state.wz = alpha * state.wz + (1 - alpha) * wz_filt_last;
    wx_filt_last = state.wx;
    wy_filt_last = state.wy;
    wz_filt_last = state.wz;

    // if (!bno055.readEuler(state.heading, state.roll, state.pitch)) return;
    // state.roll = wrapAngle(state.roll - roll0);
    // state.pitch = wrapAngle(state.pitch - pitch0);
    // // Deadband for small angles
    // if (abs(state.roll) < 0.02) state.roll = 0;
    // if (abs(state.pitch) < 0.02) state.pitch = 0;


    // bno055.readGyro(gx, gy, gz);
    // // Convert gyro to rad/s
    // state.wx = gx * DEG_TO_RAD;
    // state.wy = gy * DEG_TO_RAD;
    // state.wz = gz * DEG_TO_RAD;
    // //low pass filter
    // state.wx = alpha * state.wx + (1 - alpha) * wx_filt_last;
    // state.wy = alpha * state.wy + (1 - alpha) * wy_filt_last;
    // state.wz = alpha * state.wz + (1 - alpha) * wz_filt_last;
    // wx_filt_last = state.wx;
    // wy_filt_last = state.wy;
    // wz_filt_last = state.wz;
}