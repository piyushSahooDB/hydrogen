#include <Arduino.h>
#include <Wire.h>
#include <7Semi_BNO055.h>
#include <MadgwickAHRS.h>

/* ================= IMU ================= */
BNO055_7Semi imu;
Madgwick filter;
/* ================= Timing ================= */
const float dt = 0.005;   // 200 Hz
unsigned long lastMicros;
/* ================= State ================= */
float roll = 0, pitch = 0;

int i = 0;

void printCalib() {
  uint8_t sys, gyr, acc, mag;
  imu.calibBreakdown(sys, gyr, acc, mag);
  Serial.print("Calib SYS:");
  Serial.print(sys);
  Serial.print(" G:");
  Serial.print(gyr);
  Serial.print(" A:");
  Serial.print(acc);
  Serial.print(" M:");
  Serial.println(mag);
}
void setup() {

  Serial.begin(115200);

  delay(500);
  Serial.println(F("\nsearching BNO055"));
  if (!imu.begin(Wire, 21, 22, true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }
  Serial.println(F("Found BNO055"));
  imu.setMode(Mode::AMG);
  Serial.print("Calibrating");
  imu.waitCalibrated(10000, 200);
  Serial.println(" done");
  printCalib();

  filter.begin(1.0f / dt);
  filter.setBeta(0.5);

  lastMicros = micros();
}

/* ================= Read IMU ================= */
void readIMU() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  imu.readAccel(ax, ay, az);
  imu.readGyro(gx, gy, gz);

  /* Convert units */
  float axf = ax * 0.01f;
  float ayf = ay * 0.01f;
  float azf = az * 0.01f;

  float gxf = gx * DEG_TO_RAD * 0.01f;
  float gyf = gy * DEG_TO_RAD * 0.01f;
  float gzf = gz * DEG_TO_RAD * 0.01f;

  filter.updateIMU(gxf, gyf, gzf, axf, ayf, azf);

  roll = filter.getRoll() * DEG_TO_RAD;
  pitch = filter.getPitch() * DEG_TO_RAD;
}
/* ================= Main Loop ================= */
void loop() {
  if ((micros() - lastMicros) < dt * 1e6) return;
  lastMicros = micros();

  readIMU();
  i++;

  if (i == 10) {
    Serial.print(roll);
    Serial.print("    ");
    Serial.println(pitch);
    i = 0;
  }
}