#include <Wire.h>
#include <7Semi_BNO055.h>
#include "pico_esc.hpp"

BNO055_7Semi imu;

#define RP_TXPIN   17
#define RP_RXPIN   16
#define RP_BAUDRATE  115200

/* ================= TIMING ================= */
unsigned long lastTime = 0;
const float dt = 0.1;   // 100 ms control loop

/* ================= NON-BLOCKING SERIAL ================= */
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200; // 5 Hz logging

/* ================= ORIENTATION ================= */
float roll, pitch, heading;
float roll0 = 0.0;
float pitch0 = 0.0;

/* ================= GYRO ================= */
int16_t gx, gy, gz;
float wx_filt_last = 0, wy_filt_last = 0, wz_filt_last = 0;
const float alpha = 0.1; // low-pass filter coefficient

/* ================= YAW PID ================= */
float Kp_yaw = 5;
float Ki_yaw = 0.01;
float Kd_yaw = 0.05;
float yawIntegral = 0;
float yawLastError = 0;
const float I_MAX = 100.0;

/* ================= OUTER LOOP PI (ANGLES) ================= */
float Kp_ang = 5.0;
float Ki_ang = 1.0;
float rollInt = 0;
float pitchInt = 0;

/* ================= INNER LOOP LQR (RATES) ================= */
float K_lqr[3][2] = {
  { 1.5, -0.1},
  { -1.5, -0.1 },
  { 0.0, 2.0 }
};
const float U_MAX = 1.0;
float u_smooth[3] = { 0, 0, 0 };
const float beta = 0.2; // LQR output smoothing factor

/* ================= PWM ================= */
const int PWM_NEUTRAL = 1500;

/* ================= FUNCTIONS ================= */
int clampPWM(int value) {
  if (value >= 0)
    return constrain(value + 48, 0, 1000);
  else if (value <= 0)
    return constrain(-value + 1049, 1001, 2000);
}

float wrapAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

int pwm_HL = 0, pwm_HR = 0, zoffset = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial2.begin(RP_BAUDRATE, SERIAL_8N1, RP_RXPIN, RP_TXPIN);
  delay(500);

  Wire.begin(21, 22, 100000);

  if (!imu.begin(Wire, 21, 22, false)) {
    Serial.println("BNO055 NOT DETECTED");
    while (1);
  }

  imu.setMode(Mode::CONFIG);
  delay(50);
  imu.setMode(Mode::IMUPLUS); // use IMUPLUS for fast rate feedback
  delay(50);

  imu.readEuler(heading, roll, pitch);
  roll0 = roll;
  pitch0 = pitch;
  Serial.println("Roll & Pitch locked");

  pico_esc::pico_uart_init();
  pico_esc::arm_thrusters();

  lastTime = millis();
  lastPrintTime = millis();
}

void loop() {

  if (Serial2.available() > 0) {
    uint8_t rec = Serial2.read();
    while (Serial2.available() > 0) {
      Serial2.read();
    }
    if (rec >= 0 && rec <= 6) {
      Serial.print("rec");
      Serial.print(rec, HEX);
      if (rec == 1) {
        pwm_HL = 300;
        pwm_HR = 300;
      }
      else if (rec == 2) {
        pwm_HL = 1300;
        pwm_HR = 1300;
      }
      else if (rec == 3) {
        pwm_HL = 1300;
        pwm_HR = 300;
      }
      else if (rec == 4) {
        pwm_HL = 300;
        pwm_HR = 1300;
      }
      else if (rec == 5) {
        zoffset += 5;
        zoffset = constrain(zoffset, 0, 500);
      }
      else if (rec == 6) {
        zoffset -= 5;
        zoffset = constrain(zoffset, 0, 500);
      }
    }
    else {
      Serial.print("transmission error");
      Serial.println(rec);
    }
  }

  pwm_HL = 300;
  pwm_HR = 300;

  if (millis() - lastTime < dt * 1000) return;
  lastTime = millis();

  /* ================= READ IMU ================= */
  if (!imu.readEuler(heading, roll, pitch)) return;
  imu.readGyro(gx, gy, gz);

  // Wrap angles relative to initial
  roll = wrapAngle(roll - roll0);
  pitch = wrapAngle(pitch - pitch0);

  // Deadband for small angles
  if (abs(roll) < 0.02) roll = 0;
  if (abs(pitch) < 0.02) pitch = 0;

  // Convert gyro to rad/s
  float wx = gx * DEG_TO_RAD;
  float wy = gy * DEG_TO_RAD;
  float wz = gz * DEG_TO_RAD;

  // ================= LOW-PASS FILTER =================
  wx = alpha * wx + (1 - alpha) * wx_filt_last;
  wy = alpha * wy + (1 - alpha) * wy_filt_last;
  wz = alpha * wz + (1 - alpha) * wz_filt_last;
  wx_filt_last = wx;
  wy_filt_last = wy;
  wz_filt_last = wz;

  // ================= OUTER LOOP PI =================
  float roll_rad = roll * DEG_TO_RAD;
  float pitch_rad = pitch * DEG_TO_RAD;

  rollInt += roll_rad * dt;
  pitchInt += pitch_rad * dt;

  // Limit integrator to prevent windup
  rollInt = constrain(rollInt, -0.02, 0.02);
  pitchInt = constrain(pitchInt, -0.02, 0.02);

  float wx_ref = Kp_ang * roll_rad + Ki_ang * rollInt;
  float wy_ref = Kp_ang * pitch_rad + Ki_ang * pitchInt;

  // ================= INNER LOOP LQR =================
  float omega_err[2] = { wx - wx_ref, wy - wy_ref };

  // Deadband for gyro errors
  for (int i = 0; i < 2; i++) {
    if (abs(omega_err[i]) < 0.1) omega_err[i] = 0;
  }

  float u[3];
  u[0] = -(K_lqr[0][0] * omega_err[0] + K_lqr[0][1] * omega_err[1]);
  u[1] = -(K_lqr[1][0] * omega_err[0] + K_lqr[1][1] * omega_err[1]);
  u[2] = -(K_lqr[2][0] * omega_err[0] + K_lqr[2][1] * omega_err[1]);

  // Normalize LQR outputs
  float umax = max(max(abs(u[0]), abs(u[1])), abs(u[2]));
  if (umax > U_MAX) {
    u[0] *= U_MAX / umax;
    u[1] *= U_MAX / umax;
    u[2] *= U_MAX / umax;
  }

  // ================= SMOOTH LQR OUTPUT =================
  for (int i = 0; i < 3; i++) {
    u_smooth[i] = beta * u[i] + (1 - beta) * u_smooth[i];
  }

  // ================= YAW PID =================
  float yawRateError = -wz;
  yawIntegral += yawRateError * dt;
  yawIntegral = constrain(yawIntegral, -I_MAX, I_MAX);

  float yaw_u = Kp_yaw * yawRateError + Ki_yaw * yawIntegral +
    Kd_yaw * ((yawRateError - yawLastError) / dt);
  yawLastError = yawRateError;

  // ================= THRUSTER MIXING =================
  // Vertical thrusters: LQR only
  int pwm_T1 = clampPWM(u_smooth[0] * 0 + zoffset);
  int pwm_T2 = clampPWM(u_smooth[1] * 0 + zoffset);
  int pwm_T3 = clampPWM(u_smooth[2] * 0 + zoffset);

  // Horizontal thrusters: yaw only
  // int pwm_HL = clampPWM(+yaw_u * 5);
  // int pwm_HR = clampPWM(-yaw_u * 5);

  // Write to servos
  uint16_t throttle[5] = { pwm_T1,pwm_T2,pwm_T3,pwm_HR,pwm_HL };
  for (int i = 0;i < 5;i++) {
    Serial1.write(0b00010000 | i);      //sending address
    pico_esc::send_escframe(throttle[i]);
  }

  // ================= TELEMETRY =================
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = millis();
    Serial.print("Roll_rel:"); Serial.print(roll, 2);
    Serial.print(" Pitch_rel:"); Serial.print(pitch, 2);
    Serial.print(" YawRate(rad/s):"); Serial.print(wz, 3);
    Serial.print(" | ");
    Serial.print("T1:"); Serial.print(pwm_T1);
    Serial.print(" T2:"); Serial.print(pwm_T2);
    Serial.print(" T3:"); Serial.print(pwm_T3);
    Serial.print(" HL:"); Serial.print(pwm_HL);
    Serial.print(" HR:"); Serial.println(pwm_HR);
  }
  pwm_HL = 0;
  pwm_HR = 0;
}