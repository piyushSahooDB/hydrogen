#include <Wire.h>
#include <7Semi_BNO055.h>
#include <ESP32Servo.h>

BNO055_7Semi imu;

/* ================= ORIENTATION ================= */
float roll, pitch, heading;
float roll0 = 0.0;
float pitch0 = 0.0;

/* ================= GYRO ================= */
int16_t gx, gy, gz;   // deg/s

/* ================= PID GAINS ================= */
// Roll (angle)
float Kp_roll = 2.0;
float Ki_roll = 0.05;
float Kd_roll = 0.2;

// Pitch (angle)
float Kp_pitch = 2.0;
float Ki_pitch = 0.05;
float Kd_pitch = 0.2;

// Yaw (RATE control → keep small)
float Kp_yaw = 1.2;
float Ki_yaw = 0.01;
float Kd_yaw = 0.05;

/* ================= PID STATES ================= */
float rollError, rollLastError = 0, rollIntegral = 0;
float pitchError, pitchLastError = 0, pitchIntegral = 0;
float yawRateError, yawLastError = 0, yawIntegral = 0;

/* ================= LIMITS ================= */
const float I_MAX = 100.0;

/* ================= TIMING ================= */
unsigned long lastTime = 0;
const float dt = 0.1;   // 100 ms (stable for water)

/* ================= THRUSTERS ================= */
// Vertical
Servo T1;  // back right
Servo T2;  // back left
Servo FM;  // front middle

// Horizontal
Servo HL;  // left
Servo HR;  // right

/* ================= PINS ================= */
const int PIN_T1 = 18;
const int PIN_T2 = 19;
const int PIN_FM = 26;
const int PIN_HL = 27;
const int PIN_HR = 14;

/* ================= PWM ================= */
const int PWM_NEUTRAL = 1500;
const int PWM_MIN = 1300;
const int PWM_MAX = 1700;

/* ================= ANGLE WRAP ================= */
float angleError(float angle, float reference) {
  float err = angle - reference;
  if (err > 180.0) err -= 360.0;
  if (err < -180.0) err += 360.0;
  return err;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22, 100000);

  if (!imu.begin(Wire, 21, 22, false)) {
    Serial.println(" BNO055 NOT DETECTED");
    while (1);
  }

  imu.setMode(Mode::CONFIG);
  delay(50);
  imu.setMode(Mode::IMUPLUS);   // NO magnetometer
  delay(50);

  Serial.println(" Hold ROV still (5s) to lock roll & pitch...");
  delay(5000);

  if (imu.readEuler(heading, roll, pitch)) {
    roll0  = roll;
    pitch0 = pitch;
  }

  Serial.println(" Roll & Pitch locked");

  /* ================= ATTACH THRUSTERS ================= */
  T1.attach(PIN_T1);
  T2.attach(PIN_T2);
  FM.attach(PIN_FM);
  HL.attach(PIN_HL);
  HR.attach(PIN_HR);

  T1.writeMicroseconds(PWM_NEUTRAL);
  T2.writeMicroseconds(PWM_NEUTRAL);
  FM.writeMicroseconds(PWM_NEUTRAL);
  HL.writeMicroseconds(PWM_NEUTRAL);
  HR.writeMicroseconds(PWM_NEUTRAL);

  lastTime = millis();

  Serial.println("Roll Pitch | rPID pPID yPID | T1 T2 FM HL HR");
}

void loop() {
  if (millis() - lastTime < dt * 1000) return;
  lastTime = millis();

  if (!imu.readEuler(heading, roll, pitch)) {
    Serial.println(" IMU READ FAILED");
    return;
  }

  imu.readGyro(gx, gy, gz);   // deg/s

  /* ================= ANGLE ERRORS ================= */
  rollError  = angleError(roll, roll0);
  pitchError = angleError(pitch, pitch0);

  /* ================= YAW RATE ERROR ================= */
  yawRateError = -gz;   // want yaw rate = 0 deg/s

  /* ================= INTEGRALS ================= */
  rollIntegral  += rollError * dt;
  pitchIntegral += pitchError * dt;
  yawIntegral   += yawRateError * dt;

  rollIntegral  = constrain(rollIntegral,  -I_MAX, I_MAX);
  pitchIntegral = constrain(pitchIntegral, -I_MAX, I_MAX);
  yawIntegral   = constrain(yawIntegral,   -I_MAX, I_MAX);

  /* ================= DERIVATIVES ================= */
  float rollD  = (rollError  - rollLastError)  / dt;
  float pitchD = (pitchError - pitchLastError) / dt;
  float yawD   = (yawRateError - yawLastError) / dt;

  rollLastError  = rollError;
  pitchLastError = pitchError;
  yawLastError   = yawRateError;

  /* ================= PID OUTPUTS ================= */
  float rollPID =
      Kp_roll * rollError +
      Ki_roll * rollIntegral +
      Kd_roll * rollD;

  float pitchPID =
      Kp_pitch * pitchError +
      Ki_pitch * pitchIntegral +
      Kd_pitch * pitchD;

  float yawPID =
      Kp_yaw * yawRateError +
      Ki_yaw * yawIntegral +
      Kd_yaw * yawD;

  /* ================= VERTICAL MIXING ================= */
  int pwm_T1 = PWM_NEUTRAL + rollPID - pitchPID;
  int pwm_T2 = PWM_NEUTRAL - rollPID - pitchPID;
  int pwm_FM = PWM_NEUTRAL + pitchPID;

  /* ================= HORIZONTAL (YAW RATE) ================= */
  int pwm_HL = PWM_NEUTRAL + yawPID;
  int pwm_HR = PWM_NEUTRAL - yawPID;

  pwm_T1 = constrain(pwm_T1, PWM_MIN, PWM_MAX);
  pwm_T2 = constrain(pwm_T2, PWM_MIN, PWM_MAX);
  pwm_FM = constrain(pwm_FM, PWM_MIN, PWM_MAX);
  pwm_HL = constrain(pwm_HL, PWM_MIN, PWM_MAX);
  pwm_HR = constrain(pwm_HR, PWM_MIN, PWM_MAX);

  /* ================= WRITE ================= */
  T1.writeMicroseconds(pwm_T1);
  T2.writeMicroseconds(pwm_T2);
  FM.writeMicroseconds(pwm_FM);
  HL.writeMicroseconds(pwm_HL);
  HR.writeMicroseconds(pwm_HR);

  /* ================= DEBUG ================= */
  Serial.print(roll,1); Serial.print(" ");
  Serial.print(pitch,1); Serial.print(" | ");
  Serial.print(rollPID,1); Serial.print(" ");
  Serial.print(pitchPID,1); Serial.print(" ");
  Serial.print(yawPID,1); Serial.print(" | ");
  Serial.print(pwm_T1); Serial.print(" ");
  Serial.print(pwm_T2); Serial.print(" ");
  Serial.print(pwm_FM); Serial.print(" ");
  Serial.print(pwm_HL); Serial.print(" ");
  Serial.println(pwm_HR);
}
