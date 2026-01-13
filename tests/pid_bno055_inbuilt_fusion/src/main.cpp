#include <Arduino.h>
#include <Wire.h>
#include <7Semi_BNO055.h>
#include <ESP32Servo.h>

BNO055_7Semi imu;

/* ================= ORIENTATION ================= */
float roll, pitch, heading;
float roll0 = 0.0;
float pitch0 = 0.0;

/* ================= PID GAINS ================= */
// Roll
float Kp_roll = 2.0;
float Ki_roll = 0.05;   // REDUCED (important)
float Kd_roll = 0.2;

// Pitch
float Kp_pitch = 2.0;
float Ki_pitch = 0.05;  // REDUCED (important)
float Kd_pitch = 0.2;

/* ================= PID STATES ================= */
float rollError, rollLastError = 0, rollIntegral = 0;
float pitchError, pitchLastError = 0, pitchIntegral = 0;

/* ================= ANTI-WINDUP ================= */
const float I_MAX = 100.0;

/* ================= TIMING ================= */
unsigned long lastTime = 0;
const float dt = 0.1;   // 100 ms

/* ================= THRUSTERS ================= */
Servo T1;  // back right
Servo T2;  // back left
Servo T3;  // front middle

const int PIN_T1 = 18;
const int PIN_T2 = 19;
const int PIN_T3 = 26;

/* ================= PWM LIMITS ================= */
const int PWM_NEUTRAL = 1500;
const int PWM_MIN = 1100;
const int PWM_MAX = 1900;

/* ================= ANGLE WRAP FUNCTION ================= */
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
    Serial.println("❌ BNO055 NOT DETECTED");
    while (1);
  }

  imu.setMode(Mode::CONFIG);
  delay(50);
  imu.setMode(Mode::IMUPLUS);
  delay(50);

  Serial.println("📌 Hold ROV still (5s) to lock orientation...");
  delay(5000);

  if (imu.readEuler(heading, roll, pitch)) {
    roll0 = roll;
    pitch0 = pitch;
    Serial.print("✅ Locked Roll0 = "); Serial.println(roll0, 2);
    Serial.print("✅ Locked Pitch0 = "); Serial.println(pitch0, 2);
  }

  T1.attach(PIN_T1);
  T2.attach(PIN_T2);
  T3.attach(PIN_T3);

  T1.writeMicroseconds(PWM_NEUTRAL);
  T2.writeMicroseconds(PWM_NEUTRAL);
  T3.writeMicroseconds(PWM_NEUTRAL);

  lastTime = millis();

  Serial.println("Roll | Pitch | rPID | pPID | T1 T2 T3");
}

void loop() {
  if (millis() - lastTime < dt * 1000) return;
  lastTime = millis();

  if (!imu.readEuler(heading, roll, pitch)) {
    Serial.println("IMU READ FAILED");
    return;
  }

  /* ================= ANGLE ERRORS (WRAP SAFE) ================= */
  rollError = angleError(roll, roll0);
  pitchError = angleError(pitch, pitch0);

  /* ================= INTEGRAL (ANTI-WINDUP) ================= */
  rollIntegral += rollError * dt;
  pitchIntegral += pitchError * dt;

  rollIntegral = constrain(rollIntegral, -I_MAX, I_MAX);
  pitchIntegral = constrain(pitchIntegral, -I_MAX, I_MAX);

  /* ================= DERIVATIVE ================= */
  float rollDerivative = (rollError - rollLastError) / dt;
  float pitchDerivative = (pitchError - pitchLastError) / dt;

  rollLastError = rollError;
  pitchLastError = pitchError;

  /* ================= PID OUTPUT ================= */
  float rollPID =
    Kp_roll * rollError +
    Ki_roll * rollIntegral +
    Kd_roll * rollDerivative;

  float pitchPID =
    Kp_pitch * pitchError +
    Ki_pitch * pitchIntegral +
    Kd_pitch * pitchDerivative;

  /* ================= 3-THRUSTER MIXING ================= */
  int pwm_T1 = PWM_NEUTRAL + rollPID - pitchPID;  // back right
  int pwm_T2 = PWM_NEUTRAL - rollPID - pitchPID;  // back left
  int pwm_T3 = PWM_NEUTRAL + pitchPID;            // front middle

  pwm_T1 = constrain(pwm_T1, PWM_MIN, PWM_MAX);
  pwm_T2 = constrain(pwm_T2, PWM_MIN, PWM_MAX);
  pwm_T3 = constrain(pwm_T3, PWM_MIN, PWM_MAX);

  /* ================= WRITE PWM ================= */
  T1.writeMicroseconds(pwm_T1);
  T2.writeMicroseconds(pwm_T2);
  T3.writeMicroseconds(pwm_T3);

  /* ================= DEBUG ================= */
  Serial.print(roll, 2); Serial.print(" | ");
  Serial.print(pitch, 2); Serial.print(" | ");
  Serial.print(rollPID, 2); Serial.print(" | ");
  Serial.print(pitchPID, 2); Serial.print(" | ");
  Serial.print(pwm_T1); Serial.print(" ");
  Serial.print(pwm_T2); Serial.print(" ");
  Serial.println(pwm_T3);
}