#include <Wire.h>
#include <7Semi_BNO055.h>
#include <MadgwickAHRS.h>
#include <Servo.h>

/* ================= IMU ================= */
BNO055_7Semi imu;
Madgwick filter;

/* ================= Thrusters ================= */
Servo T1, T2, T5;
const int PIN_T1 = 21;
const int PIN_T2 = 22;
const int PIN_T5 = 23;

/* ================= Timing ================= */
const float dt = 0.005;   // 200 Hz
unsigned long lastMicros;

/* ================= Madgwick ================= */
float beta = 0.05f;

/* ================= Control Gains ================= */
float Kp_roll = 5.0;
float Ki_roll = 1.0;
float Kp_pitch = 5.0;
float Ki_pitch = 1.0;

float K_rp[3][2] = {
  {10.0,  0.0},   // T1
  { 0.0, 10.0},   // T2
  { 5.0,  5.0}    // T5
};

/* ================= State ================= */
float roll = 0, pitch = 0;
float p = 0, q = 0;
float int_roll = 0, int_pitch = 0;

/* ================= Desired ================= */
float roll_des = 0.0;
float pitch_des = 0.0;

/* ================= Thrust ================= */
float F_trim[3] = { 10.0, 10.0, 10.0 };
float max_thrust = 40.0;

/* ================= Utility ================= */
int thrustToPWM(float u) {
  u = constrain(u, 0.0, max_thrust);
  float norm = u / max_thrust;
  return 1500 + norm * 160;
}

/* ================= Calibration Print ================= */
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

/* ================= Setup ================= */
void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println(F("\nBNO055 demo"));

  // Initialize (Wire, address, use external crystal)
  if (!imu.begin(Wire, 32, 33, /*useExtCrystal=*/true)) {
    Serial.println(F("BNO055 not found"));
    while (1) delay(1000);
  }

  /* Raw sensor mode (NO fusion) */
  imu.setMode(Mode::AMG);

  /* Wait for calibration */
  Serial.print("Calibrating");
  imu.waitCalibrated(10000, 200);
  Serial.println(" done");
  printCalib();

  filter.begin(1.0f / dt);

  T1.attach(PIN_T1);
  T2.attach(PIN_T2);
  T5.attach(PIN_T5);

  T1.writeMicroseconds(1500);
  T2.writeMicroseconds(1500);
  T5.writeMicroseconds(1500);
  delay(5000);

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

  p = gxf;
  q = gyf;
}

/* ================= Control Loop ================= */
void controlLoop() {

  float e_roll = roll_des - roll;
  float e_pitch = pitch_des - pitch;

  int_roll += e_roll * dt;
  int_pitch += e_pitch * dt;

  float p_ref = Kp_roll * e_roll + Ki_roll * int_roll;
  float q_ref = Kp_pitch * e_pitch + Ki_pitch * int_pitch;

  float omega_err[2] = {
    p - p_ref,
    q - q_ref
  };

  float thrust[3];
  for (int i = 0; i < 3; i++) {
    thrust[i] = F_trim[i]
      - K_rp[i][0] * omega_err[0]
      - K_rp[i][1] * omega_err[1];

    thrust[i] = constrain(thrust[i], 0.0, max_thrust);
  }

  T1.writeMicroseconds(thrustToPWM(thrust[0]));
  T2.writeMicroseconds(thrustToPWM(thrust[1]));
  T5.writeMicroseconds(thrustToPWM(thrust[2]));
}

/* ================= Main Loop ================= */
void loop() {
  if ((micros() - lastMicros) < dt * 1e6) return;
  lastMicros = micros();

  readIMU();
  controlLoop();
}