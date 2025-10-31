#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

const int addr = 0x68;

int16_t ax, ay, az, gx, gy, gz;

Matrix<3,6> K = {
  0.4472, 0.1757, -0.0000, -0.0000, -0.0000, -0.0000,
  0.0000, 0.0000,  0.4472,  0.1348, -0.0000, -0.0000,
  0.0000, -0.0000, 0.0000,  0.0000,  0.4472,  0.6760
};

float pitch = 0.0, roll = 0.0;
float vz = 0.0, z = 0.0;
unsigned long lastTime = 0;
const float alpha = 0.98;
const float g = 9.80665;

const float x1 = 0.37f / 2.0f;
const float y1 = 0.24f;
const float y2 = 0.08f;

Matrix<3,3> FtoX = {
  y2,  y1,  y2,
 -x1, 0.0,  x1,
  1.0, 1.0, 1.0
};
Matrix<3,3> XtoF;

void setup() {
  Wire.begin(21, 22); 
  Serial.begin(9600);

  Wire.beginTransmission(addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  XtoF = Invert(FtoX);
  delay(100);
  lastTime = millis();
}

void loop() {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("I2C error");
    delay(500);
    return;
  }

  if (Wire.requestFrom(addr, 14, true) != 14) {
    Serial.println("Read error");
    delay(500);
    return;
  }

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  ax /= 16384.0;
  ay /= 16384.0;
  az /= 16384.0;
  gx /= 131.0;
  gy /= 131.0;
  gz /= 131.0;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
  float accelRoll  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  pitch += gx * dt;
  roll  += gy * dt;

  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  roll  = alpha * roll  + (1.0 - alpha) * accelRoll;

  float pitchRad = pitch * PI / 180.0;
  float rollRad  = roll * PI / 180.0;
  float az_corrected = az * g;
  float az_linear = az_corrected - g * cos(pitchRad) * cos(rollRad);

  vz += az_linear * dt;
  z  += vz * dt;
  vz *= 0.999; 

  Matrix<6,1> x = {pitch, gx, roll, gy, z, vz};
  Matrix<3,1> u = K * x;
  u = u * 0.05f;
  Matrix<3,1> Forces = XtoF * u;

  for (int i = 0; i < 3; i++) {
    float v = Forces(i);
    v = fmaxf(-0.7f, fminf(v, 0.85f));
    Forces(i) = v;
  }

  // get the values for the forces from F

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("\tRoll: "); Serial.print(roll);
  Serial.print("\tZ: "); Serial.print(z);
  Serial.print("\tVz: "); Serial.print(vz);
  Serial.print("\tF1: "); Serial.print(F(0));
  Serial.print("\tF2: "); Serial.print(F(1));
  Serial.print("\tF3: "); Serial.println(F(2));

  delay(50);
}
