#include "pico_esc.hpp"
#include "rp_comms.hpp"
#include "imu.hpp"
#include "config.hpp"
#include "structs.hpp"
#include "control.hpp"

State state;
Throttle throttle = { 0, 0, 0, 0, 0, 0 }; ;

void setup() {
  Serial.begin(115200);
  delay(500);

  rp_comms::init();
  delay(500);

  imu::init();

  pico_esc::init();
  pico_esc::arm_thrusters();
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
        throttle.HL = 300;
        throttle.HR = 300;
      }
      else if (rec == 2) {
        throttle.HL = 1300;
        throttle.HR = 1300;
      }
      else if (rec == 3) {
        throttle.HL = 1300;
        throttle.HR = 300;
      }
      else if (rec == 4) {
        throttle.HL = 300;
        throttle.HR = 1300;
      }
      else if (rec == 5) {
        throttle.zoffset += 5;
        throttle.zoffset = constrain(throttle.zoffset, 0, 500);
      }
      else if (rec == 6) {
        throttle.zoffset -= 5;
        throttle.zoffset = constrain(throttle.zoffset, 0, 500);
      }
    }
    else {
      Serial.print("transmission error");
      Serial.println(rec);
    }
  }

  imu::update();
  control::update();

  uint16_t throttleesc[5] = { throttle.VB,throttle.VR,throttle.VL,throttle.HR,throttle.HL };
  for (int i = 0;i < 5;i++) {
    Serial1.write(0b00010000 | i);      //sending address
    pico_esc::send_escframe(throttleesc[i]);
  }

  throttle.HL = 0;
  throttle.HR = 0;
}