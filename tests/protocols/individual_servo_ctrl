#include <Servo.h>

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);

  servo1.attach(5);
  servo2.attach(11);

  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);

  Serial.setTimeout(5000); // you have 5 seconds to type each value
  delay(2000);
}

int readPWM(const char* prompt) {
  Serial.println(prompt);

  while (Serial.available() == 0);  // wait for input

  int val = Serial.parseInt();

  // clear leftover characters (like newline)
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.print("Read: ");
  Serial.println(val);

  return val;
}

void loop() {
  int pwm1 = readPWM("Enter PWM1 (1400 - 1600):");
  if (pwm1 >= 1400 && pwm1 <= 1600) {
    servo1.writeMicroseconds(pwm1);
  } else {
    Serial.println("PWM1 ignored (out of range).");
  }

  int pwm2 = readPWM("Enter PWM2 (1400 - 1600):");
  if (pwm2 >= 1400 && pwm2 <= 1600) {
    servo2.writeMicroseconds(pwm2);
  } else {
    Serial.println("PWM2 ignored (out of range).");
  }
}
