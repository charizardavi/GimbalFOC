#include <SimpleFOC.h>

// GM4108: 22 poles  ⇒ 11 pp,  11.1 Ω line-to-line  ⇒ 5.55 Ω per phase
BLDCMotor       motor  = BLDCMotor(11, 5.55f, 100);
BLDCDriver3PWM  driver = BLDCDriver3PWM(D1, D2, D3, D0);

void setup() {
  Serial.begin(115200);

  driver.voltage_power_supply = 15;
  driver.pwm_frequency        = 25000;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_limit = 6;
  motor.controller    = MotionControlType::velocity_openloop;

  motor.init();
  motor.initFOC();
  motor.target = 2.0f;           // 10 rad/s
}

void loop() {
  motor.loopFOC();
  motor.move();                   // uses motor.target
}