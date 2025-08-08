#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

// GM4108: 22 poles -> 11 pp,  11.1 ohm line-to-line -> 5.55 ohm per phase
BLDCMotor       motor  = BLDCMotor(11, 5.55f, 100);
BLDCDriver3PWM  driver = BLDCDriver3PWM(D1, D2, D3, D0);

void setup() {
  Serial.begin(115200);
  
  sensor.init();
  
  driver.voltage_power_supply = 16;
  driver.pwm_frequency        = 25000;
  driver.init();
  motor.linkDriver(&driver);
  
  // motor.voltage_limit = 6;
  motor.controller = MotionControlType::velocity_openloop;

  motor.init();
  motor.initFOC();
  motor.target = 6.0f;
}

void loop() {
  sensor.update();
  motor.loopFOC();
  motor.move();
  Serial.print(sensor.getAngle());
  Serial.print('\t');
  Serial.println(sensor.getVelocity());
}