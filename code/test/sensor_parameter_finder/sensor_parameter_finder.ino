#include <Wire.h>
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

BLDCMotor      motor  (11, 5.55f, 100);
BLDCDriver3PWM driver (D1, D2, D3, D0);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();

  driver.voltage_power_supply = 15;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);

  sensor.init();
  motor.linkSensor(&sensor);

  motor.current_limit = 0.7;
  motor.voltage_limit = 4;
  motor.controller = MotionControlType::torque;

  motor.init();

  motor.initFOC();

  Serial.println(F("Calibration finished"));
  Serial.print  (F("zero_electric_offset = "));
  Serial.println(motor.zero_electric_angle, 6);
  Serial.print  (F("sensor_direction = Direction::"));
  Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");

  motor.disable();
  Serial.println(F("Driver disabled. Calibration complete."));
}


void loop() {}
