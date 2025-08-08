#include "MT6701.hpp"
#include <SimpleFOC.h>

MT6701  encoder;
float   readMt6701() { return encoder.getAngleRadians(); }
void    initMt6701() { encoder.begin(); }
GenericSensor sensor(readMt6701, initMt6701);

BLDCMotor       motor  = BLDCMotor(11, 5.55f, 100);
BLDCDriver3PWM  driver = BLDCDriver3PWM(D1, D2, D3, D0);

Commander commander = Commander(Serial);

void onMotor(char* cmd){ commander.motor(&motor, cmd); }

void zeroPids(char*) {
  motor.PID_velocity.reset();
  motor.P_angle.reset();
  Serial.println(F("PIDs reset"));
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  driver.voltage_power_supply = 15;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);

  sensor.init();
  motor.linkSensor(&sensor);

  motor.controller = MotionControlType::angle;
  motor.current_limit = 0.7;
  motor.voltage_limit = 6;
  // motor.velocity_limit = 4;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.LPF_velocity.Tf = 0.01; 

  motor.P_angle.P = 5;

  motor.useMonitoring(Serial);

  motor.zero_electric_angle = 2.251883;
  motor.sensor_direction = Direction::CCW;

  motor.init();
  motor.initFOC();
  Serial.println("FOC ready (position control)");


  commander.add('M', onMotor);
  commander.add('Z', zeroPids, "zero pids");
}

void loop() {
  motor.loopFOC();
  motor.move(2.0f);

  commander.run();
}
