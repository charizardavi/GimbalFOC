#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(D1, D2, D3, D0);

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor, cmd); }

void zeroPids(char*) {
  motor.PID_velocity.reset();
  Serial.println(F("Velocity PID reset"));
}


void setup() {
  Serial.begin(115200);

  driver.voltage_power_supply = 16;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);

  sensor.init();
  motor.linkSensor(&sensor);

  motor.phase_resistance = 5.55;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // motor.zero_electric_angle = 2.066273;
  // motor.sensor_direction    = Direction::CCW;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

  Serial.println(F("FOC ready (torque control)"));

  commander.add('M', onMotor);
  commander.add('Z', zeroPids, "zero pids");
}

void loop() {
  motor.loopFOC();
  motor.move(.5);
  commander.run();

  Serial.print(sensor.getAngle());
  Serial.print('\t');
  Serial.println(sensor.getVelocity());
}
