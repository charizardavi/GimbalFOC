#include <SimpleFOC.h>


MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(D1, D2, D3, D0);

Commander commander = Commander(Serial);
void onMotor(char* cmd) {
  commander.motor(&motor, cmd);
}

static uint64_t lastFOC = 0;         // micros() is 64-bit on ESP32-S3
const uint32_t FOC_PERIOD_US = 200;  // 5 kHz update rate

void zeroPids(char*) {
  motor.PID_velocity.reset();
  Serial.println(F("Velocity PID reset"));
}

void setup() {
  Serial.begin(115200);


  // — DRIVER —
  driver.voltage_power_supply = 16;  // 15 V rail
  driver.pwm_frequency = 25000;      // 25 kHz PWM
  driver.init();
  motor.linkDriver(&driver);

  // — SENSOR —
  sensor.init();
  motor.linkSensor(&sensor);
  
  // — MOTOR CONTROL —
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = 7.0;  // ≤0.7 A ➜ <3 W copper loss

  // motor.voltage_limit = 6;                // extra safety
  // motor.velocity_limit= 20;               // rad s-1 (≈190 rpm)
  // Velocity PID (rad s-1)
  motor.PID_velocity.P = 0.4;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  // manual calibration (optional, remove to auto-cal):
  // motor.zero_electric_angle = 2.066273;
  // motor.sensor_direction = Direction::CCW;
  // Init FOC
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
  Serial.println("FOC ready (velocity)");
  // Commander CLI
  commander.add('M', onMotor);
  commander.add('Z', zeroPids, "zero pids");
  Serial.println("\nType  M  <enter>  for command list");
}

// ────────── MAIN LOOP ──────────────────────────────────────────
void loop() {
  motor.loopFOC();
  motor.move(2.0);     // default target = motor.target
  // commander.run();  // handle CLI
                    // debug stream (angle [rad]  |  velocity [rad s-1])
                    // Serial.print(sensor.getAngle());
                    // Serial.print('\t');
                    // Serial.println(sensor.getVelocity());
}
