/******************************************************************
 *  GM4108 + DRV8313 + MT6701 -- closed-loop POSITION control
 *  with SimpleFOC Commander interface
 ******************************************************************/
#include "MT6701.hpp"
#include <SimpleFOC.h>

// ------------ sensor wrapper (GenericSensor) -------------------
MT6701  encoder;
float   readMt6701() { return encoder.getAngleRadians(); }
void    initMt6701() { encoder.begin(); }
GenericSensor sensor(readMt6701, initMt6701);

// ------------ motor + driver -----------------------------------
BLDCMotor       motor  = BLDCMotor(11, 5.55f, 100);     // 11 pp, 5.55 Ω, 100 rpm/V
BLDCDriver3PWM  driver = BLDCDriver3PWM(D1, D2, D3, D0); // phase A,B,C, enable

// ------------ Commander ----------------------------------------
Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor, cmd); } // callback
void zeroPids(char*) {
  motor.PID_velocity.reset();   // clears integrator & dTerm memory
  motor.P_angle.reset();
  Serial.println(F("PIDs reset"));
}

// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while(!Serial);              // USB-CDC: wait for PC to open the port

  // DRIVER -------------------------------------------------------
  driver.voltage_power_supply = 15;      // real supply voltage
  driver.pwm_frequency        = 25000;   // 25 kHz keeps DRV8313 quiet
  driver.init();
  motor.linkDriver(&driver);

  // SENSOR -------------------------------------------------------
  sensor.init();
  motor.linkSensor(&sensor);

  // MOTOR CONTROL ------------------------------------------------
  motor.controller     = MotionControlType::angle;
  motor.current_limit  = 0.7;   // ≤0.7 A → <3 W copper loss
  motor.voltage_limit  = 6;     // safety net for start-up
  // motor.velocity_limit = 4;     // max 4 rad s-¹ slew

  motor.PID_velocity.P  = 0.2;
  motor.PID_velocity.I  = 20;
  motor.LPF_velocity.Tf = 0.01; // 10 ms

  motor.P_angle.P = 5;

  motor.useMonitoring(Serial);  // live prints if you enable it in Commander

  motor.zero_electric_angle = 2.251883;
  motor.sensor_direction = Direction::CCW;

  motor.init();
  motor.initFOC();
  Serial.println("FOC ready");

  // COMMANDER ----------------------------------------------------
  commander.add('M', onMotor);  // ’M’ = motor command tree
  commander.add('Z', zeroPids, "zero pids");
  Serial.println("\nType  M  then <enter> for the command list");
}

// ----------------------------------------------------------------
void loop() {
  motor.loopFOC();          // keep this fast (>1 kHz)
  motor.move(2.0f);         // hold 2 rad (~115 °)

  commander.run();          // poll the serial port
}
