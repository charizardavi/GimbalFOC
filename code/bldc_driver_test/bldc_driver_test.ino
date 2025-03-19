#include <SimpleFOC.h>
#include "AS5600.h"

// Motor instance
BLDCMotor motor = BLDCMotor(11);

// Driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);


AS5600 as5600;


float target_velocity = 5;  // Adjust this as needed

void setup() {
  // Serial monitor for debugging
  Serial.begin(115200);

  // Initialize sensor
  Wire.begin();  // Start I2C communication


  // Configure driver hardware
  driver.voltage_power_supply = 16;  // Set power supply voltage
  driver.voltage_limit = 15;         // Set safe voltage limit

  // Initialize driver
  if (!driver.init()) {
    Serial.println("Failed driver init");
    return;
  }

  // Link motor to driver
  motor.linkDriver(&driver);
  motor.voltage_limit = 10;

  // Set control loop type to velocity open-loop
  motor.controller = MotionControlType::velocity_openloop;


  // Initialize motor
  if (!motor.init()) {
    Serial.println("Motor initialization failed!");
  } else {
    Serial.println("Motor and driver initialized.");
    Serial.println("Motor will start rotating at the set target velocity.");
  }

  // Set target velocity
  motor.target = target_velocity;
  _delay(1000);


  as5600.begin(4);                         //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
}

void loop() {
  // Move motor based on open-loop velocity
  motor.move();

  if (Wire.requestFrom(0x36, 1) == 1) {
    Serial.print(as5600.readAngle());
    Serial.print("\t");
    Serial.println(as5600.rawAngle() * AS5600_RAW_TO_RADIANS);
  } else {
    Serial.println("AS5600 not responding!");
  }
}
