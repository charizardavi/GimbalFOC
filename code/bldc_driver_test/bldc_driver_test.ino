#include <SimpleFOC.h>

// Motor instance
// Replace 'pole_pairs' with the actual number of pole pairs in your motor.
BLDCMotor motor = BLDCMotor(7);

// Driver instance
// Connect your motor to the appropriate pins.
// Replace 'pwmA', 'pwmB', 'pwmC' with the pin numbers you've wired IN1, IN2, IN3 to.
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

// target velocity in radians per second
float target_velocity = 6.28; // Adjust this value to your needs

void setup() {
  // Serial to display data
  Serial.begin(9600);

  // configure driver hardware
  driver.voltage_power_supply = 16; // Set your power supply voltage
  driver.voltage_limit = 6;         // Set a safe voltage limit

  // initialize driver
  if (!driver.init()){
    Serial.println("failed driver init");
    return;
  }

  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.voltage_limit = 1;

  // set control loop type to velocity
  motor.controller = MotionControlType::velocity_openloop;

  // initialize motor
  if (!motor.init()) {
    Serial.println("Motor initialization failed!");
  } else {
    Serial.println("Motor and driver initialized.");
    Serial.println("Motor will start rotating at the set target velocity.");
  }

  motor.target = target_velocity;
  _delay(1000);
}

void loop() {
  // set motor target velocity
  motor.move();

  // // for debugging, print the actual motor velocity
  Serial.println(motor.shaft_velocity);

  // // a small delay to make the serial output readable
  delay(100);
}
