#include <SimpleFOC.h>
#include <SoftwareSerial.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);


float target_angle = 0;
SoftwareSerial serial1 (8, 9);

void setup() {
  serial1.begin(9600);
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 15;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 10;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 2;
  
  // maximal velocity of the position control
  motor.velocity_limit = 20;
  motor.current_limit = 1.2;

  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


void loop() {
  motor.loopFOC();

  motor.move(target_angle);
  
  if (serial1.available() >= sizeof(float)) {
      // Receive the float value from the first Arduino
      float receivedValue;
      serial1.readBytes((char*)&receivedValue, sizeof(receivedValue));
      
      // Print the received value
      Serial.print("Received: ");
      Serial.println(receivedValue);

      target_angle = receivedValue;
  }
}