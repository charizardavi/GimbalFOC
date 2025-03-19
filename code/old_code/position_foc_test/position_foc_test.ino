#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);


float target_angle = 0;
float target_offset = 0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doP_Angle(char* cmd) { command.scalar(&motor.P_angle.P, cmd); }
void onMotor(char* cmd){command.motor(&motor, cmd);}

void setup() {

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
  
  // maximal voltage to be set to the motor
  motor.voltage_limit = 15;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // PITCH:
  motor.PID_velocity.P = .25f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;

  motor.P_angle.P = 35.0F;
  motor.P_angle.I = 4.0F;
  motor.P_angle.D = 0.0F;
  

  
  // maximal velocity of the position control
  motor.velocity_limit = 300;

  motor.useMonitoring(Serial);

  // motor.sensor_direction=Direction::CCW;
  // motor.zero_electric_angle=6.19;


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  command.add('T', doTarget, "target angle");
  command.add('P', doP_Angle, "angle controller P");
  command.add('M',onMotor,"full motor config");
  
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);


  target_offset = motor.shaft_angle;
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle + target_offset);

  
  command.run();
}