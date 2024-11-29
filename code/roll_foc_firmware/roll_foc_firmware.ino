#include <math.h>
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

float target_angle = 0;
float target_offset = 0;

volatile uint16_t risingEdgeTime = 0;
volatile uint16_t fallingEdgeTime = 0;
volatile uint16_t highTime = 0;
volatile uint16_t lowTime = 0;
volatile bool newMeasurementAvailable = false;

void setup() {
  delay(500);

  pinMode(8, INPUT);  // Pin 8 (ICP1) as input
  Serial.begin(115200);

  // Configure Timer1
  TCCR1A = 0;                           // Normal mode
  TCCR1B = (1 << ICES1) | (1 << CS11);  // Input capture on rising edge, prescaler = 8
  TIMSK1 = (1 << ICIE1);                // Enable input capture interrupt

  sei();  // Enable global interrupts

  SimpleFOCDebug::enable(&Serial);
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
  motor.voltage_limit = 20;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // PITCH:
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = .05;
  motor.PID_velocity.D = 0;

  motor.P_angle.P = 200.0f;
  motor.P_angle.I = 4.0F;
  motor.P_angle.D = 0.0F;

  motor.velocity_limit = 300;

  motor.useMonitoring(Serial);

  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 6.19;

  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));

  delay(200);

  target_offset = motor.shaft_angle;
}

ISR(TIMER1_CAPT_vect) {
  static bool risingEdgeDetected = false;
  if (risingEdgeDetected) {
    // Falling edge detected
    fallingEdgeTime = ICR1;                       // Capture time of falling edge
    highTime = fallingEdgeTime - risingEdgeTime;  // Calculate HIGH time
    risingEdgeDetected = false;
    TCCR1B &= ~(1 << ICES1);  // Switch to detect rising edge
  } else {
    // Rising edge detected
    risingEdgeTime = ICR1;                       // Capture time of rising edge
    lowTime = risingEdgeTime - fallingEdgeTime;  // Calculate LOW time
    risingEdgeDetected = true;
    TCCR1B |= (1 << ICES1);          // Switch to detect falling edge
    newMeasurementAvailable = true;  // Indicate new measurement is available
  }
}

void loop() {
  if (newMeasurementAvailable) {
    newMeasurementAvailable = false;

    uint32_t period = highTime + lowTime;

    float frequency = 16000000.0 / (8 * period);

    float dutyCycle = (float)lowTime / period;

    target_angle = constrain((int)round(dutyCycle * 255.0f) - 90, -90, 90);

    Serial.print("Target angle: ");
    Serial.println(target_angle);
  }

  motor.loopFOC();

  motor.move(-target_angle * M_PI / 180 + target_offset);
}
