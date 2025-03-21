#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);

float target_angle = 0;
float target_offset = 0;

volatile byte receivedData[4] = {0};
volatile byte index = 0;
volatile bool receiving = false;
volatile bool dataReady = false;

ISR(SPI_STC_vect) {
  byte receivedByte = SPDR;

  if (!receiving && receivedByte == 0xAA) {
    index = 0;
    receiving = true;
  } else if (receiving) {
    receivedData[index] = receivedByte;
    index++;

    if (index >= 4) {
      receiving = false;
      dataReady = true;
    }
  }

  SPDR = receivedByte;
}

void setup() {
  Serial.begin(115200);

  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  digitalWrite(SS, HIGH);

  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

  sei();

  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 15;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  motor.voltage_limit = 15;

  motor.LPF_velocity.Tf = 0.01f;
  motor.PID_velocity.P = .25f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;

  motor.P_angle.P = 35.0F;
  motor.P_angle.I = 4.0F;
  motor.P_angle.D = 0.0F;
  
  motor.velocity_limit = 300;

  // motor.sensor_direction=Direction::CCW;
  // motor.zero_electric_angle=6.19;

  motor.init();
  motor.initFOC();

  target_offset = motor.shaft_angle;

  Serial.print("inital angle: ");
  Serial.println(target_offset);
}

void loop() {
  if (dataReady) {
    union {
      float value;
      byte bytes[4];
    } floatData;

    for (int i = 0; i < 4; i++) {
      floatData.bytes[i] = receivedData[i];
    }

    Serial.print("Received angle: ");
    Serial.println(floatData.value, 2);

    target_angle = floatData.value * M_PI / 180.0;

    dataReady = false;
  }

  motor.loopFOC();
  motor.move(target_angle + target_offset);
}
