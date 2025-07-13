#include <Wire.h>
#include <SimpleFOC.h>


MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

BLDCMotor      motor  (11, 5.55f, 100);
BLDCDriver3PWM driver (D1, D2, D3, D0);

void setup() {
  Serial.begin(115200);
  while (!Serial);                               // wait for USB-CDC

  Wire.begin();                                  // default SDA/SCL pins

  // DRIVER ----------------------------------------------------------------
  driver.voltage_power_supply = 15;              // your supply voltage
  driver.pwm_frequency        = 25000;           // quiet PWM
  driver.init();
  motor.linkDriver(&driver);

  // SENSOR ----------------------------------------------------------------
  sensor.init();                                 // MT6701 I²C
  motor.linkSensor(&sensor);

  // SAFE LIMITS (gentle calibration) --------------------------------------
  motor.current_limit = 0.7;                     // ≤0.7 A  → <3 W
  motor.voltage_limit = 4;                       // low voltage
  motor.controller    = MotionControlType::torque;

  motor.init();                                  // *do not* call initFOC yet

  // ── RUN CALIBRATION ────────────────────────────────────────────────────
  Serial.println(F("\nRunning sensor calibration …"));
  motor.initFOC();                               // performs slow half-turn

  // ── PRINT RESULTS ─────────────────────────────────────────────────────
  Serial.println(F("\nCalibration finished\n"));
  Serial.print  (F("zero_electric_offset = "));
  Serial.println(motor.zero_electric_angle, 6);
  Serial.print  (F("sensor_direction     = Direction::"));
  Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");

  Serial.println(F("\nCopy this into your real sketch:\n"));
  Serial.print  (F("  motor.initFOC("));
  Serial.print  (motor.zero_electric_angle, 6);
  Serial.print  (F(", Direction::"));
  Serial.print  (motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial.println(F(");"));

  // ── DISABLE OUTPUTS ───────────────────────────────────────────────────
  motor.disable();                               // driver off, rotor free
  Serial.println(F("\nDriver disabled. Calibration complete.\n"));
}

// ────────── LOOP ────────────────────────────────────────────────
void loop() { /* nothing – calibration is one-shot */ }
