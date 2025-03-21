#include <SimpleFOC.h>

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

void setup() {
  Serial.begin(115200);

  as5600.init();

  Serial.println("AS5600 ready");
  
  _delay(1000);
}

void loop() {
  Serial.print(as5600.getAngle() * 57.2958);
  Serial.print("\t");
  Serial.println(as5600.getVelocity());
  as5600.update();
}