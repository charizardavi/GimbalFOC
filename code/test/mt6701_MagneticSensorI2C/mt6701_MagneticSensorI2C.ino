#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(MT6701_I2C);

void setup() {
  Serial.begin(115200);

  sensor.init();

  Serial.println("MT6701 ready");
  _delay(1000);
}

void loop() {
  sensor.update();
  Serial.print(sensor.getAngle(), 5);
  Serial.print("\t");
  Serial.println(sensor.getVelocity(), 5);
}