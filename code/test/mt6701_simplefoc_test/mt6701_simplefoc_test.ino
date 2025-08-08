#include <Wire.h>
#include <MT6701.h>
#include <SimpleFOC.h>

MT6701 encoder;

float read_mt6701_radians() {
  return encoder.angleRead() * PI / 180.0f;
}

void init_mt6701() {
  Wire.setClock(400000);
  Wire.begin();
  encoder.initializeI2C();
}

GenericSensor sensor = GenericSensor(read_mt6701_radians, init_mt6701);


void setup() {
  Serial.begin(115200);
  sensor.init();
  Serial.println(F("MT6701 + SimpleFOC ready"));
}

void loop() {
  sensor.update();
  Serial.println(sensor.getAngle(), 6);
}