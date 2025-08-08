#include <Wire.h>
#include <MT6701.h>
MT6701 encoder;

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  encoder.initializeI2C();
}

void loop() {
  Serial.println(encoder.angleRead() * PI / 180.0f, 10);
}