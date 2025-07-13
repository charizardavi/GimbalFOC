#include "MT6701.hpp"
#include <SimpleFOC.h>

MT6701 encoder;

void setup() {
  Serial.begin(115200);
  encoder.begin();
}

void loop() {
    float angleRadians = encoder.getAngleRadians();
    Serial.print("Angle in Radians: ");
    Serial.println(angleRadians);
}