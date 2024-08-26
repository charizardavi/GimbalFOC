#include <SoftwareSerial.h>

// RX, TX
SoftwareSerial mySerial(10, 11); // Define SoftwareSerial pins: RX = 10, TX = 11

void setup() {
  // Start the hardware serial to communicate with the PC
  Serial.begin(9600);
  // Start the software serial
  mySerial.begin(9600);
}

void loop() {
  if (mySerial.available()) {
    char c = mySerial.read();
    Serial.print(c);
  }
}
