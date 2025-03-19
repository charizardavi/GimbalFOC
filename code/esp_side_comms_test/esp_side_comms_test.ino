/*
   ESP32-S3 SPI Master - Send Angle (Float) with Synchronization
*/

#include <SPI.h>

// Define SPI pins
static const int PIN_MOSI = 9;
static const int PIN_MISO = 8;
static const int PIN_SCK  = 7;
static const int PIN_SS   = 10; // Slave Select (if needed)

// Define a constant angle to send
const float angleToSend = 45.67;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 SPI Master - Send Float with Synchronization");

  // Initialize SPI
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // Configure SPI settings (1MHz, MSB first, Mode 0)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  pinMode(PIN_SS, OUTPUT);
  digitalWrite(PIN_SS, HIGH); // Set SS high initially
}

void loop() {
  // Convert float to 4 bytes
  union {
    float value;
    byte bytes[4];
  } floatData;
  floatData.value = angleToSend;

  Serial.print("Sending angle: ");
  Serial.println(floatData.value, 2);

  // Enable slave select (if used)
  digitalWrite(PIN_SS, LOW);

  // Send start byte (sync byte)
  SPI.transfer(0xAA); // 0xAA = Start Transmission Signal

  // Send the 4 bytes of the float
  for (int i = 0; i < 4; i++) {
    SPI.transfer(floatData.bytes[i]);
  }

  // Disable slave select (if used)
  digitalWrite(PIN_SS, HIGH);

  delay(10); // Small delay to allow proper sync before next transmission
}
