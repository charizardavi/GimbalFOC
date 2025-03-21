#include <SPI.h>

// Define SPI pins
static const int PIN_MOSI_ROLL = 9;
static const int PIN_MISO_ROLL = 8;
static const int PIN_SCK_ROLL = 7;

static const int PIN_MOSI_PITCH = 2;
static const int PIN_MISO_PITCH = 3;
static const int PIN_SCK_PITCH = 4;
// Define a constant angle to send
const float angleToSend = 45.67;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 SPI Master - Send Float with Synchronization");

  // Initialize SPI
  SPI.begin(PIN_MOSI_PITCH, PIN_MISO_PITCH, PIN_SCK_PITCH);

  // Configure SPI settings (1MHz, MSB first, Mode 0)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  byte recieved = SPI.transfer(0x12);  // Start Transmission Signal
  Serial.println(recieved, HEX);
}
