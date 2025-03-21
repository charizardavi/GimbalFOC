#include <Arduino.h>

ISR(SPI_STC_vect) {
  byte receivedByte = SPDR; // Read the received byte from SPI
  Serial.print("Slave received: ");
  Serial.println(receivedByte, HEX);

  SPDR = 0xAB; // Reply with 0xAB
}

void setup() {
  Serial.begin(115200);
  Serial.println("ATmega328P SPI Slave Initialized");

  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  digitalWrite(SS, HIGH);  // Keep SS high initially

  SPCR |= _BV(SPE);  // Enable SPI
  SPCR |= _BV(SPIE); // Enable SPI interrupt

  sei(); // Enable global interrupts
}

void loop() {
  // Nothing needed in loop, SPI handled by ISR
}
