/*
   ATmega328P SPI Slave - Receive Float with Synchronization
*/

#include <Arduino.h>

// Buffer to hold received float bytes
volatile byte receivedData[4] = {0};
volatile byte index = 0;
volatile bool receiving = false;
volatile bool dataReady = false;

ISR(SPI_STC_vect) {
  byte receivedByte = SPDR; // Read the received byte from SPI

  if (!receiving && receivedByte == 0xAA) {
    // Start byte received, reset index to start receiving
    index = 0;
    receiving = true;
  } else if (receiving) {
    // Store received data
    receivedData[index] = receivedByte;
    index++;

    // Once 4 bytes are received, process the float
    if (index >= 4) {
      receiving = false; // Stop receiving
      dataReady = true;  // Flag for processing in loop
    }
  }

  // Echo received byte back (optional)
  SPDR = receivedByte;
}

void setup() {
  Serial.begin(115200);

  // Configure SPI as Slave
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  digitalWrite(SS, HIGH); // Prevent entering master mode

  // Enable SPI in Slave Mode + SPI Interrupt
  SPCR |= _BV(SPE);   // Enable SPI
  SPCR |= _BV(SPIE);  // Enable SPI interrupt

  Serial.println("ATmega328P SPI Slave Initialized");
  sei(); // Enable global interrupts
}

void loop() {
  if (dataReady) {
    // Convert received bytes back to float
    union {
      float value;
      byte bytes[4];
    } floatData;

    // Copy received bytes
    for (int i = 0; i < 4; i++) {
      floatData.bytes[i] = receivedData[i];
    }

    // Print the received float
    Serial.print("Received angle: ");
    Serial.println(floatData.value, 2);

    dataReady = false; // Reset flag
  }
}
