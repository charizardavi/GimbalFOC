#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);    // Initialize hardware serial to communicate with the PC
  mySerial.begin(9600);  // Initialize software serial to communicate with the second Arduino
  Serial.println("Enter a float value:");
}

void loop() {
  if (Serial.available()) {
    // Check if valid float is available
    if (Serial.peek() >= '0' && Serial.peek() <= '9') {
      // Read the float from the serial monitor
      float value = Serial.parseFloat();
      
      // Clear any remaining characters in the buffer
      while (Serial.available()) {
        Serial.read();
      }

      // Send the float value to the second Arduino
      mySerial.write((byte*)&value, sizeof(value));

      // Print the sent value for confirmation
      Serial.print("Sent: ");
      Serial.println(value);
    }
  }
}
