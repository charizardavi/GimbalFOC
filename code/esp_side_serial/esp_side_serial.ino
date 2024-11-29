#define PWM_PIN 1          // GPIO 1
#define PWM_FREQUENCY 1000 // 1 kHz
#define PWM_RESOLUTION 8   // 8-bit resolution

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Attach GPIO 1 to PWM
  ledcAttach(PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

  // Configure PWM with 1 kHz frequency and 8-bit resolution
  // Set initial duty cycle to 0%
  ledcWrite(PWM_PIN, 0); // 0% duty cycle
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming value as a string
    String input = Serial.readStringUntil('\n');
    
    // Convert the input string to an integer
    int dutyCycle = input.toInt();

    // Ensure the duty cycle is within valid range (0 to 255 for 8-bit resolution)
    if (dutyCycle >= 0 && dutyCycle <= 255) {
      // Write the duty cycle to the PWM pin
      ledcWrite(PWM_PIN, dutyCycle);

      // Print confirmation message
      Serial.print("Duty cycle set to: ");
      Serial.println(dutyCycle);
    } else {
      // Print error message if input is invalid
      Serial.println("Invalid input. Please enter a value between 0 and 255.");
    }
  }
}
