#define PIN_D0 1
#define PIN_D1 2

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);

  // Configure pins D0 and D1 as input
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);

  // Print a message to indicate the setup is complete
  Serial.println("Setup complete. Reading digital signals from D0 and D1...");
}

void loop() {
  // Read the digital state of pins D0 and D1
  int stateD0 = digitalRead(PIN_D0);
  int stateD1 = digitalRead(PIN_D1);

  // Print the states to the Serial Monitor
  Serial.print("D0 State: ");
  Serial.print(stateD0);
  Serial.print(" | D1 State: ");
  Serial.println(stateD1);
}
