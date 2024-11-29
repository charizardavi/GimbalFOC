#include <math.h>

volatile uint16_t risingEdgeTime = 0;
volatile uint16_t fallingEdgeTime = 0;
volatile uint16_t highTime = 0;
volatile uint16_t lowTime = 0;
volatile bool newMeasurementAvailable = false;

void setup() {
  delay(500);
  
  pinMode(8, INPUT); // Pin 8 (ICP1) as input
  Serial.begin(9600);

  // Configure Timer1
  TCCR1A = 0; // Normal mode
  TCCR1B = (1 << ICES1) | (1 << CS11); // Input capture on rising edge, prescaler = 8
  TIMSK1 = (1 << ICIE1); // Enable input capture interrupt

  sei(); // Enable global interrupts
}

ISR(TIMER1_CAPT_vect) {
  static bool risingEdgeDetected = false;
  if (risingEdgeDetected) {
    // Falling edge detected
    fallingEdgeTime = ICR1; // Capture time of falling edge
    highTime = fallingEdgeTime - risingEdgeTime; // Calculate HIGH time
    risingEdgeDetected = false;
    TCCR1B &= ~(1 << ICES1); // Switch to detect rising edge
  } else {
    // Rising edge detected
    risingEdgeTime = ICR1; // Capture time of rising edge
    lowTime = risingEdgeTime - fallingEdgeTime; // Calculate LOW time
    risingEdgeDetected = true;
    TCCR1B |= (1 << ICES1); // Switch to detect falling edge
    newMeasurementAvailable = true; // Indicate new measurement is available
  }
}

void loop() {
  if (newMeasurementAvailable) {
    newMeasurementAvailable = false;

    uint32_t period = highTime + lowTime;

    float frequency = 16000000.0 / (8 * period);

    float dutyCycle = (float)lowTime / period;

    Serial.print("Duty Cycle: ");
    Serial.println((int)round(dutyCycle*255.0f));
  }
}

