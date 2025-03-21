#include <Arduino.h>

#define TEST_PIN_1 2
#define TEST_PIN_2 3
#define TEST_PIN_3 4

void setup() {
    pinMode(TEST_PIN_1, OUTPUT);
    pinMode(TEST_PIN_2, OUTPUT);
    pinMode(TEST_PIN_3, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    Serial.println("Toggling HIGH...");
    digitalWrite(TEST_PIN_1, HIGH);
    digitalWrite(TEST_PIN_2, HIGH);
    digitalWrite(TEST_PIN_3, HIGH);
    delay(1000);

    Serial.println("Toggling LOW...");
    digitalWrite(TEST_PIN_1, LOW);
    digitalWrite(TEST_PIN_2, LOW);
    digitalWrite(TEST_PIN_3, LOW);
    delay(1000);
}
