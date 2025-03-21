#include <Arduino.h>

#define INPUT_PIN_1 11  // PB3 on ATMega328P
#define INPUT_PIN_2 12  // PB4 on ATMega328P
#define INPUT_PIN_3 13  // PB5 on ATMega328P

void setup() {
    pinMode(INPUT_PIN_1, INPUT);
    pinMode(INPUT_PIN_2, INPUT);
    pinMode(INPUT_PIN_3, INPUT);
    Serial.begin(115200);
}

void loop() {
    int state1 = digitalRead(INPUT_PIN_1);
    int state2 = digitalRead(INPUT_PIN_2);
    int state3 = digitalRead(INPUT_PIN_3);

    Serial.print("PB3: "); Serial.print(state1);
    Serial.print(" | PB4: "); Serial.print(state2);
    Serial.print(" | PB5: "); Serial.println(state3);

    delay(10);
}
