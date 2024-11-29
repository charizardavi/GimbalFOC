const int RX_PIN = 8;
const int TX_PIN = 9;

void setup() {
  Serial.begin(9600);
  pinMode(RX_PIN, OUTPUT);
  pinMode(TX_PIN, OUTPUT);
}

void loop() {
  digitalWrite(RX_PIN, HIGH);
  digitalWrite(TX_PIN, HIGH);
  Serial.println("HIGH");
  delay(1000);
  digitalWrite(RX_PIN, LOW);
  digitalWrite(TX_PIN, LOW);
  Serial.println("LOW");
  delay(1000);
}
