// Keyestudio HC-SR04: VCC->5V, GND->GND, TRIG->D12, ECHO->D13.
// The Arduino connects to the Raspberry Pi over USB; no sensor signal goes to Pi GPIO.
constexpr uint8_t TRIG_PIN = 12;
constexpr uint8_t ECHO_PIN = 13;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  const float cm = duration * 0.0343f / 2.0f;
  if (duration == 0 || cm < 2.0f || cm > 400.0f) {
    Serial.println("NA");
  } else {
    Serial.println(cm, 1);
  }
  delay(100);
}
