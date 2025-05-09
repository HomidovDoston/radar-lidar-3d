#define BUZZER_PIN 9

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void playAlert() {
  tone(BUZZER_PIN, 1000); // 1 kHz signal
  delay(500);
  noTone(BUZZER_PIN);
}

void loop() {
  // Shartli ogohlantirish
  playAlert();
  delay(2000);
}
