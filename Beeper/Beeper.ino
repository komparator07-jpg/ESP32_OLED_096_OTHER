#define BUTTON_PIN 4
#define BUZZER_PIN 5

#define BEEPER_FREQ 1500   // Гц
#define BEEPER_DUR  30     // мс

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
}

void loop() {
  static bool lastButtonState = HIGH;
  static bool pressed = false;
  static unsigned long lastDebounce = 0;
  const unsigned long debounceDelay = 50;

  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) lastDebounce = millis();

  if (millis() - lastDebounce > debounceDelay) {
    if (reading == LOW && !pressed) {
      pressed = true;
      tone(BUZZER_PIN, BEEPER_FREQ, BEEPER_DUR);
    }
    if (reading == HIGH) pressed = false;
  }
  lastButtonState = reading;
}
