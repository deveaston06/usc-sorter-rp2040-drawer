#include <led_manager.h>

void setupLED() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void blinkLED() {
  for (uint8_t i = 0; i < BLINK_COUNT; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_ON_MS);
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_OFF_MS);
  }
}
