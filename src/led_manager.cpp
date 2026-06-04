// ─────────────────────────────────────────────────────────────
// led_manager.cpp — RP2040 Drawer
//
// Hardware (all active low):
//   Red   LED → GP10
//   Green LED → GP11
//   Blue  LED → GP12
//
// Behaviour:
//   Red  = default state
//   Green = relay from ESP32 LED command
//   Blue  = address assignment in progress
//   Blink red = address just assigned
// ─────────────────────────────────────────────────────────────

#include <led_manager.h>

// ─────────────────────────────────────────────────────────────
// PUBLIC: INIT
// ─────────────────────────────────────────────────────────────
void led_init() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  RED_ON();
  GREEN_OFF();
  BLUE_OFF();
}

// ─────────────────────────────────────────────────────────────
// PUBLIC: SET STATES
// ─────────────────────────────────────────────────────────────
void led_setRed() {
  GREEN_OFF();
  BLUE_OFF();
  RED_ON();
}

void led_setGreen() {
  RED_OFF();
  BLUE_OFF();
  GREEN_ON();
}

void led_setBlue() {
  RED_OFF();
  GREEN_OFF();
  BLUE_ON();
}

// ─────────────────────────────────────────────────────────────
// PUBLIC: BLINK RED ON ASSIGNMENT
// ─────────────────────────────────────────────────────────────
void led_blinkRedOnAssignment() {
  GREEN_OFF();
  BLUE_OFF();
  for (uint8_t i = 0; i < BLINK_COUNT; i++) {
    RED_ON();
    delay(BLINK_ON_MS);
    RED_OFF();
    delay(BLINK_OFF_MS);
  }
  RED_ON();
}
