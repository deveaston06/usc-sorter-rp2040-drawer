#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <Arduino.h>

#define PIN_LED_RED 5
#define PIN_LED_GREEN 6
#define PIN_LED_BLUE 7

#define BLINK_COUNT 3
#define BLINK_ON_MS 150
#define BLINK_OFF_MS 150

// ── Active low helpers ────────────────────────────────────────
#define RED_ON() digitalWrite(PIN_LED_RED, LOW)
#define RED_OFF() digitalWrite(PIN_LED_RED, HIGH)
#define GREEN_ON() digitalWrite(PIN_LED_GREEN, LOW)
#define GREEN_OFF() digitalWrite(PIN_LED_GREEN, HIGH)
#define BLUE_ON() digitalWrite(PIN_LED_BLUE, LOW)
#define BLUE_OFF() digitalWrite(PIN_LED_BLUE, HIGH)

// ── External references ───────────────────────────────────────
void led_init();
void led_setGreen();
void led_setRed();
void led_setBlue();
void led_blinkRedOnAssignment();

#endif // !LED_MANAGER_H
