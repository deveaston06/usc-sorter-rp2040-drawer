#ifndef LED_MANAGER_H
#define LED_MANAGER_H

#include <Arduino.h>
#define LED_PIN 29

#define BLINK_COUNT 3
#define BLINK_ON_MS 150
#define BLINK_OFF_MS 150

void setupLED();
void blinkLED();

#endif // !LED_MANAGER_H
