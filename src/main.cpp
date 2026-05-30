#include <Arduino.h>
#include <iic_manager.h>
#include <led_manager.h>
/*
 * RP2040 I2C Slave — Blink on Command
 *
 * Hardware:
 *   I2C Bus
 *     SDA → GP4 (Wire default on arduino-pico)
 *     SCL → GP5 (Wire default on arduino-pico)
 *
 *   LED
 *     Built-in LED on GP25 (standard RP2040)
 *     OR external LED on GP15 via 330Ω to GND
 *
 * I2C Slave Address:
 *   Change SLAVE_ADDR below to a unique address per module
 *   Valid range: 0x08 to 0x77
 *   Example: module 1 = 0x10, module 2 = 0x11, module 3 = 0x12
 *
 * Commands received from master:
 *   0x01 → blink LED 3 times
 *
 * Note:
 *   In the final framework this address will be assigned
 *   dynamically by the enumeration protocol. For this demo
 *   it is set manually per device.
 */

void setup() {
  Serial.begin(115200);

  setupI2C();
  setupLED();

  // startup blink confirms address and boot
  Serial.print("RP2040 slave ready at 0x");
  Serial.println(SLAVE_ADDR, HEX);
  blinkLED();
}

void loop() {
  if (blinkPending) {
    blinkPending = false;
    blinkLED();
  }
}
