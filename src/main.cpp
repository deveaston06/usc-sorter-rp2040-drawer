// ─────────────────────────────────────────────────────────────
// main.cpp — RP2040 Drawer (Intermediary)
// PlatformIO + arduino-pico (Earle Philhower)
//
// platformio.ini:
//   [env:rp2040]
//   platform  = https://github.com/maxgerhardt/platform-raspberrypi.git
//   board     = rpipico
//   framework = arduino
//   board_build.core = earlephilhower
//
// Board manager URL for Arduino IDE:
//   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
//
// First-time setup:
//   Call iic_writeUDID(uniqueSerial) ONCE per device before normal use.
//   Uncomment the block in setup(), flash with unique value, recomment.
//
// Bus wiring:
//   Wire  (Bus 0)
//   Wire1 (Bus 1)
//   ALERT up (open-drain out)
//   ALERT down (input with external pull-up)
// ─────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <iic_manager.h>
#include <led_manager.h>

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  led_init();

  iic_init();
  // iic_init() asserts ALERT up to ESP32 if this RP2040 has
  // no stored address, triggering automatic enumeration by ESP32.
  // iic_init() also sets up Wire1 master and ALERT down interrupt
  // for ATtiny85 hot-plug detection.

  // ── FIRST TIME ONLY: write unique UDID ────────────────────
  // iic_writeUDID(0x00000010); // drawer 1
  // iic_writeUDID(0x00000011); // drawer 2
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  iic_update();
  // iic_update() handles:
  //   - upstream commands from ESP32 (via rxPending flag)
  //   - downstream ALERT from ATtiny85 (via alertDownPending flag)
  //   - periodic ATtiny85 scanner every 5 seconds
}
