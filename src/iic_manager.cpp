#include <iic_manager.h>
#include <led_manager.h>

// Flag set in ISR, actioned in loop
volatile bool blinkPending = false;

// ─────────────────────────────────────────────────────────────
// I2C RECEIVE CALLBACK (runs in interrupt context)
// keep short, no blocking calls here
// ─────────────────────────────────────────────────────────────
void onReceive(int numBytes) {
  while (Wire.available()) {
    uint8_t cmd = Wire.read();
    if (cmd == CMD_BLINK) {
      blinkPending = true;
    }
  }
}

// ─────────────────────────────────────────────────────────────
// I2C REQUEST CALLBACK
// master requested data from this slave
// send back own address as acknowledgement
// ─────────────────────────────────────────────────────────────
void onRequest() { Wire.write(SLAVE_ADDR); }

void setupI2C() {
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
}
