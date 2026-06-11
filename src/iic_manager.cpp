// ─────────────────────────────────────────────────────────────
// iic_manager.cpp — RP2040 Drawer
// arduino-pico Wire library (PlatformIO)
//
// Two I2C buses:
//   Wire (Bus 0) — master to ATtiny85
//   Wire1 (Bus 1) — slave to ESP32
//
// Responsibilities:
//   Upstream (Wire, slave side):
//     - Respond to GET_UDID with own UDID (arbitration)
//     - Accept ASSIGN_ADDR from ESP32
//     - Accept PREPARE_ARP, reset AV flag
//     - Accept SCAN_MODULES, reply with ATtiny85 address list
//     - Accept SET_PSA, SYNC_PSA, store in EEPROM
//     - Accept LED commands, relay downstream to ATtiny85
//
//   Downstream (Wire1, master side):
//     - Run GET_UDID cycles on ATtiny85 slaves
//     - PSA lookup before assigning new address
//     - Send PREPARE_ARP to all ATtiny85 on new ALERT
//     - Periodic scanner to detect removed modules
//     - ALERT line monitoring for new ATtiny85 arrivals
// ─────────────────────────────────────────────────────────────

#include <iic_manager.h>

// ── ATtiny85 module list ──────────────────────────────────────
static uint8_t tinyAddrs[MAX_TINY_MODULES];
static uint8_t tinyUdids[MAX_TINY_MODULES][UDID_SIZE];
static uint8_t tinyCount = 0;

// ── Own state ─────────────────────────────────────────────────
static uint8_t udid[UDID_SIZE];
static bool addressResolved = false;
static uint8_t assignedAddr = ADDR_ARP_DEFAULT;

// ── Receive buffer (upstream, ISR safe) ───────────────────────
static volatile uint8_t rxBuf[RX_BUF_SIZE];
static volatile uint8_t rxLen = 0;
static volatile bool rxPending = false;

// ── Reply buffer for CMD_SCAN_MODULES ─────────────────────────────
static uint8_t replyBuf[1 + MAX_TINY_MODULES * (1 + UDID_SIZE)];
static uint8_t replyLen = 0;
static bool replyReady = false;

// ── Timing ────────────────────────────────────────────────────
static uint32_t lastScanMs = 0;

// ─────────────────────────────────────────────────────────────
// ALERT LINE HELPERS
// ─────────────────────────────────────────────────────────────
static void alertUp_assert() {
  pinMode(PIN_ALERT_UP, OUTPUT);
  digitalWrite(PIN_ALERT_UP, LOW);
}

static void alertUp_release() { pinMode(PIN_ALERT_UP, INPUT); }

// ─────────────────────────────────────────────────────────────
// UDID HELPERS
// ─────────────────────────────────────────────────────────────
static void udid_load() {
  for (uint8_t i = 0; i < UDID_SIZE; i++) {
    udid[i] = EEPROM.read(EE_UDID_START + i);
  }
}

void iic_writeUDID(uint32_t serialNumber) {
  uint16_t pv = PROTOCOL_VERSION;
  uint16_t dt = DEVICE_TYPE_RP2040;
  EEPROM.write(EE_UDID_START + 0, (uint8_t)(pv >> 8));
  EEPROM.write(EE_UDID_START + 1, (uint8_t)(pv));
  EEPROM.write(EE_UDID_START + 2, (uint8_t)(dt >> 8));
  EEPROM.write(EE_UDID_START + 3, (uint8_t)(dt));
  EEPROM.write(EE_UDID_START + 4, (uint8_t)(serialNumber >> 24));
  EEPROM.write(EE_UDID_START + 5, (uint8_t)(serialNumber >> 16));
  EEPROM.write(EE_UDID_START + 6, (uint8_t)(serialNumber >> 8));
  EEPROM.write(EE_UDID_START + 7, (uint8_t)(serialNumber));
  EEPROM.write(EE_UDID_START + 8, CAPABILITIES);
  EEPROM.commit();
}

static bool udid_matches(const volatile uint8_t *buf) {
  for (uint8_t i = 0; i < UDID_SIZE; i++) {
    if (buf[i] != udid[i])
      return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────
// PSA TABLE
// Entry: [udid 9][psa 1][persistent 1]
// persistent=1 means ESP32 has authorised this address to be reused
// persistent=0 means this entry is a working record only
// ─────────────────────────────────────────────────────────────
static uint8_t psa_lookup(const uint8_t *targetUdid) {
  for (uint8_t i = 0; i < PSA_MAX_ENTRIES; i++) {
    uint16_t base = EE_PSA_TABLE_START + (i * PSA_ENTRY_SIZE);
    bool match = true;
    for (uint8_t j = 0; j < UDID_SIZE; j++) {
      if (EEPROM.read(base + j) != targetUdid[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      // Return address regardless of persistent flag
      // (persistent flag only controls whether the address is RESERVED)
      return EEPROM.read(base + UDID_SIZE);
    }
  }
  return 0xFF;
}

static void psa_store(const uint8_t *targetUdid, uint8_t psa,
                      uint8_t persistent) {
  // Check if entry already exists — preserve persistent flag
  for (uint8_t i = 0; i < PSA_MAX_ENTRIES; i++) {
    uint16_t base = EE_PSA_TABLE_START + (i * PSA_ENTRY_SIZE);
    bool match = true;
    for (uint8_t j = 0; j < UDID_SIZE; j++) {
      if (EEPROM.read(base + j) != targetUdid[j]) {
        match = false;
        break;
      }
    }
    if (match) {
      uint8_t existingPersistent = EEPROM.read(base + UDID_SIZE + 1);
      if (existingPersistent == 0x01 && persistent == 0x00) {
        // Don't downgrade a persistent entry to non-persistent
        // But DO update the address if it changed
        EEPROM.write(base + UDID_SIZE, psa);
        EEPROM.commit();
        return;
      }
      EEPROM.write(base + UDID_SIZE, psa);
      EEPROM.write(base + UDID_SIZE + 1, persistent);
      EEPROM.commit();
      return;
    }
  }
  // find empty slot (all 0xFF) or use slot 0
  for (uint8_t i = 0; i < PSA_MAX_ENTRIES; i++) {
    uint16_t base = EE_PSA_TABLE_START + (i * PSA_ENTRY_SIZE);
    if (EEPROM.read(base) == 0xFF) {
      for (uint8_t j = 0; j < UDID_SIZE; j++) {
        EEPROM.write(base + j, targetUdid[j]);
      }
      EEPROM.write(base + UDID_SIZE, psa);
      EEPROM.write(base + UDID_SIZE + 1, persistent);
      EEPROM.commit();
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────────
// CMD_SYNC_PSA HANDLER
// Receives list of addresses ESP32 considers persistent.
// Updates persistent flag on all PSA table entries accordingly.
// Entries whose address is not in the received list are cleared to 0.
// ─────────────────────────────────────────────────────────────
static void handleSyncPSA(const volatile uint8_t *buf, uint8_t len) {
  if (len < 2)
    return;
  uint8_t count = buf[1];

  // Clear all persistent flags first
  for (uint8_t i = 0; i < PSA_MAX_ENTRIES; i++) {
    uint16_t base = EE_PSA_TABLE_START + (i * PSA_ENTRY_SIZE);
    if (EEPROM.read(base) != 0xFF) {
      EEPROM.write(base + UDID_SIZE + 1, 0x00);
    }
  }

  // Write received entries
  uint8_t offset = 2;
  for (uint8_t e = 0; e < count; e++) {
    if (offset + UDID_SIZE >= len)
      break;
    uint8_t entryUdid[UDID_SIZE];
    for (uint8_t j = 0; j < UDID_SIZE; j++) {
      entryUdid[j] = buf[offset++];
    }
    uint8_t psa = buf[offset++];
    psa_store(entryUdid, psa, 0x01);

    // ── NEW: Check for conflicts with currently assigned ATtiny85s ──
    for (uint8_t i = 0; i < tinyCount; i++) {
      if (tinyAddrs[i] == psa) {
        // A device is at this PSA address
        // Is it the PSA owner?
        if (memcmp(tinyUdids[i], entryUdid, UDID_SIZE) != 0) {
          // CONFLICT: different device at PSA address — evict it
          Wire.beginTransmission(psa);
          Wire.write(CMD_PREPARE_ARP);
          Wire.endTransmission();
          delayMicroseconds(200);
          tiny_remove(psa);
        }
      }
    }
  }
  EEPROM.commit();

  // Re-enumerate any evicted devices
  if (tinyCount < MAX_TINY_MODULES) {
    downstream_enumerate();
  }
}

// ─────────────────────────────────────────────────────────────
// ALERT DOWN ISR
// ─────────────────────────────────────────────────────────────
static volatile bool alertDownPending = false;
static volatile uint32_t alertDownDebounceMs = 0;
static volatile bool downstreamBusBusy = false;
static bool scanPending = false;

static void onAlertDown() {
  alertDownPending = true;
  alertDownDebounceMs = millis();
  downstreamBusBusy = true;
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: ATTINY85 MODULE LIST HELPERS
// ─────────────────────────────────────────────────────────────
static bool tiny_isKnown(uint8_t addr) {
  for (uint8_t i = 0; i < tinyCount; i++) {
    if (tinyAddrs[i] == addr)
      return true;
  }
  return false;
}

static void tiny_add(uint8_t addr) {
  if (tinyCount < (uint8_t)MAX_TINY_MODULES && !tiny_isKnown(addr)) {
    tinyAddrs[tinyCount++] = addr;
  }
}

static void tiny_remove(uint8_t addr) {
  for (uint8_t i = 0; i < tinyCount; i++) {
    if (tinyAddrs[i] == addr) {
      // shift left
      for (uint8_t j = i; j < tinyCount - 1; j++) {
        tinyAddrs[j] = tinyAddrs[j + 1];
      }
      tinyCount--;
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: NEXT AVAILABLE ADDRESS
// simple sequential allocator starting at 0x20
// ─────────────────────────────────────────────────────────────
static uint8_t nextAvailableAddr = TINY_START_ADDRESS;

static bool psa_isReserved(uint8_t addr) {
  for (uint8_t i = 0; i < PSA_MAX_ENTRIES; i++) {
    uint16_t base = EE_PSA_TABLE_START + (i * PSA_ENTRY_SIZE);
    if (EEPROM.read(base) == 0xFF)
      continue;
    if (EEPROM.read(base + UDID_SIZE + 1) == 0x01 &&
        EEPROM.read(base + UDID_SIZE) == addr) {
      return true;
    }
  }
  return false;
}

static uint8_t getNextAddr() {
  while (nextAvailableAddr <= 0x77) {
    Serial.print("getNextAddr checking 0x");
    Serial.print(nextAvailableAddr, HEX);
    Serial.print(" known=");
    Serial.print(tiny_isKnown(nextAvailableAddr));
    Serial.print(" reserved=");
    Serial.println(psa_isReserved(nextAvailableAddr));

    if (!tiny_isKnown(nextAvailableAddr) &&
        !psa_isReserved(nextAvailableAddr)) {
      uint8_t ret = nextAvailableAddr++;
      Serial.print("getNextAddr returning 0x");
      Serial.println(ret, HEX);
      return ret;
    }
    nextAvailableAddr++;
  }
  return 0xFF;
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM WIRE RETRY WRAPPERS
// Retries up to WIRE_RETRY_COUNT times if ALERT bouncing during transmission
// ─────────────────────────────────────────────────────────────

static uint8_t wire0_write(uint8_t addr, uint8_t cmd) {
  for (uint8_t i = 0; i < WIRE_RETRY_COUNT; i++) {
    if (alertDownPending &&
        (millis() - alertDownDebounceMs) < ALERT_DEBOUNCE_MS) {
      delay(ALERT_DEBOUNCE_MS);
      continue;
    }
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    if (Wire.endTransmission() == 0)
      return 0;
    delayMicroseconds(500);
  }
  return 1;
}

static uint8_t wire0_write_buf(uint8_t addr, const uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < WIRE_RETRY_COUNT; i++) {
    if (alertDownPending &&
        (millis() - alertDownDebounceMs) < ALERT_DEBOUNCE_MS) {
      delay(ALERT_DEBOUNCE_MS);
      continue;
    }
    Wire.beginTransmission(addr);
    Wire.write(data, len);
    if (Wire.endTransmission() == 0)
      return 0;
    delayMicroseconds(500);
  }
  return 1;
}

static uint8_t wire0_request(uint8_t addr, uint8_t len, uint8_t *buf) {
  for (uint8_t i = 0; i < WIRE_RETRY_COUNT; i++) {
    if (alertDownPending &&
        (millis() - alertDownDebounceMs) < ALERT_DEBOUNCE_MS) {
      delay(ALERT_DEBOUNCE_MS);
      continue;
    }
    uint8_t received = Wire.requestFrom(addr, len);
    if (received >= len) {
      for (uint8_t j = 0; j < len; j++)
        buf[j] = Wire.read();
      return received;
    }
    while (Wire.available())
      Wire.read(); // drain partial
    delayMicroseconds(500);
  }
  return 0;
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: GET_UDID CYCLE FOR ATTINY85
// PSA conflict resolution:
//   1. Read winner UDID
//   2. PSA lookup — if persistent, that address is RESERVED
//   3. If another ATtiny85 currently uses that reserved address,
//      reassign it FIRST (send PREPARE_ARP, re-enumerate it)
//   4. Then assign the PSA address to the winning device
// ─────────────────────────────────────────────────────────────
static bool downstream_getUdidCycle() {
  if (downstreamBusBusy) {
    return false;
  }

  // Step 1: send GET_UDID command
  if (wire0_write(ADDR_ARP_DEFAULT, CMD_GET_UDID) != 0)
    return false;

  delayMicroseconds(200);

  // Step 2: read winner UDID
  uint8_t winnerUdid[UDID_SIZE];
  if (wire0_request(ADDR_ARP_DEFAULT, UDID_SIZE, winnerUdid) < UDID_SIZE)
    return false;

  // Validate UDID - reject all 0xFF or all 0x00 (noise)
  bool validUdid = false;
  for (uint8_t i = 0; i < UDID_SIZE; i++) {
    if (winnerUdid[i] != 0xFF && winnerUdid[i] != 0x00) {
      validUdid = true;
      break;
    }
  }
  if (!validUdid)
    return false;

  // Step 3: PSA lookup
  uint8_t psaAddr = psa_lookup(winnerUdid);
  uint8_t newAddr;

  if (psaAddr != 0xFF) {
    // ── PSA exists for this UDID ──────────────────────────
    // Check if another ATtiny85 currently occupies this address
    int8_t conflictIdx = -1;
    for (uint8_t i = 0; i < tinyCount; i++) {
      if (tinyAddrs[i] == psaAddr) {
        // Is this the SAME device (same UDID)?
        if (memcmp(tinyUdids[i], winnerUdid, UDID_SIZE) != 0) {
          conflictIdx = i; // Different device at PSA address!
        }
        break;
      }
    }

    if (conflictIdx >= 0) {
      // ── Conflict: evict existing device ────────────────
      uint8_t conflictAddr = tinyAddrs[conflictIdx];

      // Send PREPARE_ARP to the conflicting device
      Wire.beginTransmission(conflictAddr);
      Wire.write(CMD_PREPARE_ARP);
      Wire.endTransmission();
      delayMicroseconds(200);

      // Remove from our list
      tiny_remove(conflictAddr);

      // Give it a moment to reset to ARP address
      delay(5);
    }

    newAddr = psaAddr;
  } else {
    // ── No PSA — get next available ──────────────────────
    newAddr = getNextAddr();
  }
  if (newAddr == 0xFF) {
    // if still exhausted
    return false;
  }

  // Step 4: send ASSIGN_ADDR
  uint8_t payload[UDID_SIZE + 2];
  payload[0] = CMD_ASSIGN_ADDR;
  memcpy(&payload[1], winnerUdid, UDID_SIZE);
  payload[UDID_SIZE + 1] = newAddr;

  if (wire0_write_buf(ADDR_ARP_DEFAULT, payload, sizeof(payload)) != 0) {
    return false;
  }

  delayMicroseconds(200);

  // Step 5: Verify by probing new address
  Wire.beginTransmission(newAddr);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    memcpy(tinyUdids[tinyCount], winnerUdid, UDID_SIZE);
    tiny_add(newAddr);
    psa_store(winnerUdid, newAddr, 0x00);

    // If we evicted a device, run one more cycle to reassign it
    // The evicted device is now at ARP address waiting

    return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: FULL ENUMERATION SEQUENCE
// Repeats GET_UDID cycles until no more unresolved devices
// After main loop, handle any evicted devices
// ─────────────────────────────────────────────────────────────
static void downstream_enumerate() {
  detachInterrupt(digitalPinToInterrupt(PIN_ALERT_DOWN));

  delay(ALERT_DEBOUNCE_MS);
  downstreamBusBusy = false;

  // First pass: discover initial devices
  Wire.beginTransmission(ADDR_ARP_DEFAULT);
  Wire.write(CMD_PREPARE_ARP);
  Wire.endTransmission();
  delayMicroseconds(200);

  uint8_t maxCycles = (uint8_t)MAX_TINY_MODULES;
  while (maxCycles-- > 0) {
    if (!downstream_getUdidCycle())
      break;
  }

  delay(5);

  if (digitalRead(PIN_ALERT_DOWN) == LOW) {
    // Evicted devices are waiting — run second pass
    Wire.beginTransmission(ADDR_ARP_DEFAULT);
    Wire.write(CMD_PREPARE_ARP);
    Wire.endTransmission();
    delayMicroseconds(200);

    maxCycles = (uint8_t)MAX_TINY_MODULES;
    while (maxCycles-- > 0) {
      if (!downstream_getUdidCycle())
        break;
    }
  }

  attachInterrupt(digitalPinToInterrupt(PIN_ALERT_DOWN), onAlertDown, FALLING);
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: PERIODIC MODULE SCANNER
// Probes all known addresses, removes those that no longer ACK
// ─────────────────────────────────────────────────────────────
static void downstream_scan() {
  for (uint8_t i = tinyCount; i-- > 0;) {
    Wire.beginTransmission(tinyAddrs[i]);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
      tiny_remove(tinyAddrs[i]);
    }
  }
}

// ─────────────────────────────────────────────────────────────
// UPSTREAM: WIRE CALLBACKS (ISR context)
// ─────────────────────────────────────────────────────────────
static void onReceive(int numBytes) {
  rxLen = 0;
  while (Wire1.available() && rxLen < RX_BUF_SIZE) {
    rxBuf[rxLen++] = Wire1.read();
  }

  // CMD_SCAN_MODULES: prepare reply immediately in ISR from cached arrays
  // so replyBuf is ready before onRequest fires — eliminates repeat-address bug
  // downstream_scan() runs asynchronously via scanPending flag in main loop
  if (rxLen > 0 && rxBuf[0] == CMD_SCAN_MODULES) {
    replyBuf[0] = tinyCount;
    uint8_t offset = 1;
    for (uint8_t i = 0; i < tinyCount; i++) {
      replyBuf[offset++] = tinyAddrs[i];
      for (uint8_t j = 0; j < UDID_SIZE; j++) {
        replyBuf[offset++] = tinyUdids[i][j];
      }
    }
    replyLen = offset;
    replyReady = true;
    scanPending = true;
    return; // do not set rxPending — handled entirely here
  }

  rxPending = true;
}

static void onRequest() {
  if (replyReady) {
    Wire1.write(replyBuf, replyLen);
    replyReady = false;
  } else if (!addressResolved) {
    Wire1.write(udid, UDID_SIZE);
  } else {
    Wire1.write(assignedAddr);
  }
}

// ─────────────────────────────────────────────────────────────
// UPSTREAM: PROCESS COMMAND FROM ESP32
// ─────────────────────────────────────────────────────────────
static void processUpstreamCommand() {
  if (rxLen == 0)
    return;
  uint8_t cmd = rxBuf[0];

  switch (cmd) {

  case CMD_GET_UDID:
    // response via onRequest()
    break;

  case CMD_ASSIGN_ADDR:
    if (rxLen >= UDID_SIZE + 2 && !addressResolved) {
      if (udid_matches(&rxBuf[1])) {
        assignedAddr = rxBuf[UDID_SIZE + 1];
        addressResolved = true;
        EEPROM.write(EE_ASSIGNED_ADDR, assignedAddr);
        EEPROM.commit();

        Wire1.end();

        Wire1.setSDA(BUS1_SDA);
        Wire1.setSCL(BUS1_SCL);
        Wire1.begin(assignedAddr);
        Wire1.onReceive(onReceive);
        Wire1.onRequest(onRequest);

        alertUp_release();
        led_blinkRedOnAssignment();
      } else {
        delay(5);
        alertUp_assert();
      }
    }
    break;

  case CMD_PREPARE_ARP:
    addressResolved = false;

    Wire1.end();

    Wire1.setSDA(BUS1_SDA);
    Wire1.setSCL(BUS1_SCL);
    Wire1.begin(ADDR_ARP_DEFAULT);
    Wire1.onReceive(onReceive);
    Wire1.onRequest(onRequest);
    break;

  case CMD_SYNC_PSA:
    // ESP32 is pushing its authoritative PSA list
    // update persistent flags in our PSA table
    handleSyncPSA(rxBuf, rxLen);
    break;

  case CMD_LED_GREEN:
    // rxBuf[1] = target ATtiny85 address
    if (rxLen >= 2) {
      Wire.beginTransmission(rxBuf[1]);
      Wire.write(CMD_LED_GREEN);
      Wire.endTransmission();
    }
    led_setGreen();
    break;

  case CMD_LED_RED:
    if (rxLen >= 2) {
      Wire.beginTransmission(rxBuf[1]);
      Wire.write(CMD_LED_RED);
      Wire.endTransmission();
    }
    led_setRed();
    break;

  case CMD_LED_BLUE:
    led_setBlue();
    break;

  default:
    break;
  }
}

// ─────────────────────────────────────────────────────────────
// PUBLIC: INIT
// ─────────────────────────────────────────────────────────────
void iic_init() {
  alertUp_release();

  EEPROM.begin(EEPROM_SIZE);

  addressResolved = false;

  udid_load();

  // Bus 1 — slave to ESP32, always start at ARP default
  Wire1.setSDA(BUS1_SDA);
  Wire1.setSCL(BUS1_SCL);
  Wire1.begin(ADDR_ARP_DEFAULT);
  Wire1.onReceive(onReceive);
  Wire1.onRequest(onRequest);

  while (true) {
    // synchronization, to synzhronize with other rp2040s
    if (millis() > WAIT_FOR_ESP32_MS || digitalRead(PIN_ALERT_UP) == LOW) {
      alertUp_assert();
      break;
    }
  }

  // Bus 0 — master to ATtiny85
  Wire.setSDA(BUS0_SDA);
  Wire.setSCL(BUS0_SCL);
  Wire.begin();
  Wire.setClock(100000);

  pinMode(PIN_ALERT_DOWN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ALERT_DOWN), onAlertDown, FALLING);
}

// ─────────────────────────────────────────────────────────────
// PUBLIC: UPDATE (call every loop iteration)
// ─────────────────────────────────────────────────────────────
void iic_update() {
  // process upstream command
  if (rxPending) {
    rxPending = false;
    processUpstreamCommand();
  }

  // handle downstream ALERT — new ATtiny85 plugged in
  if (alertDownPending) {
    if (millis() - alertDownDebounceMs >= ALERT_DEBOUNCE_MS) {
      alertDownPending = false;
      if (digitalRead(PIN_ALERT_DOWN) == LOW) {
        // confirmed stable LOW — real hot-plug event
        downstream_enumerate();
      }
      // else: bounce, discard
    }
  }

  if (scanPending) {
    scanPending = false;
    downstream_scan();
  }

  // periodic scanner
  uint32_t now = millis();
  if (now - lastScanMs >= SCAN_INTERVAL_MS) {
    lastScanMs = now;
    downstream_scan();
  }
}

// ─────────────────────────────────────────────────────────────
// PUBLIC: GETTERS
// ─────────────────────────────────────────────────────────────
bool iic_isResolved() { return addressResolved; }
uint8_t iic_getAddr() { return assignedAddr; }
uint8_t iic_getTinyCount() { return tinyCount; }
uint8_t iic_getTinyAddr(uint8_t idx) {
  return (idx < tinyCount) ? tinyAddrs[idx] : 0xFF;
}
