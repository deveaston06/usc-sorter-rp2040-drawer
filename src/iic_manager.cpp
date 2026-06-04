// ─────────────────────────────────────────────────────────────
// iic_manager.cpp — RP2040 Drawer
// arduino-pico Wire library (PlatformIO)
//
// Two I2C buses:
//   Wire  (Bus 0) — slave to ESP32     SDA GP4 / SCL GP5
//   Wire1 (Bus 1) — master to ATtiny85 SDA GP2 / SCL GP3
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
static uint8_t replyBuf[MAX_TINY_MODULES + 1];
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

static void alertUp_release() { digitalWrite(PIN_ALERT_UP, HIGH); }

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
      uint8_t persistent = EEPROM.read(base + UDID_SIZE + 1);
      // only return stored address if ESP32 has marked it persistent
      if (persistent == 0x01)
        return EEPROM.read(base + UDID_SIZE);
    }
  }
  return 0xFF;
}

static void psa_store(const uint8_t *targetUdid, uint8_t psa,
                      uint8_t persistent) {
  // update existing entry
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
  // buf[0] = CMD_SYNC_PSA already consumed
  // buf[1] = count, buf[2..] = addresses
  if (len < 2)
    return;
  uint8_t count = buf[1];
  if (count > PSA_MAX_ENTRIES)
    count = PSA_MAX_ENTRIES;

  for (uint8_t i = 0; i < PSA_MAX_ENTRIES; i++) {
    uint16_t base = EE_PSA_TABLE_START + (i * PSA_ENTRY_SIZE);
    if (EEPROM.read(base) == 0xFF)
      continue; // empty slot

    uint8_t storedPSA = EEPROM.read(base + UDID_SIZE);
    bool isInList = false;

    for (uint8_t j = 0; j < count; j++) {
      if (buf[2 + j] == storedPSA) {
        isInList = true;
        break;
      }
    }

    uint8_t newFlag = isInList ? 0x01 : 0x00;
    if (EEPROM.read(base + UDID_SIZE + 1) != newFlag) {
      EEPROM.write(base + UDID_SIZE + 1, newFlag);
    }
  }
  EEPROM.commit();
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
  if (tinyCount < MAX_TINY_MODULES && !tiny_isKnown(addr)) {
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
static uint8_t nextAvailableAddr = 0x20;

static uint8_t getNextAddr() {
  // skip addresses already in use
  while (tiny_isKnown(nextAvailableAddr) && nextAvailableAddr < 0x40) {
    nextAvailableAddr++;
  }
  return nextAvailableAddr++;
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: GET_UDID CYCLE FOR ATTINY85
// Stores working PSA entry (persistent=0) after assignment.
// psa_lookup() only returns the address if ESP32 later marks it persistent.
// ─────────────────────────────────────────────────────────────
static bool downstream_getUdidCycle() {
  // send CMD_GET_UDID to ARP default address
  Wire.beginTransmission(ADDR_ARP_DEFAULT);
  Wire.write(CMD_GET_UDID);
  uint8_t err = Wire.endTransmission();
  if (err != 0)
    return false; // no device at 0x55

  // request UDID bytes
  uint8_t received = Wire.requestFrom(ADDR_ARP_DEFAULT, (uint8_t)UDID_SIZE);
  if (received < UDID_SIZE)
    return false;

  uint8_t winnerUdid[UDID_SIZE];
  for (uint8_t i = 0; i < UDID_SIZE; i++) {
    winnerUdid[i] = Wire.read();
  }

  // PSA lookup — only succeeds if ESP32 marked this address persistent
  uint8_t newAddr = psa_lookup(winnerUdid);
  if (newAddr == 0xFF) {
    newAddr = getNextAddr();
  }

  // send ASSIGN_ADDR: [CMD][UDID 9 bytes][new addr]
  Wire.beginTransmission(ADDR_ARP_DEFAULT);
  Wire.write(CMD_ASSIGN_ADDR);
  for (uint8_t i = 0; i < UDID_SIZE; i++) {
    Wire.write(winnerUdid[i]);
  }
  Wire.write(newAddr);
  Wire.endTransmission();

  // small delay for slave to reinitialize its Wire
  delay(10);

  // verify by probing new address
  Wire.beginTransmission(newAddr);
  err = Wire.endTransmission();
  if (err == 0) {
    tiny_add(newAddr);
    // store working record, not persistent until ESP32 syncs
    psa_store(winnerUdid, newAddr, 0x00);
    return true;
  }
  return false;
}

// ─────────────────────────────────────────────────────────────
// DOWNSTREAM: FULL ENUMERATION SEQUENCE
// Repeats GET_UDID cycles until no more unresolved devices
// ─────────────────────────────────────────────────────────────
static void downstream_enumerate() {
  // reset all devices first
  Wire.beginTransmission(ADDR_ARP_DEFAULT);
  Wire.write(CMD_PREPARE_ARP);
  Wire.endTransmission();
  delay(5);

  // run cycles until no more responses
  uint8_t maxCycles = MAX_TINY_MODULES;
  while (maxCycles-- > 0) {
    if (!downstream_getUdidCycle())
      break;
  }
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
        EEPROM.write(EE_AV_FLAG, 0x01);
        EEPROM.write(EE_ASSIGNED_ADDR, assignedAddr);
        EEPROM.commit();

        Wire1.end();
        Wire1.begin(assignedAddr);
        Wire1.onReceive(onReceive);
        Wire1.onRequest(onRequest);

        alertUp_release();
        led_blinkRedOnAssignment();
      }
    }
    break;

  case CMD_PREPARE_ARP:
    addressResolved = false;
    EEPROM.write(EE_AV_FLAG, 0x00);
    EEPROM.commit();

    Wire1.end();
    Wire1.begin(ADDR_ARP_DEFAULT);
    Wire1.onReceive(onReceive);
    Wire1.onRequest(onRequest);
    alertUp_assert();
    break;

  // ESP32 requests list of ATtiny85 addresses
  // Response is sent via onRequest on next master read
  // Store count + list in a reply buffer for onRequest
  case CMD_SCAN_MODULES:
    downstream_scan();
    replyBuf[0] = tinyCount;
    for (uint8_t i = 0; i < tinyCount; i++)
      replyBuf[i + 1] = tinyAddrs[i];
    replyLen = tinyCount + 1;
    replyReady = true;
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
// ALERT DOWN ISR
// ─────────────────────────────────────────────────────────────
static volatile bool alertDownPending = false;

static void onAlertDown() { alertDownPending = true; }

// ─────────────────────────────────────────────────────────────
// PUBLIC: INIT
// ─────────────────────────────────────────────────────────────
void iic_init() {
  EEPROM.begin(EEPROM_SIZE);

  // ALERT up — open-drain output initially asserted
  alertUp_assert();

  // ALERT down — floating input with external pull-up, interrupt on falling
  // edge
  pinMode(PIN_ALERT_DOWN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ALERT_DOWN), onAlertDown, FALLING);

  // load own UDID
  udid_load();

  // Bus 1 — slave to ESP32
  Wire1.setSDA(BUS1_SDA);
  Wire1.setSCL(BUS1_SCL);
  uint8_t avFlag = EEPROM.read(EE_AV_FLAG);
  if (avFlag == 0x01) {
    assignedAddr = EEPROM.read(EE_ASSIGNED_ADDR);
    addressResolved = true;
    Wire1.begin(assignedAddr);
    alertUp_release();
  } else {
    Wire1.begin(ADDR_ARP_DEFAULT);
  }
  Wire1.onReceive(onReceive);
  Wire1.onRequest(onRequest);

  // Bus 0 — master to ATtiny85
  Wire.setSDA(BUS0_SDA);
  Wire.setSCL(BUS0_SCL);
  Wire.begin();
  Wire.setClock(100000);
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
    alertDownPending = false;
    downstream_enumerate();
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
