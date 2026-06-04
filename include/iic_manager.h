#ifndef IIC_MANAGER_H
#define IIC_MANAGER_H

#include <EEPROM.h>
#include <Wire.h>
#include <led_manager.h>

// ── Bus 0 pins (arduino-pico defaults) ───────────────────────
#define BUS0_SDA 0
#define BUS0_SCL 1

// ── Bus 1 pins ────────────────────────────────────────────────
#define BUS1_SDA 26
#define BUS1_SCL 27

// ── ALERT pins ────────────────────────────────────────────────
#define PIN_ALERT_UP 28  // ALERT to ESP32 (open-drain output)
#define PIN_ALERT_DOWN 2 // ALERT from ATtiny85 (input, external pull-up)

// ── I2C addresses ─────────────────────────────────────────────
#define ADDR_ARP_DEFAULT 0x55
#define ADDR_UNASSIGNED 0x55
#define ADDR_ESP32_BCAST 0x00

// ── Commands (shared with ESP32 and ATtiny85) ─────────────────
#define CMD_GET_UDID 0x01
#define CMD_ASSIGN_ADDR 0x02
#define CMD_PREPARE_ARP 0x03
#define CMD_SCAN_MODULES 0x05
#define CMD_SYNC_PSA 0x06
#define CMD_LED_GREEN 0x10
#define CMD_LED_RED 0x11
#define CMD_LED_BLUE 0x12

// ── EEPROM layout ─────────────────────────────────────────────
// 0-8:   Own UDID (9 bytes)
// 9:     AV flag (1 byte)
// 10:    Assigned addr (1 byte)
// 11-90: PSA table, 8 entries x 10 bytes
//        each entry: [serial 4 bytes][psa 1 byte][valid 1 byte][udid 9 bytes -
//        serial overlap] simplified: [udid 9 bytes][psa 1 byte]
#define EE_UDID_START 0
#define EE_AV_FLAG 9
#define EE_ASSIGNED_ADDR 10
#define EE_PSA_TABLE_START 11
#define PSA_ENTRY_SIZE 11 // 9 UDID + 1 PSA + 1 persistent flag
#define PSA_MAX_ENTRIES 8
#define EEPROM_SIZE 256

// ── UDID constants ────────────────────────────────────────────
#define UDID_SIZE 9
#define DEVICE_TYPE_RP2040 0x2040
#define PROTOCOL_VERSION 0x0001
#define CAPABILITIES 0x03 // master + slave

#define MAX_TINY_MODULES 8

#define RX_BUF_SIZE 16

#define SCAN_INTERVAL_MS 5000

void iic_init();
void iic_update();
void iic_writeUDID(uint32_t serialNumber);

#endif // !IIC_MANAGER_H
