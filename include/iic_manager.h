#ifndef IIC_MANAGER_H
#define IIC_MANAGER_H

#include <Wire.h>

#define SLAVE_ADDR 0x10

#define SCL_PIN 1
#define SDA_PIN 0

#define CMD_BLINK 0x01

extern volatile bool blinkPending;

void setupI2C();

#endif // !IIC_MANAGER_H
