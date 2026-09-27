// See cst816s.h. Registers from the CST816S register description
// (Hynitron), as linked from the Waveshare wiki for this board.

#include "cst816s.h"

#include <Arduino.h>
#include <Wire.h>

static const uint8_t ADDRESS = 0x15;
static const int PIN_RST = 13;

static const uint8_t REG_FINGER_NUM = 0x02;  // then XH, XL, YH, YL
static const uint8_t REG_CHIP_ID = 0xA7;
static const uint8_t REG_DIS_AUTO_SLEEP = 0xFE;

static bool write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t reg, uint8_t *out, size_t n) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDRESS, (int)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

bool cst816s_begin() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  delay(60);  // the chip answers for a short while after reset
  uint8_t id = 0;
  const bool answered = read_regs(REG_CHIP_ID, &id, 1);
  // Keep it awake so an idle read is "no finger", not a bus error. Best
  // effort: if this write is lost the NACK path covers it.
  write_reg(REG_DIS_AUTO_SLEEP, 0x01);
  Serial.printf("touch: CST816S %s, chip id 0x%02X\n",
                answered ? "answered" : "DID NOT ANSWER", id);
  return answered;
}

bool cst816s_read(uint16_t &x, uint16_t &y) {
  uint8_t d[5];
  if (!read_regs(REG_FINGER_NUM, d, sizeof(d))) return false;
  if (d[0] == 0) return false;
  x = (uint16_t)(((d[1] & 0x0F) << 8) | d[2]);
  y = (uint16_t)(((d[3] & 0x0F) << 8) | d[4]);
  return true;
}
