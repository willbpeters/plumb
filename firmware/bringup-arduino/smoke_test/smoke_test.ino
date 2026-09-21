// Plumb bring-up smoke test.
//
// Throwaway Arduino sketch. Proves three things before any real firmware exists:
//   1. The CH343 UART path works  (serial output appears at all)
//   2. The I2C bus is alive       (scan finds devices)
//   3. The QMI8658 IMU responds   (WHO_AM_I reads back)
//
// Expected on the bus (spec 4.2): QMI8658 IMU and CST816S touch controller,
// both on GPIO6/7. The scan tells us the real addresses rather than assuming.

#include <Wire.h>

static const int PIN_SDA = 6;   // spec 4.2
static const int PIN_SCL = 7;
static const uint32_t I2C_HZ = 400000;

static const uint8_t QMI8658_WHO_AM_I = 0x00;   // expect 0x05
static const uint8_t QMI8658_REVISION = 0x01;

// Read one register from `addr`. Returns false if the device did not ack.
static bool readReg(uint8_t addr, uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  out = Wire.read();
  return true;
}

static void scanBus() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  NONE FOUND - check wiring/pins");
  else Serial.printf("  %d device(s)\n", found);
}

static void probeImu() {
  // QMI8658 is at 0x6B when SA0 is high, 0x6A when low. Try both.
  const uint8_t candidates[] = {0x6B, 0x6A};
  for (uint8_t addr : candidates) {
    uint8_t who = 0, rev = 0;
    if (readReg(addr, QMI8658_WHO_AM_I, who)) {
      readReg(addr, QMI8658_REVISION, rev);
      Serial.printf("IMU at 0x%02X: WHO_AM_I=0x%02X REVISION=0x%02X %s\n",
                    addr, who, rev,
                    who == 0x05 ? "(QMI8658 confirmed)" : "(UNEXPECTED - check datasheet)");
      return;
    }
  }
  Serial.println("IMU: no response at 0x6B or 0x6A");
}

void setup() {
  Serial.begin(115200);
  delay(2000);                     // let the CH343 port re-enumerate after reset
  Serial.println("\n=== Plumb smoke test ===");
  Serial.printf("chip: %s rev%d, %d core(s) @ %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(),
                ESP.getChipCores(), getCpuFrequencyMhz());
  Serial.printf("flash: %u bytes\n", ESP.getFlashChipSize());
  Serial.printf("PSRAM: %u bytes  (expect ~2 MB; 0 means PSRAM is misconfigured)\n",
                ESP.getPsramSize());
  Serial.printf("free internal heap: %u bytes\n", ESP.getFreeHeap());

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  scanBus();
  probeImu();
  Serial.println("=== end ===");
}

void loop() {
  delay(5000);
  Serial.println("alive");
}
