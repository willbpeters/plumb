// Plumb IMU streaming instrument.
//
// Measures what the synthetic harness cannot: sensor axes and signs, the real
// sample rate, the gyro noise floor, and whether the grip mount is rigid
// (spec 5.5). Not product firmware.
//
// Commands, single characters over serial:
//   c  CSV output          b  binary output
//   1  stroke rate         9  maximum rate
//   s  start / stop        ?  status

#include <Wire.h>

#include "qmi8658.h"
#include "framing.h"

static const int PIN_SDA = 6;   // spec 4.2
static const int PIN_SCL = 7;
static const uint32_t I2C_HZ = 400000;

static const uint8_t BATCH_MAX = 64;

static Sample batch[BATCH_MAX];
static Format format = Format::Csv;
static Rate rate = Rate::Stroke;
static bool streaming = false;

static uint32_t seq = 0;
static uint32_t overflowCount = 0;
static uint32_t startMicros = 0;

static void printStatus() {
  const uint32_t elapsed = micros() - startMicros;
  Serial.printf("# format=%s rate=%s(%.1f Hz nominal) streaming=%d\n",
                format == Format::Csv ? "csv" : "binary",
                rate == Rate::Stroke ? "stroke" : "max",
                qmi8658::nominalRateHz(), streaming);
  Serial.printf("# samples=%lu overflows=%lu elapsed_us=%lu\n",
                (unsigned long)seq, (unsigned long)overflowCount,
                (unsigned long)elapsed);
  if (streaming && elapsed > 0) {
    Serial.printf("# measured_odr=%.2f Hz\n", seq * 1e6f / elapsed);
  }
}

static void handleCommand(char c) {
  switch (c) {
    case 'c': format = Format::Csv;    Serial.println("# format csv"); break;
    case 'b': format = Format::Binary; Serial.println("# format binary"); break;
    case '1': rate = Rate::Stroke; qmi8658::setRate(rate); Serial.println("# rate stroke"); break;
    case '9': rate = Rate::Max;    qmi8658::setRate(rate); Serial.println("# rate max"); break;
    case 's':
      streaming = !streaming;
      seq = 0;
      overflowCount = 0;
      startMicros = micros();
      Serial.printf("# streaming %d\n", streaming);
      break;
    case '?': printStatus(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("\n# Plumb IMU streaming instrument");

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  if (!qmi8658::begin(rate)) {
    Serial.println("# FATAL: QMI8658 init or config read-back failed");
    while (true) delay(1000);
  }
  Serial.println("# ready -- press s to start, ? for status");
}

void loop() {
  while (Serial.available()) handleCommand((char)Serial.read());

  if (!streaming || !qmi8658::dataReady()) return;

  const uint32_t drainMicros = micros();
  const DrainResult r = qmi8658::drain(batch, BATCH_MAX);
  if (r.count == 0) return;
  if (r.overflow) overflowCount++;

  if (format == Format::Csv) {
    stream::emitCsv(Serial, seq, batch, r.count);
  } else {
    stream::emitBinary(Serial, seq, batch, r.count, r.overflow, drainMicros);
  }
  seq += r.count;
}
