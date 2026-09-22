// Plumb IMU streaming instrument.
//
// Measures what the synthetic harness cannot: sensor axes and signs, the real
// sample rate, the gyro noise floor, and whether the grip mount is rigid
// (spec 5.5). Not product firmware.
//
// Commands, single characters over serial:
//   c  CSV output          b  binary output
//   1  stroke rate         9  maximum rate
//   f  FIFO read path      d  direct register read path
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
// Direct is the default because it is the path that has been measured: zero
// sample loss and a 0.37 dps resting noise floor, against ~20% loss and a
// position-dependent 1.07 dps through the FIFO (docs/bringup-results.md). The
// FIFO path stays for the tap test, which runs at a rate direct polling cannot
// service.
static ReadPath path = ReadPath::Direct;
static bool streaming = false;

// Samples that reached the host, and samples the sensor says it made. These
// used to be the same variable, which is why 20% sample loss reported as
// "dropped: 0": sequence numbers were assigned as seq += count, so anything
// lost inside the part was arithmetically invisible.
//
// On the direct path `produced` now comes from the sensor's own TIMESTAMP
// counter and the gap between the two IS the loss. On the FIFO path the
// counter is frozen, so there is no produced count to be had and the
// instrument says so rather than printing a zero that reads like good news.
static uint32_t delivered = 0;
static uint32_t produced = 0;
static bool producedIsMeasured = false;
static uint32_t overflowCount = 0;
static uint32_t startMicros = 0;

static void printStatus() {
  const uint32_t elapsed = micros() - startMicros;
  Serial.printf("# format=%s rate=%s(%.1f Hz nominal) path=%s streaming=%d\n",
                format == Format::Csv ? "csv" : "binary",
                rate == Rate::Stroke ? "stroke" : "max",
                qmi8658::nominalRateHz(),
                path == ReadPath::Fifo ? "fifo" : "direct", streaming);
  Serial.printf("# delivered=%lu produced=%lu lost=%ld overflows=%lu elapsed_us=%lu\n",
                (unsigned long)delivered, (unsigned long)produced,
                (long)produced - (long)delivered,
                (unsigned long)overflowCount, (unsigned long)elapsed);
  if (streaming && elapsed > 0) {
    // Two rates, because they answer different questions. The delivered rate
    // is what the host will see; the produced rate is the sensor's actual ODR,
    // which is the number spec 9.3 asks for and the one that scales every
    // integrated angle.
    Serial.printf("# delivered_hz=%.2f produced_hz=%.2f\n",
                  delivered * 1e6f / elapsed, produced * 1e6f / elapsed);
  }
}

static void startStop() {
  streaming = !streaming;
  delivered = 0;
  produced = 0;
  producedIsMeasured = false;
  overflowCount = 0;
  // Re-base the sensor's sample counter, so `produced` counts this capture
  // rather than everything since boot.
  if (!qmi8658::resetSampleClock()) {
    Serial.println("# WARNING: sample clock reset failed; produced count is unreliable");
  }
  startMicros = micros();
  Serial.printf("# streaming %d\n", streaming);
}

static void handleCommand(char c) {
  switch (c) {
    case 'c': format = Format::Csv;    Serial.println("# format csv"); break;
    case 'b': format = Format::Binary; Serial.println("# format binary"); break;
    case '1': rate = Rate::Stroke; qmi8658::setRate(rate); Serial.println("# rate stroke"); break;
    case '9': rate = Rate::Max;    qmi8658::setRate(rate); Serial.println("# rate max"); break;
    case 'f':
      path = ReadPath::Fifo;
      Serial.println(qmi8658::setReadPath(path) ? "# path fifo"
                                                : "# path fifo FAILED");
      break;
    case 'd':
      path = ReadPath::Direct;
      Serial.println(qmi8658::setReadPath(path) ? "# path direct"
                                                : "# path direct FAILED");
      break;
    case 's': startStop(); break;
    case '?': printStatus(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("\n# Plumb IMU streaming instrument");

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  if (!qmi8658::begin(rate, path)) {
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

  // Where the sequence numbers come from, and what a gap in them means.
  //
  // Direct path: from the sensor's own counter, exactly, one sample at a time.
  // A gap is a sample the part produced that nothing read -- the loss this
  // instrument exists to measure.
  //
  // FIFO path: from what has been delivered, because the sensor's counter is
  // frozen while the FIFO is enabled. A gap is then a frame lost between the
  // board and the host, and in-sensor loss does not appear at all. The flag on
  // the frame says which of the two the host is looking at, so nobody reads
  // "dropped: 0" as "nothing was lost" a second time.
  uint32_t firstSeq;
  if (r.sensorCounted) {
    firstSeq = r.produced - r.count;
    produced = r.produced;
    producedIsMeasured = true;
  } else {
    firstSeq = delivered;
  }
  delivered += r.count;

  if (format == Format::Csv) {
    stream::emitCsv(Serial, firstSeq, batch, r.count);
  } else {
    stream::emitBinary(Serial, firstSeq, batch, r.count, r.overflow,
                       r.sensorCounted, drainMicros);
  }
}
