// Register-level driver for the QMI8658 6-axis IMU.
//
// Knows nothing about serial ports or output formats. That boundary is what
// makes the eventual ESP-IDF port a translation rather than a rewrite.

#pragma once

#include <Arduino.h>

enum class Rate {
  Stroke,   // nearest supported ODR to 500 Hz -- stroke capture, noise floor
  Max,      // highest supported ODR -- tap test (spec 5.5 needs > 1 kHz)
};

// How samples get off the part. Two read paths, because the FIFO path loses
// about a fifth of its samples on this board and nobody yet knows whether that
// is the FIFO or the way this driver drives it (docs/bringup-results.md,
// defects 2 and 5). Direct reads bypass the FIFO entirely, which makes the
// comparison decisive: same sensor, same configuration, same environment, one
// difference.
enum class ReadPath {
  Fifo,     // watermark interrupt, CTRL9 read mode, batch drains (spec 6.4)
  Direct,   // poll STATUS0, burst-read the output registers, no FIFO at all
};

struct Sample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct DrainResult {
  uint8_t count;      // samples written to the caller's buffer
  bool overflow;      // FIFO overflowed since the previous drain
  // Samples the SENSOR has produced since resetSampleClock(), from its own
  // TIMESTAMP register: what the part made, not what reached the host. So
  // `produced - delivered` over a capture is the sample loss, which was
  // invisible while sequence numbers came from the delivered count.
  //
  // Only meaningful when `sensorCounted` is true. Measured on hardware, the
  // TIMESTAMP register does not advance while the FIFO is enabled -- see the
  // note in qmi8658.cpp -- so the FIFO path cannot count its own losses and
  // says so here rather than returning a number that looks like one.
  uint32_t produced;
  bool sensorCounted;
};

namespace qmi8658 {

// Returns false if WHO_AM_I does not read back 0x05, or if any configuration
// register fails to read back the value that was written.
bool begin(Rate rate, ReadPath path);

// Reconfigure the output data rate. Same read-back guarantee as begin().
bool setRate(Rate rate);

// Switch read paths. Same read-back guarantee as begin().
bool setReadPath(ReadPath path);

// Re-base the produced-sample counter on the sensor's current TIMESTAMP, and
// discard anything already buffered. Call this when a capture starts.
bool resetSampleClock();

// True when there is at least one sample to collect.
bool dataReady();

// Collects up to `capacity` samples. Never blocks.
DrainResult drain(Sample* out, uint8_t capacity);

// Nominal rate in Hz for the configured mode, from the datasheet table.
// The instrument measures the real rate anyway -- see the plan preamble.
float nominalRateHz();

}  // namespace qmi8658
