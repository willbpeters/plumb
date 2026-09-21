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

struct Sample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct DrainResult {
  uint8_t count;      // samples written to the caller's buffer
  bool overflow;      // FIFO overflowed since the previous drain
};

namespace qmi8658 {

// Returns false if WHO_AM_I does not read back 0x05, or if any configuration
// register fails to read back the value that was written.
bool begin(Rate rate);

// Reconfigure the output data rate. Same read-back guarantee as begin().
bool setRate(Rate rate);

// True when the FIFO has reached its watermark.
bool dataReady();

// Drains up to `capacity` samples. Never blocks.
DrainResult drain(Sample* out, uint8_t capacity);

// Nominal rate in Hz for the configured mode, from the datasheet table.
// The instrument measures the real rate anyway -- see the plan preamble.
float nominalRateHz();

}  // namespace qmi8658
