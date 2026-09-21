// Wire formats for the IMU streaming instrument.
//
// The binary layout is pinned in the implementation plan and implemented twice:
// here, and in analysis/tools/capture.py. Keep them in step.

#pragma once

#include <Arduino.h>

#include "qmi8658.h"

enum class Format { Csv, Binary };

namespace stream {

// One CSV line per sample: seq,ax,ay,az,gx,gy,gz
void emitCsv(Stream& out, uint32_t firstSeq, const Sample* samples, uint8_t count);

// One framed batch. Layout is in the plan: sync, header, samples, XOR checksum.
void emitBinary(Stream& out, uint32_t firstSeq, const Sample* samples,
                 uint8_t count, bool overflow, uint32_t drainMicros);

}  // namespace stream
