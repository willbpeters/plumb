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
//
// `sensorSeq` sets the flag that tells the host what the sequence numbers ARE.
// On the direct path they come from the sensor's own sample counter, so a gap
// means the part produced a sample nobody read. On the FIFO path they are a
// count of what was delivered, because the sensor's counter is frozen while
// the FIFO is enabled -- so a gap there means a frame was lost between the
// board and the host, and in-sensor loss is not measurable at all. Two
// different meanings for one field; the host should not have to guess which.
void emitBinary(Stream& out, uint32_t firstSeq, const Sample* samples,
                 uint8_t count, bool overflow, bool sensorSeq,
                 uint32_t drainMicros);

}  // namespace stream
