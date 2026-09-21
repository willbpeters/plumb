#include "framing.h"

namespace {
constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kFlagOverflow = 0x01;

// Accumulate the checksum over everything after the sync word, exactly as
// capture.py does. Writing the bytes and the checksum in one pass keeps the two
// implementations from drifting.
struct ChecksumWriter {
  Stream& out;
  uint8_t sum = 0;
  void write(uint8_t b) { out.write(b); sum ^= b; }
  void write32(uint32_t v) {
    write(v & 0xFF); write((v >> 8) & 0xFF);
    write((v >> 16) & 0xFF); write((v >> 24) & 0xFF);
  }
  void write16(int16_t v) {
    write((uint16_t)v & 0xFF); write(((uint16_t)v >> 8) & 0xFF);
  }
};
}  // namespace

void stream::emitCsv(Stream& out, uint32_t firstSeq, const Sample* s, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    out.printf("%lu,%d,%d,%d,%d,%d,%d\n", (unsigned long)(firstSeq + i),
               s[i].ax, s[i].ay, s[i].az, s[i].gx, s[i].gy, s[i].gz);
  }
}

void stream::emitBinary(Stream& out, uint32_t firstSeq, const Sample* s,
                         uint8_t count, bool overflow, uint32_t drainMicros) {
  out.write(kSync0);
  out.write(kSync1);

  ChecksumWriter w{out};
  w.write32(firstSeq);
  w.write(count);
  w.write(overflow ? kFlagOverflow : 0);
  w.write32(drainMicros);
  for (uint8_t i = 0; i < count; i++) {
    w.write16(s[i].ax); w.write16(s[i].ay); w.write16(s[i].az);
    w.write16(s[i].gx); w.write16(s[i].gy); w.write16(s[i].gz);
  }
  out.write(w.sum);
}
