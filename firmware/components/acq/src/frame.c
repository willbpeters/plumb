#include "acq/frame.h"

typedef struct {
    uint8_t *out;
    size_t n;
    uint8_t sum;
} writer;

static void put(writer *w, uint8_t b)
{
    w->out[w->n++] = b;
    w->sum ^= b;
}

static void put32(writer *w, uint32_t v)
{
    put(w, (uint8_t)(v & 0xFFu));
    put(w, (uint8_t)((v >> 8) & 0xFFu));
    put(w, (uint8_t)((v >> 16) & 0xFFu));
    put(w, (uint8_t)((v >> 24) & 0xFFu));
}

static void put16(writer *w, int16_t v)
{
    const uint16_t u = (uint16_t)v;
    put(w, (uint8_t)(u & 0xFFu));
    put(w, (uint8_t)(u >> 8));
}

size_t acq_frame_encode(uint8_t *out, uint32_t first_seq, uint8_t flags,
                        uint32_t micros, const acq_record *records,
                        uint8_t count)
{
    out[0] = 0xA5;
    out[1] = 0x5A;
    writer w = {out, 2, 0};
    put32(&w, first_seq);
    put(&w, count);
    put(&w, flags);
    put32(&w, micros);
    for (uint8_t i = 0; i < count; i++) {
        for (int axis = 0; axis < 3; axis++) {
            put16(&w, records[i].accel[axis]);
        }
        for (int axis = 0; axis < 3; axis++) {
            put16(&w, records[i].gyro[axis]);
        }
    }
    out[w.n] = w.sum;
    return w.n + 1;
}
