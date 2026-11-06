#include "rec/block.h"

#include <string.h>

#include "rec/crc32.h"

static uint32_t zigzag(int32_t d)
{
    return d >= 0 ? (uint32_t)d * 2u : (uint32_t)(-(d + 1)) * 2u + 1u;
}

/* Bits to code z with parameter k, including the escape. */
static uint32_t code_bits(uint32_t z, uint32_t k)
{
    const uint32_t q = z >> k;
    return q < REC_RICE_ESCAPE ? q + 1u + k : REC_RICE_ESCAPE + REC_RICE_RAW_BITS;
}

typedef struct {
    uint8_t *out;
    size_t byte;   /* current byte */
    uint32_t bit;  /* bits already used in it, 0..7 */
} bit_writer;

static void put_bit(bit_writer *w, uint32_t bit)
{
    if (w->bit == 0) {
        w->out[w->byte] = 0;
    }
    if (bit) {
        w->out[w->byte] |= (uint8_t)(0x80u >> w->bit);
    }
    if (++w->bit == 8) {
        w->bit = 0;
        w->byte++;
    }
}

static void put_bits(bit_writer *w, uint32_t value, uint32_t n)
{
    while (n--) {
        put_bit(w, (value >> n) & 1u);
    }
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v);
    put16(p + 2, (uint16_t)(v >> 16));
}

size_t rec_block_encode(uint8_t *out, uint32_t first_index,
                        const int16_t samples[][REC_AXES], uint8_t count)
{
    uint8_t k[REC_AXES];
    for (uint32_t a = 0; a < REC_AXES; a++) {
        uint32_t best_bits = UINT32_MAX;
        for (uint32_t kk = 0; kk <= REC_RICE_K_MAX; kk++) {
            uint32_t bits = 0;
            for (uint32_t i = 1; i < count; i++) {
                bits += code_bits(zigzag((int32_t)samples[i][a]
                                         - (int32_t)samples[i - 1][a]), kk);
            }
            if (bits < best_bits) {  /* strict: the smallest k on a tie */
                best_bits = bits;
                k[a] = (uint8_t)kk;
            }
        }
    }

    bit_writer w = {out + REC_BLOCK_HEADER_BYTES, 0, 0};
    for (uint32_t a = 0; a < REC_AXES; a++) {
        for (uint32_t i = 1; i < count; i++) {
            const uint32_t z = zigzag((int32_t)samples[i][a]
                                      - (int32_t)samples[i - 1][a]);
            const uint32_t q = z >> k[a];
            if (q < REC_RICE_ESCAPE) {
                put_bits(&w, (1u << q) - 1u, q); /* q ones */
                put_bit(&w, 0);
                put_bits(&w, z & ((1u << k[a]) - 1u), k[a]);
            } else {
                put_bits(&w, (1u << REC_RICE_ESCAPE) - 1u, REC_RICE_ESCAPE);
                put_bits(&w, z, REC_RICE_RAW_BITS);
            }
        }
    }
    const size_t payload = w.byte + (w.bit ? 1u : 0u);

    put16(out, (uint16_t)REC_BLOCK_MAGIC);
    put16(out + 2, (uint16_t)payload);
    put32(out + 4, first_index);
    out[8] = count;
    out[9] = 0;
    memcpy(out + 10, k, REC_AXES);
    for (uint32_t a = 0; a < REC_AXES; a++) {
        put16(out + 16 + 2 * a, (uint16_t)samples[0][a]);
    }
    const size_t body = REC_BLOCK_HEADER_BYTES + payload;
    put32(out + body, rec_crc32(out, body));
    return body + 4u;
}
