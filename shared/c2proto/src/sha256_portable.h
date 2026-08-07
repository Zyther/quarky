// shared/c2proto/src/sha256_portable.h
//
// Portable, dependency-free, header-only SHA-256 implementation.
//
// Vendored and adapted from Brad Conte's public-domain "crypto-algorithms"
// implementation:
//   https://github.com/B-Con/crypto-algorithms (sha256.h / sha256.c)
//
// Per that project's README:
//   "This code is released into the public domain free of any
//   restrictions. The author requests acknowledgement if the code is
//   used, but does not require it."
//
// Adaptations made here (algorithm itself is unchanged from the original):
//   - Renamed SHA256_CTX -> sha256_ctx, BYTE -> uint8_t, WORD -> uint32_t
//   - Converted from a .h/.c pair into a single header of `inline`
//     functions so it compiles identically under host "native" test
//     builds and the ESP32 Arduino toolchain without a separate
//     translation unit or build-system changes.
//   - Added a one-shot sha256_buf() convenience wrapper (init+update+final).
//
// Algorithm reference: FIPS 180-4 (SHA-256).
//
#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace c2proto {

struct sha256_ctx {
    uint8_t data[64];
    uint32_t datalen;
    unsigned long long bitlen;
    uint32_t state[8];
};

namespace detail {

inline uint32_t sha256_rotleft(uint32_t a, uint32_t b) {
    return (a << b) | (a >> (32 - b));
}
inline uint32_t sha256_rotright(uint32_t a, uint32_t b) {
    return (a >> b) | (a << (32 - b));
}

inline uint32_t sha256_ch(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (~x & z);
}
inline uint32_t sha256_maj(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (x & z) ^ (y & z);
}
inline uint32_t sha256_ep0(uint32_t x) {
    return sha256_rotright(x, 2) ^ sha256_rotright(x, 13) ^ sha256_rotright(x, 22);
}
inline uint32_t sha256_ep1(uint32_t x) {
    return sha256_rotright(x, 6) ^ sha256_rotright(x, 11) ^ sha256_rotright(x, 25);
}
inline uint32_t sha256_sig0(uint32_t x) {
    return sha256_rotright(x, 7) ^ sha256_rotright(x, 18) ^ (x >> 3);
}
inline uint32_t sha256_sig1(uint32_t x) {
    return sha256_rotright(x, 17) ^ sha256_rotright(x, 19) ^ (x >> 10);
}

inline const uint32_t *sha256_k() {
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    return k;
}

inline void sha256_transform(sha256_ctx *ctx, const uint8_t data[]) {
    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
    const uint32_t *k = sha256_k();

    for (i = 0, j = 0; i < 16; ++i, j += 4)
        m[i] = (static_cast<uint32_t>(data[j]) << 24) |
               (static_cast<uint32_t>(data[j + 1]) << 16) |
               (static_cast<uint32_t>(data[j + 2]) << 8) |
               (static_cast<uint32_t>(data[j + 3]));
    for (; i < 64; ++i)
        m[i] = sha256_sig1(m[i - 2]) + m[i - 7] + sha256_sig0(m[i - 15]) + m[i - 16];

    a = ctx->state[0];
    b = ctx->state[1];
    c = ctx->state[2];
    d = ctx->state[3];
    e = ctx->state[4];
    f = ctx->state[5];
    g = ctx->state[6];
    h = ctx->state[7];

    for (i = 0; i < 64; ++i) {
        t1 = h + sha256_ep1(e) + sha256_ch(e, f, g) + k[i] + m[i];
        t2 = sha256_ep0(a) + sha256_maj(a, b, c);
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
    ctx->state[5] += f;
    ctx->state[6] += g;
    ctx->state[7] += h;
}

} // namespace detail

inline void sha256_init(sha256_ctx *ctx) {
    ctx->datalen = 0;
    ctx->bitlen = 0;
    ctx->state[0] = 0x6a09e667;
    ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372;
    ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f;
    ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab;
    ctx->state[7] = 0x5be0cd19;
}

inline void sha256_update(sha256_ctx *ctx, const uint8_t data[], size_t len) {
    for (size_t i = 0; i < len; ++i) {
        ctx->data[ctx->datalen] = data[i];
        ctx->datalen++;
        if (ctx->datalen == 64) {
            detail::sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

inline void sha256_final(sha256_ctx *ctx, uint8_t hash[32]) {
    uint32_t i = ctx->datalen;

    // Pad whatever data is left in the buffer.
    if (ctx->datalen < 56) {
        ctx->data[i++] = 0x80;
        while (i < 56) ctx->data[i++] = 0x00;
    } else {
        ctx->data[i++] = 0x80;
        while (i < 64) ctx->data[i++] = 0x00;
        detail::sha256_transform(ctx, ctx->data);
        std::memset(ctx->data, 0, 56);
    }

    // Append to the padding the total message's length in bits and transform.
    ctx->bitlen += static_cast<unsigned long long>(ctx->datalen) * 8;
    ctx->data[63] = static_cast<uint8_t>(ctx->bitlen);
    ctx->data[62] = static_cast<uint8_t>(ctx->bitlen >> 8);
    ctx->data[61] = static_cast<uint8_t>(ctx->bitlen >> 16);
    ctx->data[60] = static_cast<uint8_t>(ctx->bitlen >> 24);
    ctx->data[59] = static_cast<uint8_t>(ctx->bitlen >> 32);
    ctx->data[58] = static_cast<uint8_t>(ctx->bitlen >> 40);
    ctx->data[57] = static_cast<uint8_t>(ctx->bitlen >> 48);
    ctx->data[56] = static_cast<uint8_t>(ctx->bitlen >> 56);
    detail::sha256_transform(ctx, ctx->data);

    // This implementation uses little-endian internal byte ordering and
    // SHA-256 output is big-endian, so reverse bytes when copying out.
    for (i = 0; i < 4; ++i) {
        hash[i] = (ctx->state[0] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 4] = (ctx->state[1] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 8] = (ctx->state[2] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 20] = (ctx->state[5] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 24] = (ctx->state[6] >> (24 - i * 8)) & 0x000000ff;
        hash[i + 28] = (ctx->state[7] >> (24 - i * 8)) & 0x000000ff;
    }
}

// One-shot convenience wrapper: init + update + final.
inline void sha256_buf(const uint8_t *data, size_t len, uint8_t out[32]) {
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, data, len);
    sha256_final(&ctx, out);
}

} // namespace c2proto
