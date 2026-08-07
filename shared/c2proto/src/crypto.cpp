// shared/c2proto/src/crypto.cpp
#include "crypto.h"
#include "sha256_portable.h"
#include <cstring>

#if defined(ARDUINO)
#include <esp_random.h>
#endif

namespace c2proto {

static uint32_t portable_random32() {
#if defined(ARDUINO)
    return esp_random();
#else
    // Host-native test build only: not cryptographically relevant here,
    // since PSK generation is exercised for structure not entropy quality
    // in the native test suite. On-device builds always use esp_random().
    static uint32_t seed = 0x9E3779B9u;
    seed = seed * 1103515245u + 12345u;
    return seed;
#endif
}

void generate_psk(uint8_t out[16]) {
    for (int i = 0; i < 4; i++) {
        uint32_t r = portable_random32();
        memcpy(out + i * 4, &r, 4);
    }
}

// HMAC-SHA256 per RFC 2104, built on the vendored portable sha256 block functions.
void hmac_sha256(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  uint8_t out[32]) {
    uint8_t k_ipad[64], k_opad[64], key_block[64] = {0};

    if (key_len > 64) {
        sha256_buf(key, key_len, key_block); // reduces long keys to 32 bytes, rest stays 0
    } else {
        memcpy(key_block, key, key_len);
    }

    for (int i = 0; i < 64; i++) {
        k_ipad[i] = key_block[i] ^ 0x36;
        k_opad[i] = key_block[i] ^ 0x5c;
    }

    uint8_t inner[32];
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, k_ipad, 64);
    sha256_update(&ctx, msg, msg_len);
    sha256_final(&ctx, inner);

    sha256_init(&ctx);
    sha256_update(&ctx, k_opad, 64);
    sha256_update(&ctx, inner, 32);
    sha256_final(&ctx, out);
}

bool hmac_verify(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  const uint8_t expected[32]) {
    uint8_t actual[32];
    hmac_sha256(key, key_len, msg, msg_len, actual);
    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) diff |= actual[i] ^ expected[i]; // constant-time compare
    return diff == 0;
}

} // namespace c2proto
