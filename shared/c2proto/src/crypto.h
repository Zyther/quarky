// shared/c2proto/src/crypto.h
#pragma once
#include <cstdint>
#include <cstddef>

namespace c2proto {

void generate_psk(uint8_t out[16]);
void hmac_sha256(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  uint8_t out[32]);
bool hmac_verify(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  const uint8_t expected[32]);

} // namespace c2proto
