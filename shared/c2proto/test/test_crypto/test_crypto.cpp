// shared/c2proto/test/test_crypto/test_crypto.cpp
#include <unity.h>
#include "crypto.h"
#include "sha256_portable.h"
#include <cstring>

using namespace c2proto;

void test_psk_is_nonzero_and_16_bytes() {
    uint8_t psk[16] = {0};
    generate_psk(psk);
    bool all_zero = true;
    for (int i = 0; i < 16; i++) if (psk[i] != 0) all_zero = false;
    TEST_ASSERT_FALSE(all_zero);
}

void test_hmac_roundtrip_valid() {
    const uint8_t key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    const uint8_t msg[] = "start_feature:wifi_scan";
    uint8_t mac[32];
    hmac_sha256(key, sizeof(key), msg, sizeof(msg) - 1, mac);
    TEST_ASSERT_TRUE(hmac_verify(key, sizeof(key), msg, sizeof(msg) - 1, mac));
}

void test_hmac_rejects_tampered_message() {
    const uint8_t key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    const uint8_t msg[] = "start_feature:wifi_scan";
    uint8_t mac[32];
    hmac_sha256(key, sizeof(key), msg, sizeof(msg) - 1, mac);

    const uint8_t tampered[] = "start_feature:wifi_scam";
    TEST_ASSERT_FALSE(hmac_verify(key, sizeof(key), tampered, sizeof(tampered) - 1, mac));
}

// Known-answer test (NIST): SHA-256("abc") =
// ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad
// This checks sha256_buf() directly against an externally-known digest,
// unlike the HMAC roundtrip/tamper tests above which are only
// self-consistent and would pass even against a broken-but-deterministic
// hash function.
void test_sha256_known_answer_abc() {
    const uint8_t expected[32] = {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
        0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
        0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
        0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
    };
    uint8_t out[32];
    sha256_buf(reinterpret_cast<const uint8_t *>("abc"), 3, out);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, out, 32);
}

// Known-answer test (RFC 4231, Test Case 1):
// key = 20 bytes of 0x0b, data = "Hi There"
// HMAC-SHA256 = b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7
void test_hmac_known_answer_rfc4231_case1() {
    uint8_t key[20];
    memset(key, 0x0b, sizeof(key));
    const uint8_t data[] = "Hi There";
    const uint8_t expected[32] = {
        0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53,
        0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
        0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7,
        0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7,
    };
    uint8_t mac[32];
    hmac_sha256(key, sizeof(key), data, sizeof(data) - 1, mac);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, mac, 32);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_psk_is_nonzero_and_16_bytes);
    RUN_TEST(test_hmac_roundtrip_valid);
    RUN_TEST(test_hmac_rejects_tampered_message);
    RUN_TEST(test_sha256_known_answer_abc);
    RUN_TEST(test_hmac_known_answer_rfc4231_case1);
    return UNITY_END();
}
