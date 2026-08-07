// shared/c2proto/test/test_crypto.cpp
#include <unity.h>
#include "crypto.h"
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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_psk_is_nonzero_and_16_bytes);
    RUN_TEST(test_hmac_roundtrip_valid);
    RUN_TEST(test_hmac_rejects_tampered_message);
    return UNITY_END();
}
