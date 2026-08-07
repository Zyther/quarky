// shared/c2proto/test/test_proto.cpp
#include <unity.h>
#include "proto.h"

using namespace c2proto;

void test_encode_decode_roundtrip() {
    Frame f{};
    f.version = 1;
    f.type = MsgType::CMD_START_FEATURE;
    f.seq = 42;
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    memcpy(f.payload, payload, sizeof(payload));
    f.payload_len = sizeof(payload);

    uint8_t buf[64];
    int n = encode(f, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);

    Frame decoded{};
    bool ok = decode(buf, (size_t)n, decoded);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(1, decoded.version);
    TEST_ASSERT_EQUAL_INT((int)MsgType::CMD_START_FEATURE, (int)decoded.type);
    TEST_ASSERT_EQUAL_UINT16(42, decoded.seq);
    TEST_ASSERT_EQUAL_UINT16(4, decoded.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, decoded.payload, 4);
}

void test_decode_rejects_bad_magic() {
    uint8_t buf[16] = {0};
    Frame decoded{};
    TEST_ASSERT_FALSE(decode(buf, sizeof(buf), decoded));
}

void test_encode_rejects_oversized_payload() {
    Frame f{};
    f.payload_len = c2proto::kMaxPayload + 1;
    uint8_t buf[400];
    TEST_ASSERT_EQUAL_INT(-1, encode(f, buf, sizeof(buf)));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_decode_rejects_bad_magic);
    RUN_TEST(test_encode_rejects_oversized_payload);
    return UNITY_END();
}
