#include <unity.h>
#include "features/ir/ir_file_format.h"
#include "features/ir/ir_nec_encode.h"
#include <cstring>

// ===========================================================================
// Host-native tests for IrNecEncode (Phase 3 Task 18: NEC/NECext protocol
// encoder). Runs via `pio test -e native` from firmware/tab5/, same
// [env:native] target Tasks 7/21 established (see platformio.ini, extended
// this task to also build ir_nec_encode.cpp). ir_nec_encode.cpp doesn't
// include Arduino.h and depends only on IrFileFormat::IrSignal's plain
// struct definition (ir_file_format.h) -- the same host-testability shape
// as ir_file_format.cpp/rf433_protocol_decode.cpp.
//
// Fixtures are the two REAL sample .ir files read in full this session from
// the real, on-SD-card Flipper-IRDB (`/quarky/ir/flipperdb/Window_cleaners/
// HOBOT/Hobot_2S.ir` and `HOBOT.ir`) -- real community data, not synthetic,
// matching Task 7/21's own precedent for host-testable modules. Expected
// duration values below are hand-derived from the real NEC/NECext bit
// order and complement rules cited in ir_nec_encode.h's header (Flipper
// Devices' own real firmware source), not copied from this module's own
// implementation -- an independent check, the same discipline this
// project's other protocol-encode/decode tests already follow.
// ===========================================================================

namespace {

void make_signal(IrFileFormat::IrSignal *sig, const char *protocol, const uint8_t *address,
                  size_t address_len, const uint8_t *command, size_t command_len) {
    std::memset(sig, 0, sizeof(*sig));
    sig->type = IrFileFormat::SignalType::kParsed;
    std::strncpy(sig->protocol, protocol, IrFileFormat::kProtocolMaxLen - 1);
    std::memcpy(sig->address, address, address_len);
    sig->address_len = address_len;
    std::memcpy(sig->command, command, command_len);
    sig->command_len = command_len;
}

} // namespace

// ── Hobot_2S.ir, SPRAY_TOGGLE: protocol NECext, address C5 1F 00 00,
// command 0D F2 00 00 (real file content, see this task's brief for the
// full verbatim file). NECext transmits the two real stored address bytes,
// then the two real stored command bytes, literally -- no complement. Real
// frame bytes, transmission order: C5, 1F, 0D, F2.
void test_encode_necext_hobot2s_spray_toggle(void) {
    const uint8_t address[4] = {0xC5, 0x1F, 0x00, 0x00};
    const uint8_t command[4] = {0x0D, 0xF2, 0x00, 0x00};
    IrFileFormat::IrSignal sig;
    make_signal(&sig, "NECext", address, 4, command, 4);

    uint16_t durations[IrNecEncode::kDurationsPerFrame];
    size_t count = 0;
    uint32_t carrier = 0;
    TEST_ASSERT_TRUE(IrNecEncode::encode(sig, durations, IrNecEncode::kDurationsPerFrame, &count, &carrier));
    TEST_ASSERT_EQUAL_UINT32(38000, carrier);
    TEST_ASSERT_EQUAL_size_t(67, count); // 2 preamble + 32*2 bit halves + 1 terminating mark

    // Preamble.
    TEST_ASSERT_EQUAL_UINT16(9000, durations[0]);
    TEST_ASSERT_EQUAL_UINT16(4500, durations[1]);

    // Byte 0 = 0xC5 = 0b11000101 -- LSB-first bit-cells: 1,0,1,0,0,0,1,1.
    // Every mark is 560us; space is 1690 for a '1' bit, 560 for a '0' bit.
    const bool byte0_bits[8] = {true, false, true, false, false, false, true, true};
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + i * 2]);
        TEST_ASSERT_EQUAL_UINT16(byte0_bits[i] ? 1690 : 560, durations[2 + i * 2 + 1]);
    }

    // Byte 1 = 0x1F = 0b00011111 -- LSB-first bit-cells: 1,1,1,1,1,0,0,0.
    const bool byte1_bits[8] = {true, true, true, true, true, false, false, false};
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + 16 + i * 2]);
        TEST_ASSERT_EQUAL_UINT16(byte1_bits[i] ? 1690 : 560, durations[2 + 16 + i * 2 + 1]);
    }

    // Terminating mark (final element, index 66).
    TEST_ASSERT_EQUAL_UINT16(560, durations[66]);
}

// ── HOBOT.ir, UP: protocol NEC, address 00 00 00 00, command 05 00 00 00
// (real file content). NEC transmits address, ~address, command, ~command
// -- the complement bytes are DERIVED, not read from the stored fields'
// own zero-padding bytes. Real frame bytes, transmission order: 00, FF,
// 05, FA.
void test_encode_nec_hobot_up(void) {
    const uint8_t address[4] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t command[4] = {0x05, 0x00, 0x00, 0x00};
    IrFileFormat::IrSignal sig;
    make_signal(&sig, "NEC", address, 4, command, 4);

    uint16_t durations[IrNecEncode::kDurationsPerFrame];
    size_t count = 0;
    uint32_t carrier = 0;
    TEST_ASSERT_TRUE(IrNecEncode::encode(sig, durations, IrNecEncode::kDurationsPerFrame, &count, &carrier));
    TEST_ASSERT_EQUAL_UINT32(38000, carrier);
    TEST_ASSERT_EQUAL_size_t(67, count);

    TEST_ASSERT_EQUAL_UINT16(9000, durations[0]);
    TEST_ASSERT_EQUAL_UINT16(4500, durations[1]);

    // Byte 0 = address = 0x00 -- every bit 0, every space 560us.
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + i * 2]);
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + i * 2 + 1]);
    }

    // Byte 1 = ~address = 0xFF -- every bit 1, every space 1690us. This is
    // the real, concrete proof the complement is derived correctly: a
    // real stored address of 0x00 must produce an all-1690 second byte.
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + 16 + i * 2]);
        TEST_ASSERT_EQUAL_UINT16(1690, durations[2 + 16 + i * 2 + 1]);
    }

    // Byte 2 = command = 0x05 = 0b00000101 -- LSB-first: 1,0,1,0,0,0,0,0.
    const bool byte2_bits[8] = {true, false, true, false, false, false, false, false};
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + 32 + i * 2]);
        TEST_ASSERT_EQUAL_UINT16(byte2_bits[i] ? 1690 : 560, durations[2 + 32 + i * 2 + 1]);
    }

    // Byte 3 = ~command = 0xFA -- derived complement of 0x05.
    const bool byte3_bits[8] = {false, true, false, true, true, true, true, true};
    for (int i = 0; i < 8; i++) {
        TEST_ASSERT_EQUAL_UINT16(560, durations[2 + 48 + i * 2]);
        TEST_ASSERT_EQUAL_UINT16(byte3_bits[i] ? 1690 : 560, durations[2 + 48 + i * 2 + 1]);
    }

    TEST_ASSERT_EQUAL_UINT16(560, durations[66]);
}

// ── Refusals: unrecognized protocol, raw signal type, undersized buffer --
// this project's established "refuse rather than lie" convention (see
// ir_nec_encode.h's header).

void test_encode_rejects_unrecognized_protocol(void) {
    const uint8_t address[4] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t command[4] = {0x00, 0x00, 0x00, 0x00};
    IrFileFormat::IrSignal sig;
    // RC5 is a real protocol name the .ir format documents (see the plan's
    // Task 18 section) but this module deliberately does not implement it.
    make_signal(&sig, "RC5", address, 4, command, 4);

    uint16_t durations[IrNecEncode::kDurationsPerFrame];
    size_t count = 0;
    uint32_t carrier = 0;
    TEST_ASSERT_FALSE(IrNecEncode::encode(sig, durations, IrNecEncode::kDurationsPerFrame, &count, &carrier));
}

void test_encode_rejects_raw_signal_type(void) {
    IrFileFormat::IrSignal sig{};
    sig.type = IrFileFormat::SignalType::kRaw; // NEC-family encoding never
                                                // applies to a raw capture,
                                                // even if `protocol` were
                                                // somehow set
    std::strncpy(sig.protocol, "NEC", IrFileFormat::kProtocolMaxLen - 1);

    uint16_t durations[IrNecEncode::kDurationsPerFrame];
    size_t count = 0;
    uint32_t carrier = 0;
    TEST_ASSERT_FALSE(IrNecEncode::encode(sig, durations, IrNecEncode::kDurationsPerFrame, &count, &carrier));
}

void test_encode_rejects_undersized_buffer(void) {
    const uint8_t address[4] = {0xC5, 0x1F, 0x00, 0x00};
    const uint8_t command[4] = {0x0D, 0xF2, 0x00, 0x00};
    IrFileFormat::IrSignal sig;
    make_signal(&sig, "NECext", address, 4, command, 4);

    uint16_t durations[IrNecEncode::kDurationsPerFrame - 1]; // one short of a real frame
    size_t count = 0;
    uint32_t carrier = 0;
    TEST_ASSERT_FALSE(IrNecEncode::encode(sig, durations, IrNecEncode::kDurationsPerFrame - 1, &count, &carrier));
}

// ── Integration: decode a real, verbatim .ir file's text (Hobot_2S.ir),
// then encode the named signal it produces -- proves IrFileFormat::decode()
// -> IrNecEncode::encode() works end to end on real file content, not just
// a hand-built IrSignal.
void test_decode_then_encode_real_hobot2s_file(void) {
    static const char kHobot2sIr[] =
        "Filetype: IR signals file\n"
        "Version: 1\n"
        "#\n"
        "# by Angel 0ff Death\n"
        "# \n"
        "# HOBOT-2S \n"
        "# \n"
        "name: SPRAY_TOGGLE\n"
        "type: parsed\n"
        "protocol: NECext\n"
        "address: C5 1F 00 00\n"
        "command: 0D F2 00 00\n"
        "# \n"
        "name: UP\n"
        "type: parsed\n"
        "protocol: NECext\n"
        "address: C5 1F 00 00\n"
        "command: 05 FA 00 00\n";

    IrFileFormat::IrSignal signals[4];
    bool truncated = false;
    size_t n = IrFileFormat::decode(kHobot2sIr, std::strlen(kHobot2sIr), signals, 4, &truncated);
    TEST_ASSERT_EQUAL_size_t(2, n);
    TEST_ASSERT_FALSE(truncated);
    TEST_ASSERT_EQUAL_STRING("SPRAY_TOGGLE", signals[0].name);

    uint16_t durations[IrNecEncode::kDurationsPerFrame];
    size_t count = 0;
    uint32_t carrier = 0;
    TEST_ASSERT_TRUE(IrNecEncode::encode(signals[0], durations, IrNecEncode::kDurationsPerFrame, &count, &carrier));
    TEST_ASSERT_EQUAL_size_t(67, count);
    TEST_ASSERT_EQUAL_UINT32(38000, carrier);
    TEST_ASSERT_EQUAL_UINT16(9000, durations[0]);
    TEST_ASSERT_EQUAL_UINT16(4500, durations[1]);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_encode_necext_hobot2s_spray_toggle);
    RUN_TEST(test_encode_nec_hobot_up);
    RUN_TEST(test_encode_rejects_unrecognized_protocol);
    RUN_TEST(test_encode_rejects_raw_signal_type);
    RUN_TEST(test_encode_rejects_undersized_buffer);
    RUN_TEST(test_decode_then_encode_real_hobot2s_file);
    return UNITY_END();
}
