#include <unity.h>
#include "subghz_protocol_decode.h"
#include <cstring>

// ===========================================================================
// Host-native tests for SubghzProto::decode() (Phase 10 Task 2). Runs via
// `pio test -e native` from shared/subghz_proto/.
//
// SubghzProto::decode() operates directly on a raw alternating-duration
// array (unsigned int *dur, uint16_t count), unlike firmware/tab5's
// Rf433ProtocolDecode::decode() (which additionally converts a
// Rf433Scan::CapturedSignal's EdgeSample[] into that duration array via its
// own build_durations() merge step, kept there since it's RF433-specific
// plumbing -- see rf433_protocol_decode.cpp's updated header comment). So
// these tests build duration arrays directly rather than EdgeSample arrays.
//
// The CAME positive-match test reuses the EXACT same real cited constants
// (te_short=320, te_long=640, te_delta=150, sync multiple 56, end-of-frame
// multiple 4) as firmware/tab5/test/test_rf433_protocol_decode/
// test_rf433_protocol_decode.cpp's own test_decode_accepts_synthesized_
// came_signal -- that test's EdgeSample sequence never repeats a level
// (build_durations()'s merge step is therefore a no-op on it), so the
// derived duration array is identical to what's constructed here directly.
// ===========================================================================

// ── Boundary / defensive-behavior tests ────────────────────────────────────

void test_decode_rejects_null_out() {
    unsigned int dur[8] = {320, 640, 320, 640, 320, 640, 320, 640};
    TEST_ASSERT_FALSE(SubghzProto::decode(dur, 8, nullptr));
}

void test_decode_rejects_too_few_durations() {
    // count < 8 -- SubGhzDecoders::decode()'s own guard (SubGhzDecoders.cpp:1866).
    unsigned int dur[5] = {320, 640, 320, 640, 320};
    SubghzProto::Match out{};
    TEST_ASSERT_FALSE(SubghzProto::decode(dur, 5, &out));
}

void test_decode_rejects_empty() {
    SubghzProto::Match out{};
    TEST_ASSERT_FALSE(SubghzProto::decode(nullptr, 0, &out));
}

void test_decode_rejects_uniform_pulse_train() {
    // Enough entries to clear the count >= 8 guard, but a flat/uniform pulse
    // width matches no ported brand's alternating short/long signature or
    // preamble-multiple sync window.
    unsigned int dur[40];
    for (int i = 0; i < 40; i++) dur[i] = 500;
    SubghzProto::Match out{};
    std::memset(&out, 0xAA, sizeof(out));
    SubghzProto::Match sentinel;
    std::memset(&sentinel, 0xAA, sizeof(sentinel));
    TEST_ASSERT_FALSE(SubghzProto::decode(dur, 40, &out));
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &out, sizeof(out)); // *out left untouched on no match
}

// ── Positive test: CAME (one of the six extracted decoders) ───────────────
// Same real construction as test_rf433_protocol_decode.cpp's
// test_decode_accepts_synthesized_came_signal (see this file's header
// comment) -- 12-bit code 0xFFF, encoding a real CAME "1" bit twelve times:
// decode_came()'s CheckDur branch (SaveDur=te_long(640) followed by
// CheckDur=te_short(320) => data = data<<1 | 1), preceded by the real sync
// gap (56 * te_short) and start pulse (te_short), followed by the real
// end-of-frame gap (te_short * 4).
void test_decode_accepts_synthesized_came_signal() {
    unsigned int dur[2 + 12 * 2 + 1];
    size_t idx = 0;
    dur[idx++] = 320u * 56u; // sync gap
    dur[idx++] = 320u;       // start pulse
    for (int bit = 0; bit < 12; bit++) {
        dur[idx++] = 640u; // SaveDur LOW pulse == te_long
        dur[idx++] = 320u; // CheckDur HIGH pulse == te_short -> bit '1'
    }
    dur[idx++] = 320u * 4u; // end-of-frame gap

    SubghzProto::Match out{};
    TEST_ASSERT_TRUE(SubghzProto::decode(dur, static_cast<uint16_t>(idx), &out));
    TEST_ASSERT_EQUAL_STRING("CAME", out.name);
    TEST_ASSERT_TRUE(out.key == 0xFFFULL);
    TEST_ASSERT_EQUAL_UINT8(12, out.bits);
    TEST_ASSERT_EQUAL_UINT16(320, out.te);
}

// ── Positive test: Princeton (the one freshly-ported decoder) ─────────────
// Constructed directly from decode_princeton()'s own real, cited timing
// constants (te_short=390, te_long=1170, te_delta=300 --
// ~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.cpp:78-126, ported
// verbatim in subghz_protocol_decode.cpp's decode_princeton()) -- nothing
// invented, matching this project's "real sources only" discipline.
//
// Princeton requires a DOUBLE FRAME (last_data == data, Flipper's own
// anti-noise guard, same double-frame shape as decode_holtek_ht12x's
// last_data check) -- so this builds TWO consecutive identical 24-bit
// frames, all-1s (bit=1 per decode_princeton(): te_last==te_long(1170) &&
// duration==te_short(390) => data = data<<1|1). Each bit is one HIGH pulse
// (te_long) then one LOW pulse (te_short); after 24 bits, one more HIGH
// pulse (any value -- unused by the end-of-frame branch) then a LOW pulse
// >= te_long*2 (2340us) ends the frame (SubGhzDecoders.cpp's own
// `duration >= te_long * 2` check). The real sync gap (te_short*36) only
// precedes the FIRST frame -- the state machine returns to SaveDur (not
// Reset) after an end-of-frame gap, so the second frame's bits follow
// directly with no second sync gap needed, matching the donor's own state
// transitions exactly.
void test_decode_accepts_synthesized_princeton_signal() {
    constexpr uint32_t te_short = 390, te_long = 1170;
    unsigned int dur[1 + (24 * 2 + 2) * 2];
    size_t idx = 0;

    dur[idx++] = te_short * 36u; // sync gap (Reset -> SaveDur)

    for (int frame = 0; frame < 2; frame++) {
        for (int bit = 0; bit < 24; bit++) {
            dur[idx++] = te_long;  // SaveDur HIGH pulse == te_long
            dur[idx++] = te_short; // CheckDur LOW pulse == te_short -> bit '1'
        }
        dur[idx++] = te_short;       // extra HIGH pulse (value irrelevant to the end-of-frame branch)
        dur[idx++] = te_long * 2u + 60u; // end-of-frame LOW gap (>= te_long * 2)
    }

    SubghzProto::Match out{};
    TEST_ASSERT_TRUE(SubghzProto::decode(dur, static_cast<uint16_t>(idx), &out));
    TEST_ASSERT_EQUAL_STRING("Princeton", out.name);
    TEST_ASSERT_TRUE(out.key == 0xFFFFFFULL); // 24 bits, all 1s
    TEST_ASSERT_EQUAL_UINT8(24, out.bits);
    TEST_ASSERT_EQUAL_UINT16(390, out.te);
}

// A single Princeton frame (no repeat) must NOT match -- the double-frame
// guard is a real anti-noise requirement, not an incidental side effect of
// how the test above happens to be built.
void test_decode_rejects_single_princeton_frame() {
    constexpr uint32_t te_short = 390, te_long = 1170;
    unsigned int dur[1 + 24 * 2 + 2];
    size_t idx = 0;
    dur[idx++] = te_short * 36u;
    for (int bit = 0; bit < 24; bit++) {
        dur[idx++] = te_long;
        dur[idx++] = te_short;
    }
    dur[idx++] = te_short;
    dur[idx++] = te_long * 2u + 60u;

    SubghzProto::Match out{};
    std::memset(&out, 0xAA, sizeof(out));
    SubghzProto::Match sentinel;
    std::memset(&sentinel, 0xAA, sizeof(sentinel));
    TEST_ASSERT_FALSE(SubghzProto::decode(dur, static_cast<uint16_t>(idx), &out));
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &out, sizeof(out));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_rejects_null_out);
    RUN_TEST(test_decode_rejects_too_few_durations);
    RUN_TEST(test_decode_rejects_empty);
    RUN_TEST(test_decode_rejects_uniform_pulse_train);
    RUN_TEST(test_decode_accepts_synthesized_came_signal);
    RUN_TEST(test_decode_accepts_synthesized_princeton_signal);
    RUN_TEST(test_decode_rejects_single_princeton_frame);
    return UNITY_END();
}
