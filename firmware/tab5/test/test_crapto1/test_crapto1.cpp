#include <unity.h>

extern "C" {
#include <crapto1.h>
}

#include <cstdint>
#include <cstdlib>

// ===========================================================================
// Host-native tests for the vendored crapto1 library (lib/crapto1/) --
// Phase 3 Task 9 (MIFARE Classic key recovery). Runs via
// `pio test -e native` from firmware/tab5/, the same [env:native] target
// Task 7 established.
//
// WHAT IS AND IS NOT PROVEN HERE -- read this before trusting a green bar.
//
// No published known-answer test vector for CRYPTO1 / lfsr_recovery32 was
// found in reach (crapto1 ships no test suite of its own, and the canonical
// mfcuk/mfoc redistributions carry none either). Rather than invent one --
// a self-constructed "expected" value proves nothing about correctness, only
// that the code still does what it did yesterday -- these tests assert
// STRUCTURAL properties that are independently checkable:
//
//   1. prng_successor()'s 16-bit LFSR has period 65535. This is a property
//      of the MIFARE Classic tag PRNG itself (a 16-bit LFSR with taps
//      x^16 + x^14 + x^13 + x^11 + 1, whose state cycles through all 2^16-1
//      nonzero states), not a property invented for this test.
//   2. lfsr_rollback_word() is the exact inverse of crypto1_word(). These
//      are two independently written functions living in two different
//      files (crypto1.c forward, crapto1.c backward); a round trip through
//      both is a real cross-check, not a tautology.
//   3. lfsr_recovery32() inverts the forward cipher: given only the 32-bit
//      keystream word crypto1_word() emits while a known key's cipher is fed
//      (uid ^ nt), it recovers a candidate list containing that key. This is
//      the ONE property the whole attack rests on, and again it is a
//      cross-check between two separately implemented halves of the library
//      rather than a hardcoded magic number.
//
// What these do NOT prove: that the vendored code matches upstream bit for
// bit (that was established by direct source comparison instead -- see
// lib/crapto1/src/crapto1.h's provenance header), and that any attack in
// features/nfc/nfc_mifare_crack.cpp works against a real card (that needs
// real hardware; see this task's report).
//
// Test 3 is also what settled a REAL BUG in the donor attack code -- see
// test_recovery32_matches_first_keystream_word_not_second() below.
// ===========================================================================

namespace {

// Arbitrary but fixed inputs. They are inputs, not expected outputs: nothing
// below asserts a precomputed keystream value, so these carry no claim of
// being a "known vector".
constexpr uint64_t kKey = 0xA0A1A2A3A4A5ULL; // a real MIFARE default key
                                             // (COMMON_KEYS[1]); any 48-bit
                                             // value would do
constexpr uint32_t kUid = 0xDEADBEEFU;
constexpr uint32_t kNt  = 0x01200145U;

// Walks the candidate-state list lfsr_recovery32() returns exactly the way
// the ported attacks do (rollback by the same `in` word, then read the LFSR
// out as a 48-bit key) and reports whether `expected` appears.
bool recovered_list_contains(uint32_t ks, uint32_t in, uint64_t expected,
                             int *out_candidates) {
    struct Crypto1State *rev = lfsr_recovery32(ks, in);
    if (rev == nullptr) {
        if (out_candidates != nullptr) *out_candidates = -1;
        return false;
    }
    bool found = false;
    int n = 0;
    for (struct Crypto1State *rs = rev; rs->odd != 0 || rs->even != 0; rs++) {
        lfsr_rollback_word(rs, in, 0);
        uint64_t cand = 0;
        crypto1_get_lfsr(rs, &cand);
        if (cand == expected) found = true;
        if (++n > 2000000) break; // same shape as the ported attacks' own
                                  // 100k safety limit; generous here
    }
    if (out_candidates != nullptr) *out_candidates = n;
    std::free(rev);
    return found;
}

} // namespace

// --- 1. PRNG ---------------------------------------------------------------

void test_prng_successor_identity_and_period(void) {
    // n == 0 must be the identity.
    TEST_ASSERT_EQUAL_HEX32(kNt, prng_successor(kNt, 0));

    // PERIOD. The tag PRNG is a 16-bit LFSR whose nonzero state space has
    // period 65535, so 65535 successor steps must return to the start.
    //
    // The 32-bit word prng_successor() operates on is NOT 32 bits of LFSR
    // state, which matters for how this is asserted. Reading the shift in
    // crypto1.c (`x = x >> 1 | (x >> 16 ^ x >> 18 ^ x >> 19 ^ x >> 21) << 31`,
    // between two SWAPENDIANs) shows only bits 16/18/19/21 feed back: the
    // upper half is the real 16-bit LFSR and the lower half is a 16-step
    // delay line of it. So an ARBITRARY 32-bit word is generally not a state
    // the PRNG could actually be in, and asserting a 65535-step round trip on
    // one is asserting something untrue -- exactly what an earlier draft of
    // this test did, and it correctly failed on 0x00000001.
    //
    // Normalising a raw word with 16 steps first refills that delay line from
    // the LFSR itself, producing a genuinely reachable state; the period
    // property then holds on the full 32-bit word.
    const uint32_t raw_seeds[] = {kNt, 0x00000001U, 0x12345678U, 0xCAFEBABEU};
    for (unsigned i = 0; i < sizeof(raw_seeds) / sizeof(raw_seeds[0]); i++) {
        const uint32_t state = prng_successor(raw_seeds[i], 16);
        // 0 is the LFSR's single degenerate fixed point (all-zero state never
        // leaves itself), so a seed normalising to it would pass vacuously.
        TEST_ASSERT_NOT_EQUAL_UINT32(0U, state);
        TEST_ASSERT_EQUAL_HEX32(state, prng_successor(state, 65535));
        // ...and 65534 steps must NOT, or "period 65535" would be a weaker
        // claim than it looks (any divisor of 65535 would also pass above).
        TEST_ASSERT_NOT_EQUAL_UINT32(state, prng_successor(state, 65534));
    }

    // Stepping is additive: successor(successor(x, a), b) == successor(x, a+b).
    // This is what both nested attacks rely on when they walk candidate PRNG
    // distances (`prng_successor(nt1, d)` for d in [0, 65535)).
    TEST_ASSERT_EQUAL_HEX32(prng_successor(kNt, 96),
                            prng_successor(prng_successor(kNt, 32), 64));
}

// --- 2. Forward cipher vs. rollback ----------------------------------------

void test_lfsr_rollback_word_inverts_crypto1_word(void) {
    struct Crypto1State *s = crypto1_create(kKey);
    TEST_ASSERT_NOT_NULL(s);

    uint64_t before = 0;
    crypto1_get_lfsr(s, &before);
    TEST_ASSERT_EQUAL_HEX64(kKey, before); // create() then get_lfsr() must
                                           // round-trip the key itself

    const uint32_t in = kUid ^ kNt;
    (void)crypto1_word(s, in, 0);
    lfsr_rollback_word(s, in, 0);

    uint64_t after = 0;
    crypto1_get_lfsr(s, &after);
    TEST_ASSERT_EQUAL_HEX64(kKey, after);
    crypto1_destroy(s);
}

// --- 3. State recovery ------------------------------------------------------

// THIS TEST EXISTS BECAUSE IT FOUND A REAL BUG IN THE DONOR ATTACK CODE.
//
// UniGeek's NestedAttack.cpp:313-320 and StaticNestedAttack.cpp:392-399 both
// software-cross-check a candidate key against their extra collected nonces
// like this:
//
//     Crypto1State* test = crypto1_create(candidateKey);
//     crypto1_word(test, uid ^ nt, 0);          // <-- return value DISCARDED
//     uint32_t testKs = crypto1_word(test, 0, 0);
//     if ((encNt ^ nt) != testKs) softMatch = false;
//
// i.e. they compare the observed keystream (encNt ^ nt) against the SECOND
// 32-bit word the cipher emits. But those same functions feed the FIRST word
// to lfsr_recovery32() as `ks`, so the two halves cannot both be right. This
// test resolves it from the library itself: recovery succeeds for the first
// word and fails for the second, so the cross-check was comparing against the
// wrong word and would have rejected the CORRECT key whenever more than one
// nonce was collected -- which is exactly what both attacks try to do
// (COLLECT_NR == 3). Fixed in features/nfc/nfc_mifare_crack.cpp; see the
// "DONOR BUG" comment at that call site.
void test_recovery32_matches_first_keystream_word_not_second(void) {
    const uint32_t in = kUid ^ kNt;

    struct Crypto1State *s = crypto1_create(kKey);
    TEST_ASSERT_NOT_NULL(s);
    const uint32_t ks_first = crypto1_word(s, in, 0);
    const uint32_t ks_second = crypto1_word(s, 0, 0);
    crypto1_destroy(s);

    // Sanity: they really are different words, so the assertions below are
    // distinguishing something.
    TEST_ASSERT_NOT_EQUAL_UINT32(ks_first, ks_second);

    int candidates_first = 0;
    const bool found_first = recovered_list_contains(ks_first, in, kKey,
                                                     &candidates_first);
    TEST_ASSERT_NOT_EQUAL_INT(-1, candidates_first); // allocation succeeded
    TEST_ASSERT_TRUE_MESSAGE(found_first,
        "lfsr_recovery32() failed to recover the key from the FIRST keystream "
        "word -- the ported nested/static-nested attacks cannot work");
    TEST_ASSERT_GREATER_THAN_INT(0, candidates_first);

    int candidates_second = 0;
    const bool found_second = recovered_list_contains(ks_second, in, kKey,
                                                      &candidates_second);
    TEST_ASSERT_FALSE_MESSAGE(found_second,
        "the SECOND keystream word also recovered the key -- if this ever "
        "fires, the donor cross-check bug this test documents was not a bug, "
        "and nfc_mifare_crack.cpp's fix needs revisiting");
}

// --- Runner -----------------------------------------------------------------

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_prng_successor_identity_and_period);
    RUN_TEST(test_lfsr_rollback_word_inverts_crypto1_word);
    RUN_TEST(test_recovery32_matches_first_keystream_word_not_second);
    return UNITY_END();
}
