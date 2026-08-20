#include <unity.h>
#include "features/rf433/rf433_common.h"
#include "features/rf433/rf433_protocol_decode.h"
#include "features/rf433/rf433_scan.h"
#include <cstring>

// ===========================================================================
// Host-native tests for Rf433ProtocolDecode::decode() (Phase 3 Task 7).
// Runs via `pio test -e native` from firmware/tab5/ -- see platformio.ini's
// [env:native] (added this task, patterned on shared/feature_contract's own
// [env:native]) for why this module, and only this module, can build host-
// native at all: it does not include Arduino.h and only depends on
// rf433_scan.h/rf433_common.h's plain struct definitions, not their .cpp
// implementations.
// ===========================================================================

// ── Boundary / defensive-behavior tests ────────────────────────────────────
// Synthetic inputs, but NOT presented as real captures of any brand's
// protocol -- these exercise decode()'s own guard clauses (empty signal, too
// few edges to derive even one duration, too few durations for any decoder
// to self-sync, a uniform pulse train no brand's alternating-width signature
// matches), not brand/timing recognition. See the fixture test below for the
// real-hardware case this file cannot complete without controller input.

void test_decode_rejects_empty_signal() {
    Rf433Scan::CapturedSignal sig{};
    sig.edge_count = 0;
    Rf433ProtocolDecode::DecodedCode out{};
    TEST_ASSERT_FALSE(Rf433ProtocolDecode::decode(sig, &out));
}

void test_decode_rejects_single_edge() {
    // One edge => zero derivable durations (a duration is the gap between
    // TWO consecutive edges -- see rf433_protocol_decode.cpp's
    // build_durations() header comment).
    Rf433Scan::CapturedSignal sig{};
    sig.edges[0] = Rf433Common::EdgeSample{1000, true};
    sig.edge_count = 1;
    Rf433ProtocolDecode::DecodedCode out{};
    TEST_ASSERT_FALSE(Rf433ProtocolDecode::decode(sig, &out));
}

void test_decode_rejects_short_noise() {
    // Fewer than 8 derivable durations -- decode()'s own guard (mirroring
    // SubGhzDecoders::decode()'s count < 8 check, SubGhzDecoders.cpp:1866):
    // not enough edges for ANY decoder to self-sync, regardless of brand.
    Rf433Scan::CapturedSignal sig{};
    uint32_t t = 0;
    for (size_t i = 0; i < 5; i++) {
        sig.edges[i] = Rf433Common::EdgeSample{t, (i % 2) == 0};
        t += 100;
    }
    sig.edge_count = 5;
    Rf433ProtocolDecode::DecodedCode out{};
    TEST_ASSERT_FALSE(Rf433ProtocolDecode::decode(sig, &out));
}

void test_decode_rejects_uniform_pulse_train() {
    // Enough edges to clear the count >= 8 guard, but a flat/uniform pulse
    // width matches no ported brand's alternating short/long signature or
    // preamble-multiple sync window. This asserts "no decoder in the table
    // claims this input", not any particular protocol's real timing.
    Rf433Scan::CapturedSignal sig{};
    uint32_t t = 0;
    for (size_t i = 0; i < 40; i++) {
        sig.edges[i] = Rf433Common::EdgeSample{t, (i % 2) == 0};
        t += 500;
    }
    sig.edge_count = 40;
    Rf433ProtocolDecode::DecodedCode out{};
    TEST_ASSERT_FALSE(Rf433ProtocolDecode::decode(sig, &out));
}

// ── Real-hardware fixture ────────────────────────────────────────────────
// Real capture, burst #17, 354 edges. Captured 2026-08-20 by the controller
// directly from the on-device RF433 Scan screen (RF433R unit on PORT.A, a
// single real remote-control press) during this session's hardware testing,
// extracted via a temporary Serial hex-dump of the CapturedSignal's raw
// EdgeSample bytes (added and reverted in features/rf433/rf433_scan.cpp --
// see the SDD ledger's 2026-08-20 entries for the full extraction record).
// Real data, transcribed mechanically from the real byte dump -- not
// invented, per this project's "real sources only" discipline.
constexpr size_t kFixtureEdgeCount = 354;
constexpr Rf433Common::EdgeSample kFixtureEdges[kFixtureEdgeCount] = {
    {70083986u, false}, {70083991u, false}, {70084005u, false}, {70084022u, false},
    {70084514u, false}, {70084522u, false}, {70084529u, false}, {70084541u, false},
    {70084546u, false}, {70084554u, false}, {70084559u, true}, {70084563u, false},
    {70084572u, true}, {70084577u, true}, {70084581u, false}, {70084586u, true},
    {70084591u, true}, {70084596u, true}, {70084600u, true}, {70084605u, true},
    {70084611u, false}, {70084616u, true}, {70084621u, true}, {70084626u, true},
    {70084630u, true}, {70084635u, true}, {70084640u, true}, {70084647u, true},
    {70084652u, true}, {70084657u, true}, {70084664u, true}, {70084669u, true},
    {70084683u, true}, {70084831u, false}, {70085205u, true}, {70085210u, true},
    {70085643u, false}, {70085998u, true}, {70086441u, false}, {70086799u, true},
    {70087240u, false}, {70087601u, true}, {70088039u, false}, {70088402u, true},
    {70088982u, false}, {70089203u, true}, {70089782u, false}, {70090018u, true},
    {70090594u, false}, {70090820u, true}, {70091394u, false}, {70091622u, true},
    {70092191u, false}, {70093222u, true}, {70093790u, false}, {70094028u, true},
    {70094594u, false}, {70094824u, true}, {70095794u, false}, {70096428u, true},
    {70096847u, false}, {70097230u, true}, {70098060u, false}, {70101686u, false},
    {70101697u, false}, {70101701u, false}, {70101706u, false}, {70101716u, true},
    {70101721u, false}, {70101726u, false}, {70101731u, false}, {70101735u, true},
    {70101740u, true}, {70101747u, true}, {70101751u, true}, {70101756u, true},
    {70101761u, true}, {70101765u, true}, {70101770u, true}, {70101775u, true},
    {70101780u, true}, {70101785u, true}, {70101789u, true}, {70101794u, true},
    {70101799u, true}, {70101804u, true}, {70101810u, true}, {70101815u, true},
    {70101820u, true}, {70101825u, true}, {70101829u, true}, {70101838u, true},
    {70101843u, true}, {70101848u, true}, {70101853u, true}, {70101868u, true},
    {70101883u, true}, {70102064u, false}, {70102857u, true}, {70103662u, false},
    {70104453u, true}, {70105260u, false}, {70106055u, true}, {70107001u, false},
    {70107659u, true}, {70108603u, false}, {70109258u, true}, {70109816u, false},
    {70110069u, true}, {70111018u, false}, {70111271u, true}, {70111815u, false},
    {70112473u, true}, {70113288u, true}, {70113293u, false}, {70113676u, true},
    {70114076u, false}, {70114918u, false}, {70114933u, false}, {70114945u, false},
    {70114949u, false}, {70114959u, false}, {70114968u, false}, {70114982u, false},
    {70118995u, false}, {70119006u, true}, {70119011u, false}, {70119015u, true},
    {70119020u, false}, {70119025u, true}, {70119030u, true}, {70119036u, true},
    {70119045u, true}, {70119051u, true}, {70119062u, true}, {70119083u, true},
    {70119293u, false}, {70119695u, true}, {70120092u, false}, {70120493u, true},
    {70121292u, false}, {70121707u, true}, {70122111u, false}, {70122116u, true},
    {70122121u, false}, {70122125u, false}, {70122130u, false}, {70122141u, false},
    {70122158u, false}, {70122163u, false}, {70122169u, false}, {70122176u, true},
    {70122180u, true}, {70122185u, true}, {70122190u, true}, {70122195u, true},
    {70122199u, true}, {70122204u, true}, {70122210u, true}, {70122215u, true},
    {70122220u, true}, {70122225u, true}, {70122229u, true}, {70122234u, false},
    {70122239u, false}, {70122244u, false}, {70122911u, true}, {70123846u, false},
    {70124109u, true}, {70124645u, false}, {70125309u, true}, {70126248u, false},
    {70126916u, true}, {70127447u, false}, {70127712u, true}, {70128248u, false},
    {70128512u, true}, {70129064u, false}, {70129326u, true}, {70130123u, false},
    {70130527u, true}, {70130923u, false}, {70135648u, false}, {70135677u, false},
    {70135682u, false}, {70135687u, true}, {70135692u, false}, {70135696u, true},
    {70135701u, true}, {70135706u, true}, {70135711u, true}, {70135715u, true},
    {70135720u, true}, {70135725u, true}, {70135730u, true}, {70135734u, true},
    {70135739u, false}, {70136159u, true}, {70136164u, false}, {70136169u, true},
    {70136940u, false}, {70137745u, true}, {70138141u, false}, {70138546u, true},
    {70139481u, false}, {70140148u, true}, {70141084u, false}, {70141747u, true},
    {70142284u, false}, {70142549u, true}, {70143484u, false}, {70144150u, true},
    {70145086u, false}, {70145751u, true}, {70146569u, true}, {70146574u, false},
    {70147368u, true}, {70148160u, false}, {70152684u, false}, {70152692u, false},
    {70152697u, false}, {70152701u, true}, {70152706u, true}, {70152711u, true},
    {70152716u, true}, {70152721u, true}, {70152726u, true}, {70152737u, true},
    {70152747u, true}, {70152753u, true}, {70152964u, false}, {70153784u, true},
    {70154578u, false}, {70154984u, true}, {70155378u, false}, {70156185u, true},
    {70156719u, false}, {70156985u, true}, {70157920u, false}, {70158188u, true},
    {70158720u, false}, {70158987u, true}, {70159522u, false}, {70160187u, true},
    {70160734u, false}, {70161001u, true}, {70161534u, false}, {70161801u, true},
    {70162738u, false}, {70163002u, true}, {70163399u, false}, {70164204u, true},
    {70164796u, true}, {70164813u, true}, {70164818u, true}, {70164823u, false},
    {70164827u, true}, {70164832u, false}, {70169366u, false}, {70169371u, true},
    {70169375u, false}, {70169380u, true}, {70169385u, true}, {70169389u, true},
    {70169394u, true}, {70169399u, true}, {70169403u, true}, {70169420u, false},
    {70169831u, true}, {70169836u, true}, {70170217u, false}, {70170621u, true},
    {70171415u, false}, {70171822u, true}, {70172215u, false}, {70173022u, true},
    {70173556u, false}, {70173823u, true}, {70174762u, false}, {70175824u, true},
    {70176359u, false}, {70176624u, true}, {70177161u, false}, {70177438u, true},
    {70178379u, false}, {70179039u, true}, {70179574u, false}, {70179840u, true},
    {70180634u, false}, {70186017u, false}, {70186026u, true}, {70186031u, false},
    {70186038u, true}, {70186043u, false}, {70186048u, true}, {70186052u, true},
    {70186057u, true}, {70186062u, true}, {70186070u, true}, {70186075u, true},
    {70186080u, true}, {70186085u, true}, {70186090u, true}, {70186094u, true},
    {70186099u, true}, {70186104u, true}, {70186108u, true}, {70186113u, true},
    {70186119u, true}, {70186134u, true}, {70186252u, false}, {70187062u, true},
    {70187860u, false}, {70188660u, true}, {70189595u, false}, {70190260u, true},
    {70191197u, false}, {70191862u, true}, {70192395u, false}, {70192662u, true},
    {70193598u, false}, {70193863u, true}, {70194398u, false}, {70195065u, true},
    {70196011u, false}, {70196281u, true}, {70196672u, false}, {70197479u, true},
    {70197873u, false}, {70203122u, true}, {70203127u, true}, {70203889u, false},
    {70204297u, true}, {70204690u, false}, {70205497u, true}, {70206433u, false},
    {70206698u, true}, {70207246u, false}, {70207911u, true}, {70208848u, false},
    {70209513u, true}, {70210048u, false}, {70210313u, true}, {70210852u, false},
    {70211114u, true}, {70211648u, false}, {70211915u, true}, {70212852u, false},
    {70213116u, true}, {70213513u, false}, {70214330u, true}, {70214703u, true},
    {70214717u, true}, {70214722u, false}
};

// Ground truth established by actually running this fixture through
// decode() (re-confirmed after the round-2 review's build_durations() merge
// fix and the addition of decode_holtek_ht12x -- both re-checked against
// this exact fixture, not assumed to still hold): it does NOT match any of
// the 7 ported brands (Holtek/Holtek HT12X/CAME/Nice FLO/Chamberlain/
// Ansonic/Linear). Per this file's own header note ("decode() returning
// false is itself real, useful information"), that is exactly what's
// asserted here -- this is real negative information about a real remote,
// not an unimplemented test.
//
// The most likely concrete cause, checked directly rather than left as pure
// speculation: after the merge fix collapses this fixture's 353 raw
// edge-to-edge gaps down to 210 real alternating pulses, the LARGEST
// resulting gap is 5392us (5.392ms). Every ported decoder's Reset state
// requires a gap at or above a brand-specific minimum sync/preamble
// threshold before it will even leave Reset -- CAME's is the widest-net
// among the original six at 8470us minimum, so none of the original six
// ever leave Reset on this data at all. The one exception is the newly
// ported decode_holtek_ht12x, whose window (4960-12960us) is the only one
// of the seven that 5392us falls inside -- its Reset state DOES trigger
// (twice, at merged-duration indices 164 and 188 under phase 1), but the
// state machine never completes a full 12-bit double-frame match (HT12X
// requires the same 12-bit value twice in a row) before running out of
// data. So this fixture reaches further into a decoder's state machine than
// it did before this fix round, without ever producing a false match --
// itself a small piece of evidence the merge fix and the HT12X ordering fix
// are both behaving as intended, not just cosmetic.
//
// The remote's actual protocol remains unidentified: could be one of the
// ~42 still-unported SubGhzDecoders.cpp brands (this project's OOK-only
// front end rules out the 2 FSK-only ones but not the ~40 other OOK
// brands, including Princeton -- see rf433_protocol_decode.h's SCOPE note),
// could be a brand whose real sync gap this single button-press's capture
// window simply didn't include intact, or could be an artifact of receiver
// noise on this particular capture. Whoever next extends the ported decoder
// set should treat this fixture as a real, still-open candidate to
// identify, not assume it's already covered.
void test_decode_real_capture_fixture() {
    Rf433Scan::CapturedSignal sig{};
    for (size_t i = 0; i < kFixtureEdgeCount; i++) {
        sig.edges[i] = kFixtureEdges[i];
    }
    sig.edge_count = kFixtureEdgeCount;
    sig.captured_at_ms = 0;
    sig.capture_id = 17;
    sig.truncated = false;

    // Sentinel-fill *out first so a false return that nonetheless writes
    // through the pointer (violating the header's "on no match, *out is
    // left untouched" contract) would be caught here rather than silently
    // passing just because the return value happened to be right.
    Rf433ProtocolDecode::DecodedCode out{};
    std::memset(&out, 0xAA, sizeof(out));
    Rf433ProtocolDecode::DecodedCode sentinel;
    std::memset(&sentinel, 0xAA, sizeof(sentinel));

    TEST_ASSERT_FALSE(Rf433ProtocolDecode::decode(sig, &out));
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &out, sizeof(out));
}

// ── Positive test: decode() can actually return true ────────────────────
// Every test above this line asserts TEST_ASSERT_FALSE -- decode() could be
// permanently stubbed to `return false` and all of them would still pass.
// This one is constructed, not a hardware capture: synthesized directly
// from decode_came()'s own real, cited timing constants (te_short=320,
// te_long=640, te_delta=150; sync-gap multiple 56; end-of-frame gap
// >= te_short*4 -- all from SubGhzDecoders.cpp:14-73, ported verbatim in
// rf433_protocol_decode.cpp's decode_came()), matching this project's
// existing discipline for distinguishing real captures from
// real-constant-derived synthetic test vectors: every value here traces to
// a real cited source, nothing is invented.
//
// Encodes a real CAME "1" bit twelve times (decode_came()'s CheckDur branch:
// SaveDur=te_long(640) followed by CheckDur=te_short(320) => data = data<<1
// | 1), giving the 12-bit code 0xFFF -- one of decode_came()'s real valid
// bit counts (12/18/24/25/42), which reports plain "CAME" (Prastel is
// 25/42, Airforce is 18). Preceded by the real sync gap (56 * te_short) and
// start pulse (te_short), followed by the real end-of-frame gap
// (te_short * 4) that makes decode_came() see cnt == 12 and return true.
void test_decode_accepts_synthesized_came_signal() {
    Rf433Scan::CapturedSignal sig{};
    size_t idx = 0;
    uint32_t t = 0;

    auto push_edge = [&](uint32_t duration, bool level_after) {
        t += duration;
        sig.edges[idx++] = Rf433Common::EdgeSample{t, level_after};
    };

    sig.edges[idx++] = Rf433Common::EdgeSample{t, false}; // held LOW during the sync gap
    push_edge(320u * 56u, true);                          // sync gap: te_short * 56
    push_edge(320u, false);                                // start pulse: te_short

    for (int bit = 0; bit < 12; bit++) {
        push_edge(640u, true);  // SaveDur LOW pulse == te_long
        push_edge(320u, false); // CheckDur HIGH pulse == te_short -> bit '1'
    }
    push_edge(320u * 4u, true); // end-of-frame gap: te_short * 4

    sig.edge_count = idx;
    sig.captured_at_ms = 0;
    sig.capture_id = 0;
    sig.truncated = false;

    Rf433ProtocolDecode::DecodedCode out{};
    TEST_ASSERT_TRUE(Rf433ProtocolDecode::decode(sig, &out));
    TEST_ASSERT_EQUAL_STRING("CAME", out.protocol_name);
    TEST_ASSERT_TRUE(out.code == 0xFFFULL);
    TEST_ASSERT_EQUAL_UINT8(12, out.bit_length);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_decode_rejects_empty_signal);
    RUN_TEST(test_decode_rejects_single_edge);
    RUN_TEST(test_decode_rejects_short_noise);
    RUN_TEST(test_decode_rejects_uniform_pulse_train);
    RUN_TEST(test_decode_real_capture_fixture);
    RUN_TEST(test_decode_accepts_synthesized_came_signal);
    return UNITY_END();
}
