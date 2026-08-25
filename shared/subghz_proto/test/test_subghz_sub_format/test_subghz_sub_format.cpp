#include <unity.h>
#include "subghz_sub_format.h"
#include "subghz_protocol_decode.h"
#include <cstring>
#include <cstdio>

// ===========================================================================
// Host-native tests for SubghzProto::encode_sub()/decode_sub() (Phase 10
// Task 2). Runs via `pio test -e native` from shared/subghz_proto/.
//
// Reuses the same real fixtures firmware/tab5/test/test_rf433_sub_format/
// test_rf433_sub_format.cpp already established for Rf433SubFormat
// (Task 21) -- the simple 7-edge hand-built signal and the real 354-edge
// on-device RF433 capture (burst #17, 2026-08-20) -- adapted to
// SubghzProto::EdgeSample (same two fields, same semantics, see
// subghz_sub_format.h's own header comment for why this is a separate
// type) and to this module's generalized (parameterized freq/preset,
// caller-supplied edges_out buffer) signature, rather than inventing new
// fixtures from scratch.
// ===========================================================================

// A real CC1101 test frequency within this phase's module range
// (855-925MHz per the spec) -- not RF433's fixed 433.92MHz, to actually
// exercise the "variable frequency" generalization this module exists for.
constexpr uint32_t kTestFreqHz = 868350000u;
constexpr char kTestPreset[] = "FuriHalSubGhzPresetOok650Async";

// ── Simple hand-built signal (no chatter, starts HIGH) ─────────────────────
// Same 7 edges as test_rf433_sub_format.cpp's build_simple_signal().
static void build_simple_signal(SubghzProto::EdgeSample *edges, size_t *count) {
    const SubghzProto::EdgeSample src[] = {
        {0u, true},    {320u, false},  {960u, true},  {1280u, false},
        {1920u, true}, {2240u, false}, {3520u, true},
    };
    for (size_t i = 0; i < 7; i++) edges[i] = src[i];
    *count = 7;
}

void test_encode_rejects_too_few_edges() {
    SubghzProto::EdgeSample edges[1] = {{0u, true}};
    char buf[256];
    size_t len = 0;
    TEST_ASSERT_FALSE(SubghzProto::encode_sub(kTestFreqHz, kTestPreset, edges, 1, buf, sizeof(buf), &len));
}

void test_encode_rejects_null_preset() {
    SubghzProto::EdgeSample edges[7];
    size_t count = 0;
    build_simple_signal(edges, &count);
    char buf[256];
    size_t len = 0;
    TEST_ASSERT_FALSE(SubghzProto::encode_sub(kTestFreqHz, nullptr, edges, count, buf, sizeof(buf), &len));
}

void test_encode_produces_expected_header_and_raw_data() {
    SubghzProto::EdgeSample edges[7];
    size_t count = 0;
    build_simple_signal(edges, &count);
    char buf[512];
    size_t len = 0;
    TEST_ASSERT_TRUE(SubghzProto::encode_sub(kTestFreqHz, kTestPreset, edges, count, buf, sizeof(buf), &len));

    // Real spec's own field order/literal text (see rf433_sub_format.h /
    // subghz_sub_format.h), generalized frequency/preset written verbatim.
    const char *expected =
        "Filetype: Flipper SubGhz RAW File\n"
        "Version: 1\n"
        "Frequency: 868350000\n"
        "Preset: FuriHalSubGhzPresetOok650Async\n"
        "Protocol: RAW\n"
        "RAW_Data: 320 -640 320 -640 320 -1280\n";
    TEST_ASSERT_EQUAL_STRING(expected, buf);
    TEST_ASSERT_EQUAL_UINT32(std::strlen(expected), len);
}

void test_encode_decode_round_trip_simple_signal() {
    SubghzProto::EdgeSample edges[7];
    size_t count = 0;
    build_simple_signal(edges, &count);
    char buf[512];
    size_t len = 0;
    TEST_ASSERT_TRUE(SubghzProto::encode_sub(kTestFreqHz, kTestPreset, edges, count, buf, sizeof(buf), &len));

    uint32_t freq_out = 0;
    SubghzProto::EdgeSample result[16]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_TRUE(
        SubghzProto::decode_sub(buf, len, &freq_out, result, 16, &edge_count_out));

    TEST_ASSERT_EQUAL_UINT32(kTestFreqHz, freq_out);
    TEST_ASSERT_EQUAL_UINT32(count, edge_count_out);
    for (size_t i = 0; i < count; i++) {
        TEST_ASSERT_EQUAL_UINT32(edges[i].timestamp_us, result[i].timestamp_us);
        TEST_ASSERT_EQUAL(edges[i].level, result[i].level);
    }
}

void test_decode_rejects_wrong_filetype() {
    const char *text = "Filetype: Something Else\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 100 -100\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[8]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_FALSE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 8, &edge_count_out));
}

void test_decode_rejects_wrong_version() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 2\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 100 -100\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[8]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_FALSE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 8, &edge_count_out));
}

void test_decode_rejects_non_raw_protocol() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Princeton\nKey: 12345\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[8]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_FALSE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 8, &edge_count_out));
}

void test_decode_rejects_zero_valued_duration() {
    // Real spec: "Values must be non-zero."
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 100 0 -100\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[8]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_FALSE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 8, &edge_count_out));
}

void test_decode_rejects_out_of_int32_range_duration() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n"
                        "RAW_Data: 100 -4294967296 100\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[8]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_FALSE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 8, &edge_count_out));
}

// Real spec's own example RAW_Data line (see rf433_sub_format.h's SOURCE
// comment / test_rf433_sub_format.cpp's own test for the self-contradiction
// note: "29262 361" are both positive back-to-back, contradicting the same
// doc's "interleaved" wording) -- decode_sub() does not enforce pairwise
// sign alternation on read, same real leniency Rf433SubFormat::decode()
// already established.
void test_decode_accepts_real_spec_example_fragment() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n"
                        "RAW_Data: 29262 361 -68 2635 -66 24113 -66 11\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[16]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_TRUE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 16, &edge_count_out));
    TEST_ASSERT_EQUAL_UINT32(868350000u, freq_out);
    TEST_ASSERT_EQUAL_UINT32(9, edge_count_out); // 8 values -> 9 edges
}

void test_decode_parses_multiple_raw_data_lines() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n"
                        "RAW_Data: 320 -640 320\nRAW_Data: -640 320 -1280\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[16]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_TRUE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 16, &edge_count_out));
    TEST_ASSERT_EQUAL_UINT32(7, edge_count_out); // 6 total values across both lines -> 7 edges
}

// Real spec's own "RAW file, custom preset" shape -- Preset:
// FuriHalSubGhzPresetCustom plus Custom_preset_module:/Custom_preset_data:
// lines inserted between Preset: and Protocol:. Same real fragment as
// test_decode_accepts_real_spec_example_fragment above.
void test_decode_accepts_real_custom_preset_header() {
    const char *text =
        "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
        "Preset: FuriHalSubGhzPresetCustom\n"
        "Custom_preset_module: CC1101\n"
        "Custom_preset_data: 02 0D 03 07 08 32 0B 06 14 00 13 00 12 30 11 32 10 17 18 18 19 18 1D 91 "
        "1C 00 1B 07 20 FB 22 11 21 B6 00 00 00 C0 00 00 00 00 00 00\n"
        "Protocol: RAW\n"
        "RAW_Data: 29262 361 -68 2635 -66 24113 -66 11\n";
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[16]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_TRUE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 16, &edge_count_out));
    TEST_ASSERT_EQUAL_UINT32(868350000u, freq_out);
    TEST_ASSERT_EQUAL_UINT32(9, edge_count_out);
}

// New behavior specific to this module's generalized, caller-supplied-buffer
// signature (rf433_sub_format.cpp has no equivalent -- it owns a fixed-size
// CapturedSignal instead): when the real file would produce more edges than
// edges_capacity, the excess is silently dropped and *edge_count_out is
// capped at edges_capacity -- documented in subghz_sub_format.h's own
// decode_sub() comment.
void test_decode_caps_output_at_edges_capacity() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 868350000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n"
                        "RAW_Data: 320 -640 320 -640 320 -1280\n"; // 6 values -> 7 edges if uncapped
    uint32_t freq_out = 0;
    SubghzProto::EdgeSample out[3]{};
    size_t edge_count_out = 0;
    TEST_ASSERT_TRUE(SubghzProto::decode_sub(text, std::strlen(text), &freq_out, out, 3, &edge_count_out));
    TEST_ASSERT_EQUAL_UINT32(3, edge_count_out);
}

// ── Real-hardware fixture round trip ───────────────────────────────────────
// The SAME real capture as test_rf433_protocol_decode.cpp's / Task 21's
// test_rf433_sub_format.cpp's kFixtureEdges (burst #17, 354 edges, captured
// 2026-08-20 from the on-device RF433 Scan screen). Reused here verbatim
// (adapted to SubghzProto::EdgeSample's own type), per this task's own
// instruction to reuse real fixtures rather than invent new ones.
constexpr size_t kFixtureEdgeCount = 354;
constexpr SubghzProto::EdgeSample kFixtureEdges[kFixtureEdgeCount] = {
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

// Same fixed-point property test_rf433_sub_format.cpp's own
// test_encode_decode_real_fixture_is_a_fixed_point establishes for
// Rf433SubFormat: encode a real capture, decode the result, then re-encode
// that -- the two encoded texts must be byte-identical, AND the edge count
// must match the same real, independently-computed expected value (210
// edges: 354 raw edges merge down to 209 signed durations after folding
// same-level chatter runs and dropping the leading LOW pulse).
void test_encode_decode_real_fixture_is_a_fixed_point() {
    static char text1[8192];
    static char text2[8192];
    size_t len1 = 0, len2 = 0;
    TEST_ASSERT_TRUE(SubghzProto::encode_sub(kTestFreqHz, kTestPreset, kFixtureEdges, kFixtureEdgeCount,
                                              text1, sizeof(text1), &len1));

    uint32_t freq_out = 0;
    static SubghzProto::EdgeSample roundtripped[512];
    size_t edge_count_out = 0;
    TEST_ASSERT_TRUE(SubghzProto::decode_sub(text1, len1, &freq_out, roundtripped, 512, &edge_count_out));
    TEST_ASSERT_EQUAL_UINT32(kTestFreqHz, freq_out);
    TEST_ASSERT_EQUAL_UINT32(210, edge_count_out);

    TEST_ASSERT_TRUE(SubghzProto::encode_sub(kTestFreqHz, kTestPreset, roundtripped, edge_count_out,
                                              text2, sizeof(text2), &len2));

    TEST_ASSERT_EQUAL_UINT32(len1, len2);
    TEST_ASSERT_EQUAL_MEMORY(text1, text2, len1);
}

// ── RAW is a first-class path (project owner, 2026-08-25) ─────────────────
// encode_sub() must work directly off raw edges with no dependency on a
// successful SubghzProto::decode() protocol match -- confirmed directly:
// the real 354-edge fixture above is the SAME fixture
// test_rf433_protocol_decode.cpp's test_decode_real_capture_fixture proves
// decode()s to false (matches none of the 8 ported brands) -- yet it
// encodes to a full, valid, non-empty Protocol: RAW .sub file here, with no
// protocol-decode call anywhere in this path.
void test_encode_sub_succeeds_on_undecodable_raw_capture() {
    char buf[8192];
    size_t len = 0;
    TEST_ASSERT_TRUE(
        SubghzProto::encode_sub(kTestFreqHz, kTestPreset, kFixtureEdges, kFixtureEdgeCount, buf, sizeof(buf), &len));
    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_TRUE(std::strstr(buf, "Protocol: RAW") != nullptr);
}

// ── Protocol-keyed Princeton .sub support (2026-08-25) ─────────────────────
// Real file that surfaced this gap, verbatim (project owner's own report --
// a genuine Flipper SubGhz-DB entry, "LED/Aurora_RGB"):
//
//   Filetype: Flipper SubGhz Key File
//   Version: 1
//   Frequency: 433920000
//   Preset: FuriHalSubGhzPresetOok650Async
//   Protocol: Princeton
//   Bit: 24
//   Key: 00 00 00 00 00 BE AF 03
//   TE: 412
//
// Strongest available verification with no physical hardware in the loop:
// decode this exact real file, then feed the resulting edges into this
// project's OWN SubghzProto::decode() (Princeton branch) and confirm it
// comes back out as a Princeton match with the SAME key (0xBEAF03) --
// end-to-end round-trip through both directions of this codebase's real
// Princeton logic, not just "decode_sub() returned true".
void test_decode_sub_accepts_real_princeton_keyed_file() {
    const char *text =
        "Filetype: Flipper SubGhz Key File\n"
        "Version: 1\n"
        "Frequency: 433920000\n"
        "Preset: FuriHalSubGhzPresetOok650Async\n"
        "Protocol: Princeton\n"
        "Bit: 24\n"
        "Key: 00 00 00 00 00 BE AF 03\n"
        "TE: 412\n";
    size_t len = std::strlen(text);

    static SubghzProto::EdgeSample edges[512];
    uint32_t freq_out = 0;
    size_t edge_count = 0;
    TEST_ASSERT_TRUE(SubghzProto::decode_sub(text, len, &freq_out, edges, 512, &edge_count));
    TEST_ASSERT_EQUAL_UINT32(433920000u, freq_out);
    TEST_ASSERT_TRUE(edge_count > 2);

    // edges[] -> duration[] (gap between consecutive edges), same
    // conversion every real caller in this codebase performs before calling
    // SubghzProto::decode().
    static unsigned int durations[512];
    for (size_t i = 1; i < edge_count; i++) {
        durations[i - 1] = static_cast<unsigned int>(edges[i].timestamp_us - edges[i - 1].timestamp_us);
    }

    SubghzProto::Match m{};
    TEST_ASSERT_TRUE(SubghzProto::decode(durations, static_cast<uint16_t>(edge_count - 1), &m));
    TEST_ASSERT_EQUAL_STRING("Princeton", m.name);
    TEST_ASSERT_EQUAL_UINT32(0xBEAF03u, static_cast<uint32_t>(m.key));
    TEST_ASSERT_EQUAL_UINT8(24, m.bits);
}

void test_decode_sub_rejects_unsupported_keyed_protocol() {
    const char *text =
        "Filetype: Flipper SubGhz Key File\n"
        "Version: 1\n"
        "Frequency: 433920000\n"
        "Preset: FuriHalSubGhzPresetOok650Async\n"
        "Protocol: KeeLoq\n"
        "Bit: 64\n"
        "Key: 00 00 00 00 00 BE AF 03\n"
        "TE: 400\n";
    size_t len = std::strlen(text);
    static SubghzProto::EdgeSample edges[512];
    uint32_t freq_out = 0;
    size_t edge_count = 0;
    TEST_ASSERT_FALSE(SubghzProto::decode_sub(text, len, &freq_out, edges, 512, &edge_count));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_rejects_too_few_edges);
    RUN_TEST(test_encode_rejects_null_preset);
    RUN_TEST(test_encode_produces_expected_header_and_raw_data);
    RUN_TEST(test_encode_decode_round_trip_simple_signal);
    RUN_TEST(test_decode_rejects_wrong_filetype);
    RUN_TEST(test_decode_rejects_wrong_version);
    RUN_TEST(test_decode_rejects_non_raw_protocol);
    RUN_TEST(test_decode_rejects_zero_valued_duration);
    RUN_TEST(test_decode_rejects_out_of_int32_range_duration);
    RUN_TEST(test_decode_accepts_real_spec_example_fragment);
    RUN_TEST(test_decode_parses_multiple_raw_data_lines);
    RUN_TEST(test_decode_accepts_real_custom_preset_header);
    RUN_TEST(test_decode_caps_output_at_edges_capacity);
    RUN_TEST(test_encode_decode_real_fixture_is_a_fixed_point);
    RUN_TEST(test_encode_sub_succeeds_on_undecodable_raw_capture);
    RUN_TEST(test_decode_sub_accepts_real_princeton_keyed_file);
    RUN_TEST(test_decode_sub_rejects_unsupported_keyed_protocol);
    return UNITY_END();
}
