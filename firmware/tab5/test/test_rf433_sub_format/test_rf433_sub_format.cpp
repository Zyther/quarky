#include <unity.h>
#include "features/rf433/rf433_common.h"
#include "features/rf433/rf433_scan.h"
#include "features/rf433/rf433_sub_format.h"
#include "features/ir/ir_file_format.h"
#include "hal/istorage.h"
#include <cstring>
#include <cstdio>
#include <cmath>

// ===========================================================================
// Host-native tests for Rf433SubFormat (.sub) and IrFileFormat (.ir) --
// Phase 3 Task 21 (SD file format interop). Runs via `pio test -e native`
// from firmware/tab5/, same [env:native] target Task 7 established (see
// platformio.ini, extended this task to also build these two modules'
// .cpp files). Neither module includes Arduino.h; both depend only on
// IStorage's pure-virtual interface (hal/istorage.h has no Arduino
// dependency itself) plus rf433_scan.h/rf433_common.h's plain structs, the
// same host-testability shape Task 7 established for
// rf433_protocol_decode.cpp.
// ===========================================================================

// ── Fake IStorage: single in-memory "file", for write()/read() round trips ─
// A real StorageSD can't be exercised host-natively (it depends on
// SD_MMC/Arduino). This stands in for it -- both format modules take
// IStorage& by dependency injection specifically so this is possible; see
// rf433_sub_format.h's "SD-backed convenience wrappers" comment.
class FakeStorage : public IStorage {
public:
    bool mount() override { return true; }
    bool write_test_file() override { return true; }
    bool write_capture_file(const char *path, const uint8_t *data, size_t len) override {
        if (len > sizeof(buf_)) return false;
        std::memcpy(buf_, data, len);
        len_ = len;
        std::strncpy(path_, path, sizeof(path_) - 1);
        path_[sizeof(path_) - 1] = '\0';
        return true;
    }
    bool append_capture_file(const char *, const uint8_t *, size_t) override {
        return false; // unused by these tests
    }
    bool read_file(const char *path, uint8_t *out, size_t max_len, size_t *out_len) override {
        if (std::strcmp(path, path_) != 0) return false;
        size_t n = (len_ < max_len) ? len_ : max_len;
        std::memcpy(out, buf_, n);
        if (out_len != nullptr) *out_len = n;
        return true;
    }
    int list_files(const char *, const char *, char[][64], int) override { return 0; }

private:
    char path_[128] = {};
    uint8_t buf_[16384] = {};
    size_t len_ = 0;
};

// ── Rf433SubFormat: hand-built strictly-alternating signal ────────────────
// Deliberately starts HIGH (edges[0].level == true) and never repeats a
// level on consecutive edges, so no build_signed_durations() merge or
// leading-LOW-drop applies -- the simplest case where write() then read()
// should reproduce the ORIGINAL edges exactly (timestamps included, since
// edges[0].timestamp_us is already 0). See rf433_sub_format.h's "SOURCE"
// comment section for why more complex real-hardware capture cases (chatter,
// non-HIGH starts) are covered separately below via the real fixture.
static void build_simple_signal(Rf433Scan::CapturedSignal *sig) {
    *sig = Rf433Scan::CapturedSignal{};
    Rf433Common::EdgeSample edges[] = {
        {0u, true},    {320u, false},  {960u, true},  {1280u, false},
        {1920u, true}, {2240u, false}, {3520u, true},
    };
    for (size_t i = 0; i < 7; i++) sig->edges[i] = edges[i];
    sig->edge_count = 7;
    sig->captured_at_ms = 12345;
    sig->capture_id = 99;
    sig->truncated = false;
}

void test_encode_rejects_too_few_edges() {
    Rf433Scan::CapturedSignal sig{};
    sig.edge_count = 1;
    char buf[256];
    size_t len = 0;
    TEST_ASSERT_FALSE(Rf433SubFormat::encode(sig, buf, sizeof(buf), &len));
}

void test_encode_produces_expected_header_and_raw_data() {
    Rf433Scan::CapturedSignal sig;
    build_simple_signal(&sig);
    char buf[512];
    size_t len = 0;
    TEST_ASSERT_TRUE(Rf433SubFormat::encode(sig, buf, sizeof(buf), &len));
    // Real spec's own field order/literal text (see rf433_sub_format.h).
    const char *expected =
        "Filetype: Flipper SubGhz RAW File\n"
        "Version: 1\n"
        "Frequency: 433920000\n"
        "Preset: FuriHalSubGhzPresetOok650Async\n"
        "Protocol: RAW\n"
        "RAW_Data: 320 -640 320 -640 320 -1280\n";
    TEST_ASSERT_EQUAL_STRING(expected, buf);
    TEST_ASSERT_EQUAL_UINT32(std::strlen(expected), len);
}

void test_write_read_round_trip_simple_signal() {
    Rf433Scan::CapturedSignal sig;
    build_simple_signal(&sig);

    FakeStorage storage;
    TEST_ASSERT_TRUE(Rf433SubFormat::write(storage, "/quarky/captures/rf433/sig.sub", sig));

    Rf433Scan::CapturedSignal result{};
    TEST_ASSERT_TRUE(Rf433SubFormat::read(storage, "/quarky/captures/rf433/sig.sub", &result));

    TEST_ASSERT_EQUAL_UINT32(sig.edge_count, result.edge_count);
    for (size_t i = 0; i < sig.edge_count; i++) {
        TEST_ASSERT_EQUAL_UINT32(sig.edges[i].timestamp_us, result.edges[i].timestamp_us);
        TEST_ASSERT_EQUAL(sig.edges[i].level, result.edges[i].level);
    }
    TEST_ASSERT_FALSE(result.truncated);
    // Documented lossy behavior (rf433_sub_format.h): the .sub format
    // carries no capture timestamp/session-id, so these are NOT
    // round-tripped -- confirmed here rather than just asserted in a
    // comment, using a source signal that deliberately set them non-zero.
    TEST_ASSERT_EQUAL_UINT32(0, result.captured_at_ms);
    TEST_ASSERT_EQUAL_UINT32(0, result.capture_id);
}

void test_decode_rejects_wrong_filetype() {
    const char *text = "Filetype: Something Else\nVersion: 1\nFrequency: 433920000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 100 -100\n";
    Rf433Scan::CapturedSignal out{};
    TEST_ASSERT_FALSE(Rf433SubFormat::decode(text, std::strlen(text), &out));
}

void test_decode_rejects_wrong_version() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 2\nFrequency: 433920000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 100 -100\n";
    Rf433Scan::CapturedSignal out{};
    TEST_ASSERT_FALSE(Rf433SubFormat::decode(text, std::strlen(text), &out));
}

void test_decode_rejects_non_raw_protocol() {
    // Real spec's own protocol-keyed (non-RAW) .sub files -- explicitly out
    // of scope, see rf433_sub_format.h's SOURCE comment.
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Princeton\nKey: 12345\n";
    Rf433Scan::CapturedSignal out{};
    TEST_ASSERT_FALSE(Rf433SubFormat::decode(text, std::strlen(text), &out));
}

void test_decode_rejects_zero_valued_duration() {
    // Real spec: "Values must be non-zero."
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\nRAW_Data: 100 0 -100\n";
    Rf433Scan::CapturedSignal out{};
    TEST_ASSERT_FALSE(Rf433SubFormat::decode(text, std::strlen(text), &out));
}

// Real spec's own example RAW_Data line, quoted verbatim in
// task-21-controller-notes.md: "RAW_Data: 29262 361 -68 2635 -66 24113 -66
// 11 ...". NOTE, a real discrepancy worth flagging: this literal example's
// first two values (29262, 361) are BOTH positive -- back-to-back same-sign
// -- which contradicts the same source's own "interleaved... two same-signed
// values in a row is not valid" wording, quoted immediately above it. Rather
// than guess which half of the real documentation is authoritative, decode()
// deliberately does not enforce pairwise sign alternation on READ (only
// non-zero-ness, per the one rule this file's own real example does not
// contradict) -- each token's sign is consumed independently to derive that
// edge's level, exactly as the WRITE side already guarantees alternation by
// construction (build_signed_durations() can only ever emit an alternating
// sequence). This test exists to prove that leniency against the spec's own
// example, not to claim the example is a semantically complete capture.
void test_decode_accepts_real_spec_example_fragment() {
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n"
                        "RAW_Data: 29262 361 -68 2635 -66 24113 -66 11\n";
    Rf433Scan::CapturedSignal out{};
    TEST_ASSERT_TRUE(Rf433SubFormat::decode(text, std::strlen(text), &out));
    TEST_ASSERT_EQUAL_UINT32(9, out.edge_count); // 8 values -> 9 edges
    TEST_ASSERT_FALSE(out.truncated);
}

void test_decode_parses_multiple_raw_data_lines() {
    // Real spec's own continuation convention for captures over 512 values
    // per line -- exercised here with two short lines rather than an actual
    // 512-value line, same "prove the mechanism, not the exact threshold"
    // approach the rest of this project's tests use for boundary behavior.
    const char *text = "Filetype: Flipper SubGhz RAW File\nVersion: 1\nFrequency: 433920000\n"
                        "Preset: FuriHalSubGhzPresetOok650Async\nProtocol: RAW\n"
                        "RAW_Data: 320 -640 320\nRAW_Data: -640 320 -1280\n";
    Rf433Scan::CapturedSignal out{};
    TEST_ASSERT_TRUE(Rf433SubFormat::decode(text, std::strlen(text), &out));
    TEST_ASSERT_EQUAL_UINT32(7, out.edge_count); // 6 total values across both lines -> 7 edges
}

// ── Real-hardware fixture round trip ───────────────────────────────────────
// The SAME real capture as test_rf433_protocol_decode.cpp's kFixtureEdges
// (burst #17, 354 edges, captured 2026-08-20 from the on-device RF433 Scan
// screen -- see that file's own header comment for the full extraction
// provenance). Reused here, per this task's own plan text's explicitly
// acceptable fallback ("a fresh real capture from this session's own
// hardware testing, re-encoded to the real Flipper format, is an acceptable
// fixture if no external sample is available" -- no donor checkout has a
// real .sub sample file at all, see rf433_sub_format.h's SOURCE comment).
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

// Fixed-point property: encode a real capture, decode the result, then
// re-encode that -- the two encoded texts must be byte-identical. This
// proves the full round trip is stable for real, messy hardware data
// (including this fixture's own documented same-level chatter runs -- see
// test_rf433_protocol_decode.cpp's kFixtureEdges comment) without requiring
// a hand-computed expected edge sequence for all 354 real edges: encode()'s
// build_signed_durations() merge+leading-LOW-drop logic and decode()'s
// edge-reconstruction logic are each other's real inverse if and only if
// this holds.
void test_encode_decode_real_fixture_is_a_fixed_point() {
    Rf433Scan::CapturedSignal sig{};
    for (size_t i = 0; i < kFixtureEdgeCount; i++) sig.edges[i] = kFixtureEdges[i];
    sig.edge_count = kFixtureEdgeCount;
    sig.captured_at_ms = 0;
    sig.capture_id = 17;
    sig.truncated = false;

    static char text1[8192];
    static char text2[8192];
    size_t len1 = 0, len2 = 0;
    TEST_ASSERT_TRUE(Rf433SubFormat::encode(sig, text1, sizeof(text1), &len1));

    Rf433Scan::CapturedSignal roundtripped{};
    TEST_ASSERT_TRUE(Rf433SubFormat::decode(text1, len1, &roundtripped));

    TEST_ASSERT_TRUE(Rf433SubFormat::encode(roundtripped, text2, sizeof(text2), &len2));

    TEST_ASSERT_EQUAL_UINT32(len1, len2);
    TEST_ASSERT_EQUAL_MEMORY(text1, text2, len1);
}

void test_write_read_real_fixture_via_storage() {
    Rf433Scan::CapturedSignal sig{};
    for (size_t i = 0; i < kFixtureEdgeCount; i++) sig.edges[i] = kFixtureEdges[i];
    sig.edge_count = kFixtureEdgeCount;
    sig.captured_at_ms = 0;
    sig.capture_id = 17;
    sig.truncated = false;

    FakeStorage storage;
    TEST_ASSERT_TRUE(Rf433SubFormat::write(storage, "/quarky/captures/rf433/burst17.sub", sig));

    Rf433Scan::CapturedSignal result{};
    TEST_ASSERT_TRUE(Rf433SubFormat::read(storage, "/quarky/captures/rf433/burst17.sub", &result));
    TEST_ASSERT_TRUE(result.edge_count > 0);
    TEST_ASSERT_FALSE(result.truncated);
}

// ===========================================================================
// IrFileFormat (.ir) tests
// ===========================================================================

// Real spec sample text, quoted verbatim in task-21-controller-notes.md
// (from InfraredFileFormats.md) -- both signal shapes it documents.
static const char kRealSampleIrFile[] =
    "Filetype: IR signals file\n"
    "Version: 1\n"
    "#\n"
    "name: Button_1\n"
    "type: parsed\n"
    "protocol: NECext\n"
    "address: EE 87 00 00\n"
    "command: 5D A0 00 00\n"
    "#\n"
    "name: Button_2\n"
    "type: raw\n"
    "frequency: 38000\n"
    "duty_cycle: 0.330000\n"
    "data: 504 3432 502 483 500 484 510 502 502 482 501 485 509 1452 504 1458 509\n";

void test_ir_decode_real_sample_file() {
    IrFileFormat::IrSignal signals[4]{};
    bool truncated = false;
    size_t n = IrFileFormat::decode(kRealSampleIrFile, std::strlen(kRealSampleIrFile), signals, 4, &truncated);

    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_FALSE(truncated);

    TEST_ASSERT_EQUAL_STRING("Button_1", signals[0].name);
    TEST_ASSERT_TRUE(IrFileFormat::SignalType::kParsed == signals[0].type);
    TEST_ASSERT_EQUAL_STRING("NECext", signals[0].protocol);
    TEST_ASSERT_EQUAL_UINT32(4, signals[0].address_len);
    const uint8_t expected_addr[] = {0xEE, 0x87, 0x00, 0x00};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_addr, signals[0].address, 4);
    TEST_ASSERT_EQUAL_UINT32(4, signals[0].command_len);
    const uint8_t expected_cmd[] = {0x5D, 0xA0, 0x00, 0x00};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_cmd, signals[0].command, 4);

    TEST_ASSERT_EQUAL_STRING("Button_2", signals[1].name);
    TEST_ASSERT_TRUE(IrFileFormat::SignalType::kRaw == signals[1].type);
    TEST_ASSERT_EQUAL_UINT32(38000, signals[1].frequency_hz);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.33f, signals[1].duty_cycle);
    const uint16_t expected_data[] = {504, 3432, 502, 483, 500, 484, 510, 502,
                                       502, 482,  501, 485, 509, 1452, 504, 1458, 509};
    TEST_ASSERT_EQUAL_UINT32(17, signals[1].data_count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(expected_data, signals[1].data, 17);
    TEST_ASSERT_FALSE(signals[1].truncated);
}

void test_ir_decode_rejects_wrong_filetype() {
    const char *text = "Filetype: Something Else\nVersion: 1\n";
    IrFileFormat::IrSignal signals[2]{};
    bool truncated = false;
    TEST_ASSERT_EQUAL_UINT32(0, IrFileFormat::decode(text, std::strlen(text), signals, 2, &truncated));
}

void test_ir_decode_rejects_wrong_version() {
    const char *text = "Filetype: IR signals file\nVersion: 2\n";
    IrFileFormat::IrSignal signals[2]{};
    bool truncated = false;
    TEST_ASSERT_EQUAL_UINT32(0, IrFileFormat::decode(text, std::strlen(text), signals, 2, &truncated));
}

void test_ir_decode_truncates_when_max_signals_exceeded() {
    IrFileFormat::IrSignal signals[1]{};
    bool truncated = false;
    size_t n = IrFileFormat::decode(kRealSampleIrFile, std::strlen(kRealSampleIrFile), signals, 1, &truncated);
    TEST_ASSERT_EQUAL_UINT32(1, n);
    TEST_ASSERT_TRUE(truncated);
    TEST_ASSERT_EQUAL_STRING("Button_1", signals[0].name); // first signal, not dropped
}

void test_ir_decode_truncates_oversized_data_field() {
    // Real spec's own stated max for `data` is 1024 elements. Built
    // programmatically (1030 real, valid, non-fabricated tokens -- just
    // repeated -- rather than hand-typing over a thousand literals).
    static char buf[16384];
    int pos = std::snprintf(buf, sizeof(buf),
                             "Filetype: IR signals file\nVersion: 1\n#\nname: BigRaw\ntype: raw\n"
                             "frequency: 38000\nduty_cycle: 0.330000\ndata:");
    for (int i = 0; i < 1030; i++) {
        pos += std::snprintf(buf + pos, sizeof(buf) - pos, " 100");
    }
    pos += std::snprintf(buf + pos, sizeof(buf) - pos, "\n");

    IrFileFormat::IrSignal signals[2]{};
    bool truncated = false;
    size_t n = IrFileFormat::decode(buf, static_cast<size_t>(pos), signals, 2, &truncated);
    TEST_ASSERT_EQUAL_UINT32(1, n);
    TEST_ASSERT_TRUE(truncated);
    TEST_ASSERT_EQUAL_UINT32(IrFileFormat::kMaxRawSamples, signals[0].data_count);
    TEST_ASSERT_TRUE(signals[0].truncated);
}

void test_ir_encode_decode_round_trip() {
    IrFileFormat::IrSignal signals[2]{};
    std::strncpy(signals[0].name, "Button_1", sizeof(signals[0].name) - 1);
    signals[0].type = IrFileFormat::SignalType::kParsed;
    std::strncpy(signals[0].protocol, "NECext", sizeof(signals[0].protocol) - 1);
    const uint8_t addr[] = {0xEE, 0x87, 0x00, 0x00};
    const uint8_t cmd[] = {0x5D, 0xA0, 0x00, 0x00};
    std::memcpy(signals[0].address, addr, 4);
    signals[0].address_len = 4;
    std::memcpy(signals[0].command, cmd, 4);
    signals[0].command_len = 4;

    std::strncpy(signals[1].name, "Button_2", sizeof(signals[1].name) - 1);
    signals[1].type = IrFileFormat::SignalType::kRaw;
    signals[1].frequency_hz = 38000;
    signals[1].duty_cycle = 0.33f;
    const uint16_t data[] = {504, 3432, 502, 483, 500, 484, 510, 502,
                              502, 482,  501, 485, 509, 1452, 504, 1458, 509};
    std::memcpy(signals[1].data, data, sizeof(data));
    signals[1].data_count = 17;

    char buf[1024];
    size_t len = 0;
    TEST_ASSERT_TRUE(IrFileFormat::encode(signals, 2, buf, sizeof(buf), &len));

    // encode()'s output uses the exact real field syntax/formatting (e.g.
    // "%02X" hex bytes, "%f"'s default 6-decimal duty_cycle) that happens to
    // reproduce the real sample file byte-for-byte for these exact values --
    // asserted directly since it's a strong, checkable claim, not just an
    // aspiration.
    TEST_ASSERT_EQUAL_STRING(kRealSampleIrFile, buf);

    IrFileFormat::IrSignal decoded[2]{};
    bool truncated = false;
    size_t n = IrFileFormat::decode(buf, len, decoded, 2, &truncated);
    TEST_ASSERT_EQUAL_UINT32(2, n);
    TEST_ASSERT_FALSE(truncated);
    TEST_ASSERT_EQUAL_STRING(signals[0].name, decoded[0].name);
    TEST_ASSERT_EQUAL_STRING(signals[1].name, decoded[1].name);
    TEST_ASSERT_EQUAL_UINT32(signals[1].data_count, decoded[1].data_count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(signals[1].data, decoded[1].data, signals[1].data_count);
}

void test_ir_write_read_round_trip_via_storage() {
    IrFileFormat::IrSignal signals[1]{};
    std::strncpy(signals[0].name, "Learned", sizeof(signals[0].name) - 1);
    signals[0].type = IrFileFormat::SignalType::kRaw;
    signals[0].frequency_hz = 38000;
    signals[0].duty_cycle = 0.33f;
    const uint16_t data[] = {900, 450, 900, 450, 1800};
    std::memcpy(signals[0].data, data, sizeof(data));
    signals[0].data_count = 5;

    FakeStorage storage;
    TEST_ASSERT_TRUE(IrFileFormat::write(storage, "/quarky/ir/learned.ir", signals, 1));

    IrFileFormat::IrSignal result[1]{};
    bool truncated = false;
    size_t n = IrFileFormat::read(storage, "/quarky/ir/learned.ir", result, 1, &truncated);
    TEST_ASSERT_EQUAL_UINT32(1, n);
    TEST_ASSERT_FALSE(truncated);
    TEST_ASSERT_EQUAL_STRING("Learned", result[0].name);
    TEST_ASSERT_EQUAL_UINT32(5, result[0].data_count);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(data, result[0].data, 5);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_rejects_too_few_edges);
    RUN_TEST(test_encode_produces_expected_header_and_raw_data);
    RUN_TEST(test_write_read_round_trip_simple_signal);
    RUN_TEST(test_decode_rejects_wrong_filetype);
    RUN_TEST(test_decode_rejects_wrong_version);
    RUN_TEST(test_decode_rejects_non_raw_protocol);
    RUN_TEST(test_decode_rejects_zero_valued_duration);
    RUN_TEST(test_decode_accepts_real_spec_example_fragment);
    RUN_TEST(test_decode_parses_multiple_raw_data_lines);
    RUN_TEST(test_encode_decode_real_fixture_is_a_fixed_point);
    RUN_TEST(test_write_read_real_fixture_via_storage);
    RUN_TEST(test_ir_decode_real_sample_file);
    RUN_TEST(test_ir_decode_rejects_wrong_filetype);
    RUN_TEST(test_ir_decode_rejects_wrong_version);
    RUN_TEST(test_ir_decode_truncates_when_max_signals_exceeded);
    RUN_TEST(test_ir_decode_truncates_oversized_data_field);
    RUN_TEST(test_ir_encode_decode_round_trip);
    RUN_TEST(test_ir_write_read_round_trip_via_storage);
    return UNITY_END();
}
