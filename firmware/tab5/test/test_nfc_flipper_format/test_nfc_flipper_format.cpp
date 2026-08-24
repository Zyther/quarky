#include <unity.h>
#include "features/nfc/nfc_flipper_format.h"
#include "hal/istorage.h"
#include <cstring>

// ===========================================================================
// Host-native tests for NfcFlipperFormat -- the Flipper Zero ".nfc" file
// exporter/importer for the ISO14443-4A device type (Phase 3 Task 13's
// save/export extension). Runs via `pio test -e native`, the same target
// Tasks 7/10/21 established.
//
// The sample text in test_decode_real_sample_file() is hand-constructed to
// the real format read directly out of Flipper's own dev-branch source on
// 2026-08-23 (nfc_device.c, iso14443_3a.c, iso14443_4a.c and
// flipper_format_stream.c -- every key string, its order, the '# ' comment
// syntax, the "%02X "-separated hex and the MSB-order ATQA quirk are cited in
// nfc_flipper_format.h's header comment). It is NOT a guess at a
// plausible-looking format: each line below corresponds to one specific
// write call in that real save path.
// ===========================================================================

// ── Fake IStorage, adapted from test_nfc_tag_library.cpp's ────────────────
class FakeStorage : public IStorage {
public:
    bool mount() override { return true; }
    bool write_test_file() override { return true; }

    bool write_capture_file(const char *path, const uint8_t *data, size_t len) override {
        if (len > sizeof(buf_)) return false;
        std::strncpy(path_, path, sizeof(path_) - 1);
        path_[sizeof(path_) - 1] = '\0';
        std::memcpy(buf_, data, len);
        len_ = len;
        used_ = true;
        return true;
    }
    bool append_capture_file(const char *, const uint8_t *, size_t) override { return false; }

    bool read_file(const char *path, uint8_t *out, size_t max_len, size_t *out_len) override {
        if (!used_ || std::strcmp(path, path_) != 0) return false;
        size_t n = (len_ < max_len) ? len_ : max_len;
        std::memcpy(out, buf_, n);
        if (out_len != nullptr) *out_len = n;
        return true;
    }
    int list_files(const char *, const char *, char[][64], int, bool * = nullptr) override {
        return 0;
    }
    int list_dirs(const char *, char[][64], int, bool * = nullptr) override { return 0; }

    const char *path() const { return path_; }
    const char *text() const { return reinterpret_cast<const char *>(buf_); }
    size_t len() const { return len_; }

private:
    bool used_ = false;
    char path_[128] = {};
    uint8_t buf_[2048] = {};
    size_t len_ = 0;
};

// A real ISO14443-4A card's identity: 7-byte UID, ATQA 0x0344 (wire/LSB order
// 44 03), SAK 0x20 (the "compliant with ISO/IEC 14443-4" SAK), and a 5-byte
// ATS TL=05 T0=78 (TA(1)+TB(1)+TC(1) all present) TA=80 TB=82 TC=02 -- the
// shape a real contactless payment card presents.
static NfcFlipperFormat::Iso14443_4aRecord sample_record() {
    NfcFlipperFormat::Iso14443_4aRecord r{};
    const uint8_t uid[7] = {0x04, 0xA2, 0xB3, 0xC4, 0xD5, 0xE6, 0xF7};
    std::memcpy(r.uid, uid, sizeof(uid));
    r.uid_len = 7;
    r.atqa[0] = 0x44; // wire order, LSB first
    r.atqa[1] = 0x03;
    r.sak = 0x20;
    const uint8_t ats[5] = {0x05, 0x78, 0x80, 0x82, 0x02};
    std::memcpy(r.ats, ats, sizeof(ats));
    r.ats_len = 5;
    return r;
}

void test_encode_matches_real_format_line_for_line() {
    char buf[NfcFlipperFormat::kMaxEncodedTextBytes];
    size_t len = 0;
    TEST_ASSERT_TRUE(NfcFlipperFormat::encode(sample_record(), buf, sizeof(buf), &len));
    TEST_ASSERT_EQUAL_UINT(std::strlen(buf), len);

    TEST_ASSERT_NOT_NULL(std::strstr(buf, "Filetype: Flipper NFC device\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "Version: 4\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "Device type: ISO14443-4A\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "UID: 04 A2 B3 C4 D5 E6 F7\n"));
    // ATQA is written MSB-first -- iso14443_3a_save()'s own documented quirk
    // ("Save ATQA in MSB order for correct companion apps display"). Wire
    // order 44 03 must therefore appear as "03 44".
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "ATQA: 03 44\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "SAK: 20\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "T0: 78\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "TA(1): 80\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "TB(1): 82\n"));
    TEST_ASSERT_NOT_NULL(std::strstr(buf, "TC(1): 02\n"));
    // The file must start with the header, not merely contain it.
    TEST_ASSERT_EQUAL_INT(0, std::strncmp(buf, "Filetype: Flipper NFC device\n", 29));
}

void test_encode_decode_round_trip() {
    const NfcFlipperFormat::Iso14443_4aRecord in = sample_record();
    char buf[NfcFlipperFormat::kMaxEncodedTextBytes];
    size_t len = 0;
    TEST_ASSERT_TRUE(NfcFlipperFormat::encode(in, buf, sizeof(buf), &len));

    NfcFlipperFormat::Iso14443_4aRecord out{};
    TEST_ASSERT_TRUE(NfcFlipperFormat::decode(buf, len, &out));

    TEST_ASSERT_EQUAL_UINT8(in.uid_len, out.uid_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in.uid, out.uid, in.uid_len);
    TEST_ASSERT_EQUAL_UINT8(in.atqa[0], out.atqa[0]);
    TEST_ASSERT_EQUAL_UINT8(in.atqa[1], out.atqa[1]);
    TEST_ASSERT_EQUAL_UINT8(in.sak, out.sak);
    TEST_ASSERT_EQUAL_UINT8(in.ats_len, out.ats_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in.ats, out.ats, in.ats_len);
}

// A real-shaped file written by hand rather than by encode(), so decode() is
// tested against the format itself and not just against this module's own
// output. Includes the historical-bytes key and comment lines.
void test_decode_real_sample_file() {
    static const char kSample[] =
        "Filetype: Flipper NFC device\n"
        "Version: 4\n"
        "# Device type can be ISO14443-3A, ISO14443-3B, ISO14443-4A, ISO14443-4B,"
        " ISO15693-3, FeliCa, NTAG/Ultralight, Mifare Classic, Mifare Plus,"
        " Mifare DESFire, SLIX, ST25TB\n"
        "Device type: ISO14443-4A\n"
        "# UID is common for all formats\n"
        "UID: 04 11 22 33\n"
        "# ISO14443-3A specific data\n"
        "ATQA: 00 04\n"
        "SAK: 28\n"
        "# ISO14443-4A specific data\n"
        "T0: 75\n"
        "TA(1): 77\n"
        "TB(1): 81\n"
        "TC(1): 02\n"
        "T1...Tk: 80 31 80 66 B1 84 0C 01 6E 01 83 00 90 00\n";

    NfcFlipperFormat::Iso14443_4aRecord r{};
    TEST_ASSERT_TRUE(NfcFlipperFormat::decode(kSample, std::strlen(kSample), &r));

    TEST_ASSERT_EQUAL_UINT8(4, r.uid_len);
    const uint8_t want_uid[4] = {0x04, 0x11, 0x22, 0x33};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want_uid, r.uid, 4);
    // File stores MSB-first "00 04" -> wire order 04 00.
    TEST_ASSERT_EQUAL_UINT8(0x04, r.atqa[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, r.atqa[1]);
    TEST_ASSERT_EQUAL_UINT8(0x28, r.sak);

    // ATS rebuilt TL-first: TL(=19) T0 TA TB TC then 14 historical bytes.
    TEST_ASSERT_EQUAL_UINT8(19, r.ats_len);
    TEST_ASSERT_EQUAL_UINT8(19, r.ats[0]);
    TEST_ASSERT_EQUAL_UINT8(0x75, r.ats[1]);
    TEST_ASSERT_EQUAL_UINT8(0x77, r.ats[2]);
    TEST_ASSERT_EQUAL_UINT8(0x81, r.ats[3]);
    TEST_ASSERT_EQUAL_UINT8(0x02, r.ats[4]);
    TEST_ASSERT_EQUAL_UINT8(0x80, r.ats[5]);
    TEST_ASSERT_EQUAL_UINT8(0x00, r.ats[18]);
}

// tl <= 1 (or no ATS at all) -- Flipper's own save() omits every ISO14443-4A
// key in that case, and this must still round-trip.
void test_record_without_ats_round_trips() {
    NfcFlipperFormat::Iso14443_4aRecord in{};
    const uint8_t uid[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    std::memcpy(in.uid, uid, 4);
    in.uid_len = 4;
    in.atqa[0] = 0x04;
    in.atqa[1] = 0x00;
    in.sak = 0x20;
    in.ats_len = 0;

    char buf[NfcFlipperFormat::kMaxEncodedTextBytes];
    size_t len = 0;
    TEST_ASSERT_TRUE(NfcFlipperFormat::encode(in, buf, sizeof(buf), &len));
    TEST_ASSERT_NULL(std::strstr(buf, "T0:"));

    NfcFlipperFormat::Iso14443_4aRecord out{};
    TEST_ASSERT_TRUE(NfcFlipperFormat::decode(buf, len, &out));
    TEST_ASSERT_EQUAL_UINT8(0, out.ats_len);
    TEST_ASSERT_EQUAL_UINT8(0x20, out.sak);
}

void test_decode_rejects_wrong_filetype_and_device_type() {
    static const char kWrongFiletype[] =
        "Filetype: Flipper SubGhz RAW File\n"
        "Version: 4\n"
        "Device type: ISO14443-4A\n"
        "UID: 04 11 22 33\n"
        "ATQA: 00 04\n"
        "SAK: 28\n";
    NfcFlipperFormat::Iso14443_4aRecord r{};
    TEST_ASSERT_FALSE(
        NfcFlipperFormat::decode(kWrongFiletype, std::strlen(kWrongFiletype), &r));

    // A real Mifare Classic .nfc file is a different device type carrying data
    // this module cannot represent -- rejected rather than half-parsed.
    static const char kMfc[] =
        "Filetype: Flipper NFC device\n"
        "Version: 4\n"
        "Device type: Mifare Classic\n"
        "UID: 04 11 22 33\n"
        "ATQA: 00 04\n"
        "SAK: 08\n";
    TEST_ASSERT_FALSE(NfcFlipperFormat::decode(kMfc, std::strlen(kMfc), &r));
}

void test_encode_rejects_bad_uid_length_and_small_buffer() {
    NfcFlipperFormat::Iso14443_4aRecord r = sample_record();
    r.uid_len = 5; // not 4/7/10
    char buf[NfcFlipperFormat::kMaxEncodedTextBytes];
    size_t len = 0;
    TEST_ASSERT_FALSE(NfcFlipperFormat::encode(r, buf, sizeof(buf), &len));

    r = sample_record();
    char tiny[16];
    TEST_ASSERT_FALSE(NfcFlipperFormat::encode(r, tiny, sizeof(tiny), &len));
}

void test_write_then_read_via_storage() {
    FakeStorage storage;
    const NfcFlipperFormat::Iso14443_4aRecord in = sample_record();

    char path[96];
    TEST_ASSERT_TRUE(NfcFlipperFormat::build_path(in, path, sizeof(path)));
    TEST_ASSERT_EQUAL_STRING("/quarky/captures/nfc/04A2B3C4D5E6F7.nfc", path);

    TEST_ASSERT_TRUE(NfcFlipperFormat::write(storage, path, in));
    TEST_ASSERT_EQUAL_STRING(path, storage.path());

    NfcFlipperFormat::Iso14443_4aRecord out{};
    TEST_ASSERT_TRUE(NfcFlipperFormat::read(storage, path, &out));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in.uid, out.uid, in.uid_len);
    TEST_ASSERT_EQUAL_UINT8(in.sak, out.sak);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(in.ats, out.ats, in.ats_len);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_matches_real_format_line_for_line);
    RUN_TEST(test_encode_decode_round_trip);
    RUN_TEST(test_decode_real_sample_file);
    RUN_TEST(test_record_without_ats_round_trips);
    RUN_TEST(test_decode_rejects_wrong_filetype_and_device_type);
    RUN_TEST(test_encode_rejects_bad_uid_length_and_small_buffer);
    RUN_TEST(test_write_then_read_via_storage);
    return UNITY_END();
}
