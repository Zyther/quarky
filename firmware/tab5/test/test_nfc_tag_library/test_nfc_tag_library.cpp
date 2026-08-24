#include <unity.h>
#include "features/nfc/nfc_common.h"
#include "features/nfc/nfc_tag_library.h"
#include "hal/istorage.h"
#include <cstdio>
#include <cstring>

// ===========================================================================
// Host-native tests for NfcTagLibrary -- Phase 3 Task 10 (SD-backed tag
// library save/list/load). Runs via `pio test -e native` from
// firmware/tab5/, same [env:native] target Tasks 7/21 established (see
// platformio.ini, extended this task to also build nfc_tag_library.cpp).
// nfc_tag_library.cpp doesn't include Arduino.h and depends only on
// IStorage's pure-virtual interface (hal/istorage.h has no Arduino
// dependency itself, injected via IStorage& parameters -- see
// nfc_tag_library.h's header comment on why this is DI rather than an
// `extern StorageSD storage;` global) plus nfc_common.h's plain TagInfo
// struct -- the same host-testability shape Task 21 established for
// rf433_sub_format.cpp/ir_file_format.cpp.
//
// register_module()/the browse screen (LVGL + FeatureRegistry) live in the
// separate nfc_tag_library_ui.cpp, which is NOT part of [env:native]'s
// build_src_filter and is not exercised here.
// ===========================================================================

// ── Fake IStorage: a small in-memory "directory", multiple named files ────
// A real StorageSD can't be exercised host-natively (it depends on
// SD_MMC/Arduino). Adapted from test_rf433_sub_format.cpp's single-slot
// FakeStorage (sufficient there for a lone write()-then-read() pair) --
// this task's own list() forwards straight to IStorage::list_files(), so
// this double needs to actually hold several files at once and implement
// list_files()'s real directory+extension filtering, not just stub it to
// return 0.
class FakeStorage : public IStorage {
public:
    bool mount() override { return true; }
    bool write_test_file() override { return true; }

    bool write_capture_file(const char *path, const uint8_t *data, size_t len) override {
        int slot = find_slot(path);
        if (slot < 0) slot = find_free_slot();
        if (slot < 0 || len > sizeof(entries_[0].buf)) return false;
        std::strncpy(entries_[slot].path, path, sizeof(entries_[slot].path) - 1);
        entries_[slot].path[sizeof(entries_[slot].path) - 1] = '\0';
        std::memcpy(entries_[slot].buf, data, len);
        entries_[slot].len = len;
        entries_[slot].used = true;
        return true;
    }

    bool append_capture_file(const char *, const uint8_t *, size_t) override {
        return false; // unused by these tests, same as test_rf433_sub_format.cpp's double
    }

    bool read_file(const char *path, uint8_t *out, size_t max_len, size_t *out_len) override {
        int slot = find_slot(path);
        if (slot < 0) return false;
        size_t n = (entries_[slot].len < max_len) ? entries_[slot].len : max_len;
        std::memcpy(out, entries_[slot].buf, n);
        if (out_len != nullptr) *out_len = n;
        return true;
    }

    int list_files(const char *dir, const char *ext_filter, char names_out[][64], int max_names,
                   bool * = nullptr) override {
        int count = 0;
        size_t dir_len = std::strlen(dir);
        for (int i = 0; i < kMaxEntries && count < max_names; i++) {
            if (!entries_[i].used) continue;
            const char *path = entries_[i].path;
            // Must live directly under `dir` (path == "<dir>/<name>", no
            // further '/' in the remainder) and its name must end with
            // ext_filter -- same two conditions the real StorageSD/
            // wifi_evil_portal.cpp template-picker precedent relies on.
            if (std::strncmp(path, dir, dir_len) != 0 || path[dir_len] != '/') continue;
            const char *name = path + dir_len + 1;
            if (std::strchr(name, '/') != nullptr) continue;
            size_t name_len = std::strlen(name);
            size_t ext_len = std::strlen(ext_filter);
            if (name_len < ext_len || std::strcmp(name + (name_len - ext_len), ext_filter) != 0) continue;
            std::strncpy(names_out[count], name, 63);
            names_out[count][63] = '\0';
            count++;
        }
        return count;
    }

    int list_dirs(const char *, char[][64], int, bool * = nullptr) override { return 0; } // unused by these tests

private:
    struct Entry {
        bool used = false;
        char path[128] = {};
        // Sized FROM the real record rather than hardcoded: TagInfo grew from
        // 35 to 38 bytes (Task 24's sak/atqa) and then to ~963 (2026-08-24's
        // page-content extension), and a fixed 256-byte buffer would have made
        // every save() in this suite silently start returning false. The +64
        // headroom is what test_load_rejects_oversized_corrupt_record() below
        // needs to write a deliberately oversized record.
        uint8_t buf[sizeof(NfcCommon::TagInfo) + 64] = {};
        size_t len = 0;
    };
    static constexpr int kMaxEntries = 16;
    Entry entries_[kMaxEntries];

    int find_slot(const char *path) {
        for (int i = 0; i < kMaxEntries; i++) {
            if (entries_[i].used && std::strcmp(entries_[i].path, path) == 0) return i;
        }
        return -1;
    }
    int find_free_slot() {
        for (int i = 0; i < kMaxEntries; i++) {
            if (!entries_[i].used) return i;
        }
        return -1;
    }
};

static void build_tag(NfcCommon::TagInfo *tag, const uint8_t *uid, uint8_t uid_len,
                       const char *type_name) {
    *tag = NfcCommon::TagInfo{};
    std::memcpy(tag->uid, uid, uid_len);
    tag->uid_len = uid_len;
    std::strncpy(tag->type_name, type_name, sizeof(tag->type_name) - 1);
}

void test_save_then_load_round_trip() {
    FakeStorage storage;
    const uint8_t uid[] = {0x04, 0xA3, 0xF1, 0xD2};
    NfcCommon::TagInfo tag;
    build_tag(&tag, uid, sizeof(uid), "MIFARE Classic 1K");

    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag));

    char names[8][64];
    int count = NfcTagLibrary::list(storage, names, 8);
    TEST_ASSERT_EQUAL_INT(1, count);

    NfcCommon::TagInfo loaded{};
    TEST_ASSERT_TRUE(NfcTagLibrary::load(storage, names[0], &loaded));

    TEST_ASSERT_EQUAL_UINT8(tag.uid_len, loaded.uid_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tag.uid, loaded.uid, tag.uid_len);
    TEST_ASSERT_EQUAL_STRING(tag.type_name, loaded.type_name);
}

void test_load_returns_false_for_missing_file() {
    FakeStorage storage;
    NfcCommon::TagInfo out{};
    TEST_ASSERT_FALSE(NfcTagLibrary::load(storage, "does_not_exist.tag", &out));
}

void test_save_rejects_zero_length_uid() {
    FakeStorage storage;
    NfcCommon::TagInfo tag{};
    tag.uid_len = 0;
    std::strncpy(tag.type_name, "Unknown", sizeof(tag.type_name) - 1);

    TEST_ASSERT_FALSE(NfcTagLibrary::save(storage, tag));

    char names[8][64];
    TEST_ASSERT_EQUAL_INT(0, NfcTagLibrary::list(storage, names, 8));
}

void test_resaving_same_uid_overwrites_rather_than_duplicates() {
    FakeStorage storage;
    const uint8_t uid[] = {0x01, 0x02, 0x03, 0x04};
    NfcCommon::TagInfo first;
    build_tag(&first, uid, sizeof(uid), "MIFARE Classic 1K");
    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, first));

    NfcCommon::TagInfo second;
    build_tag(&second, uid, sizeof(uid), "MIFARE Ultralight");
    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, second));

    char names[8][64];
    int count = NfcTagLibrary::list(storage, names, 8);
    TEST_ASSERT_EQUAL_INT(1, count); // same UID -> same filename -> overwrite, not a 2nd entry

    NfcCommon::TagInfo loaded{};
    TEST_ASSERT_TRUE(NfcTagLibrary::load(storage, names[0], &loaded));
    TEST_ASSERT_EQUAL_STRING("MIFARE Ultralight", loaded.type_name); // latest write wins
}

void test_list_returns_multiple_distinct_saved_tags() {
    FakeStorage storage;
    const uint8_t uid_a[] = {0xAA, 0xBB, 0xCC, 0xDD};
    const uint8_t uid_b[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};

    NfcCommon::TagInfo tag_a;
    build_tag(&tag_a, uid_a, sizeof(uid_a), "MIFARE Classic 1K");
    NfcCommon::TagInfo tag_b;
    build_tag(&tag_b, uid_b, sizeof(uid_b), "ISO14443A SAK 20");

    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag_a));
    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag_b));

    char names[8][64];
    int count = NfcTagLibrary::list(storage, names, 8);
    TEST_ASSERT_EQUAL_INT(2, count);

    // Order isn't guaranteed -- load both by whatever names came back and
    // confirm each of the two originals is present exactly once.
    bool found_a = false, found_b = false;
    for (int i = 0; i < count; i++) {
        NfcCommon::TagInfo loaded{};
        TEST_ASSERT_TRUE(NfcTagLibrary::load(storage, names[i], &loaded));
        if (loaded.uid_len == tag_a.uid_len &&
            std::memcmp(loaded.uid, tag_a.uid, tag_a.uid_len) == 0) {
            TEST_ASSERT_EQUAL_STRING(tag_a.type_name, loaded.type_name);
            found_a = true;
        } else if (loaded.uid_len == tag_b.uid_len &&
                   std::memcmp(loaded.uid, tag_b.uid, tag_b.uid_len) == 0) {
            TEST_ASSERT_EQUAL_STRING(tag_b.type_name, loaded.type_name);
            found_b = true;
        }
    }
    TEST_ASSERT_TRUE(found_a);
    TEST_ASSERT_TRUE(found_b);
}

void test_load_rejects_oversized_corrupt_record() {
    FakeStorage storage;
    const uint8_t uid[] = {0x01, 0x02, 0x03, 0x04};
    NfcCommon::TagInfo tag;
    build_tag(&tag, uid, sizeof(uid), "MIFARE Classic 1K");
    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag));

    char names[8][64];
    int count = NfcTagLibrary::list(storage, names, 8);
    TEST_ASSERT_EQUAL_INT(1, count);

    // Overwrite the same file with MORE bytes than a real TagInfo record --
    // simulates SD corruption or a stray oversized file landing under this
    // module's own directory+extension. IStorage::read_file()'s documented
    // contract caps *out_len at the read buffer's size when the real file is
    // longer, so without the +1-byte read buffer in load(), this oversized
    // file would be indistinguishable from an exact match. load() must
    // reject it outright rather than hand back a truncated/wrong record.
    char path[128];
    std::snprintf(path, sizeof(path), "/quarky/captures/nfc/%s", names[0]);
    uint8_t oversized[sizeof(NfcCommon::TagInfo) + 16];
    std::memset(oversized, 0xAB, sizeof(oversized));
    TEST_ASSERT_TRUE(storage.write_capture_file(path, oversized, sizeof(oversized)));

    NfcCommon::TagInfo loaded{};
    TEST_ASSERT_FALSE(NfcTagLibrary::load(storage, names[0], &loaded));
}

void test_list_respects_max_names_cap() {
    FakeStorage storage;
    for (int i = 0; i < 5; i++) {
        uint8_t uid[4] = {0x10, 0x10, 0x10, (uint8_t)i};
        NfcCommon::TagInfo tag;
        build_tag(&tag, uid, sizeof(uid), "Test Tag");
        TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag));
    }
    char names[2][64];
    int count = NfcTagLibrary::list(storage, names, 2);
    TEST_ASSERT_EQUAL_INT(2, count); // capped at max_names even though 5 were saved
}

// ── 2026-08-24 page-content extension ────────────────────────────────────
// NfcCommon::TagInfo now also carries a real captured NFC Forum Type 2 Tag
// page image (see nfc_common.h). The save/load format is still a raw
// fixed-layout struct dump, so these exercise the parts of that which are
// genuinely testable off-hardware: that the page bytes survive the round trip
// byte for byte, that a record with no content stays at the page_count == 0
// sentinel, and that load()'s clamp defends the Listen Mode responder against
// an out-of-range page_count in a corrupt/hand-edited record.

void test_page_content_round_trips_byte_for_byte() {
    FakeStorage storage;
    const uint8_t uid[] = {0x04, 0x73, 0x45, 0xE4, 0x3A, 0x02, 0x89};
    NfcCommon::TagInfo tag;
    build_tag(&tag, uid, sizeof(uid), "Ultralight/NTAG 135 pg");
    tag.sak = 0x00;
    tag.atqa[0] = 0x44;
    tag.atqa[1] = 0x00;

    // A recognisably non-uniform image, so a shifted/truncated copy fails
    // rather than accidentally matching.
    tag.page_count = 135; // NTAG215's real page count
    for (uint8_t p = 0; p < tag.page_count; p++) {
        for (uint8_t b = 0; b < NfcCommon::kT2tPageLen; b++) {
            tag.pages[p][b] = static_cast<uint8_t>((p * 7u) + (b * 31u) + 1u);
        }
    }

    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag));

    char names[8][64];
    TEST_ASSERT_EQUAL_INT(1, NfcTagLibrary::list(storage, names, 8));

    NfcCommon::TagInfo loaded{};
    TEST_ASSERT_TRUE(NfcTagLibrary::load(storage, names[0], &loaded));

    TEST_ASSERT_EQUAL_UINT8(tag.sak, loaded.sak);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tag.atqa, loaded.atqa, 2);
    TEST_ASSERT_EQUAL_UINT8(135, loaded.page_count);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&tag.pages[0][0], &loaded.pages[0][0],
                                  tag.page_count * NfcCommon::kT2tPageLen);
}

void test_tag_without_page_content_keeps_zero_sentinel() {
    FakeStorage storage;
    const uint8_t uid[] = {0x08, 0x92, 0x2F, 0x97};
    NfcCommon::TagInfo tag;
    build_tag(&tag, uid, sizeof(uid), "ISO14443A SAK 20");
    tag.sak = 0x20; // an EMV-style card: not a Type 2 tag, nothing to capture

    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag));

    char names[8][64];
    TEST_ASSERT_EQUAL_INT(1, NfcTagLibrary::list(storage, names, 8));

    NfcCommon::TagInfo loaded{};
    TEST_ASSERT_TRUE(NfcTagLibrary::load(storage, names[0], &loaded));
    TEST_ASSERT_EQUAL_UINT8(0, loaded.page_count);
}

void test_load_clamps_out_of_range_page_count() {
    FakeStorage storage;
    const uint8_t uid[] = {0x01, 0x02, 0x03, 0x04};
    NfcCommon::TagInfo tag;
    build_tag(&tag, uid, sizeof(uid), "Corrupt");
    TEST_ASSERT_TRUE(NfcTagLibrary::save(storage, tag));

    char names[8][64];
    TEST_ASSERT_EQUAL_INT(1, NfcTagLibrary::list(storage, names, 8));

    // Rewrite the SAME record, correctly sized (so it still passes load()'s
    // exact-size check) but with a page_count larger than pages[] can hold --
    // what a corrupt SD sector or a hand-edited .tag file could produce.
    // Without the clamp this value would reach ListenConfig::page_count and
    // then index past the driver's page image.
    NfcCommon::TagInfo corrupt = tag;
    corrupt.page_count = 255;
    char path[128];
    std::snprintf(path, sizeof(path), "/quarky/captures/nfc/%s", names[0]);
    TEST_ASSERT_TRUE(storage.write_capture_file(
        path, reinterpret_cast<const uint8_t *>(&corrupt), sizeof(corrupt)));

    NfcCommon::TagInfo loaded{};
    TEST_ASSERT_TRUE(NfcTagLibrary::load(storage, names[0], &loaded));
    TEST_ASSERT_EQUAL_UINT8(NfcCommon::kMaxT2tPages, loaded.page_count);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_save_then_load_round_trip);
    RUN_TEST(test_load_returns_false_for_missing_file);
    RUN_TEST(test_save_rejects_zero_length_uid);
    RUN_TEST(test_resaving_same_uid_overwrites_rather_than_duplicates);
    RUN_TEST(test_list_returns_multiple_distinct_saved_tags);
    RUN_TEST(test_load_rejects_oversized_corrupt_record);
    RUN_TEST(test_list_respects_max_names_cap);
    RUN_TEST(test_page_content_round_trips_byte_for_byte);
    RUN_TEST(test_tag_without_page_content_keeps_zero_sentinel);
    RUN_TEST(test_load_clamps_out_of_range_page_count);
    return UNITY_END();
}
