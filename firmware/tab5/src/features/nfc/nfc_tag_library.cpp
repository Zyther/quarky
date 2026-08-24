// Pure storage half of NfcTagLibrary (save()/list()/load()) -- deliberately
// kept in its own translation unit, free of Arduino/LVGL/FeatureRegistry
// dependencies, so it can build for `pio test -e native`
// (platformio.ini's [env:native] has no LVGL available at all -- confirmed,
// .pio/libdeps/native only ever pulls in Unity). This mirrors the existing
// project convention of a pure format/storage module (rf433_sub_format.cpp,
// Task 21) kept separate from its own feature's UI module
// (rf433_scan.cpp) -- register_module() and the browse screen live in
// nfc_tag_library_ui.cpp instead, which is NOT part of [env:native]'s
// build_src_filter. Both files share the single nfc_tag_library.h this
// task's plan specifies, so callers (main.cpp) see one API regardless of
// the physical file split.
//
// Takes IStorage& by dependency injection rather than reading an
// `extern StorageSD storage;` global -- see nfc_tag_library.h's header
// comment for why (a global would be a hard link-time dependency every
// native test binary in this env would have to satisfy, not just this
// module's own test).
#include "nfc_tag_library.h"

#include <cstdio>
#include <cstring>

namespace NfcTagLibrary {

namespace {

constexpr char kLibraryDir[] = "/quarky/captures/nfc";
constexpr char kExt[] = ".tag";

// Builds "<dir>/<HEX-UID>.tag" from a tag's own UID bytes into path. No
// separate name parameter (see header): re-scanning the same physical tag
// overwrites its existing entry rather than accumulating duplicates. Returns
// false (leaving path untouched) if uid_len is 0 or out of range, since
// there'd be nothing valid to key the filename on.
bool build_path_for_tag(const NfcCommon::TagInfo &tag, char *path, size_t path_len) {
    if (tag.uid_len == 0 || tag.uid_len > sizeof(tag.uid)) {
        return false;
    }
    char hex[2 * sizeof(tag.uid) + 1];
    for (uint8_t i = 0; i < tag.uid_len; i++) {
        std::snprintf(hex + (i * 2), 3, "%02X", tag.uid[i]);
    }
    hex[tag.uid_len * 2] = '\0';
    std::snprintf(path, path_len, "%s/%s%s", kLibraryDir, hex, kExt);
    return true;
}

} // namespace

bool save(IStorage &storage, const NfcCommon::TagInfo &tag) {
    char path[96];
    if (!build_path_for_tag(tag, path, sizeof(path))) {
        return false;
    }
    return storage.write_capture_file(path, reinterpret_cast<const uint8_t *>(&tag), sizeof(tag));
}

int list(IStorage &storage, char names_out[][64], int max_names) {
    return storage.list_files(kLibraryDir, kExt, names_out, max_names);
}

bool load(IStorage &storage, const char *name, NfcCommon::TagInfo *out) {
    if (name == nullptr || out == nullptr) {
        return false;
    }
    char path[96];
    std::snprintf(path, sizeof(path), "%s/%s", kLibraryDir, name);

    // Read-buffer is deliberately ONE byte larger than TagInfo. IStorage::
    // read_file()'s own documented contract (istorage.h) caps *out_len at
    // max_len when the real file is LONGER than the buffer -- so reading
    // into a buffer exactly sizeof(TagInfo) would make an oversized/
    // corrupt file indistinguishable from an exact match (both report
    // out_len == sizeof(TagInfo)). The +1 byte turns that ambiguity into a
    // detectable case: an oversized file reports out_len ==
    // sizeof(TagInfo)+1, which now correctly fails the exact-match check
    // below instead of silently accepting a truncated/wrong record. Same
    // reasoning this project already applies via the `truncated` idiom in
    // rf433_sub_format.cpp/ir_file_format.cpp (Task 21) and via
    // wifi_evil_portal.cpp's `len < kMaxTemplateBytes` check.
    uint8_t raw[sizeof(NfcCommon::TagInfo) + 1];
    size_t out_len = 0;
    if (!storage.read_file(path, raw, sizeof(raw), &out_len)) {
        return false;
    }
    if (out_len != sizeof(NfcCommon::TagInfo)) {
        return false;
    }
    // Copied straight into *out rather than via a local TagInfo staging copy.
    // That staging copy was harmless while a record was 38 bytes; as of
    // 2026-08-24's page-content extension a record is ~1 KB, and holding two
    // of them plus the read buffer on the stack of an LVGL event callback
    // (nfc_tag_library_ui.cpp's show_tag()) is real stack pressure for no
    // benefit -- the exact-size check above has already passed by this point,
    // so there is nothing left that could reject the record and leave *out
    // needing to be untouched.
    std::memcpy(out, raw, sizeof(NfcCommon::TagInfo));
    // Defense in depth: force a NUL within type_name regardless of what was
    // actually on disk, so a caller's %s formatting (nfc_tag_library_ui.cpp)
    // can never read past this struct even if some other bug ever manages
    // to write a non-terminated record.
    out->type_name[sizeof(out->type_name) - 1] = '\0';
    // Same reasoning for page_count: a corrupt or hand-edited record must not
    // be able to make the Listen Mode responder index past pages[]. Clamped
    // rather than rejected, matching this module's existing "repair what can
    // be repaired, reject only a wrong-sized record" policy.
    if (out->page_count > NfcCommon::kMaxT2tPages) {
        out->page_count = NfcCommon::kMaxT2tPages;
    }
    // Same clamp, same reasoning, for the 2026-08-24 GET_VERSION addition.
    if (out->get_version_len > NfcCommon::kMaxGetVersionLen) {
        out->get_version_len = NfcCommon::kMaxGetVersionLen;
    }
    return true;
}

} // namespace NfcTagLibrary
