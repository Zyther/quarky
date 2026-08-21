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

    NfcCommon::TagInfo tmp{};
    size_t out_len = 0;
    if (!storage.read_file(path, reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp), &out_len)) {
        return false;
    }
    // A size mismatch means this file isn't one of this module's own
    // fixed-layout records (e.g. a short read, or something else entirely
    // landed under the same directory/extension) -- don't hand back a
    // partially-populated or garbage TagInfo.
    if (out_len != sizeof(tmp)) {
        return false;
    }
    *out = tmp;
    return true;
}

} // namespace NfcTagLibrary
