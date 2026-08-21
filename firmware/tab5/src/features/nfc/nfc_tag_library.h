#pragma once

#include "../../hal/istorage.h"
#include "nfc_common.h"

// Phase 3 Task 10: SD-backed tag library ("Tag-o-matic" in the Bruce donor,
// per docs/superpowers/specs/2026-08-06-phase3-tab5-nfc-rf433-design.md's
// Section 1 table) -- save a scanned NfcCommon::TagInfo to SD, list what's
// already saved, and load one back. A UI/storage feature more than a radio
// feature (same spec citation): everything below is plain IStorage file I/O,
// following wifi_evil_portal.cpp's already-proven "save/list/load small files
// via IStorage" shape, not new radio logic.
//
// DEVIATION FROM THE PLAN'S LITERAL SIGNATURE, noted explicitly: the plan's
// own Task 10 text writes these three as taking no storage parameter at all
// (implying an `extern StorageSD storage;` global, matching
// wifi_evil_portal.cpp's real precedent). That shape was tried first and
// does not build: any host-native test binary that links
// nfc_tag_library.cpp (via [env:native]'s build_src_filter) would then need
// a real definition of the global `StorageSD storage` and all of StorageSD's
// virtual method bodies -- a hard link-time dependency that broke the
// OTHER, unrelated native test suites in this same env (test_rf433_*), which
// have no reason to define a StorageSD stub of their own. Task 21's own
// rf433_sub_format.cpp/ir_file_format.cpp sidestep this exact problem by
// taking `IStorage &storage` as a parameter (dependency injection) instead
// of reading a global -- the plan's own Step 2 text and the controller notes
// both point at that as the pattern to reuse "rather than inventing a
// different mocking approach". This module follows that established,
// already-reviewed convention instead: save()/list()/load() take an
// IStorage& explicitly. Callers on real hardware (nfc_tag_library_ui.cpp)
// simply pass the real global `storage` (hal/storage_sd.h's StorageSD IS-A
// IStorage) at the call site -- no functional difference for them, and no
// separate "convenience no-arg overload" is needed.
namespace NfcTagLibrary {

// Saves tag under /quarky/captures/nfc/<hex-uid>.tag as a fixed-layout binary
// dump of the TagInfo struct itself (uid[10] + uid_len + type_name[24] = 35
// bytes; every member is byte-sized, so there is no compiler padding to worry
// about -- writing sizeof(tag) raw bytes is a faithful, lossless record).
// Deliberately no separate "name" parameter: the filename is derived from the
// tag's own UID, so re-saving the same physical tag overwrites its prior
// library entry (via write_capture_file()'s overwrite semantics) rather than
// accumulating duplicates. Returns false if uid_len is 0 (nothing to key the
// filename on) or the underlying SD write fails.
bool save(IStorage &storage, const NfcCommon::TagInfo &tag);

// Lists saved tag filenames (basenames, e.g. "04A3F1D2.tag", as
// IStorage::list_files() returns them) under /quarky/captures/nfc/, writing
// up to max_names entries into names_out. Returns the count actually
// written -- 0 if the directory doesn't exist, is empty, or mount() was
// never called. Mirrors wifi_evil_portal.cpp's storage.list_files()
// template-picker precedent exactly.
int list(IStorage &storage, char names_out[][64], int max_names);

// Loads the tag record named `name` (as returned by list()) from
// /quarky/captures/nfc/ into *out. Returns false if the file doesn't exist,
// can't be read, or isn't exactly sizeof(NfcCommon::TagInfo) bytes (a size
// mismatch means it isn't one of this module's own fixed-layout records).
bool load(IStorage &storage, const char *name, NfcCommon::TagInfo *out);

// Registers the "NFC: Tag Library" browse-screen launcher tile (list +
// tap-to-view saved tags). No register_module() call yet wires a "Save"
// action from a live scan into this library -- see nfc_tag_library_ui.cpp's
// header comment on that gap.
void register_module();

} // namespace NfcTagLibrary
