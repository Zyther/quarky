#pragma once
#include <cstdint>
#include <cstddef>

class IStorage {
public:
    virtual ~IStorage() = default;
    virtual bool mount() = 0;
    virtual bool write_test_file() = 0;
    // Writes (overwriting if it exists) data[0..len) to path, creating any
    // missing parent directories first. Returns false on any failure (mount
    // not called, directory creation failed, write failed, or the written
    // byte count didn't match len). Used for one-shot files -- e.g. a pcap
    // global header written once at capture-file creation.
    virtual bool write_capture_file(const char *path, const uint8_t *data, size_t len) = 0;
    // Appends data[0..len) to path (creating it, and any missing parent
    // directories, if it doesn't exist yet). Returns false on any failure.
    // Distinct from write_capture_file's overwrite semantics: this is for a
    // live-growing capture file, where a promiscuous-mode ring buffer is
    // drained across many poll() calls into the same file without
    // clobbering what was already written (see wifi_pmkid.cpp).
    virtual bool append_capture_file(const char *path, const uint8_t *data, size_t len) = 0;

    // Reads up to max_len bytes from path into out. Returns false if path
    // doesn't exist or can't be opened. *out_len (if non-null) receives the
    // actual byte count read -- may be less than max_len if the file is
    // shorter, or capped at max_len if it's longer (callers wanting to
    // detect truncation should compare *out_len against a known/expected
    // file size). Added for wifi_evil_portal.cpp's user-supplied HTML
    // template loading -- whole-buffer read, not streaming, matching this
    // interface's existing whole-buffer write shape rather than exposing
    // the underlying filesystem type through the interface.
    virtual bool read_file(const char *path, uint8_t *out, size_t max_len, size_t *out_len) = 0;

    // Lists filenames (basenames, not full paths) in dir whose name ends
    // with ext_filter (e.g. ".html"), writing up to max_names entries (each
    // up to 63 chars + NUL) into names_out. Returns the number of entries
    // actually written -- 0 if dir doesn't exist, is empty, has no matches,
    // or mount() was never called. Added alongside read_file() for the same
    // reason (wifi_evil_portal.cpp's template picker).
    //
    // out_read_failed (optional, default nullptr -- existing callers need no
    // changes): set to true if a 0 result is suspected to be a REAL storage
    // read failure rather than a genuinely empty/nonexistent directory.
    // Added after a real hardware finding (Task 18, 2026-08-22/23 -- see
    // the SDD ledger's "Task 18: real-hardware crash investigation"
    // section): a busy WiFi/BLE radio session can consume nearly all of
    // this board's DMA-capable internal memory, making every SD_MMC block
    // read fail (`allocate_dma_buf: not enough mem`) -- and a failed read
    // and a genuinely empty directory were previously indistinguishable at
    // this API, silently showing "no files" for what was really "couldn't
    // read". Best-effort (a pre-flight low-DMA-memory check, not a
    // guarantee every possible real SD error is caught) -- left untouched
    // (never set to false) on a call this implementation doesn't suspect.
    virtual int list_files(const char *dir, const char *ext_filter, char names_out[][64], int max_names,
                           bool *out_read_failed = nullptr) = 0;

    // Lists immediate SUBDIRECTORY basenames (not full paths, not files) of
    // dir, writing up to max_names entries (each up to 63 chars + NUL) into
    // names_out. Non-recursive -- one directory level only. Returns the
    // number of entries actually written -- 0 if dir doesn't exist, has no
    // subdirectories, or mount() was never called. Added for Task 18
    // (ir_clone.cpp)'s real folder-by-folder Flipper-IRDB navigator: the
    // real on-SD-card database (`/quarky/ir/flipperdb/...`) is deep and
    // inconsistent in depth (2 levels in one observed real branch, 4 in
    // another), so a flat list_files()-only browser (Task 22's
    // ui/file_browser.h, deliberately scoped out of recursing -- see its own
    // header) cannot navigate it; this primitive plus repeated list_files()
    // calls, one directory at a time, is what lets ir_clone.cpp do so
    // without ever walking the whole tree in one call (a whole-tree walk is
    // what caused a real task-watchdog reset this session -- see the Task 18
    // plan section).
    //
    // Same dotfile-rejection contract as list_files() (see StorageSD's real
    // implementation): any entry whose basename starts with '.' is skipped,
    // not just AppleDouble sidecar files (`._Something`) -- the real SD copy
    // of the Flipper-IRDB contains thousands of these, one per real file,
    // from being copied onto the SD card via a Mac/Finder.
    //
    // out_read_failed: same real-read-failure-vs-empty-directory signal as
    // list_files()'s own parameter above -- see its comment for the full
    // citation.
    virtual int list_dirs(const char *dir, char names_out[][64], int max_names,
                          bool *out_read_failed = nullptr) = 0;
};
