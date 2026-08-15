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
    virtual int list_files(const char *dir, const char *ext_filter, char names_out[][64], int max_names) = 0;
};
