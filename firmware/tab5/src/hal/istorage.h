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
};
