#pragma once
#include "istorage.h"

// microSD bring-up for the Tab5's SDMMC host **slot 0** (fixed IOMUX pins,
// GPIO 39/40/41/42/43/44) -- confirmed to be a physically separate hardware
// slot from the ESP32-C6 co-processor's esp-hosted SDIO link, which uses
// slot 1 (GPIO-matrix-routed, entirely different pins on real Tab5 hardware:
// GPIO 8/9/10/11/12/13). See storage_sd.cpp and
// .superpowers/sdd/2026-08-06-tab5-foundation-plan/task-10-report.md for the
// full research trail (SD/C6 SDIO-sharing question).
class StorageSD : public IStorage {
public:
    bool mount() override;
    bool write_test_file() override;
    bool write_capture_file(const char *path, const uint8_t *data, size_t len) override;
    bool append_capture_file(const char *path, const uint8_t *data, size_t len) override;
};
