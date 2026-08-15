#include "storage_sd.h"
#include <SD_MMC.h> // Tab5's SD is SDIO-attached, not SPI
#include <cstring> // strncpy (ensure_parent_dirs)
// boards/tab5/pins_config.h documents the real Tab5 SD/C6 SDIO pins and the
// SDIO-host-sharing research below in named constants (TAB5_SD_*,
// TAB5_C6_SDIO_*). Not included/consumed directly here: SD_MMC's own
// ESP32-P4 slot-0 defaults (from pins_arduino.h's BOARD_SDMMC_SLOT=0)
// already resolve to the correct real Tab5 SD pins, so no explicit
// setPins() call -- and no macro use -- is needed (see caveat (c) below).

// SD/C6 SDIO-host-sharing research (Task 10), sourced from four independent
// places, cross-checked against each other rather than assumed. (Fix report,
// review round 2: point 1's espp citation is clarified below -- it is NOT
// itself evidence of slot 0 -- and point 4 was added to close the citation
// gap between "these #defines exist" and "these #defines are what actually
// select the runtime hardware slot".)
//
// 1. espp/m5stack-tab5 BSP source (esp-cpp/espp, `main` branch, fetched
//    2026-08-07 via raw.githubusercontent.com):
//      https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/include/m5stack-tab5.hpp
//      https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/sdcard.cpp
//    gives the real Tab5 schematic pins for BOTH buses:
//      microSD (SDIO/SDMMC): clk=GPIO43, cmd=GPIO44, d0=GPIO39, d1=GPIO40,
//        d2=GPIO41, d3=GPIO42.
//      ESP32-C6 link ("SDIO2" net names in the BSP's own comments):
//        clk=GPIO12, cmd=GPIO13, d0=GPIO11, d1=GPIO10, d2=GPIO9, d3=GPIO8,
//        reset=GPIO15.
//    These two pin sets are entirely disjoint -- no shared wire between them.
//    CLARIFICATION (this pin evidence is all espp's sdcard.cpp corroborates
//    -- NOT the slot number): espp's own `initialize_sdcard()` calls
//    `SDMMC_HOST_DEFAULT()` unmodified, which sets `.slot = SDMMC_HOST_SLOT_1`
//    (confirmed: esp_driver_sdmmc/include/driver/sdmmc_default_configs.h,
//    `SDMMC_HOST_DEFAULT()` macro). So if espp's own reference code ran
//    as-is, its SD card would land on slot 1 -- the SAME slot esp-hosted
//    uses for the C6 (see point 3) -- reproducing exactly the conflict this
//    task is ruling out. espp's file is NOT itself evidence of "SD on slot
//    0"; only its pin numbers (which do match the fixed slot-0 IOMUX pins,
//    see point 2) corroborate the physical wiring. It's THIS PROJECT's
//    `SD_MMC.begin()` (via `BOARD_SDMMC_SLOT=0`, see point 4) that actually
//    avoids the conflict by explicitly overriding to slot 0 -- not anything
//    in espp's sdcard.cpp.
//
// 2. ESP-IDF's own ESP32-P4 headers confirm *why* pin-disjointness isn't a
//    coincidence: the P4's single physical SDMMC host peripheral exposes two
//    independent hardware slots (soc/esp32p4/include/soc/sdmmc_pins.h,
//    esp_driver_sdmmc/include/driver/sdmmc_host.h):
//      - Slot 0: fixed IOMUX pins, SDMMC_SLOT0_IOMUX_PIN_NUM_{CLK,CMD,D0..D3}
//        = 43/44/39/40/41/42 -- an EXACT match to the BSP's real SD pins
//        above. Full 8-bit-capable, highest throughput, but pins are fixed
//        in silicon (can't be moved).
//      - Slot 1: "doesn't go through IOMUX" (sdmmc_host.h) -- routed via
//        GPIO matrix to any pins, 1- or 4-bit only.
//    sdmmc_host.h's own API (sdmmc_host_init_slot(), sdmmc_host_do_transaction(),
//    etc.) takes an explicit `slot` parameter (SDMMC_HOST_SLOT_0 /
//    SDMMC_HOST_SLOT_1) and documents them as independently addressable --
//    this is the IDF-supported "one SD card + one SDIO peripheral
//    concurrently" pattern, not a shared/multiplexed bus.
//
// 3. The pioarduino toolchain's actual precompiled config for this project
//    confirms which slot esp-hosted uses: the esp32p4 sdkconfig.h bakes in
//    CONFIG_ESP_HOSTED_SDIO_SLOT_1=1 / CONFIG_ESP_HOSTED_SDIO_SLOT=1 (grepped
//    from framework-arduinoespressif32-libs/esp32p4/qio_qspi/include/sdkconfig.h).
//    Combined with #2, this places the C6 link on slot 1 and confirms the SD
//    card (slot 0, via BOARD_SDMMC_SLOT=0 below) is on the *other* hardware
//    slot, not a shared one.
//
// 4. The missing link: what actually turns the #defines in points 1/3 into
//    a real runtime hardware-slot assignment, not just documentation:
//      - SD_MMC.cpp (framework-arduinoespressif32/libraries/SD_MMC/src/SD_MMC.cpp,
//        SDMMCFS::begin(), ~line 230): under
//        `#if defined(CONFIG_IDF_TARGET_ESP32P4) && defined(BOARD_SDMMC_SLOT)
//        && (BOARD_SDMMC_SLOT == 0)`, sets `host.slot = SDMMC_HOST_SLOT_0;`
//        (and reconfigures slot_config to use the fixed IOMUX pins instead
//        of GPIO-matrix pins). This is the exact line that makes this
//        project's `SD_MMC.begin()` call (below) actually request slot 0 at
//        runtime, not merely define a constant that says so.
//      - port_esp_hosted_host_config.h line 218
//        (framework-arduinoespressif32-libs/esp32p4/include/espressif__esp_hosted/host/port/esp/freertos/include/port_esp_hosted_host_config.h):
//        `#define H_SDMMC_HOST_SLOT CONFIG_ESP_HOSTED_SDIO_SLOT` -- this is
//        the line that makes esp-hosted's SDIO transport driver actually
//        consume `CONFIG_ESP_HOSTED_SDIO_SLOT` (=1, from point 3) as its
//        real slot number at init time, not just a config value sitting
//        unused in sdkconfig.h.
//    Both confirmed present in the actual installed toolchain used to build
//    this project (not just plausible-sounding header names).
//
// CONCLUSION: the SD card and the C6 co-processor are on two separate
// hardware SDIO slots of the ESP32-P4 (slot 0 vs slot 1), with zero
// overlapping GPIO pins on real Tab5 hardware. There is no electrical bus
// contention between them -- resolved from source, not deferred. This
// directly answers the foundation spec's flagged risk.
//
// RESIDUAL CAVEATS (genuinely open, not resolved from source):
//   a) Both slots are still served by the *same* physical SDMMC/DMA/interrupt
//      hardware block on the P4 SoC. The IDF docs/headers found here don't
//      say whether truly simultaneous slot-0 + slot-1 transactions can incur
//      indirect resource contention (shared DMA bandwidth/interrupt latency)
//      under heavy concurrent load, even though there's no pin-level
//      conflict. This is a much smaller risk than the plan's feared "shared
//      bus needing time-multiplexing" scenario, but not provably zero from
//      source alone -- genuinely needs the hardware pass (Step 3 of the
//      brief) to confirm under real concurrent WiFi + SD traffic.
//   b) IMPORTANT SEPARATE FINDING, flagged forward (not fixed in this task):
//      this project currently builds against PlatformIO's generic
//      `esp32-p4-evboard` board (Task 9's deliberate choice, see its
//      report), whose `variants/esp32p4/pins_arduino.h` defines
//      BOARD_SDIO_ESP_HOSTED_{CLK,CMD,D0,D1,D2,D3,RESET} =
//      18/19/14/15/16/17/54 -- the *generic ESP32-P4-Function-EV-Board's*
//      C6 pins, confirmed consumed at compile time by
//      cores/esp32/esp32-hal-hosted.c (which Task 9's build log shows is
//      compiled in). These do NOT match the real Tab5 BSP's C6 pins
//      (12/13/8-11/15) found above. This means Task 9's WiFi bring-up, as
//      currently configured, is very likely wired to the WRONG physical
//      GPIOs for real Tab5 hardware -- independent of, and arguably more
//      urgent than, the SD/C6 sharing question this task was scoped to
//      answer. Not fixed here (would mean overriding pins_arduino.h or
//      switching board definitions, a board-config decision affecting
//      Task 9's radio too, out of this task's scope) -- see
//      task-10-report.md for detail.
//   c) By contrast, SD_MMC's own slot-0 default pins happen to already be
//      correct for Tab5: SD_MMC.cpp's ESP32-P4 branch
//      (`BOARD_HAS_SDMMC` + `BOARD_SDMMC_SLOT == 0`, both set by this same
//      pins_arduino.h) defaults `SD_MMC`'s pins to
//      SDMMC_SLOT0_IOMUX_PIN_NUM_* -- which are fixed in silicon and
//      therefore identical for the EV-board and Tab5 alike. So the
//      plain `SD_MMC.begin()` below (no explicit setPins() call) already
//      targets the real Tab5 SD pins correctly -- no divergence like (b).
//   d) The EV-board's pins_arduino.h also defines an SD power-enable pin
//      (BOARD_SDMMC_POWER_CHANNEL=4, BOARD_SDMMC_POWER_PIN=45, active LOW).
//      The Tab5 BSP source does not document an equivalent dedicated SD
//      power-switch GPIO -- genuinely unconfirmed whether Tab5's SD socket
//      needs/has one, or is always-powered. Left as-is (SD_MMC.begin() will
//      toggle GPIO45 per the EV-board default); flagged as a TODO to verify
//      against Tab5's schematic, not fabricated.
//
// See task-10-report.md for the full trail and citations.

bool StorageSD::mount() {
    // SD_MMC pin assignment is confirmed (see above) to already resolve to
    // Tab5's real SD pins via BOARD_SDMMC_SLOT=0 in pins_arduino.h -- no
    // explicit setPins() call needed. Confirmed to be on a *separate* SDIO
    // host slot from the C6 co-processor link (Task 9): slot 0 vs slot 1.
    return SD_MMC.begin();
}

bool StorageSD::write_test_file() {
    File f = SD_MMC.open("/quarky_bringup_test.txt", FILE_WRITE);
    if (!f) return false;
    f.println("quarky foundation bring-up test");
    f.close();
    return true;
}

// Walks path, creating each directory component in turn -- e.g.
// "/quarky/captures/wifi/x.pcap" -> mkdir /quarky, then /quarky/captures,
// then /quarky/captures/wifi. SD_MMC.mkdir() on an already-existing directory
// is a harmless no-op (confirmed against the ESP32 SD_MMC/FS library), so no
// existence check is needed before each call.
static void ensure_parent_dirs(const char *path) {
    char buf[128];
    strncpy(buf, path, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *p = buf + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            SD_MMC.mkdir(buf);
            *p = '/';
        }
    }
}

bool StorageSD::write_capture_file(const char *path, const uint8_t *data, size_t len) {
    ensure_parent_dirs(path);
    File f = SD_MMC.open(path, FILE_WRITE); // truncates/creates
    if (!f) return false;
    size_t written = f.write(data, len);
    f.close();
    return written == len;
}

bool StorageSD::append_capture_file(const char *path, const uint8_t *data, size_t len) {
    // FILE_APPEND is the ESP32 SD_MMC/FS library's append-mode open flag --
    // seeks to end-of-file (creating the file if it doesn't exist yet)
    // instead of truncating like FILE_WRITE does. This is what lets poll()
    // call this repeatedly across many drain cycles without clobbering
    // packet records already written to the same capture file.
    //
    // Task review finding (2026-08-12): ensure_parent_dirs() used to run
    // unconditionally here, adding 3 redundant SD_MMC.mkdir() RPC calls to
    // every single drain -- potentially several times a second for the
    // whole duration of an active capture -- even though the one caller in
    // this codebase (wifi_pmkid.cpp's poll(), draining after start()'s
    // write_capture_file() call already created the same directory tree
    // once) never needs it again. Try the open first; only pay for
    // ensure_parent_dirs() (and retry) on the rarer path where it's
    // actually missing, e.g. a future caller that appends without an
    // earlier write_capture_file() call.
    File f = SD_MMC.open(path, FILE_APPEND);
    if (!f) {
        ensure_parent_dirs(path);
        f = SD_MMC.open(path, FILE_APPEND);
        if (!f) return false;
    }
    size_t written = f.write(data, len);
    f.close();
    return written == len;
}

bool StorageSD::read_file(const char *path, uint8_t *out, size_t max_len, size_t *out_len) {
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) return false;
    size_t n = f.read(out, max_len);
    f.close();
    if (out_len) *out_len = n;
    return true;
}

int StorageSD::list_files(const char *dir, const char *ext_filter, char names_out[][64], int max_names) {
    File d = SD_MMC.open(dir);
    if (!d || !d.isDirectory()) return 0;

    // Task review finding (2026-08-15, Minor): the loop used to stop only
    // once max_names MATCHES were found, so a directory containing many
    // non-matching entries (no reasonable real-world limit) made this scan
    // unbounded -- called synchronously from build_screen(), itself run
    // from the launcher tile's click handler inside a single loop()
    // iteration, against this project's own ~50ms loop() budget
    // constraint. kMaxEntriesScanned bounds total directory entries visited
    // regardless of match count, independent of max_names.
    constexpr int kMaxEntriesScanned = 256;
    size_t ext_len = strlen(ext_filter);
    int count = 0;
    int visited = 0;
    File entry = d.openNextFile();
    while (entry && count < max_names && visited < kMaxEntriesScanned) {
        visited++;
        if (!entry.isDirectory()) {
            const char *name = entry.name(); // basename, not full path (FS.h)
            size_t name_len = strlen(name);
            if (name_len > ext_len && strcmp(name + name_len - ext_len, ext_filter) == 0) {
                strncpy(names_out[count], name, 63);
                names_out[count][63] = '\0';
                count++;
            }
        }
        entry = d.openNextFile();
    }
    return count;
}
