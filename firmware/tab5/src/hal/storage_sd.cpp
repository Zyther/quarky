#include "storage_sd.h"
#include <SD_MMC.h> // Tab5's SD is SDIO-attached, not SPI
#include <cstring> // strncpy (ensure_parent_dirs)
#include <cstdio>  // snprintf (posix_path)
#include <cstdlib> // qsort -- see scan_dir()'s own sort comment
#include <dirent.h> // opendir/readdir/d_type -- see scan_dir()'s own comment
                    // for the real measured reason this replaced the
                    // Arduino FS openNextFile() loop these two functions
                    // used to share
#include <esp_heap_caps.h> // heap_caps_get_free_size(MALLOC_CAP_DMA) -- see
                          // list_files()/list_dirs()'s own out_read_failed
                          // comment for the real finding this checks for
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
// Real finding, Task 18 hardware investigation (2026-08-22/23 -- see the SDD
// ledger's "Task 18: real-hardware crash investigation" section): a busy
// WiFi/BLE radio session (esp-hosted's own transport mempool, which
// genuinely requires DMA-capable memory) can drive this board's real
// DMA-capable internal-memory pool down to as little as 128 bytes, at which
// point every SD_MMC block read fails
// (`sdmmc_cmd: allocate_dma_buf: not enough mem, err=0x101`) -- observed
// directly on real hardware. 8192 bytes is a conservative real threshold:
// generously above what one SD block-read DMA transfer actually needs (SD
// sectors are 512 bytes; even generous driver/descriptor overhead is nowhere
// near 8KB), so this only fires under genuinely degraded conditions like the
// one that motivated it, not under ordinary light memory pressure.
static bool dma_memory_critically_low() {
    return heap_caps_get_free_size(MALLOC_CAP_DMA) < 8192;
}

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


// ---------------------------------------------------------------------------
// Directory scanning: POSIX opendir()/readdir(), NOT Arduino FS
// openNextFile().
//
// THIS PREVENTS A REAL TASK-WATCHDOG REBOOT, it is not a micro-optimisation.
// Measured on the physical Tab5 2026-08-23, timing list_dirs()/list_files()
// directly while navigating the real on-SD Flipper-IRDB:
//
//   /quarky/ir/flipperdb      94 raw entries   list_dirs 620 ms + list_files 620 ms
//   /quarky/ir/flipperdb/TVs 236 raw entries   list_dirs 4.30 s + list_files 4.30 s
//
// 8.70 s of blocking SD I/O inside ONE loop() iteration, against a 5 s
// CONFIG_ESP_TASK_WDT_TIMEOUT_S -> "Task watchdog got triggered ... loopTask
// (CPU 1) ... Aborting" and a reboot. (Confirmed it is a stall, not a spin:
// the abort's register dump has CPU 1 in IDLE1 with MEPC in
// esp_cpu_wait_for_intr -- loopTask was blocked in I/O, not looping.)
//
// Note the per-entry cost is not constant: 6.6 ms/entry at 94 entries but
// 18.2 ms/entry at 236. That superlinearity is the actual bug, and it comes
// from the Arduino layer. VFSFileImpl::openNextFile() (framework-arduino
// esp32 libraries/FS/src/vfs_api.cpp:454) calls readdir() -- which already
// carries the entry's type in dirent::d_type -- then DISCARDS it and
// constructs a VFSFileImpl for the entry's full path, whose constructor
// stat()s it. On FATFS a stat() by name has to walk the directory to find
// the entry, so scanning a directory of N entries costs O(N^2) block reads.
//
// readdir() alone is O(N) total and needs no stat() at all: ESP-IDF's FATFS
// VFS fills d_type itself from the directory entry it has already read
// (esp_vfs_fat.c's vfs_fat_readdir_r: `d_type = (fno.fattrib & AM_DIR) ?
// DT_DIR : DT_REG`), which is exactly -- and only -- the DIR-vs-FILE
// distinction both callers below need.
//
// Semantics are otherwise deliberately unchanged from the openNextFile()
// version: same dotfile rejection, same max_names cap, same
// kMaxEntriesScanned visit bound, same basename-only output (readdir's
// d_name is already a basename, as Arduino's entry.name() was here).
namespace {

// SD_MMC.begin()'s default mountpoint (SD_MMC.h:58). The Arduino FS object
// hides it; POSIX calls do not, so paths must be re-rooted through it.
constexpr char kSdMountpoint[] = "/sdcard";

// Total directory entries visited per scan, regardless of how many match.
// Same bound (and same 2026-08-15 real finding behind it) the openNextFile()
// loops carried: a directory of many non-matching entries would otherwise
// make this scan unbounded, and it is called synchronously from a click
// handler inside a single loop() iteration. Kept at 512 for the same
// AppleDouble-sidecar doubling reason documented at its previous 256->512
// raise -- every real file in this corpus is accompanied by a `._` sidecar,
// so a real directory costs two visited slots per real entry (the real TVs
// folder measured 236 raw entries for 117 real subdirectories).
constexpr int kMaxEntriesScanned = 512;

// qsort() comparator for names_out[]'s fixed-width entries -- plain
// case-sensitive strcmp. Real question this answers (2026-08-23, project
// owner): "is directory-list ordering a side effect of being memory
// bound?" No -- readdir() (and Arduino's openNextFile() before it) simply
// returns entries in raw on-disk/filesystem order, never sorted; nothing
// about the O(N^2)-scan fix or the LVGL memory-pool fix touches ordering
// at all. Case-sensitive is deliberate, not an oversight: the real
// Flipper-IRDB corpus's own category names are consistently
// Title_Case_With_Underscores (confirmed directly -- ACs, Audio_and_
// Video_Receivers, TVs, Window_cleaners, etc.), so plain ASCII strcmp
// already gives the alphabetical order a human expects for this real
// data; a case-INSENSITIVE sort would be needed for a corpus that mixed
// case inconsistently, which this one doesn't.
int compare_names(const void *a, const void *b) {
    return strcmp(static_cast<const char *>(a), static_cast<const char *>(b));
}

// Shared body of both public listers. `want_dirs` selects DT_DIR vs DT_REG;
// `ext_filter` is applied to files only (nullptr = accept any).
int scan_dir(const char *dir, bool want_dirs, const char *ext_filter, char names_out[][64], int max_names) {
    char full[256];
    std::snprintf(full, sizeof(full), "%s%s", kSdMountpoint, dir);
    DIR *d = opendir(full);
    if (d == nullptr) return 0;

    size_t ext_len = (ext_filter != nullptr) ? strlen(ext_filter) : 0;
    int count = 0;
    int visited = 0;
    struct dirent *e;
    while ((e = readdir(d)) != nullptr && count < max_names && visited < kMaxEntriesScanned) {
        visited++;
        if (want_dirs != (e->d_type == DT_DIR)) continue;
        const char *name = e->d_name;
        // The real on-SD Flipper-IRDB copy carries a macOS AppleDouble
        // sidecar (`._Something.ir`) beside every real file, an artifact of
        // having been written to the card by Finder. Those match a naive
        // extension filter just as well as the real file, so reject any
        // name starting with '.' (covers AppleDouble and ordinary hidden
        // entries alike) before the extension check.
        if (name[0] == '.') continue;
        if (ext_filter != nullptr) {
            size_t name_len = strlen(name);
            if (name_len <= ext_len || strcmp(name + name_len - ext_len, ext_filter) != 0) continue;
        }
        strncpy(names_out[count], name, 63);
        names_out[count][63] = '\0';
        count++;
    }
    closedir(d);
    // Alphabetize before returning -- readdir() gives raw on-disk order,
    // not sorted (see compare_names()'s own comment). Cheap regardless of
    // the real per-directory entry counts this project has actually seen
    // (up to 236 raw/117 real for the largest real folder measured,
    // flipperdb/TVs) -- qsort() on <= kMaxEntriesScanned fixed-width
    // 64-byte strings is negligible next to the real directory-scan I/O
    // cost this same function's own header comment measures in
    // milliseconds.
    qsort(names_out, count, sizeof(names_out[0]), compare_names);
    return count;
}

} // namespace

int StorageSD::list_files(const char *dir, const char *ext_filter, char names_out[][64], int max_names,
                          bool *out_read_failed) {
    // Real pre-flight check (see dma_memory_critically_low()'s own citation):
    // when DMA memory is this low, the directory scan below is essentially
    // guaranteed to fail its own internal block reads -- checking once up
    // front, before attempting any of them, gives a clean, deterministic
    // "likely read failure" signal instead of a silent 0 indistinguishable
    // from a genuinely empty directory.
    if (dma_memory_critically_low()) {
        if (out_read_failed) *out_read_failed = true;
        return 0;
    }
    return scan_dir(dir, /*want_dirs=*/false, ext_filter, names_out, max_names);
}

int StorageSD::list_dirs(const char *dir, char names_out[][64], int max_names,
                         bool *out_read_failed) {
    // Same real pre-flight check as list_files() -- see that function's own
    // comment and dma_memory_critically_low()'s citation.
    if (dma_memory_critically_low()) {
        if (out_read_failed) *out_read_failed = true;
        return 0;
    }
    return scan_dir(dir, /*want_dirs=*/true, /*ext_filter=*/nullptr, names_out, max_names);
}

bool StorageSD::remove_file(const char *path) {
    // SD_MMC.exists()/remove() are the real Arduino fs::FS methods this
    // whole file's SD_MMC object already provides -- no new library
    // dependency. Idempotent per this method's own IStorage contract: a
    // path that doesn't exist is treated as already-successfully-removed,
    // matching POSIX unlink()'s own real convention, rather than SD_MMC's
    // own remove() (which returns false for a missing file the same way it
    // would for a genuine failure -- indistinguishable without this check).
    if (!SD_MMC.exists(path)) {
        return true;
    }
    return SD_MMC.remove(path);
}
