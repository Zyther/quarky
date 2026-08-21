#pragma once

#include "rf433_common.h"
#include <cstddef>
#include <cstdint>

namespace Rf433Scan {

// Maximum number of edge samples stored per captured signal burst.
//
// RAISED 2026-08-21 (real-hardware finding): the original 512 was never
// derived from real RF433 signal-length research -- it traces back to
// rf433_common.cpp's kRingSize (the ISR-to-main-task transfer ring, sized
// for "headroom between polling ticks", a completely different concern from
// the total length of a whole captured burst) and was carried over here
// without independent justification. A real Flipper Zero .sub capture of a
// genuine Tesla charge-port remote (project owner's own file,
// Tesla_charge_AM270.sub) contains 2395 RAW_Data duration values (2396
// edges) -- already 4.7x the old cap, and would have been silently
// truncated to a non-functional fraction of the real signal. Raised to
// 4096 (~1.7x headroom over that real capture), then to 8192 the same day
// once real PSRAM headroom was confirmed on this hardware (ESP.getPsramSize()
// -- 32MB total, ~33.5MB free at boot before any of this project's own
// allocations run) -- doubling every buffer this constant sizes costs on the
// order of ~1MB total, trivial against that budget, for extra margin against
// real-world signals longer than the Tesla capture that motivated the first
// raise.
//
// This constant sizes several buffers across rf433_scan.cpp,
// rf433_bruteforce.cpp, rf433_sub_format.cpp, and rf433_protocol_decode.cpp.
// At this size, a naive plain `static`/global array lands squarely back in
// the internal-DRAM-exhaustion crash class this project already found and
// fixed once for real (see s_signals' own allocation comment in
// rf433_scan.cpp) -- every one of those buffers was therefore converted to
// heap allocation (plain `new`, which this project's real, already-verified
// sdkconfig routes to PSRAM automatically once an allocation exceeds 4096
// bytes -- CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096, confirmed directly in
// framework-arduinoespressif32-libs/esp32p4/sdkconfig during Task 9's
// crapto1 research) as part of this same change -- not left as plain
// statics at the new, much larger size. This includes function-LOCAL
// stack variables too, not just globals/statics: a real crash was found
// and fixed on this same hardware from a stack-local CapturedSignal inside
// an LVGL click-handler callback (rf433_scan.cpp's "Combine -> .sub"
// button) that was fine at the old 512 (~4KB) but overflowed the main
// task's stack the moment this constant grew past it -- audit any NEW
// buffer sized by this constant for the same risk before assuming a local
// variable is safe.
constexpr size_t kMaxEdgesPerSignal = 8192;

// Maximum number of signals retained in memory for the scan list
constexpr size_t kMaxCapturedSignals = 16;

// Maximum signals the "Select"/"Combine -> .sub" daisy-chain feature can
// combine into one file (rf433_scan.cpp). Exposed here (rather than staying
// a rf433_scan.cpp-local constant) so rf433_sub_format.h can size its own
// combine-specific buffers off kMaxChainSignals * kMaxEdgesPerSignal -- the
// real worst case for a combined file, not an arbitrary separate guess.
constexpr int kMaxChainSignals = 8;

struct CapturedSignal {
    Rf433Common::EdgeSample edges[kMaxEdgesPerSignal];
    size_t edge_count;
    uint32_t captured_at_ms;

    // Monotonically increasing, never reused within a session. Identifies
    // this capture independent of its slot in the (ring-buffered) session
    // array, so a UI element created for this signal stays correct even
    // after later captures shift it out of its original array index.
    uint32_t capture_id;

    // True if the source burst hit kMaxEdgesPerSignal before its actual
    // end-of-burst gap arrived -- edges beyond the cap were dropped and this
    // signal's edges[]/edge_count do not represent the whole burst.
    bool truncated;
};

// Register the RF433 Scan feature module with the registry
void register_module();

// Feature module entry point (builds and pushes the screen)
void start();

// Called from main.cpp's loop() to drain the ISR ring buffer and update UI
void poll();

// Number of signals currently in the session buffer
size_t signal_count();

// Access a captured signal by index (returns nullptr if out of bounds)
const CapturedSignal *get_signal(size_t index);

} // namespace Rf433Scan
