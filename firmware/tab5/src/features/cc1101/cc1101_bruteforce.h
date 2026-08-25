#pragma once

#include <cstdint>

// ===========================================================================
// CC1101 fixed-code bruteforce (Phase 10 Task 6).
//
// Donor reference: Bruce rf_bruteforce.cpp/rf_bruteforce.h
// (~/src/firmware/src/modules/rf/rf_bruteforce.{cpp,h}, real local checkout,
// read directly for this port -- NOT re-derived). Protocol bit-timing
// constants (zero/one/pilot/stop HIGH/LOW microsecond pairs) below are
// COPIED VERBATIM from rf_bruteforce.h's real `brute_protocols[]` table --
// same 6 protocols as SubghzProto's decode table minus Princeton (bruteforce
// needs to GENERATE what decode needs to RECOGNIZE, per this phase's plan
// Task 6 Context note -- Princeton is intentionally not in Bruce's own
// bruteforce table either, matching that reasoning).
//
// Same background-FreeRTOS-task shape as cc1101_replay.cpp (see that file's
// header comment for the full "why not poll()" citation): a full 12-bit
// sweep is thousands of codes, each needing microsecond-precision bit-bang
// timing that cannot survive being interleaved with LVGL on the main task.
// start() launches the task and returns immediately; poll() (called from
// main.cpp's loop()) reports progress.
// ===========================================================================

namespace Cc1101Bruteforce {

enum class Protocol {
    kCame12,
    kNice12,
    kAnsonic12,
    kHoltek12,
    kLinear10,
    kChamberlain9,
};

struct BruteforceStatus {
    bool running = false;
    const char *protocol_name = "";
    uint32_t code = 0;   // current code index (0-based)
    uint32_t total = 0;  // 1 << bits for the running protocol
};

// Starts a bruteforce sweep of `p` on `freq_mhz` from a dedicated FreeRTOS
// task. Refuses (no-op) if a sweep is already running. `repeats` is how many
// times each code's waveform is sent before advancing (Bruce's own
// rf_brute_repeats() menu allows 1-5; this port fixes it at a caller-chosen
// value rather than exposing a full menu, per this task's "thin review"
// scope -- see cc1101_bruteforce.cpp for the default used by the UI).
void start(Protocol p, float freq_mhz, int repeats);

// Requests the running sweep to stop after its current code. Safe to call
// whether or not a sweep is running.
void stop();

// Called from main.cpp's loop(). No-ops unless a sweep is in flight. Fills
// *out with the current progress; returns true if a sweep is (or was, up to
// this call) running.
bool poll(BruteforceStatus *out);

const char *protocol_name(Protocol p);

// FeatureRegistry wiring -- same shape as every other Tab5 feature module
// (rf433_bruteforce.cpp): register_module() adds the launcher tile;
// open_screen() (the tile's on_start, zero-arg per FeatureStartFn) pushes a
// protocol-pick + progress/stop screen that calls start()/stop()/poll()
// above.
void register_module();
void open_screen();

} // namespace Cc1101Bruteforce
