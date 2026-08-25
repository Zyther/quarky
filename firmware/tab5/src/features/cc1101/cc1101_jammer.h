#pragma once

#include <cstdint>

// ===========================================================================
// CC1101 jammer (Phase 10 Task 7): full + intermittent modes.
//
// Donor reference: Bruce rf_jammer.cpp/rf_jammer.h
// (~/src/firmware/src/modules/rf/rf_jammer.cpp, real local checkout, read
// directly for this port). Bruce's real jammer has 4 modes (FULL, ITMT,
// NOISE, SWEEP) -- this phase's spec (Phase 5 spec Section 1, cited by this
// phase's plan Task 7) scopes CC1101 jammer to "full/intermittent" only, so
// NOISE (CC1101 PN9 hardware mode, needs ELECHOUSE-specific register pokes
// this project's RadioLib-based Cc1101Hw does not wrap) and SWEEP are
// intentionally not ported here -- a real scope decision, not an oversight.
// run_full_jammer()'s 3-phase pattern and run_itmt_jammer()'s
// sweep+random-burst pattern (including send_optimized_pulse()/
// send_random_pattern()) are ported with the SAME real timing constants
// Bruce uses, adapted from Bruce's own blocking `while (sendRF)` shape into
// a background FreeRTOS task with a polled stop flag (this project's house
// rule: no blocking loop on the main/LVGL task).
//
// LEGAL/SAFETY: RF jamming has real regulatory exposure. This module must
// only run under the project owner's own authorized test-lab/RF-shielded
// environment -- see this phase's plan Task 7 PAUSE-FOR-HARDWARE note. The
// UI must make unambiguous that the radio is actively transmitting, same
// clarity bar Task 8 (KeeLoq) sets, per the project owner's explicit
// instruction.
// ===========================================================================

namespace Cc1101Jammer {

enum class JamMode { kFull, kIntermittent };

struct JammerStatus {
    bool running = false;
    JamMode mode = JamMode::kFull;
    uint32_t pulse_count = 0;
    uint32_t elapsed_ms = 0;
};

// Starts jamming `mode` on `freq_mhz` from a dedicated FreeRTOS task. Runs
// until stop() is called (jamming has no natural "done" state). Refuses
// (no-op) if already running or Cc1101Hw setup fails.
void start(JamMode mode, float freq_mhz);

// Requests the running jam session to stop. Safe to call whether or not one
// is running.
void stop();

// Called from main.cpp's loop(). No-ops unless a session is in flight.
bool poll(JammerStatus *out);

// FeatureRegistry wiring, same shape as every other Tab5 feature module.
void register_module();
void open_screen();

} // namespace Cc1101Jammer
