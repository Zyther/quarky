#pragma once

// ===========================================================================
// RF433 bruteforce (Phase 3 Task 8): walks a fixed-code remote's keyspace
// (2^bits candidate codes for a given protocol) and transmits each candidate
// in turn -- the standard attack against cheap OOK garage-door/gate/barrier
// remotes that have no rolling-code protection, which is what Bruce's
// `rf_bruteforce.cpp` (ported from below) and UniGeek's equivalent tool both
// implement.
//
// SOURCE: ~/src/firmware/src/modules/rf/rf_bruteforce.h / .cpp (Bruce). The
// donor's `constexpr BruteProtocol brute_protocols[]` table
// (rf_bruteforce.h:17-25) -- 6 real entries: Came 12bit, Nice 12bit,
// Ansonic 12bit, Holtek 12bit, Linear 10bit, Chamberlain 9bit -- is ported
// verbatim in rf433_bruteforce.cpp, along with the real keyspace-walk /
// pulse-encoding sequence (rf_bruteforce.cpp:63-83, inside rf_brute_start()).
// See that file's header comment for the exact line citations.
//
// ARCHITECTURE, REQUIRED BY THE CONTROLLER (this plan's task-8-controller-
// notes.md): Bruce's own rf_brute_start() bit-bangs the TX pin directly via
// a local sendPulse() helper (rf_bruteforce.cpp:36-44) -- because Bruce's
// firmware has none of this project's safety machinery (GPIO53 arbiter,
// core-pinned transmit task, RX/TX mutual exclusion; see rf433_replay.h's
// header comment for the full real-hardware citation trail behind all
// three). Re-deriving a second, separate bit-bang path here would either
// duplicate that safety-critical machinery or -- worse -- bypass it,
// reopening exactly the hazards Task 6's two review rounds found and fixed
// (waveform corruption under LVGL preemption, GPIO53 contention with the
// external I2C bus, capture-vs-transmit collision).
//
// Instead: each candidate code's pulse sequence (pilot + per-bit zero/one +
// stop, built from the ported timing table) is synthesized into a
// Rf433Scan::CapturedSignal / Rf433Common::EdgeSample array -- the exact
// same shape a real RF433 capture produces -- and handed to
// Rf433Replay::transmit(), the exact same call Rf433Scan's Replay button
// makes. transmit() is asynchronous (spawns its own background task,
// returns immediately). This module's poll() waits for the in-flight
// transmit to finish (Rf433Replay::is_busy()/state()) before firing the
// next candidate -- that wait is what gives this feature its "one bounded
// step per tick" shape, without any separate rate-limiting logic of its own.
// ===========================================================================

namespace Rf433Bruteforce {

// Registers this module's launcher tile (Category::RF433, Affinity::
// TAB5_NATIVE). Call once from setup(), before Shell::build() -- same
// convention as every other register_module() in this codebase.
void register_module();

// Called from main.cpp's loop(), after Rf433Replay::poll() (so this tick's
// is_busy()/state() reads are fresh) and after Rf433Scan::poll(). No-ops
// unless a bruteforce run is active (Start has been tapped and the run
// hasn't finished/been stopped/failed yet).
void poll();

} // namespace Rf433Bruteforce
