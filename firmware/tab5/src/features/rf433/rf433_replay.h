#pragma once

#include "rf433_scan.h"

// ===========================================================================
// RF433 replay (Phase 3 Task 6): bit-bangs TAB5_RF433T_PIN (GPIO53) to
// reproduce a Task 5 CapturedSignal's edge timing via digitalWrite() +
// delayMicroseconds() -- the same real technique Bruce's rf_send.cpp/
// emit.cpp (via rc-switch) uses, per the plan's own donor citation.
//
// NOT registered with FeatureRegistry / g_registry, and deliberately has no
// register_module() -- despite the plan's Task 6 "Produces" line listing
// one. Two real reasons, not a shortcut:
//   1. ui/shell.cpp's build_category_screen() documents (Global
//      Constraints) that every module registered with g_registry is
//      required to have a real on_start -- a tapped tile calls it
//      unconditionally-if-non-null and a nullptr on_start would silently
//      render a dead tile. Replay has no screen of its own to launch:
//      Task 6's own Step 2 wires it into rf433_scan.cpp's EXISTING
//      scan-result list, not a new screen, so there is no real on_start to
//      give it.
//   2. features/ble/ble_target_picker.h is this codebase's exact existing
//      precedent for this shape: a component with its own start()/poll()
//      that is reachable only from another feature's screen, explicitly not
//      registered -- that header documents it as a shared picker with no
//      launcher tile of its own, a component rather than a feature in
//      FeatureRegistry's sense (paraphrased here, not a verbatim quote).
// See task-6-report.md for the full reasoning.
//
// Architecture note (see rf433_replay.cpp's header comment for the full
// real-hardware-crash citation this follows): a captured signal can
// legitimately take hundreds of milliseconds to replay, so the actual
// bit-bang loop runs on a dedicated FreeRTOS task, never on the caller's
// (main/LVGL) task. transmit() returns immediately; poll() (called from
// main.cpp's loop(), same as every other async feature module here) reports
// the outcome via state()/failure_reason().
// ===========================================================================

namespace Rf433Replay {

enum class State { kIdle, kTransmitting, kDone, kFailed };

// Starts transmitting `sig` on TAB5_RF433T_PIN (GPIO53) from a dedicated
// FreeRTOS task. Returns immediately -- call is_busy()/state()/
// failure_reason(), or just poll from the UI, to observe the outcome.
//
// Refuses (starts nothing new, leaves any in-flight transmit untouched) and
// moves to State::kFailed if:
//   - a transmit is already in flight (is_busy() was already true)
//   - a capture is currently active (Rf433Common::is_capturing()) -- RX and
//     TX share TAB5_RF433T_PIN/TAB5_RF433R_PIN (both GPIO53) and the GPIO53
//     arbiter's Owner::kRf433 token cannot by itself tell them apart (it is
//     idempotent per-owner, and RX+TX are the same owner against I2C); this
//     is the explicit second check that actually keeps a live capture's
//     attachInterrupt() and a transmit's pinMode(OUTPUT)/digitalWrite() off
//     the pin at the same time. See rf433_common.cpp's capture_start() for
//     the mirror-image check and the full hazard writeup.
//   - sig.edge_count is 0
//   - the GPIO53 arbiter (hal/gpio53_arbiter.h) is currently held by the
//     external I2C bus (an NFC/RFID2 session in progress)
//   - the background task fails to start (xTaskCreate() out-of-memory)
//
// Does NOT refuse on sig.truncated: a truncated capture's edges[] only ever
// drops the TAIL of a burst (see rf433_scan.cpp's kMaxEdgesPerSignal
// comment), so the recorded prefix is still a valid partial reconstruction.
// Real captures truncate often enough (Task 1 measured 512 edges in 509ms of
// sustained transmission -- any button held past ~0.5s hits the cap) that
// outright refusal left ordinary real-world captures unreplayable with no
// user remedy. transmit() proceeds and replays exactly the edges it has;
// last_transmit_was_truncated() reports it so the UI can show a warning
// instead of silently pretending the replayed burst was complete.
void transmit(const Rf433Scan::CapturedSignal &sig);

// True from a successful transmit() launch until poll() has processed the
// task's completion (i.e. true for the whole "Transmitting..." window).
// Also consulted by rf433_common.cpp's capture_start() as the RX-side half
// of the RX/TX exclusion described above.
bool is_busy();

// True if the most recently STARTED transmit() call was of a signal flagged
// sig.truncated -- i.e. the burst just transmitted (or currently
// transmitting) is only the captured prefix, not the whole original burst.
// Valid until the next transmit() call, same lifetime contract as
// failure_reason(). Meant for the UI to render a "partial replay" warning
// alongside State::kTransmitting/kDone -- this is not a failure, so it is
// not folded into failure_reason()/State::kFailed.
bool last_transmit_was_truncated();

// Current status, for UI polling. Persists (kDone/kFailed) until the next
// transmit() call.
State state();

// Human-readable reason for the most recent State::kFailed. Empty string
// otherwise. Valid until the next transmit() call.
const char *failure_reason();

// Called from main.cpp's loop(). No-ops unless a transmit task is in
// flight or has just finished. On completion, releases the GPIO53 arbiter
// claim taken by transmit() -- Gpio53Arbiter::claim()/release() are
// main-task-only (see hal/gpio53_arbiter.h), so this cannot happen from the
// background transmit task itself.
void poll();

} // namespace Rf433Replay
