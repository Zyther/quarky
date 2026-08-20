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
//      registered ("No launcher tile of its own -- it is a component, not a
//      feature").
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
//   - sig.truncated is set -- a truncated capture's edges[] do not
//     represent the real burst, so this refuses rather than silently
//     replaying a partial signal as if it were complete
//   - sig.edge_count is 0
//   - the GPIO53 arbiter (hal/gpio53_arbiter.h) is currently held by the
//     external I2C bus (an NFC/RFID2 session in progress)
//   - the background task fails to start (xTaskCreate() out-of-memory)
void transmit(const Rf433Scan::CapturedSignal &sig);

// True from a successful transmit() launch until poll() has processed the
// task's completion (i.e. true for the whole "Transmitting..." window).
bool is_busy();

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
