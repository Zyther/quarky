#pragma once

#include "cc1101_scan.h"

// ===========================================================================
// CC1101 replay (Phase 10 Task 4b): bit-bangs Cc1101Hw::gdo0_pin() (the
// CC1101 put into async/direct TX mode via Cc1101Hw::enable_async_tx()) to
// reproduce a Cc1101Scan::CapturedSignal's real edge timing via
// digitalWrite() + delayMicroseconds() -- the same technique Bruce's
// rf_send.cpp/rc-switch and Poseidon's subghz_scan.cpp 'r' handler
// (digitalWrite(CC1101_GDO0, ...) + delayMicroseconds(abs(pulse))) both use,
// applied through RadioLib's async-direct-mode GDO0 instead of a bare GPIO.
//
// Architecture mirrors firmware/tab5/src/features/rf433/rf433_replay.cpp
// exactly (see that file's own header comment for the full real-hardware
// crash citation this follows): the bit-bang loop runs on a dedicated,
// core-pinned, elevated-priority FreeRTOS task -- never synchronously inside
// a click handler or a poll() tick -- because a captured signal can
// legitimately take hundreds of milliseconds to replay and microsecond
// pulse timing cannot survive being interleaved with LVGL's own render tick
// on the same core. transmit() returns immediately; poll() (called from
// main.cpp's loop()) drains the task's completion and reports it via
// state()/failure_reason().
// ===========================================================================

namespace Cc1101Replay {

enum class State { kIdle, kTransmitting, kDone, kFailed };

// Starts transmitting `sig` from a dedicated FreeRTOS task. Returns
// immediately. Refuses (state() -> kFailed, nothing started) if:
//   - a transmit is already in flight
//   - Cc1101Scan::is_capturing() is true (RX/TX share the same physical
//     GDO0 pin -- see cc1101_scan.h's is_capturing() doc comment)
//   - sig.edge_count == 0
//   - Cc1101Hw::init()/enable_async_tx() fails
//   - the background task fails to start (xTaskCreate() out-of-memory)
//
// Does NOT refuse on sig.truncated -- same reasoning as rf433_replay.h's
// transmit(): truncation only ever drops the TAIL of a burst, so the
// recorded prefix is still a valid partial reconstruction.
void transmit(const Cc1101Scan::CapturedSignal &sig);

bool is_busy();
bool last_transmit_was_truncated();
State state();
const char *failure_reason();

// Called from main.cpp's loop(). No-ops unless a transmit task is in flight
// or has just finished.
void poll();

} // namespace Cc1101Replay
