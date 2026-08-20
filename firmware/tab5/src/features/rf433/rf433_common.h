#pragma once
#include <cstdint>
#include <cstddef>

// ===========================================================================
// Interrupt-driven 433MHz edge-timing capture (Phase 3, Task 1).
//
// This is the real, reusable receive front-end every later RF433 feature
// (scan, protocol decode, bruteforce/replay) is built on -- not spike-only
// throwaway code. Its first job, though, IS a spike: settling whether
// TAB5_RF433R_PIN (GPIO53) is really where the RF433R unit's data line
// lands. See pins_config.h for the full pin-research trail.
//
// WHY interrupts and not polling: Phase 1's same-day attempt to verify the
// R pin polled digitalRead() from loop(). Real OOK/ASK fixed-code remotes
// send pulses in the hundreds-of-microseconds range; a loop() that also
// runs LVGL, C2 polling, and a delay(5) samples orders of magnitude too
// slowly and too irregularly to see them. That "no signal" result is
// therefore a likely false negative, not evidence the pin is wrong. An
// attachInterrupt(CHANGE) handler timestamping edges with micros() is the
// industry-standard technique for this (it is what rc-switch -- the library
// Bruce's rf_scan.cpp is built on -- does), and is what this file
// implements.
// ===========================================================================

namespace Rf433Common {

struct EdgeSample {
    uint32_t timestamp_us;
    bool level; // GPIO level AFTER the edge (i.e. the level this sample transitioned TO)
};

// Starts capturing GPIO edges on TAB5_RF433R_PIN via attachInterrupt(CHANGE).
// Safe to call while a capture is already running (no-op, matching this
// project's established "refuse rather than lie" idempotent-start
// convention, e.g. BleHidSpike::start()). Returns true once a capture is
// active.
//
// Can also return false: TAB5_RF433R_PIN (GPIO53) is shared with the
// external I2C bus (NFC/RFID2, see hal/gpio53_arbiter.h), and this function
// claims that pin via Gpio53Arbiter before touching it. If an NFC/RFID2
// session currently owns GPIO53, the claim is refused and this returns false
// WITHOUT calling pinMode()/attachInterrupt() -- no capture is started, and
// the pin is left untouched. Callers must check the return value and must
// not assume a capture is running just because this was called.
//
// Also returns false, again WITHOUT touching the pin, if a Task 6 replay
// (Rf433Replay::transmit()) is currently transmitting. Gpio53Arbiter::claim()
// is idempotent per-OWNER (both capture and replay claim the same
// Owner::kRf433 token, since they're one subsystem sharing exclusivity
// against the external I2C bus, not against each other) -- so the arbiter
// alone cannot tell RX and TX apart, and this explicit check is what
// actually keeps a capture's attachInterrupt() handler and a replay's
// pinMode(OUTPUT)/digitalWrite() from touching the same physical pin at the
// same time. See rf433_replay.cpp's transmit() for the mirror-image check on
// the transmit side, and this file's .cpp for why that check lives THERE and
// this one lives HERE (in capture_start(), not Rf433Scan::set_capture_active()).
bool capture_start();

// Stops the interrupt handler. Safe to call whether or not a capture is
// running.
void capture_stop();

// True while an edge capture is active. Lets a caller (e.g. main.cpp's
// serial trigger) implement a single-key start/stop toggle without keeping
// its own duplicate copy of this state.
//
// NOTE: this stays true even after the ISR has self-disarmed on the edge
// ceiling (see overrun()) -- the module was never told to stop, it just
// stopped collecting. Callers that poll should check overrun() too.
bool is_capturing();

// True if this capture session hit the hard edge ceiling and the ISR masked
// its own interrupt source to stop servicing it. A caller seeing this should
// call capture_stop() to complete the teardown on the main task (the ISR
// deliberately does the minimum, not the full detach) and should treat the
// capture as suspect: at the rates real 433MHz traffic produces, reaching
// the ceiling means either the interrupt was left armed far longer than any
// legitimate capture, or the pin is seeing something that is not signal.
// Cleared by the next capture_start().
bool overrun();

// Total edges this session's ISR has serviced since capture_start(). Unlike
// the ring's occupancy this is NOT reset by capture_read() -- it is the
// runaway guard's odometer, and is useful for reporting the real interrupt
// rate a capture saw.
uint32_t edges_this_capture();

// Copies up to max samples out of the ring buffer into out, oldest first,
// and consumes exactly what it copied. Returns the number actually copied.
// A short read (max smaller than the number of unread samples) keeps the
// remaining, NEWER samples queued for the next call rather than discarding
// them, so repeated draining during a single open capture returns a
// continuous, in-order, gap-free stream -- calling this more than once per
// capture session is explicitly supported and is what the later
// scan/decode features do.
// Call from poll() / the main task -- see this function's .cpp for
// the portMUX_TYPE critical section that makes this safe against the
// interrupt handler. The handler runs in ISR context, not a FreeRTOS task,
// so it uses portENTER_CRITICAL_ISR while this function uses the matching
// non-ISR portENTER_CRITICAL -- the standard ESP-IDF pairing for a buffer
// shared between an ISR and a task.
size_t capture_read(EdgeSample *out, size_t max);

// Ring capacity, exposed so callers can size their drain buffer without
// guessing (and so a caller can tell "I read 512" from "the buffer
// overflowed and I lost the oldest edges" -- see capture_read()'s .cpp
// note on overflow behavior).
size_t capture_capacity();

} // namespace Rf433Common
