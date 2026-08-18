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
bool capture_start();

// Stops the interrupt handler. Safe to call whether or not a capture is
// running.
void capture_stop();

// True while an edge capture is active. Lets a caller (e.g. main.cpp's
// serial trigger) implement a single-key start/stop toggle without keeping
// its own duplicate copy of this state.
bool is_capturing();

// Copies up to max samples out of the ring buffer into out, in the order
// they were captured, and clears the buffer. Returns the number actually
// copied. Call from poll() / the main task -- see this function's .cpp for
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
