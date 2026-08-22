#pragma once

#include "ir_file_format.h"
#include <cstddef>
#include <cstdint>

// ===========================================================================
// IR receive/capture (Phase 3 Task 17). RAW capture only -- no named-protocol
// (NEC/RC5/Sony) bit-level decode. See this project's plan doc
// (docs/superpowers/plans/2026-08-18-phase3-nfc-rf433-ir-plan.md, Task 17
// section, 2026-08-21 note) for why: the plan's own "Produces" line already
// specified a raw pulse-width struct, not a decoded address/command struct,
// despite that section's Context paragraph mentioning protocol decode --
// this module resolves that inconsistency in favor of the Produces line,
// deliberately, because NEC/RC5/Sony bit-level decode needs real-hardware
// timing-threshold tuning this pass isn't scoped for.
//
// REAL HARDWARE THIS TARGETS (see hal/ir_unit.h for the full citation
// trail): M5Stack "Unit IR" SKU U002's RX side is an IRM-3638T
// (Everlight Electronics) -- a 38kHz-carrier-DEMODULATING receiver module.
// Its output pin (TAB5_IR_RX_GPIO / GPIO54) is already a clean digital
// mark/space signal with the 38kHz carrier stripped in hardware -- Task 15's
// real capture confirmed no 38kHz sub-structure, just clean level
// transitions matching button-press timing. This module must NOT enable
// RMT's own RX carrier demodulation (that would demodulate an
// already-demodulated signal -- see ir_learn.cpp's init comment for the
// verified-against-source reasoning).
//
// NON-BLOCKING BY DESIGN (same lesson as ir_common.h/ir_tvbgone.cpp cite
// from Task 15's real bring-up crash, hal/ir_unit.h): waiting for a human to
// press a real remote button can take any amount of time. start_capture()
// arms the RMT RX channel via rmtReadAsync() and returns immediately;
// poll() (called every loop() tick, same as every other feature module)
// checks rmtReceiveCompleted() and only then does any work. Nothing in this
// module ever blocks on the capture itself.
// ===========================================================================

namespace IrLearn {

// Sized to IrFileFormat::kMaxRawSamples (the real Flipper .ir format's own
// stated 1024-element cap for a raw signal's `data`) -- this capture only
// exists to be saved via that format (IrFileFormat::write(), see
// to_ir_signal() below), so there is no reason to ever hold more pulses in
// memory than a saved file could represent anyway.
constexpr size_t kMaxPulses = IrFileFormat::kMaxRawSamples;

// Plan's own Task 17 "Produces" line names this struct/fields
// (pulse_widths_us[kMaxPulses], count) -- kept verbatim for interface
// stability with whatever Task 18 (clone, out of scope this pass) expects.
// Extended (not just the plan's two fields) so a completed capture is
// trivially convertible to IrFileFormat::IrSignal (see to_ir_signal() below)
// without a second, incompatible raw-signal representation existing in this
// codebase -- carrier_hz/duty_cycle/truncated all map 1:1 onto IrSignal's
// own kRaw fields.
struct LearnedCode {
    uint16_t pulse_widths_us[kMaxPulses]; // alternating mark/space, mark
                                           // first -- same convention as
                                           // IrCommon::transmit_raw() and the
                                           // real .ir raw `data` field
    size_t count;

    // Neither of these is measurable from an already-demodulated RX signal
    // (see hal/ir_unit.h's IRM-3638T citation -- the carrier is stripped
    // before it ever reaches GPIO54). Both are placeholders describing "the
    // typical consumer-IR transmitter this was probably captured from", not
    // a measurement -- see ir_learn.cpp's kAssumedCarrierHz/
    // kAssumedDutyCycle for the citations.
    uint32_t carrier_hz;
    float duty_cycle;

    // True if the real burst had more pulses than kMaxPulses -- excess
    // dropped. Mirrors Rf433Scan::CapturedSignal::truncated's naming/
    // semantics.
    bool truncated;
};

enum class State { kIdle, kWaiting, kDone, kFailed };

// Registers this module's launcher tile (Category::IR, Affinity::
// TAB5_NATIVE). Call once from setup(), before Shell::build().
void register_module();

// Called from main.cpp's loop(). No-ops unless a capture is armed
// (Start Capture has been tapped and rmtReceiveCompleted() hasn't fired
// yet).
void poll();

// Current state of the most recent capture attempt (kIdle if none has ever
// been started this session).
State state();

// Valid only when state() == State::kDone. Behavior otherwise (kIdle/
// kWaiting/kFailed) is an empty/zeroed LearnedCode -- callers should check
// state() first, matching this project's existing "check before reading"
// idiom (e.g. Rf433Replay::last_transmit_was_truncated()'s doc comment).
const LearnedCode &result();

// Valid only when state() == State::kFailed. Empty string otherwise.
const char *error_reason();

// Converts a captured LearnedCode into an IrFileFormat::IrSignal (type ==
// SignalType::kRaw) for saving via IrFileFormat::write() -- so this
// capture's raw pulses can be written straight into the real Flipper .ir
// format Task 21 already implements, rather than inventing a second,
// incompatible on-disk representation. `name` is copied into the output
// signal's name field (truncated to IrFileFormat::kNameMaxLen - 1 if
// longer). Returns false if code.count == 0 (nothing captured yet).
bool to_ir_signal(const LearnedCode &code, const char *name, IrFileFormat::IrSignal *out);

} // namespace IrLearn
