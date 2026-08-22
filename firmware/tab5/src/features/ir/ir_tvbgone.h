#pragma once

// ===========================================================================
// TV-B-Gone (Phase 3 Task 16): sweeps the real Gen3 TV-B-Gone NA/EU power-off
// code database (world_ir_codes.h) over the IR unit's TX LED, transmitting
// each code in turn via ir_common.h's RMT-based IrCommon::transmit_raw().
//
// Real decode algorithm (bit-packed `codes[]` index stream ->
// `times[]`-indexed mark/space duration pairs) is cross-validated against
// TWO independent donor checkouts (Bruce's TV-B-Gone.cpp and UniGeek's
// IRUtil.cpp), which implement the identical unpacking despite each then
// handing the result to a different transmit backend (IRremoteESP8266's
// `IRsend::sendRaw()`) this project doesn't have -- see world_ir_codes.h's
// provenance header for the full citation trail.
//
// NON-BLOCKING BY DESIGN: one code is decoded+transmitted per poll() tick,
// gated by a real inter-code gap (kInterCodeGapMs, cited from both donors'
// own 205ms delay between codes) rather than looping over the whole
// database in one call. This directly applies the lesson from a real
// hardware crash Task 15's own bring-up spike caused (see hal/ir_unit.h):
// any operation holding loop() for multiple seconds straight trips the
// ESP32 Arduino task watchdog. A full NA sweep (137 codes) at ~205ms/code
// plus each code's own sub-100ms transmit time is on the order of 30+
// seconds total -- doing that in one blocking call would be the exact same
// mistake at a much longer timescale.
// ===========================================================================

namespace IrTvbGone {

// Registers this module's launcher tile (Category::IR, Affinity::
// TAB5_NATIVE). Call once from setup(), before Shell::build().
void register_module();

// Called from main.cpp's loop(). No-ops unless a sweep is active (Start has
// been tapped and the sweep hasn't finished/been stopped/failed yet).
void poll();

} // namespace IrTvbGone
