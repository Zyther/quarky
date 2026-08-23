#pragma once

// ===========================================================================
// IR jammer (Phase 3 Task 19): continuous IR "noise" transmit intended to
// saturate/blind a real IR receiver in range, the same real capability
// class as this project's own RF433/BLE jam-adjacent features (BLE Flood,
// RF433 bruteforce's continuous candidate stream) applied to the IR domain.
//
// REAL DESIGN: a real IR receiver (this project's own IR unit uses an
// IRM-3638T -- see hal/ir_unit.h) demodulates a 38kHz-ish carrier into a
// mark/space digital signal by expecting a roughly-periodic carrier
// envelope matching real remote-control protocol timing (hundreds of
// microseconds to a few milliseconds per mark/space segment -- see
// features/ir/ir_nec_encode.h's own real NEC timing citations). Continuously
// re-transmitting SHORT, RANDOMIZED mark/space durations at the same real
// 38kHz carrier keeps the receiver's demodulator/AGC saturated with
// carrier activity that never resolves into a coherent, decodable frame --
// there is no need to mimic any specific real protocol's exact structure,
// only to keep the carrier active in a pattern real receivers can't filter
// out as periodic noise. Real carrier frequency/duty-cycle citation:
// features/ir/ir_nec_encode.h's kCarrierHz (38000, the well-established
// consumer-IR figure this project already cites in three other IR
// modules without a chip-specific measurement) and 1/3 duty cycle (same
// established default ir_tvbgone.cpp/ir_learn.cpp/ir_clone.cpp already
// use for the identical reason -- this receiver class is broadly
// duty-cycle-tolerant).
//
// NON-BLOCKING BY DESIGN (same lesson as every other IR feature this
// session, ir_common.h's own header comment on Task 15's real
// watchdog-reset finding): one IrCommon::transmit_raw() call is bounded
// and short (a burst of at most kBurstDurations short segments, well
// under the ~50ms real bursts this project's other IR features already
// transmit without incident), but "continuous" jamming needs MANY such
// bursts back to back for as long as the user leaves this running --
// poll()-driven, one bounded burst per tick, never a loop over many
// bursts in one call.
// ===========================================================================

namespace IrJammer {

// Registers this module's launcher tile (Category::IR, Affinity::
// TAB5_NATIVE). Call once from setup(), before Shell::build().
void register_module();

// Called from main.cpp's loop(). No-ops unless a jam session is active
// (Start has been tapped and Stop hasn't been tapped since).
void poll();

} // namespace IrJammer
