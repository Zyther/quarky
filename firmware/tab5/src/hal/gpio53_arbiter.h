#pragma once

// Runtime ownership arbiter for the Tab5's single HY2.0-4P PORT.A
// connector -- shared between the external I2C bus (NFC/RFID2,
// hal/nfc_pn532.cpp), RF433 capture/transmit (features/rf433/
// rf433_common.cpp), and (added 2026-08-21) the IR unit (hal/ir_unit.cpp),
// because all three are wired to the same physical GPIO53/54 pins on that
// one connector. See boards/tab5/pins_config.h for the hardware story and
// features/rf433/rf433_common.cpp's header comment for the concurrent-
// access hazard this exists to close.
//
// Despite the file's name (kept for history -- GPIO53 specifically is
// where the original NFC-vs-RF433 conflict was diagnosed, see
// pins_config.h's real-hardware findings), this arbitrates ownership of
// the WHOLE port as one exclusive resource, not pin 53 in isolation:
// RF433 uses only GPIO53 (both TAB5_RF433R_PIN and TAB5_RF433T_PIN, a real
// hardware finding -- see pins_config.h), while external I2C uses
// GPIO53(SDA)/54(SCL), and the IR unit (real M5Stack "Unit IR" SKU U002,
// confirmed via its own datasheet -- see hal/ir_unit.h -- to be a plain-
// GPIO transceiver, NOT I2C) needs both GPIO53(IR_TX)/54(IR_RX). A single
// Owner variable is correct for all three because no two of them can ever
// coexist on the connector regardless of which specific pin(s) each one
// touches.
//
// Main-task only. Neither claim() nor release() is ISR-safe or
// intended to be called from one -- confirmed as of this file's
// writing that nothing calls into this from Rf433Common's ISR
// (isr_edge() only touches its own ring buffer), and that must stay
// true for any future caller too.
namespace Gpio53Arbiter {

enum class Owner { kNone, kExternalI2c, kRf433, kIr };

// Claims the pin for `owner`. Returns true if `owner` now holds it --
// either it just claimed a free pin, or it already held it (idempotent,
// matching this project's established "refuse rather than lie"
// convention, e.g. Rf433Common::capture_start()'s own idempotent
// re-arm). Returns false, and leaves the current owner untouched, if
// the pin is held by the OTHER owner -- callers must treat false as a
// real refusal, not attempt the underlying pin operation anyway.
bool claim(Owner owner);

// Releases the pin if `owner` currently holds it. No-op (not an error)
// if `owner` doesn't hold it -- safe to call defensively from a
// teardown path that isn't sure whether claim() ever succeeded.
void release(Owner owner);

Owner current_owner();

} // namespace Gpio53Arbiter
