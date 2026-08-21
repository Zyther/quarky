#pragma once

#include <cstddef>
#include <cstdint>

// ===========================================================================
// IR unit HAL (Phase 3 Task 15). Real M5Stack "Unit IR", SKU U002.
//
// CORRECTED 2026-08-21 (see this plan's Task 15 section and the SDD ledger
// for the full trail): this HAL was originally going to be an I2C driver, on
// the same wrong inherited premise CLAUDE.md itself carried. A real I2C
// census against the physically-connected unit came back completely empty
// -- the CORRECT result, because this hardware is not I2C at all.
//
// REAL HARDWARE, per the project owner-supplied datasheet
// (~/Downloads/ir.pdf) and a follow-up screenshot of M5Stack's own
// unit-compatibility page (both 2026-08-21):
//   - HY2.0-4P pinout: Black=GND, Red=5V, Yellow=IR_TX, White=IR_RX -- a
//     plain 4-wire GPIO transceiver, no I2C lines.
//   - TX side: a bare transistor (SS8050) driving a 940nm IR LED
//     (R1=150ohm, R2=1Kohm, R3=4.7Kohm) -- no chip, nothing to identify.
//   - RX side: IRM-3638T (Everlight Electronics Co., Ltd.) -- a real,
//     independently-verified part (confirmed via WebSearch against real
//     distributor listings: DigiKey, Arrow, LCSC, AllDatasheet), a
//     38kHz-carrier-demodulating IR receiver module (PIN diode +
//     preamplifier + demodulator in one epoxy-packaged part, TSOP38-family
//     equivalent) with a single digital OUT pin -- no register interface,
//     just a GPIO level to read.
//   - The datasheet's own pin-map table calls the unit-side connector
//     "PORT.B" (M5Stack's generic per-unit convention for a device's own
//     connector identity) -- NOT a claim that Tab5 has a second physical
//     connector. Tab5 has exactly ONE HY2.0-4P connector, PORT.A
//     (confirmed, boards/tab5/pins_config.h). Project owner, directly:
//     "it does not, but port A can be used as simple GPIO 'port B'".
//   - Wire-to-GPIO mapping (M5Stack unit-compatibility page screenshot):
//     Unit IR's PORT.B (GND/5V/IR_TX/IR_RX) <-> Tab5 PORT.A
//     (GND/5V/G53/G54) -- IR_TX=GPIO53, IR_RX=GPIO54. See pins_config.h's
//     TAB5_IR_TX_GPIO/TAB5_IR_RX_GPIO for the macros this maps to.
//
// This makes the IR unit a THIRD real consumer of PORT.A's GPIO53/54 pins,
// alongside external I2C (nfc_pn532.cpp) and RF433 (rf433_common.cpp,
// rf433_gpio.cpp) -- arbitrated the same way, via
// hal/gpio53_arbiter.h's Owner::kIr. Every entry point below claims before
// touching a pin and releases when done; see gpio53_arbiter.h's own header
// for why a stray unclaimed pinMode()/digitalWrite() on this pin is a real,
// previously-hit hazard (documented at length in rf433_gpio.cpp's header),
// not theoretical caution.
//
// RX POLARITY: IRM-36xx/TSOP38-family modules are, as a whole part family,
// universally active-low, open-collector-style outputs with an internal
// pull-up -- idle HIGH, pulled LOW for the duration a demodulated 38kHz
// carrier burst is present (this is how essentially every 38kHz IR-remote
// receiver module of this class behaves; it is what lets a bare
// digitalRead() reconstruct the original remote's mark/space timing
// without the receiving MCU doing any of its own carrier demodulation).
// This is the family-standard behavior, not yet independently re-confirmed
// against IRM-3638T's own primary datasheet on THIS specific board's
// wiring -- Task 15 Step 3's real GPIO-level bring-up test (a real remote,
// a real digitalRead() trace) is what turns this from "standard for the
// family" into "confirmed for this unit", and this comment should be
// updated with that result once run.
// ===========================================================================

namespace IrUnit {

// Claims Gpio53Arbiter::Owner::kIr and configures TAB5_IR_TX_GPIO as an
// OUTPUT (idle LOW) and TAB5_IR_RX_GPIO as an INPUT. Returns false (no pin
// touched) if the port is currently held by another owner (NFC/RFID2 or
// RF433) -- callers must treat false as a real refusal, matching every
// other Gpio53Arbiter consumer's convention.
bool begin();

// Releases the Owner::kIr claim if this module holds it. Safe to call
// defensively even if begin() was never called or already failed.
void end();

// Direct TX pin control for the bring-up presence test (Step 3a): drives
// TAB5_IR_TX_GPIO HIGH or LOW. No carrier modulation -- that is Task 16's
// (ir_common.cpp) scope, not this bring-up spike's.
void set_tx(bool level);

// Direct RX pin read for the bring-up presence test (Step 3b): returns the
// current level of TAB5_IR_RX_GPIO. Per this file's RX POLARITY note above,
// expect LOW while a real remote's button is held (family-standard
// active-low), HIGH at idle -- confirm this, don't assume it, when running
// the real test.
bool read_rx();

} // namespace IrUnit
