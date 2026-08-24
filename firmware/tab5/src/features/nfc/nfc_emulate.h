#pragma once

#include "nfc_common.h"

// ===========================================================================
// NFC tag emulation / Listen Mode (Phase 3 Task 24). NFC unit (ST25R3916)
// only -- the RFID2/WS1850S unit is genuinely reader-only silicon (its real
// command set, lib/MFRC522_I2C's PCD_Command enum, has no target/card-
// emulation command at all; see docs/superpowers/plans/
// 2026-08-18-phase3-nfc-rf433-ir-plan.md's Task 24 Context bullets for the
// full confirmation trail). Built entirely on this project's own
// st25r3916_driver.{h,cpp} Listen Mode addition (St25r3916::listen_start()/
// listen_poll()/listen_stop()) -- see that file's "NFC-A Listen Mode" SOURCES
// section for the real register/protocol citations.
//
// SCOPE: read-only UID/SAK/ATQA emulation of a tag already saved in Task 10's
// tag library (NfcTagLibrary). Once a reader completes SELECT this reports
// "selected" and stops there -- answering whatever the reader sends next
// (RATS, READ, etc.) is full memory-content emulation, the plan's own
// explicitly out-of-scope stretch goal, and is NOT implemented.
//
// NOT a standalone main-menu tile: unlike every other NFC feature module in
// this project, there is no register_module() here. This screen is reached
// only via the "Emulate" button nfc_tag_library_ui.cpp's saved-tag detail
// view adds (start() is called directly from that button's click handler,
// the same way "tap a saved tag" already calls show_tag() inline without its
// own top-level tile) -- there is nothing to independently launch this
// screen from the main menu, since it always needs a tag chosen from the
// library first. poll() still needs wiring into main.cpp's loop() exactly
// like every other feature's poll(), so the live "waiting for reader" /
// "selected!" status updates while this screen is open.
// ===========================================================================

namespace NfcEmulate {

// Pushes the emulate screen for `tag` and arms Listen Mode with its
// UID/SAK/ATQA on the next poll() tick (not synchronously -- same
// one-shot-bring-up-on-the-next-tick discipline nfc_read.cpp's run_bring_up()
// already uses, so this click handler itself never blocks).
void start(const NfcCommon::TagInfo &tag);

// Called from main.cpp's loop(). No-ops unless this screen is open.
void poll();

} // namespace NfcEmulate
