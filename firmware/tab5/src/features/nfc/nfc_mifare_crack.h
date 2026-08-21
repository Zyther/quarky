#pragma once

// ===========================================================================
// MIFARE Classic key recovery on the RFID2 unit (Phase 3 Task 9).
//
// Four real attacks, all ported from UniGeek's `firmware/src/utils/nfc/`
// (one of this program's three authorized donor firmwares, see CLAUDE.md),
// driving the WS1850S through the same vendored MFRC522_I2C library
// features/nfc/nfc_read.cpp already uses:
//
//   * Dictionary   -- try a built-in list of well-known keys (plus any
//                     .txt dictionaries on the SD card) against every
//                     sector/key-type. No prior knowledge needed; only finds
//                     keys that are actually in the list.
//   * Nested       -- needs ONE already-known key on ANY sector. Collects
//                     encrypted nested nonces, sweeps the 65535 PRNG
//                     distances, recovers the target key with
//                     lfsr_recovery32().
//   * Static nested-- same, for cards whose tag nonce never changes.
//   * "Darkside"   -- see the honesty note in nfc_mifare_crack.cpp's
//                     DARKSIDE section: the donor's implementation is a
//                     parity-oracle-filtered DICTIONARY attack, not the
//                     Courtois darkside key-recovery attack its name
//                     implies. It cannot recover a key that is not in the
//                     built-in list or an SD dictionary.
//
// RFID2 UNIT ONLY. MIFARE Classic authentication is an MFRC522/PN512-class
// register operation (PCD_MFAuthent + CRYPTO1 in the reader silicon); the
// Phase 3 spec explicitly flags the ST25R3916/NFC-unit path as unconfirmed
// for this, and nothing here touches St25r3916.
//
// THREADING, AND WHY IT IS NOT A PER-TICK LOOP. See the "EXECUTION MODEL"
// block at the top of nfc_mifare_crack.cpp: the attacks run on a dedicated
// FreeRTOS worker task and poll() streams their progress into the screen.
// A single dictionary key try is already ~200 ms (the donor's own card-reset
// sequence between failed authentications is two 100 ms delays), i.e. four
// times this project's ~50 ms poll() budget on its own, and lfsr_recovery32()
// is a multi-second uninterruptible block that cannot be chunked without
// rewriting the ported algorithm.
// ===========================================================================

namespace NfcMifareCrack {

// Registers this module's launcher tile (Category::NFC, Affinity::
// TAB5_NATIVE). Call once from setup(), before Shell::build() -- same
// convention as every other register_module() in this codebase.
void register_module();

// Called from main.cpp's loop(). No-ops unless the screen is open or a run
// is in flight. Owns everything that is main-task-only: the GPIO53 arbiter
// claim (via Ws1850sDriver::init() -> nfc_ensure_external_i2c_begun()), the
// worker task's launch and reaping, and every LVGL write.
void poll();

} // namespace NfcMifareCrack
