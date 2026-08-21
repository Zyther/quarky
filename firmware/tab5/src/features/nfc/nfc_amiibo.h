#pragma once

// ===========================================================================
// NFC Forum Type 2 (Ultralight-family) read/write on the RFID2 unit --
// user-facing "Amiibo" module (Phase 3 Task 11): dump an NTAG21x tag (most
// commonly encountered inside an Amiibo figure, an NTAG215), then optionally
// write that same dump back onto another writable tag of the same family.
//
// CORRECTED PREMISE, same failure class as Task 3/Task 4's donor mixups: the
// plan originally said to port Bruce's `ESP-Amiibolink`. That is a NimBLE
// CLIENT for a separate physical commercial device ("Amiibolink") over BLE --
// it contains no NFC/ISO14443 tag I/O at all and was not touched or
// referenced building this module. The real, working donor is Bruce's own
// `~/src/firmware/src/modules/rfid/RFID2.cpp`, whose Ultralight/NTAG21x path
// (no MIFARE Classic-style crypto, unlike this project's own
// nfc_mifare_crack.cpp) is ported here. See nfc_amiibo.cpp's header comment
// for the exact file:line citations.
//
// RFID2 UNIT ONLY, same scoping decision nfc_mifare_crack.cpp made and for
// the same reason: this is a proven-working composition against
// Ws1850sDriver/MFRC522_I2C. The plan's Step 1 also asks whether the NFC unit
// (ST25R3916) can address NTAG21x -- St25r3916 is a general 13.56MHz reader
// IC so likely can, but wiring that path is left as an explicit stretch goal
// / follow-up, not attempted here, so as not to duplicate a second read/write
// stack before the first is verified on real hardware.
// ===========================================================================

namespace NfcAmiibo {

// Registers the "RFID2: Amiibo (NTAG21x)" launcher tile (Category::NFC,
// Affinity::TAB5_NATIVE).
void register_module();

// Called from main.cpp's loop(). No-ops unless this screen is open.
void poll();

} // namespace NfcAmiibo
