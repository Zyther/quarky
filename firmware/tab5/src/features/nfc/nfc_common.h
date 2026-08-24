#pragma once

#include <cstddef>
#include <cstdint>

namespace NfcCommon {

// sak/atqa added by Phase 3 Task 24 (NFC tag emulation): Listen Mode
// emulation needs the tag's real SEL_RES (SAK) and SENS_RES (ATQA) to answer
// a reader's anticollision/SELECT correctly, not just its UID. Additive to
// the original Task 10 layout (every member is still byte-sized, so
// sizeof(TagInfo) still has no compiler padding) -- nfc_tag_library.cpp's
// save()/load() are generic over sizeof(TagInfo) and need no changes, but
// this DOES change that sizeof(), so a .tag file saved before this change is
// 3 bytes shorter and will now fail load()'s existing exact-size check
// (nfc_tag_library.cpp's own documented behavior for any size mismatch) --
// old library entries need a re-scan/re-save, not a migration, for this
// research firmware.
//
// atqa == {0, 0} is a real, checkable sentinel for "not captured": no valid
// ISO14443-3 ATQA is ever 0x0000 (bit 5 or 6 of the low byte is always set to
// indicate UID size -- see ISO/IEC 14443-3 Table 3), so it round-trips
// through save()/load() without needing a separate "is this valid" flag.
// Populated for real by the NFC-unit read path (nfc_read.cpp's
// try_read_nfc_uid(), which already has both bytes from
// St25r3916::nfca_detect()); left at this sentinel by the RFID2/WS1850S path
// (try_read_rfid2_uid()), whose donor library (MFRC522_I2C's Uid struct) never
// exposes the ATQA it received during PICC_RequestA() -- only the UID and SAK
// survive past PICC_ReadCardSerial(). sak has no equivalent ambiguity: 0x00 is
// simultaneously "not captured" and a real, valid SAK value (Note 1 of
// ISO/IEC 14443-3 Table 6.4: bit 3 clear across the whole byte, including
// 0x00, legitimately means "UID complete, not compliant with ISO/IEC 14443-4"
// -- ordinary MIFARE Ultralight tags report exactly this), so it is NOT a
// usable sentinel; both real save paths always populate it directly from
// their own already-verified SAK byte (s_mfrc.uid.sak / tag.sak).
struct TagInfo {
    uint8_t uid[10];
    uint8_t uid_len;
    char type_name[24];
    uint8_t sak;      // SEL_RES, real for both save paths.
    uint8_t atqa[2];  // SENS_RES, wire order (LSB first); {0,0} if not captured.
};

// Formats uid as "04:A3:F1:..." (matching the project’s hex-with-colons style).
// Returns out for convenience (and never returns nullptr).
const char *format_uid(const uint8_t *uid,
                        uint8_t len,
                        char *out,
                        size_t out_len);

} // namespace NfcCommon

