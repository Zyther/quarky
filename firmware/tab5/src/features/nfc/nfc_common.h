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
//
// page_count/pages added 2026-08-24 by Task 24's content-emulation extension,
// under the exact same additive, byte-sized discipline (and with the exact
// same disclosed consequence) as the sak/atqa addition described above: a
// .tag file written before this change is shorter than the new sizeof() and
// now fails nfc_tag_library.cpp's existing exact-size load() check, so old
// library entries need a re-scan/re-save rather than a migration.
//
// WHY they exist at all: Listen Mode emulation that answers only
// anticollision/SELECT is useless in practice -- confirmed on real hardware
// on 2026-08-24 against two real external readers (an iPhone running NFC
// Tools and a Chameleon Ultra). Both walked the emulated tag's full
// anticollision/SELECT sequence successfully (the chip's PTA state register
// reliably reached its "active" state) and then BOTH gave up and re-polled,
// because every real reader reads something back before it declares a tag
// found -- and this project had no captured page content to answer with.
// See st25r3916_driver.cpp's "NFC Forum Type 2 Tag page read" and Listen
// Mode sections for the real capture/responder halves.
//
// SIZING: kMaxT2tPages is 231, which is NTAG216's real page count -- the
// largest tag in the NTAG21x family that either of this project's two
// already-ported real donor detections recognises (nfc_amiibo.cpp's
// cc_tag_name()/page_count_tag_name(), both citing RFID2.cpp:434-447's
// Capability-Container table: 0x12 -> NTAG213/45 pages, 0x3E -> NTAG215/135,
// 0x6D -> NTAG216/231). nfc_amiibo.cpp's own kMaxPages is 256 -- the donor's
// loop BOUND (RFID2.cpp:428 iterates page 0..252 in groups of 4) rather than
// any real tag's capacity -- which is the right cap for a transient in-RAM
// dump but 100 bytes of dead weight in a struct that is stored on SD, copied
// by value between screens and held in several file-scope statics at once.
// 231 pages is 924 bytes, so a .tag record is 963 bytes.
//
// page_count == 0 is the "no page content captured" state and is what every
// non-Type-2 tag (EMV cards, MIFARE Classic) and every RFID2/WS1850S-unit
// scan still stores -- emulation of those falls back to the original
// UID/SAK/ATQA-only behaviour rather than answering with zeros.
constexpr uint8_t kT2tPageLen  = 4;   // NFC Forum T2T block length (RFAL's own
                                      // RFAL_T2T_BLOCK_LEN, rfal_t2t.h)
constexpr uint8_t kMaxT2tPages = 231; // NTAG216, see above

// get_version/get_version_len added 2026-08-24, same real-hardware
// motivation as page_count/pages above but for a DIFFERENT real gap:
// capturing every page still wasn't enough for a real Amiibo-specific
// reader app to accept the emulated tag ("amiibo not found" against a real
// NTAG215 with 135/135 real pages genuinely captured and correctly
// answered). Real root cause: an Amiibo app is NTAG215-specific tooling, and
// real such tools gate on the tag's GET_VERSION (0x60) response before
// trusting anything else -- confirmed in the real serial log ("NAKing
// unsupported command 0x60") once a reader was actually watched closely.
// st25r3916_driver.cpp's own disclosed-gap comment for GET_VERSION already
// named the honest fix before this was needed: "capture the real tag's own
// GET_VERSION response ... and replay it, not synthesise one" -- this field
// is exactly that. 8 bytes is the real, standard NTAG21x/Ultralight EV1
// GET_VERSION response length (a well-established real-world value, just
// not present in any of this project's own vendored/donor sources -- see
// that same driver comment for why the BYTES are still captured rather than
// fabricated even though the command byte itself is confidently real).
// get_version_len == 0 is "not captured" (real responses are always 8
// bytes; a real tag that supports GET_VERSION never returns 0).
constexpr uint8_t kMaxGetVersionLen = 8;

struct TagInfo {
    uint8_t uid[10];
    uint8_t uid_len;
    char type_name[24];
    uint8_t sak;      // SEL_RES, real for both save paths.
    uint8_t atqa[2];  // SENS_RES, wire order (LSB first); {0,0} if not captured.
    uint8_t page_count;                        // 0 = no page content captured
    uint8_t pages[kMaxT2tPages][kT2tPageLen];  // real captured T2T page image
    uint8_t get_version_len;                   // 0 = not captured
    uint8_t get_version[kMaxGetVersionLen];    // real captured GET_VERSION reply
};

// Formats uid as "04:A3:F1:..." (matching the project’s hex-with-colons style).
// Returns out for convenience (and never returns nullptr).
const char *format_uid(const uint8_t *uid,
                        uint8_t len,
                        char *out,
                        size_t out_len);

// Real NFC Forum Type 2 Tag page-count -> chip-family name, moved here
// (2026-08-24) from nfc_amiibo.cpp's own page_count_tag_name() so
// nfc_read.cpp's own T2T capture path (Task 24's real-hardware NTAG215/
// Amiibo emulation work) can report the same real product name instead of a
// bare page count. See nfc_amiibo.cpp's own real-hardware citation (2026-08-
// 21) for why page count, not the Capability Container byte, is this
// project's established reliable signal: a genuine Amiibo-programmed
// NTAG215 defeats CC-byte detection (Nintendo's proprietary data overwrites
// that page), but its real end-of-memory page count (135) is unaffected and
// was confirmed correct on real hardware. Returns "Unknown Ultralight/
// NTAG21x" for any page count that isn't one of the three real, standard
// NTAG21x capacities (45/135/231).
const char *t2t_page_count_tag_name(int page_count);

} // namespace NfcCommon

