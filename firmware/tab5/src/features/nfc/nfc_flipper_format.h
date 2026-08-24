#pragma once

#include "../../hal/istorage.h"
#include <cstddef>
#include <cstdint>

// ===========================================================================
// Flipper Zero "Flipper NFC device" (.nfc) file write/read, ISO14443-4A
// device type (Phase 3 Task 13's save/export extension, 2026-08-23).
//
// SOURCE: Flipper Devices' own firmware source, dev branch, fetched
// 2026-08-23 from raw.githubusercontent.com/flipperdevices/
// flipperzero-firmware/dev/... -- NOT recalled, and not a plausible-looking
// invention. The relevant real files and the exact things taken from each:
//
//   lib/nfc/nfc_device.c
//     :9   NFC_FILE_HEADER    = "Flipper NFC device"   (the Filetype value)
//     :12  NFC_DEVICE_UID_KEY = "UID"
//     :13  NFC_DEVICE_TYPE_KEY = "Device type"
//     nfc_device_save() (:167-215) writes, in this exact order:
//       1. flipper_format_write_header_cstr(ff, NFC_FILE_HEADER,
//          NFC_CURRENT_FORMAT_VERSION)
//       2. a comment listing every allowed device type ("Device type can be
//          <name>, <name>, ...", built by iterating nfc_devices[])
//       3. the "Device type:" line itself (the protocol's protocol_name)
//       4. a comment, "UID is common for all formats"
//       5. the "UID:" hex line
//       6. whatever the per-protocol save() adds after that
//
//   lib/nfc/nfc_common.h
//     NFC_CURRENT_FORMAT_VERSION = NFC_UNIFIED_FORMAT_VERSION = 4 (the
//     Version value written; NFC_LSB_ATQA_FORMAT_VERSION = 2 is the older,
//     still-loadable version whose ATQA byte order differs -- see below).
//
//   lib/nfc/protocols/iso14443_3a/iso14443_3a.c
//     :12-13 ISO14443_3A_ATQA_KEY = "ATQA", ISO14443_3A_SAK_KEY = "SAK"
//     :8     ISO14443_3A_PROTOCOL_NAME = "ISO14443-3A" (used verbatim in this
//            protocol's own "<name> specific data" comment)
//     iso14443_3a_save() (:83-104) writes a "ISO14443-3A specific data"
//     comment, then ATQA (2 bytes) then SAK (1 byte). CRITICAL DETAIL, taken
//     from that function's own in-source comment rather than assumed: "Save
//     ATQA in MSB order for correct companion apps display" --
//     `const uint8_t atqa[2] = {data->atqa[1], data->atqa[0]};`. The bytes
//     are written REVERSED relative to the on-the-wire (LSB-first) SENS_RES
//     order this project stores in NfcCommon::TagInfo::atqa and in
//     St25r3916::Iso14443aTag::atqa. iso14443_3a_load() (:60-80) mirrors it,
//     swapping back only for files newer than NFC_LSB_ATQA_FORMAT_VERSION.
//
//   lib/nfc/protocols/iso14443_4a/iso14443_4a.c
//     :5     ISO14443_4A_PROTOCOL_NAME = "ISO14443-4A" -- the "Device type"
//            value this module writes, and the one Flipper's own reader uses
//            for a plain ISO14443-4A card (which is exactly what a
//            contactless EMV payment card presents itself as at this layer).
//     :8-12  the ATS keys: "T0", "TA(1)", "TB(1)", "TC(1)", "T1...Tk"
//     iso14443_4a_save() (:148-188): calls iso14443_3a_save() FIRST, then
//     writes a "ISO14443-4A specific data" comment, then -- only `if
//     (ats_data->tl > 1)` -- T0, then TA(1)/TB(1)/TC(1) each only if the
//     corresponding presence bit is set in T0, then "T1...Tk" (the historical
//     bytes) only if non-empty. This module reproduces that conditional
//     structure exactly, driven by the real ATS bytes
//     St25r3916::iso14443_4_get_ats() captured from the card.
//
//   lib/flipper_format/flipper_format_stream.c / _stream_i.h -- the actual
//     text syntax every one of the above ultimately emits:
//       _stream_i.h:4-7  delimiter = ':', comment = '#', eol = '\n'
//       flipper_format_stream_write_key() (:17-27): "<key>" ":" " "
//       flipper_format_stream_write_value_line() (:269-337): hex values are
//         "%02X" separated by single spaces, then EOL
//       flipper_format_stream_write_comment_cstr(): "# " then the text, EOL
//     So a header line is literally `Filetype: Flipper NFC device\n` and a
//     hex line is literally `UID: 04 A2 B3 C4\n`.
//
// A real, complete file this module produces therefore looks like:
//
//   Filetype: Flipper NFC device
//   Version: 4
//   # Device type can be ISO14443-3A, ISO14443-3B, ISO14443-4A, ...
//   Device type: ISO14443-4A
//   # UID is common for all formats
//   UID: 04 A2 B3 C4 D5 E6 F7
//   # ISO14443-3A specific data
//   ATQA: 00 44
//   SAK: 20
//   # ISO14443-4A specific data
//   T0: 78
//   TA(1): 80
//   TB(1): 82
//   TC(1): 02
//   T1...Tk: 1F 2F 3F
//
// SCOPE, disclosed rather than implied: this writes/reads the ISO14443-4A
// device type ONLY. Flipper's other device types (Mifare Classic dumps,
// NTAG/Ultralight pages, DESFire application trees, ...) carry per-protocol
// memory contents this project's EMV read does not and cannot produce, and
// are not handled here -- read() rejects any file whose "Device type" is
// something else rather than mis-parsing it. There is likewise no EMV device
// type in Flipper's upstream dev branch to target (nfc_device_defs.c's
// nfc_devices[] registers 12 protocols, none of them EMV), which is why an
// EMV card is exported as what it genuinely is at the transport layer: an
// ISO14443-4A card with a real ATS.
//
// Structurally this module follows Task 21's rf433_sub_format.h/.cpp exactly
// (this project's established real-third-party-format interop pattern): pure
// encode()/decode() halves that are host-testable with no Arduino/SD
// dependency, plus write()/read() SD wrappers taking IStorage& by dependency
// injection. See test/test_nfc_flipper_format/ for the round-trip tests.
// ===========================================================================

namespace NfcFlipperFormat {

// Same SD directory the Task 10 tag library already uses -- .nfc and .tag
// files sit side by side, keyed by the same UID hex, and IStorage::
// list_files()'s extension filter keeps the two browse lists separate.
extern const char kNfcDir[];

// [SOURCE] nfc_common.h NFC_CURRENT_FORMAT_VERSION.
constexpr int kFormatVersion = 4;

// [SOURCE] iso14443_4a.c:5 ISO14443_4A_PROTOCOL_NAME.
extern const char kDeviceTypeIso14443_4a[];

// Largest ATS this module carries. Matches st25r3916_driver.cpp's own
// kAtsMaxLen (32) -- there is no point representing more than the driver can
// capture, and the real ATS of every card seen on this hardware is far
// shorter.
constexpr size_t kMaxAtsLen = 32;

// One ISO14443-4A card, in this project's own terms rather than Flipper's.
struct Iso14443_4aRecord {
    uint8_t uid[10] = {0};
    uint8_t uid_len = 0;   // 4, 7 or 10
    uint8_t atqa[2] = {0}; // WIRE order (LSB first), as St25r3916 and
                           // NfcCommon::TagInfo store it. encode()/decode()
                           // do the MSB-order swap the real format needs.
    uint8_t sak = 0;
    // Full ATS as received, TL byte FIRST (i.e. ats[0] == ats_len), exactly
    // as St25r3916::iso14443_4_get_ats() returns it. ats_len == 0 means "no
    // ATS captured": encode() then omits the whole ISO14443-4A block, which
    // is the same thing Flipper's own iso14443_4a_save() does for tl <= 1.
    uint8_t ats[kMaxAtsLen] = {0};
    uint8_t ats_len = 0;
};

// Worst-case encoded text size: the fixed header/comment lines (~250 bytes,
// dominated by the device-type-list comment) plus 10 UID bytes and up to
// kMaxAtsLen ATS bytes at 3 chars each. 1024 is comfortably past that and is
// small enough to sit in a caller's buffer without ceremony.
constexpr size_t kMaxEncodedTextBytes = 1024;

// Pure in-memory conversion, no SD I/O -- host-testable. Writes NUL-
// terminated Flipper ".nfc" text for `rec` into buf and reports its length
// (excluding the NUL) in *out_len. Returns false if uid_len is not 4/7/10,
// if ats_len is inconsistent (non-zero but not equal to its own TL byte, or
// past kMaxAtsLen), or if the text would not fit in buf_size.
bool encode(const Iso14443_4aRecord &rec, char *buf, size_t buf_size, size_t *out_len);

// Pure in-memory conversion, no SD I/O -- host-testable. Parses real Flipper
// ".nfc" text (text[0..len), need not be NUL-terminated) into *out. Returns
// false unless the file is a "Flipper NFC device" file of a supported
// Version with "Device type: ISO14443-4A" and a well-formed UID/ATQA/SAK.
// The ISO14443-4A ATS block is optional (Flipper omits it for tl <= 1); when
// present, out->ats is REBUILT from the T0/TA(1)/TB(1)/TC(1)/T1...Tk keys
// into the TL-first wire layout this project uses, since the real format
// stores the ATS decomposed rather than as one blob.
//
// ATQA byte order follows the real format's own version rule ([SOURCE]
// iso14443_3a_load(), nfc_common.h NFC_LSB_ATQA_FORMAT_VERSION = 2): files
// with Version > 2 store ATQA MSB-first and are swapped back to this
// project's wire order; Version 2 files are taken as already wire-order.
bool decode(const char *text, size_t len, Iso14443_4aRecord *out);

// SD-backed convenience wrappers, IStorage& injected (same reasoning as
// rf433_sub_format.h's own note: keeps this whole module free of any
// concrete SD_MMC/Arduino dependency so the wrappers themselves, not just
// encode()/decode(), are exercised natively against a fake IStorage).
bool write(IStorage &storage, const char *path, const Iso14443_4aRecord &rec);
bool read(IStorage &storage, const char *path, Iso14443_4aRecord *out);

// Builds "<kNfcDir>/<HEX-UID>.nfc" into path, mirroring
// nfc_tag_library.cpp's own build_path_for_tag() convention exactly (UID hex
// as the filename, so re-scanning the same physical card overwrites its own
// entry instead of accumulating duplicates). Returns false if uid_len is 0
// or larger than sizeof(rec.uid).
bool build_path(const Iso14443_4aRecord &rec, char *path, size_t path_len);

} // namespace NfcFlipperFormat
