#include "nfc_emv_read.h"

#include "st25r3916_driver.h"
#include "nfc_common.h"
#include "nfc_flipper_format.h"
#include "nfc_tag_library.h"
#include "../../hal/nfc_pn532.h" // nfc_release_external_i2c() -- GPIO53 arbiter
#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <Arduino.h>
#include <cstdio>
#include <cstring>

// ===========================================================================
// EMV/APDU contactless payment-card reader (Phase 3 Task 13).
//
// SOURCES.
//   [DONOR] ~/src/firmware/src/modules/rfid/emv_reader.cpp and
//           emv_reader.hpp (Bruce). Real, tested-against-real-cards APDU
//           byte sequences and AID dictionary, ported here verbatim; only
//           the transport (`nfc->EMVinDataExchange(...)`, a PN532 call) is
//           replaced, with St25r3916::apdu_transceive() (this task's own
//           addition to st25r3916_driver.h/.cpp, built on RATS/I-block
//           framing cited in that file's own header comment). Specifically:
//             - EMVReader::emv_ask_for_aid()'s `ask_for_aid_apdu[]` -> SELECT
//               PPSE, below.
//             - EMVReader::emv_ask_for_pdol()'s `ask_for_pdol[]` -> SELECT AID
//               (the donor's own name for this function is misleading: what
//               it actually sends, once its own memcpy overwrite is worked
//               through, is a plain SELECT-by-AID APDU, `00 A4 04 00 07 <AID>
//               00` -- not a PDOL request. This module sends that same clean
//               byte sequence directly rather than reproducing the donor's
//               confusing placeholder-array construction.)
//             - EMVReader::emv_get_processing_options_no_pdol()'s
//               `ask_for_afl[]` -> GET PROCESSING OPTIONS, below.
//             - EMVReader::read_afl() / emv_read_record() -> the AFL walk and
//               READ RECORD loop, below.
//             - `known_aid[]` (emv_reader.hpp) -> kKnownAids, below (ported
//               verbatim, vendor/scheme name only -- this module drops the
//               donor's EMV_Vendor enum since no payment logic reads it).
//             - display_emv()'s PAN-formatting convention (hex string, a
//               space inserted every 4 characters) -> format_pan(), below.
//
//   [ISO7816] ISO/IEC 7816-4 Sec 5.2.2 (tag and length encoding) / EMV Book 3
//           Annex B1 -- the BER-TLV walker below (ber_tlv_find() and its
//           helpers) is written directly against this standard, non-
//           proprietary encoding, per this task's own brief: Bruce's real
//           vendored parser (BerTlv.h/.cpp, itself from
//           https://github.com/huckor/BER-TLV per emv_reader.cpp's own
//           header comment; located at
//           ~/src/firmware/.pio/libdeps/*/BER-TLV/src/) was read and its
//           tag/length parsing logic (GetTagLength()'s 0x1F multi-byte-tag
//           marker, GetSizeOfValue()'s 0x81-0x84 multi-byte-length prefixes)
//           cross-checked against this implementation, but this is a fresh,
//           smaller walker over raw buffers rather than a port: Bruce's own
//           class is built entirely around std::vector<unsigned char>
//           construction/mutation (Add(), SetTlv(), GetTlvAsHexString()) that
//           this read-only, single-buffer, find-one-tag use case does not
//           need.
//
// PDOL-BASED GPO PATH -- IMPLEMENTED as of 2026-08-23 (this was previously
// documented here as a disclosed gap; it no longer is). Real-hardware
// evidence drove it: a real Visa card completed SELECT PPSE and SELECT AID
// (AID A0 00 00 00 03 10 10, "VISA CREDIT") and then answered the no-PDOL GPO
// with the real EMV status bytes 69 85, "Conditions of use not satisfied" --
// i.e. that card genuinely requires its PDOL echoed back. build_pdol_gpo()
// below now does exactly that, and the no-PDOL GPO remains the fallback for
// cards whose FCI carries no tag 9F38 at all (a real Mastercard-style card in
// hand behaves that way and its whole existing path is unchanged).
//   - [DONOR] EMVReader::emv_ask_for_pdol() (emv_reader.cpp:128-149) is where
//     the donor looks for tag 9F38 in the SELECT AID response; this module
//     does the same with its own ber_tlv_find().
//   - [DONOR] EMVReader::emv_read_visa() (emv_reader.cpp:151-230) is the real,
//     donor-tested Visa GPO payload every default fill value below is taken
//     from, byte for byte. See kPdolDefaults' own comment for the per-tag
//     mapping and for the ONE deliberate difference: the donor sends a FIXED
//     33-byte payload that happens to match the standard Visa PDOL, whereas
//     this module walks the card's OWN PDOL tag/length list and fills each
//     requested data object from that same value table. For a card whose PDOL
//     is the standard Visa one the two produce byte-identical output; for any
//     other PDOL ordering or length the donor's fixed payload would simply be
//     wrong, which is why this module drives it from the card's real request.
//
// SCOPE / REMAINING DISCLOSED GAPS: this is still a READ-ONLY reader -- it
// sends no GENERATE AC, computes no cryptogram, and the PDOL values below are
// deliberately inert (zero amount, fixed unpredictable number). It does not
// implement Flipper's/other tooling's deeper EMV parsing (no application
// currency/label extraction beyond the AID dictionary, no transaction log).
//
// EXECUTION MODEL: a single button-tap-triggered synchronous read, run
// entirely inside ONE poll() tick once a card is detected -- see
// attempt_read()'s own comment for the real timing budget this relies on
// (modelled on nfc_read.cpp's own precedent for a disclosed poll()-tick
// budget exception, e.g. its RFID2 bring-up's ~205 ms). Not a worker task
// (unlike nfc_mifare_crack.cpp): every step here is a single bounded APDU
// exchange (a few ms of real I2C traffic per st25r3916_driver.cpp's own
// documented costs), not a multi-second uninterruptible computation, so there
// is no watchdog-starvation risk of the kind that drove nfc_mifare_crack.cpp
// onto a separate core-1 task. See st25r3916_driver.cpp's own
// kMaxSingleExchangeMs comment for how the per-APDU timeout itself is kept
// well clear of the ~5 s ESP32 task watchdog this project has already been
// bitten by twice (hal/ir_unit.h's and hal/storage_sd.cpp's own header
// comments).
// ===========================================================================

extern FeatureRegistry g_registry;

// The real global StorageSD (defined in main.cpp, Phase 1 Task 10), handed to
// NfcTagLibrary::save() / NfcFlipperFormat::write() at their call sites --
// both take IStorage& by dependency injection (see nfc_tag_library.h's header
// comment for why), exactly as nfc_read.cpp's own "Save to Library" button
// already does. Declared at file scope, NOT inside this file's anonymous
// namespace, so it refers to that one real object rather than an
// internal-linkage declaration that would never link.
extern StorageSD storage;

namespace NfcEmvRead {

namespace {

// --- Minimal BER-TLV walker -------------------------------------------------
// See this file's header SOURCES block for the [ISO7816]/[DONOR] citations.
// Finds the first occurrence of `tag` anywhere in data[0..len), recursing
// into constructed (nested/template) tags.

bool ber_tlv_tag_len(const uint8_t *data, size_t len, size_t pos, size_t *tag_len_out) {
    if (pos >= len) {
        return false;
    }
    size_t n = 1;
    // ISO 7816-4 5.2.2: low 5 bits (0x1F) of byte 1 all set -> the tag
    // continues into subsequent bytes; each continuation byte's bit 8 (0x80)
    // set means "more tag bytes follow".
    if ((data[pos] & 0x1FU) == 0x1FU) {
        size_t i = pos + 1;
        for (;;) {
            if (i >= len) {
                return false;
            }
            n++;
            if ((data[i] & 0x80U) == 0U) {
                break;
            }
            i++;
        }
    }
    *tag_len_out = n;
    return true;
}

bool ber_tlv_len(const uint8_t *data, size_t len, size_t pos,
                 size_t *len_field_len_out, size_t *value_len_out) {
    if (pos >= len) {
        return false;
    }
    const uint8_t b0 = data[pos];
    if ((b0 & 0x80U) == 0U) {
        // Short form: the byte itself is the length, 0-127.
        *len_field_len_out = 1;
        *value_len_out = b0;
        return true;
    }
    // Long form: low 7 bits are the COUNT of following big-endian length
    // bytes. 0x80 alone (indefinite length, a BER-only construct) never
    // appears in real EMV/ISO 7816-4 card data and is rejected.
    const uint8_t n = static_cast<uint8_t>(b0 & 0x7FU);
    if (n == 0 || n > 4 || pos + 1U + n > len) {
        return false;
    }
    size_t v = 0;
    for (uint8_t i = 0; i < n; i++) {
        v = (v << 8) | data[pos + 1U + i];
    }
    *len_field_len_out = 1U + n;
    *value_len_out = v;
    return true;
}

// Real EMV/ISO 7816-4 TLV nesting in practice is shallow (a template tag
// wrapping a handful of primitive tags, rarely more than one level deep).
// This cap is a cheap defensive bound against a crafted response nesting
// constructed tags dozens of levels deep to run this recursive walker's
// stack usage up against the task's real stack budget -- not a limit any
// real card response should ever approach.
constexpr int kMaxTlvNestingDepth = 16;

bool ber_tlv_find_in(const uint8_t *data, size_t len, size_t start, size_t end,
                     const uint8_t *tag, uint8_t tag_len,
                     const uint8_t **val_out, size_t *val_len_out,
                     int depth = 0) {
    if (depth > kMaxTlvNestingDepth) {
        return false;
    }
    size_t pos = start;
    while (pos < end) {
        size_t tlen = 0;
        if (!ber_tlv_tag_len(data, len, pos, &tlen) || pos + tlen > end) {
            return false;
        }
        size_t llen = 0;
        size_t vlen = 0;
        if (!ber_tlv_len(data, len, pos + tlen, &llen, &vlen)) {
            return false;
        }
        const size_t val_pos = pos + tlen + llen;
        // Split as "val_pos > end" then "vlen > end - val_pos" rather than
        // "val_pos + vlen > end": a crafted 4-byte long-form length (up to
        // 0xFFFFFFFF, see ber_tlv_len() above) can make val_pos + vlen wrap
        // size_t's 32-bit range back under `end`, silently bypassing a
        // single-addition bounds check. This form never adds two
        // attacker-influenced values together.
        if (val_pos > end || vlen > end - val_pos) {
            return false; // truncated/malformed -- refuse rather than overread
        }

        if (tlen == tag_len && memcmp(&data[pos], tag, tag_len) == 0) {
            *val_out = &data[val_pos];
            *val_len_out = vlen;
            return true;
        }
        // ISO 7816-4 5.2.2.1: bit 6 (0x20) of the tag's first byte marks a
        // constructed (nested) tag whose value is itself a TLV sequence.
        if ((data[pos] & 0x20U) != 0U) {
            if (ber_tlv_find_in(data, len, val_pos, val_pos + vlen, tag, tag_len,
                                val_out, val_len_out, depth + 1)) {
                return true;
            }
        }
        pos = val_pos + vlen;
    }
    return false;
}

bool ber_tlv_find(const uint8_t *data, size_t len,
                  const uint8_t *tag, uint8_t tag_len,
                  const uint8_t **val_out, size_t *val_len_out) {
    return ber_tlv_find_in(data, len, 0, len, tag, tag_len, val_out, val_len_out);
}

// --- AID dictionary ----------------------------------------------------------
// Ported verbatim from [DONOR] emv_reader.hpp's known_aid[] (that file's own
// comment cites http://hartleyenterprises.com/listAID.html as its source
// list). Vendor/scheme display only.
struct KnownAid {
    uint8_t aid[7];
    const char *name;
};

constexpr KnownAid kKnownAids[] = {
    {{0xA0, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10}, "MasterCard"},
    {{0xA0, 0x00, 0x00, 0x00, 0x04, 0x22, 0x03}, "U.S Maestro"},
    {{0xA0, 0x00, 0x00, 0x00, 0x04, 0x30, 0x60}, "Maestro"},
    {{0xA0, 0x00, 0x00, 0x00, 0x04, 0x60, 0x00}, "Cirrus"},
    {{0xA0, 0x00, 0x00, 0x00, 0x04, 0x99, 0x99}, "MasterCard"},
    {{0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10}, "Visa"},
    {{0xA0, 0x00, 0x00, 0x00, 0x03, 0x20, 0x10}, "Visa Electron"},
    {{0xA0, 0x00, 0x00, 0x00, 0x03, 0x20, 0x20}, "V-Pay"},
    {{0xA0, 0x00, 0x00, 0x00, 0x03, 0x30, 0x10}, "Visa"},
    {{0xA0, 0x00, 0x00, 0x00, 0x03, 0x80, 0x10}, "Visa"},
    {{0xA0, 0x00, 0x00, 0x00, 0x98, 0x08, 0x40}, "Visa"},
};
constexpr size_t kKnownAidCount = sizeof(kKnownAids) / sizeof(kKnownAids[0]);

// --- Result / state ----------------------------------------------------------

struct EmvResult {
    bool ok = false;
    char vendor[40] = "";
    char pan[32] = "";
    char expiry[8] = "";
    char effective[8] = "";
    bool has_effective = false;
    const char *fail_reason = nullptr;
};

// Response capacity for one (possibly reassembled) R-APDU. Matches the
// driver's own kMaxReassembledLen -- St25r3916::apdu_transceive() rejects any
// rx_cap larger than that outright. Raised from the previous 255 because
// PICC->PCD I-block chaining is now handled there (real hardware: a real
// Mastercard-style card's SFI-2 READ RECORD came back as a chained I-block,
// PCB 0x12), so a single logical response can legitimately exceed one frame.
constexpr size_t kApduRxCap = 512;

// File-scope, not stack locals. Two 512-byte buffers plus this module's other
// locals inside read_emv_card() would put ~1 KB of transient APDU buffering on
// the LVGL/poll() task's stack at a non-trivial call depth (poll() ->
// attempt_read() -> read_emv_card()), for no benefit -- this module is a
// single screen driven from a single task and read_emv_card() is not
// reentrant regardless (it already keeps s_read_deadline_ms in a static). Same
// reasoning rf433_sub_format.cpp's build_signed_durations() documents for its
// own working buffer.
uint8_t s_apdu_rx[kApduRxCap];
uint8_t s_record_rx[kApduRxCap];

// Backing store for the one failure message that is FORMATTED rather than a
// string literal (the GPO status-word report). EmvResult::fail_reason is a
// `const char *` pointing at literals everywhere else; this keeps that
// contract intact without giving the struct a buffer every other path would
// waste. Safe for the same reason the two buffers above are: single screen,
// single task, and the pointer is consumed by the caller within the same
// poll() tick that set it.
char s_fail_buf[96];

// Real total wall-clock budget for one full read attempt (RATS + up to ~10
// APDU exchanges). Checked before every single apdu_step() call below, in
// addition to st25r3916_driver.cpp's own per-exchange kMaxSingleExchangeMs
// cap -- see this file's EXECUTION MODEL header comment. 2.5 s leaves wide
// margin under the ~5 s task watchdog even accounting for one exchange
// hitting its own worst-case per-exchange timeout on top of this.
constexpr uint32_t kOverallReadBudgetMs = 2500U;
uint32_t s_read_deadline_ms = 0;

bool apdu_step(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t *rx_len_out,
              size_t rx_cap) {
    *rx_len_out = 0;
    if (static_cast<int32_t>(millis() - s_read_deadline_ms) >= 0) {
        return false; // overall read budget exceeded -- give up, don't retry
    }
    return St25r3916::apdu_transceive(tx, tx_len, rx, rx_cap, rx_len_out);
}

// PAN (tag 5A) is BCD -- one byte holds two decimal digits, padded with a
// trailing 'F' nibble if the digit count is odd. [DONOR] display_emv()'s own
// convention: hex-string the raw bytes, then insert a space every 4
// characters (`if (i % 4 == 0 && i != 0) pan.insert(...)`).
void format_pan(const uint8_t *pan_bytes, size_t pan_len, char *out, size_t out_cap) {
    char digits[40];
    size_t n = 0;
    for (size_t i = 0; i < pan_len && n + 2 < sizeof(digits); i++) {
        std::snprintf(&digits[n], 3, "%02X", pan_bytes[i]);
        n += 2;
    }
    digits[n] = '\0';
    while (n > 0 && digits[n - 1] == 'F') {
        digits[--n] = '\0';
    }

    size_t oi = 0;
    for (size_t i = 0; i < n; i++) {
        if (i > 0 && (i % 4) == 0) {
            if (oi + 1 >= out_cap) {
                break;
            }
            out[oi++] = ' ';
        }
        if (oi + 1 >= out_cap) {
            break;
        }
        out[oi++] = digits[i];
    }
    out[oi] = '\0';
}

// --- PDOL (tag 9F38) handling -----------------------------------------------
// See this file's header comment for the [DONOR] citation. Every value below
// is lifted byte-for-byte from EMVReader::emv_read_visa()'s own `payload[]`
// (emv_reader.cpp:155-204), including that function's own inline labels:
//
//   9F66 (4) = 20 00 00 00       "TTQ - Visa Standard"
//   9F02 (6) = 00 00 00 00 00 00 "Amount 0"
//   9F03 (6) = 00 00 00 00 00 00 "Amount Other 0"
//   9F1A (2) = 03 80             "Country: Italy"  <- the donor's own choice
//   95   (5) = 00 00 00 00 00    "TVR: No errors"
//   5F2A (2) = 09 78             "Currency: Euro"
//   9A   (3) = 25 11 25          "Date: 25 Nov 25"
//   9C   (1) = 00                "Tx Type: Purchase"
//   9F37 (4) = 12 34 56 78       "Unpredictable Num"
//
// The country/currency/date values are the donor's, kept rather than
// "corrected": they are the combination actually tested against real cards,
// nothing here performs a transaction, and a card that answers GPO at all
// answers it for any self-consistent terminal profile. Changing them to a
// different country/currency would be an untested guess, which is exactly
// what this project's citation discipline exists to avoid.
//
// Any PDOL data object the card asks for that is NOT in this table is filled
// with zeros for its requested length -- the only defensible default, and the
// same thing the donor's fixed payload effectively does for the fields it
// zeroes.
struct PdolDefault {
    uint8_t tag[2];
    uint8_t tag_len;
    uint8_t value[6];
    uint8_t value_len;
};

constexpr PdolDefault kPdolDefaults[] = {
    {{0x9F, 0x66}, 2, {0x20, 0x00, 0x00, 0x00}, 4},
    {{0x9F, 0x02}, 2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 6},
    {{0x9F, 0x03}, 2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 6},
    {{0x9F, 0x1A}, 2, {0x03, 0x80}, 2},
    {{0x95, 0x00}, 1, {0x00, 0x00, 0x00, 0x00, 0x00}, 5},
    {{0x5F, 0x2A}, 2, {0x09, 0x78}, 2},
    {{0x9A, 0x00}, 1, {0x25, 0x11, 0x25}, 3},
    {{0x9C, 0x00}, 1, {0x00}, 1},
    {{0x9F, 0x37}, 2, {0x12, 0x34, 0x56, 0x78}, 4},
};
constexpr size_t kPdolDefaultCount = sizeof(kPdolDefaults) / sizeof(kPdolDefaults[0]);

// Real PDOLs are short (the standard Visa one is 9 entries / 33 value bytes).
// Both bounds are defensive caps against a malformed or hostile 9F38, not
// limits any real card approaches -- same discipline as the AFL cap below and
// as st25r3916_driver.cpp's own chaining bounds.
//
// Re-confirmed unchanged on 2026-08-24, when the driver gained PCD->PICC
// I-block chaining and a command larger than one frame became transmittable
// for the first time. These two numbers were never transmission limits, so
// they do not move: 120 data bytes is a 128-byte command, comfortably under
// St25r3916::apdu_transceive()'s own 254-byte tx_len ceiling (which is a
// buffer limit, not a frame limit), and the driver now fragments whatever it
// is handed across as many chained I-blocks as the card's declared FSC needs
// -- e.g. 5 fragments against a typical FSC=32 card, 1 against an FSC=128 one.
// What the caps still do is exactly what they always did: refuse a 9F38 whose
// requested lengths are absurd before any of it reaches the card.
constexpr size_t kMaxPdolEntries = 24;
constexpr size_t kMaxPdolDataLen = 120; // keeps Lc (= data + 2) < 128, so the
                                        // tag-83 length stays single-byte

// Builds the PDOL-based GET PROCESSING OPTIONS command from the card's own
// PDOL (a DOL: a bare sequence of tag/length pairs with no values, EMV Book 3
// Annex A / ISO 7816-4 5.2.2 -- the same tag and length encodings
// ber_tlv_tag_len()/ber_tlv_len() above already implement, reused here rather
// than re-written). Output shape, per [DONOR] emv_read_visa()'s payload:
//   80 A8 00 00 <Lc> 83 <data_len> <data...> 00
// Returns the total command length, or 0 if the PDOL is malformed or asks for
// more than this module is willing to build.
size_t build_pdol_gpo(const uint8_t *pdol, size_t pdol_len, uint8_t *out, size_t out_cap) {
    uint8_t data[kMaxPdolDataLen];
    size_t data_len = 0;
    size_t pos = 0;
    size_t entries = 0;

    while (pos < pdol_len) {
        if (++entries > kMaxPdolEntries) {
            return 0;
        }
        size_t tlen = 0;
        if (!ber_tlv_tag_len(pdol, pdol_len, pos, &tlen) || pos + tlen > pdol_len) {
            return 0;
        }
        size_t llen = 0;
        size_t want = 0;
        if (!ber_tlv_len(pdol, pdol_len, pos + tlen, &llen, &want)) {
            return 0;
        }
        const uint8_t *tag = &pdol[pos];
        pos += tlen + llen;

        if (want > kMaxPdolDataLen || data_len + want > sizeof(data)) {
            return 0;
        }
        memset(&data[data_len], 0, want); // zero is the default for anything
                                          // not in kPdolDefaults, and the pad
                                          // for a short known value
        for (size_t i = 0; i < kPdolDefaultCount; i++) {
            if (kPdolDefaults[i].tag_len != tlen ||
                memcmp(kPdolDefaults[i].tag, tag, tlen) != 0) {
                continue;
            }
            const size_t n = (kPdolDefaults[i].value_len < want) ? kPdolDefaults[i].value_len
                                                                 : want;
            memcpy(&data[data_len], kPdolDefaults[i].value, n);
            break;
        }
        data_len += want;
    }

    if (data_len == 0) {
        return 0; // an empty PDOL is not a PDOL path -- caller falls back
    }
    const size_t cmd_len = 5 + 2 + data_len + 1;
    if (cmd_len > out_cap) {
        return 0;
    }
    out[0] = 0x80;
    out[1] = 0xA8;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = static_cast<uint8_t>(2 + data_len); // Lc
    out[5] = 0x83;                               // command template tag
    out[6] = static_cast<uint8_t>(data_len);
    memcpy(&out[7], data, data_len);
    out[7 + data_len] = 0x00; // Le
    return cmd_len;
}

// --- Track 2 equivalent data (tag 57) ---------------------------------------
// [DONOR] emv_read_visa()'s own inline comments describe this layout exactly:
// "Index 8 is separator 'D' and first digit of ValidTo month / Index 9 is
// second digit of ValidTo month and first digit of ValidTo year / Index 10 is
// second digit of ValidTo year and first digit of Service Code". That is the
// real ISO 7813 track-2 layout: packed BCD nibbles, PAN digits first, the
// nibble 0xD as the field separator, then a 4-digit YYMM expiry, then the
// service code.
//
// The donor hardcodes the separator's position (it assumes an 8-byte/16-digit
// PAN and indexes 8/9/10 directly). This walks nibbles and finds the real
// separator instead, so a 15- or 19-digit PAN parses correctly too -- same
// layout, no fixed-offset assumption. Returns false if no separator or fewer
// than 4 expiry digits follow it.
bool parse_track2(const uint8_t *t2, size_t t2_len, char *pan_out, size_t pan_cap,
                  char *expiry_out, size_t expiry_cap) {
    uint8_t digits[40];
    size_t n = 0;
    for (size_t i = 0; i < t2_len && n + 2 <= sizeof(digits); i++) {
        digits[n++] = static_cast<uint8_t>((t2[i] >> 4) & 0x0FU);
        digits[n++] = static_cast<uint8_t>(t2[i] & 0x0FU);
    }
    size_t sep = n;
    for (size_t i = 0; i < n; i++) {
        if (digits[i] == 0x0DU) {
            sep = i;
            break;
        }
    }
    if (sep == n || sep == 0 || sep + 4 >= n) {
        return false;
    }

    // PAN digits are digits[0..sep). Re-pack them into the same BCD byte
    // layout format_pan() already consumes (it is the shared PAN-formatting
    // convention this module took from [DONOR] display_emv()), rather than
    // duplicating that function's spacing logic here.
    uint8_t pan_bcd[20];
    size_t pb = 0;
    for (size_t i = 0; i < sep && pb < sizeof(pan_bcd); i += 2) {
        const uint8_t hi = digits[i];
        const uint8_t lo = (i + 1 < sep) ? digits[i + 1] : 0x0FU; // odd count -> 'F' pad
        pan_bcd[pb++] = static_cast<uint8_t>((hi << 4) | lo);
    }
    format_pan(pan_bcd, pb, pan_out, pan_cap);

    // YYMM follows the separator; displayed MM/YY, same convention the AFL
    // path's tag-5F24 handling already uses.
    std::snprintf(expiry_out, expiry_cap, "%u%u/%u%u",
                  (unsigned)digits[sep + 3], (unsigned)digits[sep + 4],
                  (unsigned)digits[sep + 1], (unsigned)digits[sep + 2]);
    return true;
}

// The full read sequence: SELECT PPSE -> SELECT AID -> GET PROCESSING OPTIONS
// (PDOL-based when the card's FCI carries tag 9F38, otherwise no-PDOL) ->
// walk the AFL with READ RECORD. Returns false with out->fail_reason set on
// any step failing; out->vendor may still be filled in (AID is known before
// GPO/READ RECORD run) even when PAN extraction later fails, which the caller
// displays either way.
bool read_emv_card(EmvResult *out) {
    *out = EmvResult{};
    s_read_deadline_ms = millis() + kOverallReadBudgetMs;

    if (!St25r3916::iso14443_4_activate()) {
        out->fail_reason = "Card did not answer RATS (not an ISO14443-4 card?)";
        return false;
    }

    uint8_t *const rx = s_apdu_rx; // see s_apdu_rx's own comment (file-scope,
                                   // not a ~1 KB pair of stack locals)
    size_t rx_len = 0;

    // 1. SELECT PPSE ("2PAY.SYS.DDF01"). [DONOR] emv_ask_for_aid().
    static const uint8_t kSelectPpse[] = {
        0x00, 0xA4, 0x04, 0x00, 0x0E,
        0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E,
        0x44, 0x44, 0x46, 0x30, 0x31,
        0x00};
    if (!apdu_step(kSelectPpse, sizeof(kSelectPpse), rx, &rx_len, kApduRxCap)) {
        out->fail_reason = "SELECT PPSE failed (not a contactless payment card?)";
        return false;
    }

    static const uint8_t kTagAid[] = {0x4F};
    const uint8_t *aid_ptr = nullptr;
    size_t aid_len = 0;
    if (!ber_tlv_find(rx, rx_len, kTagAid, sizeof(kTagAid), &aid_ptr, &aid_len) ||
        aid_len != 7) {
        out->fail_reason = "No Application ID (tag 4F) in PPSE response";
        return false;
    }
    uint8_t aid[7];
    memcpy(aid, aid_ptr, 7);

    for (size_t i = 0; i < kKnownAidCount; i++) {
        if (memcmp(aid, kKnownAids[i].aid, 7) == 0) {
            std::snprintf(out->vendor, sizeof(out->vendor), "%s", kKnownAids[i].name);
            break;
        }
    }
    if (out->vendor[0] == '\0') {
        std::snprintf(out->vendor, sizeof(out->vendor),
                      "Unknown vendor (AID %02X%02X%02X%02X%02X%02X%02X)",
                      aid[0], aid[1], aid[2], aid[3], aid[4], aid[5], aid[6]);
    }

    // 2. SELECT AID. [DONOR] emv_ask_for_pdol() (see this file's header
    // comment on why that donor name is misleading for what it actually
    // sends).
    uint8_t select_aid[5 + 7 + 1];
    select_aid[0] = 0x00;
    select_aid[1] = 0xA4;
    select_aid[2] = 0x04;
    select_aid[3] = 0x00;
    select_aid[4] = 0x07;
    memcpy(&select_aid[5], aid, 7);
    select_aid[12] = 0x00;
    if (!apdu_step(select_aid, sizeof(select_aid), rx, &rx_len, kApduRxCap)) {
        out->fail_reason = "SELECT AID failed";
        return false;
    }

    // 3. GET PROCESSING OPTIONS. Two real variants, chosen by what the card's
    // own FCI asks for:
    //   (a) tag 9F38 (PDOL) present -> echo the requested data objects back in
    //       a tag-83 command template. [DONOR] emv_ask_for_pdol() finds the
    //       tag; emv_read_visa() supplies every fill value. Real hardware
    //       needs this: a real Visa card answered variant (b) with 69 85,
    //       "Conditions of use not satisfied".
    //   (b) no PDOL -> the plain `80 A8 00 00 02 83 00 00`. [DONOR]
    //       emv_get_processing_options_no_pdol(). Unchanged baseline; a real
    //       Mastercard-style card in hand still takes exactly this path.
    static const uint8_t kTagPdol[] = {0x9F, 0x38};
    static const uint8_t kGpoNoPdol[] = {0x80, 0xA8, 0x00, 0x00, 0x02, 0x83, 0x00, 0x00};

    const uint8_t *pdol_ptr = nullptr;
    size_t pdol_len = 0;
    uint8_t gpo_cmd[5 + 2 + kMaxPdolDataLen + 1];
    size_t gpo_len = 0;
    bool used_pdol = false;

    if (ber_tlv_find(rx, rx_len, kTagPdol, sizeof(kTagPdol), &pdol_ptr, &pdol_len) &&
        pdol_len > 0) {
        gpo_len = build_pdol_gpo(pdol_ptr, pdol_len, gpo_cmd, sizeof(gpo_cmd));
        if (gpo_len == 0) {
            out->fail_reason = "Card's PDOL (tag 9F38) is malformed or too large";
            return false;
        }
        used_pdol = true;
        Serial.printf("quarky-tab5: [nfc-emv-read] card requested a PDOL (%u bytes) "
                      "-- sending PDOL-based GPO (%u bytes)\n",
                      (unsigned)pdol_len, (unsigned)gpo_len);
    } else {
        memcpy(gpo_cmd, kGpoNoPdol, sizeof(kGpoNoPdol));
        gpo_len = sizeof(kGpoNoPdol);
    }

    if (!apdu_step(gpo_cmd, gpo_len, rx, &rx_len, kApduRxCap)) {
        // No blind retry of the other variant: the card told us which one it
        // wants via its own FCI, and a second guess would only add latency to
        // a card that is genuinely non-compliant or out of range.
        out->fail_reason = used_pdol
            ? "GET PROCESSING OPTIONS failed even with the card's own PDOL "
              "(non-compliant or unusual card)"
            : "GET PROCESSING OPTIONS failed (no PDOL in the card's FCI)";
        return false;
    }

    // A transport-level success is not an EMV-level success: the card can
    // (and, on real hardware, did) answer with a status word instead of data.
    // The real Visa card's no-PDOL GPO came back as exactly `69 85`
    // ("Conditions of use not satisfied") and the old code reported that as
    // the far less useful "No Application File Locator (tag 94)". Surface the
    // real SW1SW2 so the next failure is diagnosable from the screen alone.
    // ISO 7816-4: a normal completion is 90 00, in the LAST two bytes of the
    // R-APDU.
    if (rx_len < 2 || rx[rx_len - 2] != 0x90 || rx[rx_len - 1] != 0x00) {
        std::snprintf(s_fail_buf, sizeof(s_fail_buf),
                      "GPO rejected by card: SW=%02X%02X (%s PDOL)",
                      rx_len >= 2 ? rx[rx_len - 2] : 0,
                      rx_len >= 2 ? rx[rx_len - 1] : 0,
                      used_pdol ? "with the card's own" : "no");
        out->fail_reason = s_fail_buf;
        return false;
    }

    // The GPO response comes in one of two real EMV shapes (EMV Book 3
    // 6.5.8.4), and which one a card uses is not predictable from the AID:
    //   Format 2, template tag 77: AIP (82) and AFL (94) as ordinary nested
    //     TLVs -- what ber_tlv_find() already handled before this task.
    //   Format 1, template tag 80: a bare concatenation, AIP in the first 2
    //     bytes and the AFL in everything after, with NO inner tags at all.
    // A Format 1 response was never parseable by the tag-94 search alone, so
    // it is handled explicitly rather than reported as "no AFL".
    static const uint8_t kTagAfl[] = {0x94};
    static const uint8_t kTagGpoFmt1[] = {0x80};
    static const uint8_t kTagTrack2[] = {0x57};
    const uint8_t *afl_ptr = nullptr;
    size_t afl_len = 0;
    uint8_t afl[64];

    if (ber_tlv_find(rx, rx_len, kTagAfl, sizeof(kTagAfl), &afl_ptr, &afl_len) &&
        afl_len >= 4 && (afl_len % 4) == 0) {
        // Format 2 -- as before.
    } else if (ber_tlv_find(rx, rx_len, kTagGpoFmt1, sizeof(kTagGpoFmt1), &afl_ptr,
                            &afl_len) &&
               afl_len >= 6 && ((afl_len - 2) % 4) == 0) {
        afl_ptr += 2; // skip the AIP
        afl_len -= 2;
    } else {
        // Some cards answer a PDOL-based GPO with Track 2 equivalent data
        // (tag 57) directly and no AFL to walk at all -- which is exactly what
        // [DONOR] emv_read_visa() reads out of its own GPO response ("PAN
        // found in Track 2 Equivalent Data"). Take it and stop; there is
        // nothing further to read.
        const uint8_t *t2 = nullptr;
        size_t t2_len = 0;
        if (ber_tlv_find(rx, rx_len, kTagTrack2, sizeof(kTagTrack2), &t2, &t2_len) &&
            t2_len > 0 &&
            parse_track2(t2, t2_len, out->pan, sizeof(out->pan), out->expiry,
                         sizeof(out->expiry))) {
            out->ok = true;
            return true;
        }
        out->fail_reason = "No Application File Locator (tag 94/80) in GPO response";
        return false;
    }

    if (afl_len > sizeof(afl)) {
        afl_len = sizeof(afl); // defensive cap -- real AFLs are a handful of entries
        afl_len -= (afl_len % 4);
    }
    memcpy(afl, afl_ptr, afl_len);

    // 4. Walk the AFL, READ RECORD each entry. [DONOR] read_afl() /
    // emv_read_record(). 4 bytes per AFL entry: SFI in the top 5 bits of
    // byte 0, first/last record number in bytes 1/2 (byte 3, the "number of
    // records involved in offline data authentication", is not needed here).
    static const uint8_t kTagPan[] = {0x5A};
    static const uint8_t kTagExp[] = {0x5F, 0x24};
    static const uint8_t kTagEff[] = {0x5F, 0x25};

    bool got_pan = false;
    for (size_t i = 0; i + 4 <= afl_len; i += 4) {
        const uint8_t sfi = static_cast<uint8_t>(afl[i] >> 3);
        const uint8_t rec_start = afl[i + 1];
        const uint8_t rec_end = afl[i + 2];
        if (rec_end < rec_start) {
            continue; // malformed AFL entry -- skip rather than loop forever
        }
        for (uint8_t rec = rec_start; rec <= rec_end; rec++) {
            const uint8_t read_record[5] = {
                0x00, 0xB2, rec, static_cast<uint8_t>((sfi << 3) | 0x04U), 0x00};
            uint8_t *const rec_rx = s_record_rx; // file-scope, see its comment
            size_t rec_rx_len = 0;
            if (!apdu_step(read_record, sizeof(read_record), rec_rx, &rec_rx_len,
                          kApduRxCap)) {
                continue; // some AFL-listed records may legitimately not exist
            }

            const uint8_t *v = nullptr;
            size_t vlen = 0;
            if (!got_pan &&
                ber_tlv_find(rec_rx, rec_rx_len, kTagPan, sizeof(kTagPan), &v, &vlen) &&
                vlen > 0) {
                format_pan(v, vlen, out->pan, sizeof(out->pan));
                got_pan = true;
            }
            // Fallback for records that carry Track 2 equivalent data (tag 57)
            // but no separate tag 5A -- a real and common shape, and the one
            // [DONOR] emv_read_visa() relies on exclusively. Only consulted
            // when tag 5A has not already produced a PAN.
            if (!got_pan &&
                ber_tlv_find(rec_rx, rec_rx_len, kTagTrack2, sizeof(kTagTrack2), &v,
                             &vlen) &&
                vlen > 0 &&
                parse_track2(v, vlen, out->pan, sizeof(out->pan), out->expiry,
                             sizeof(out->expiry))) {
                got_pan = true;
            }
            if (out->expiry[0] == '\0' &&
                ber_tlv_find(rec_rx, rec_rx_len, kTagExp, sizeof(kTagExp), &v, &vlen) &&
                vlen >= 2) {
                // Card encodes YYMM (BCD). Display MM/YY -- [DONOR]
                // parse_validto()'s own comment: "format in card is
                // YEAR/MONTH but I want MONTH/YEAR since is the standard
                // format".
                std::snprintf(out->expiry, sizeof(out->expiry), "%02X/%02X", v[1], v[0]);
            }
            if (!out->has_effective &&
                ber_tlv_find(rec_rx, rec_rx_len, kTagEff, sizeof(kTagEff), &v, &vlen) &&
                vlen >= 2) {
                std::snprintf(out->effective, sizeof(out->effective), "%02X/%02X", v[1], v[0]);
                out->has_effective = true;
            }
        }
    }

    if (!got_pan) {
        out->fail_reason = "Read the card's records but found no PAN (tag 5A/57)";
        return false;
    }
    out->ok = true;
    return true;
}

// --- Screen / state machine --------------------------------------------------
// Mirrors nfc_read.cpp's own screen/poll() state machine (kIdle -> kBringUp ->
// kScanning -> kFound, with kFailed a latched bring-up failure) -- same
// reasoning: one-shot chip bring-up on the first Scan press, not re-attempted
// every tick, and a bring-up failure latches rather than retrying forever
// against an empty socket.
enum class ScanState : uint8_t {
    kIdle,
    kBringUp,
    kScanning,
    kFound,
    kFailed,
};

ScanState s_state = ScanState::kIdle;
bool s_unit_ready = false;
uint32_t s_last_attempt_ms = 0;
constexpr uint32_t kScanIntervalMs = 250; // same cadence as nfc_read.cpp

lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_result_label = nullptr;
lv_obj_t *s_save_label = nullptr;

// --- Saveable snapshot of the last successful read ---------------------------
// Both save actions below need the ISO14443-3 identity (UID/SAK/ATQA) that
// nfca_detect() already produced plus, for the .nfc export, the real ATS
// iso14443_4_activate() captured. Neither costs an extra exchange -- they are
// by-products of the read that just succeeded -- so they are snapshotted here
// the moment it succeeds and the buttons simply write them out.
bool s_have_card = false;
NfcCommon::TagInfo s_last_tag{};
NfcFlipperFormat::Iso14443_4aRecord s_last_nfc{};

void set_status(const char *text) {
    if (s_status_label != nullptr) {
        lv_label_set_text(s_status_label, text);
    }
}

void run_bring_up() {
    if (St25r3916::nfca_poller_begin()) {
        s_unit_ready = true;
        s_state = ScanState::kScanning;
        s_last_attempt_ms = millis();
        set_status("Present a payment card...");
        return;
    }
    s_unit_ready = false;
    s_state = ScanState::kFailed;
    set_status("NFC unit not responding");
    if (s_result_label != nullptr) {
        lv_label_set_text(s_result_label,
                          "Check that the NFC unit is plugged into PORT.A.\n"
                          "Leave and re-enter this screen to retry.");
    }
    Serial.println("quarky-tab5: [nfc-emv-read] bring-up failed, latched "
                   "(no 4 Hz retry)");
}

void render_result(const EmvResult &r) {
    if (s_result_label == nullptr) {
        return;
    }
    char buf[256];
    if (r.has_effective) {
        std::snprintf(buf, sizeof(buf),
                      "%s\nPAN: %s\nExpiry: %s\nEffective: %s",
                      r.vendor, r.pan, r.expiry, r.effective);
    } else {
        std::snprintf(buf, sizeof(buf), "%s\nPAN: %s\nExpiry: %s",
                      r.vendor, r.pan, r.expiry[0] != '\0' ? r.expiry : "Unknown");
    }
    lv_label_set_text(s_result_label, buf);
}

// Runs the whole RATS+APDU read sequence in ONE poll() tick.
//
// DISCLOSED POLL() BUDGET EXCEPTION (same policy nfc_read.cpp's own
// run_bring_up() documents): this single tick can run up to
// kOverallReadBudgetMs (2.5 s) in the worst case -- far past the project's
// usual ~50 ms poll() ceiling, but a one-shot cost triggered only once per
// card presentation, not something that recurs every tick. Real cost for a
// working card is expected to be a handful of milliseconds per APDU (per
// st25r3916_driver.cpp's own documented I2C-dominated timing), so low tens of
// milliseconds total; the 2.5 s ceiling exists to bound a slow or
// malfunctioning card, not to describe the common case. See
// st25r3916_driver.cpp's kMaxSingleExchangeMs comment for the matching
// per-exchange half of this reasoning, and this file's own EXECUTION MODEL
// header note for why (unlike nfc_mifare_crack.cpp) this does not need a
// separate worker task: nothing here is an uninterruptible multi-second
// computation, only a bounded sequence of short I2C exchanges.
// Snapshots the just-read card's ISO14443-3/4 identity for the two save
// actions. Everything here was already captured by the read that just
// succeeded -- nfca_detect() produced the UID/SAK/ATQA, iso14443_4_activate()
// the ATS -- so this costs no extra RF exchange.
//
// The tag-library record deliberately reuses NfcCommon::TagInfo unchanged, so
// a saved EMV card is immediately emulatable through Task 24's existing
// Listen Mode ("Emulate" in the tag-library browse screen) with no new work on
// that side: that path needs exactly UID + SAK + ATQA, all of which are real
// here. type_name marks it as an EMV card and carries the vendor from the AID
// dictionary, so it is distinguishable from a plain MIFARE/NTAG entry in the
// browse list.
void snapshot_card(const St25r3916::Iso14443aTag &tag, const EmvResult &r) {
    s_last_tag = NfcCommon::TagInfo{};
    memcpy(s_last_tag.uid, tag.uid, sizeof(s_last_tag.uid));
    s_last_tag.uid_len = tag.uid_len;
    s_last_tag.sak = tag.sak;
    s_last_tag.atqa[0] = tag.atqa[0];
    s_last_tag.atqa[1] = tag.atqa[1];
    // type_name is char[24]; snprintf truncates safely for a long "Unknown
    // vendor (AID ...)" string, which still reads as "EMV Unknown vendor..."
    // in the browse list.
    std::snprintf(s_last_tag.type_name, sizeof(s_last_tag.type_name), "EMV %s", r.vendor);

    s_last_nfc = NfcFlipperFormat::Iso14443_4aRecord{};
    memcpy(s_last_nfc.uid, tag.uid, sizeof(s_last_nfc.uid));
    s_last_nfc.uid_len = tag.uid_len;
    s_last_nfc.atqa[0] = tag.atqa[0];
    s_last_nfc.atqa[1] = tag.atqa[1];
    s_last_nfc.sak = tag.sak;
    s_last_nfc.ats_len = static_cast<uint8_t>(
        St25r3916::iso14443_4_get_ats(s_last_nfc.ats, sizeof(s_last_nfc.ats)));

    s_have_card = true;
}

void attempt_read(const St25r3916::Iso14443aTag &tag) {
    EmvResult result{};
    const bool ok = read_emv_card(&result);
    if (ok) {
        render_result(result);
        snapshot_card(tag, result);
        s_state = ScanState::kFound;
        set_status("Card read");
        if (s_save_label != nullptr) {
            lv_label_set_text(s_save_label, "");
        }
        Serial.printf("quarky-tab5: [nfc-emv-read] read OK: vendor=%s pan=%s "
                      "expiry=%s\n",
                      result.vendor, result.pan, result.expiry);
        return;
    }

    set_status(result.fail_reason != nullptr ? result.fail_reason : "EMV read failed");
    if (s_result_label != nullptr) {
        lv_label_set_text(s_result_label, "Remove the card and try again.");
    }
    Serial.printf("quarky-tab5: [nfc-emv-read] read failed: %s\n",
                  result.fail_reason != nullptr ? result.fail_reason : "(no reason)");
    // Stay in kScanning: the tag is left ACTIVE (nfca_detect() was called
    // with keep_active=true), which per ISO14443-3 does not answer a further
    // WUPA -- the natural recovery is the same "lift the card, tap again"
    // gesture presenting any new card requires, which re-powers the tag from
    // scratch. Not re-cycling the field/state machine here on every failure
    // keeps this module's state machine identical in shape to nfc_read.cpp's;
    // real-hardware use will confirm whether this recovery feels natural or
    // needs an explicit field cycle added later.
}

void set_save_status(const char *text) {
    if (s_save_label != nullptr) {
        lv_label_set_text(s_save_label, text);
    }
}

// "Save .nfc": a real Flipper-format ISO14443-4A device file. See
// nfc_flipper_format.h for the format citations.
void do_save_nfc() {
    if (!s_have_card || s_state != ScanState::kFound) {
        set_save_status("Read a card first.");
        return;
    }
    char path[96];
    if (!NfcFlipperFormat::build_path(s_last_nfc, path, sizeof(path))) {
        set_save_status("Save failed (no usable UID).");
        return;
    }
    const bool ok = NfcFlipperFormat::write(storage, path, s_last_nfc);
    char msg[128];
    std::snprintf(msg, sizeof(msg), ok ? "Saved %s" : "Failed to write %s", path);
    set_save_status(msg);
    Serial.printf("quarky-tab5: [nfc-emv-read] .nfc export %s: %s\n",
                  ok ? "OK" : "FAILED", path);
}

// "Save to Tag Library": the SAME record format Task 10 writes and Task 24's
// Listen Mode "Emulate" already consumes -- see snapshot_card()'s comment.
void do_save_library() {
    if (!s_have_card || s_state != ScanState::kFound) {
        set_save_status("Read a card first.");
        return;
    }
    const bool ok = NfcTagLibrary::save(storage, s_last_tag);
    set_save_status(ok ? "Saved to tag library (emulatable)." : "Library save failed.");
    Serial.printf("quarky-tab5: [nfc-emv-read] tag-library save %s (type=%s)\n",
                  ok ? "OK" : "FAILED", s_last_tag.type_name);
}

void teardown() {
    s_status_label = nullptr;
    s_result_label = nullptr;
    s_save_label = nullptr;
    s_state = ScanState::kIdle;
    s_last_attempt_ms = 0;
    s_have_card = false;

    if (s_unit_ready) {
        St25r3916::nfca_poller_end();
    }
    s_unit_ready = false;

    // Same GPIO53-arbiter release convention as every other NFC-unit screen
    // (nfc_read.cpp/nfc_mifare_crack.cpp/nfc_amiibo.cpp's own teardown()s):
    // unconditional, safe no-op if this session never claimed the pin.
    nfc_release_external_i2c();
}

lv_obj_t *build_screen() {
    s_state = ScanState::kIdle;
    s_unit_ready = false;
    s_last_attempt_ms = 0;
    s_have_card = false;

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("NFC: EMV Card Read", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *warn = lv_label_create(content);
    lv_label_set_text(warn,
        "Reads PAN/expiry/vendor from a contactless payment card. Read-only -- "
        "no payment or transaction logic. NFC unit only.");
    lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Idle");
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);

    lv_obj_t *scan_btn = lv_button_create(content);
    lv_obj_t *scan_lbl = lv_label_create(scan_btn);
    lv_label_set_text(scan_lbl, "Scan");
    lv_obj_add_event_cb(scan_btn, [](lv_event_t *) {
        // Non-blocking, same discipline as nfc_read.cpp's own Scan handler:
        // this only arms the state machine. Bring-up (if needed) and the
        // read itself both happen from poll().
        if (s_state == ScanState::kFailed) {
            return; // latched; re-enter the screen to retry
        }
        if (s_unit_ready) {
            s_state = ScanState::kScanning;
            set_status("Present a payment card...");
        } else {
            s_state = ScanState::kBringUp;
            set_status("Bringing up unit...");
        }
        if (s_result_label != nullptr) {
            lv_label_set_text(s_result_label, "");
        }
        s_have_card = false; // a new scan invalidates the previous snapshot
        set_save_status("");
    }, LV_EVENT_CLICKED, nullptr);

    s_result_label = lv_label_create(content);
    lv_label_set_long_mode(s_result_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_result_label, "No card read yet");

    // Save actions, same button/handler convention as nfc_read.cpp's own
    // "Save to Library" button (non-blocking handler, result reported in a
    // dedicated label below them). Both re-check s_state/s_have_card
    // themselves rather than being created/destroyed on success, matching
    // that precedent.
    lv_obj_t *save_nfc_btn = lv_button_create(content);
    lv_obj_t *save_nfc_lbl = lv_label_create(save_nfc_btn);
    lv_label_set_text(save_nfc_lbl, "Save .nfc (Flipper)");
    lv_obj_add_event_cb(save_nfc_btn, [](lv_event_t *) { do_save_nfc(); },
                        LV_EVENT_CLICKED, nullptr);

    lv_obj_t *save_lib_btn = lv_button_create(content);
    lv_obj_t *save_lib_lbl = lv_label_create(save_lib_btn);
    lv_label_set_text(save_lib_lbl, "Save to Tag Library");
    lv_obj_add_event_cb(save_lib_btn, [](lv_event_t *) { do_save_library(); },
                        LV_EVENT_CLICKED, nullptr);

    s_save_label = lv_label_create(content);
    lv_label_set_long_mode(s_save_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_save_label, "");

    lv_obj_add_event_cb(content, [](lv_event_t *) { teardown(); },
                        LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace

void register_module() {
    g_registry.register_module({"nfc_emv_read", "NFC: EMV Card Read",
                                Category::NFC, Affinity::TAB5_NATIVE,
                                start, nullptr});
}

void poll() {
    if (s_status_label == nullptr || s_result_label == nullptr) {
        return; // no screen open
    }

    if (s_state == ScanState::kBringUp) {
        run_bring_up();
        return;
    }
    if (s_state != ScanState::kScanning) {
        return;
    }

    const uint32_t now = millis();
    if (now - s_last_attempt_ms < kScanIntervalMs) {
        return;
    }
    s_last_attempt_ms = now;

    St25r3916::Iso14443aTag tag{};
    const St25r3916::NfcaResult res = St25r3916::nfca_detect(&tag, /*keep_active=*/true);
    switch (res) {
        case St25r3916::NfcaResult::kFound:
            attempt_read(tag); // single documented poll()-tick budget exception
                               // -- see attempt_read()'s own comment
            return;
        case St25r3916::NfcaResult::kNoTag:
            return; // status already says "Present a payment card..."
        case St25r3916::NfcaResult::kCollision:
            set_status("Multiple tags -- present one at a time");
            return;
        case St25r3916::NfcaResult::kProtocolError:
            set_status("Tag answered but the exchange failed");
            return;
        case St25r3916::NfcaResult::kHardwareError:
        default:
            set_status("NFC unit stopped responding");
            s_state = ScanState::kFailed;
            Serial.println("quarky-tab5: [nfc-emv-read] ST25R3916 I2C failure "
                           "mid-scan -- scanning latched off");
            return;
    }
}

} // namespace NfcEmvRead
