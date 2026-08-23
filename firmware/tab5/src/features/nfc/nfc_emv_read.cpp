#include "nfc_emv_read.h"

#include "st25r3916_driver.h"
#include "../../hal/nfc_pn532.h" // nfc_release_external_i2c() -- GPIO53 arbiter
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
// SCOPE / DISCLOSED GAP: the Visa-specific PDOL-based GET PROCESSING OPTIONS
// path (`EMVReader::emv_read_visa()` in the donor, keyed off `emv_ask_for_pdol()`
// returning a non-empty PDOL) is NOT implemented. This module always sends
// the no-PDOL GPO (`80 A8 00 00 02 83 00 00`) and, if the card responds with
// an error (a card that requires its PDOL echoed back will reject this),
// reports a clear "may require a PDOL" failure rather than reading nothing
// silently and calling it success. Per this task's brief, the no-PDOL path is
// the required baseline and this is the honestly-disclosed gap, not a
// half-working guess at the Visa path.
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

bool ber_tlv_find_in(const uint8_t *data, size_t len, size_t start, size_t end,
                     const uint8_t *tag, uint8_t tag_len,
                     const uint8_t **val_out, size_t *val_len_out) {
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
        if (val_pos + vlen > end) {
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
                                val_out, val_len_out)) {
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

// Real total wall-clock budget for one full read attempt (RATS + up to ~10
// APDU exchanges). Checked before every single apdu_step() call below, in
// addition to st25r3916_driver.cpp's own per-exchange kMaxSingleExchangeMs
// cap -- see this file's EXECUTION MODEL header comment. 2.5 s leaves wide
// margin under the ~5 s task watchdog even accounting for one exchange
// hitting its own worst-case per-exchange timeout on top of this.
constexpr uint32_t kOverallReadBudgetMs = 2500U;
uint32_t s_read_deadline_ms = 0;

bool apdu_step(const uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t *rx_len_out,
              uint8_t rx_cap) {
    *rx_len_out = 0;
    if (static_cast<int32_t>(millis() - s_read_deadline_ms) >= 0) {
        return false; // overall read budget exceeded -- give up, don't retry
    }
    size_t rx_len = 0;
    const bool ok = St25r3916::apdu_transceive(tx, tx_len, rx, rx_cap, &rx_len);
    *rx_len_out = static_cast<uint8_t>(rx_len);
    return ok;
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

// The full read sequence: SELECT PPSE -> SELECT AID -> GET PROCESSING OPTIONS
// (no-PDOL) -> walk the AFL with READ RECORD. Returns false with
// out->fail_reason set on any step failing; out->vendor may still be filled
// in (AID is known before GPO/READ RECORD run) even when PAN extraction later
// fails, which the caller displays either way.
bool read_emv_card(EmvResult *out) {
    *out = EmvResult{};
    s_read_deadline_ms = millis() + kOverallReadBudgetMs;

    if (!St25r3916::iso14443_4_activate()) {
        out->fail_reason = "Card did not answer RATS (not an ISO14443-4 card?)";
        return false;
    }

    uint8_t rx[255];
    uint8_t rx_len = 0;

    // 1. SELECT PPSE ("2PAY.SYS.DDF01"). [DONOR] emv_ask_for_aid().
    static const uint8_t kSelectPpse[] = {
        0x00, 0xA4, 0x04, 0x00, 0x0E,
        0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E,
        0x44, 0x44, 0x46, 0x30, 0x31,
        0x00};
    if (!apdu_step(kSelectPpse, sizeof(kSelectPpse), rx, &rx_len, sizeof(rx))) {
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
    if (!apdu_step(select_aid, sizeof(select_aid), rx, &rx_len, sizeof(rx))) {
        out->fail_reason = "SELECT AID failed";
        return false;
    }

    // 3. GET PROCESSING OPTIONS, no-PDOL. [DONOR]
    // emv_get_processing_options_no_pdol(). REQUIRED BASELINE per this task's
    // brief -- see this file's header SCOPE/DISCLOSED GAP note for the
    // Visa/PDOL path this deliberately does not implement.
    static const uint8_t kGpoNoPdol[] = {0x80, 0xA8, 0x00, 0x00, 0x02, 0x83, 0x00, 0x00};
    if (!apdu_step(kGpoNoPdol, sizeof(kGpoNoPdol), rx, &rx_len, sizeof(rx))) {
        out->fail_reason = "GET PROCESSING OPTIONS failed (card may require a "
                           "PDOL -- not supported by this reader)";
        return false;
    }

    static const uint8_t kTagAfl[] = {0x94};
    const uint8_t *afl_ptr = nullptr;
    size_t afl_len = 0;
    if (!ber_tlv_find(rx, rx_len, kTagAfl, sizeof(kTagAfl), &afl_ptr, &afl_len) ||
        afl_len == 0 || (afl_len % 4) != 0) {
        out->fail_reason = "No Application File Locator (tag 94) in GPO response";
        return false;
    }
    uint8_t afl[64];
    if (afl_len > sizeof(afl)) {
        afl_len = sizeof(afl); // defensive cap -- real AFLs are a handful of entries
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
            uint8_t rec_rx[255];
            uint8_t rec_rx_len = 0;
            if (!apdu_step(read_record, sizeof(read_record), rec_rx, &rec_rx_len,
                          sizeof(rec_rx))) {
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
        out->fail_reason = "Read the card's records but found no PAN (tag 5A)";
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
void attempt_read() {
    EmvResult result{};
    const bool ok = read_emv_card(&result);
    if (ok) {
        render_result(result);
        s_state = ScanState::kFound;
        set_status("Card read");
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

void teardown() {
    s_status_label = nullptr;
    s_result_label = nullptr;
    s_state = ScanState::kIdle;
    s_last_attempt_ms = 0;

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
    }, LV_EVENT_CLICKED, nullptr);

    s_result_label = lv_label_create(content);
    lv_label_set_long_mode(s_result_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_result_label, "No card read yet");

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
            attempt_read(); // single documented poll()-tick budget exception --
                            // see attempt_read()'s own comment
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
