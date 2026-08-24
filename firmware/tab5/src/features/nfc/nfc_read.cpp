#include "nfc_read.h"

#include "nfc_common.h"
#include "nfc_tag_library.h"

#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include "ws1850s_driver.h"
#include "st25r3916_driver.h"
#include "../../hal/nfc_pn532.h" // nfc_release_external_i2c() -- GPIO53 arbiter
#include "../../hal/storage_sd.h"

#include "../../../boards/tab5/pins_config.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include <Wire.h>

// Vendored at lib/MFRC522_I2C/ (see that library's src/MFRC522_I2C.h for the
// upstream repo/commit and why it is in-tree rather than fetched from another
// project's PlatformIO cache). Used ONLY for the PICC-level anticollision
// sequence -- this project owns the register level via Ws1850sDriver.
#include <MFRC522_I2C.h>

// NOTE ON WHAT IS *NOT* INCLUDED HERE. An earlier version of this file drove
// the NFC unit through ST's RFAL stack (<rfal_nfc.h> / <rfal_rfst25r3916.h>),
// constructing RfalRfST25R3916Class(&Wire1, -1) with a comment claiming RFAL
// falls back to worker-based polling without an IRQ pin. That claim was false:
// RFAL's I2C path returns ERR_PARAM when int_pin < 0 and gates every single
// interrupt read on digitalRead(int_pin), while the Tab5's HY2.0 PORT.A
// connector carries only GND/5V/SDA/SCL. The tile was non-functional by
// construction. The NFC-unit path below now uses this project's own
// St25r3916::nfca_* API, which polls the chip's IRQ *status registers* instead
// -- see st25r3916_driver.cpp's SOURCES block for the full story and citations.

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp (Phase 1 Task 10); handed to
                          // NfcTagLibrary::save() below, same extern
                          // nfc_tag_library_ui.cpp already declares for this
                          // exact real-hardware storage instance.

namespace NfcRead {

enum class TargetUnit { NFC, RFID2 };

// Screen-lifetime state machine. Chip bring-up is a one-shot (kBringUp) rather
// than something poll() re-attempts at 4 Hz, and a bring-up failure LATCHES
// (kFailed) instead of retrying forever against a socket with nothing in it --
// both required by this task's review.
enum class ScanState : uint8_t {
    kIdle,      // screen open, user has not pressed Scan
    kBringUp,   // Scan pressed; the next poll() tick performs one-shot bring-up
    kScanning,  // bring-up done, polling for a tag
    kFound,     // a tag was read; nothing further happens until Scan is pressed
    kFailed,    // bring-up failed; latched, no retry until the screen is re-entered
};

static TargetUnit s_target = TargetUnit::RFID2;
static ScanState  s_state  = ScanState::kIdle;

static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_result_label = nullptr;
static lv_obj_t *s_save_label = nullptr;

// The most recently successfully read tag, valid only while s_state ==
// kFound (the Save button's click handler re-checks s_state before using
// this, same defensive discipline as every other cross-tick state this file
// keeps -- see run_bring_up()'s own latching comment).
static NfcCommon::TagInfo s_last_found_tag{};

static uint32_t s_last_attempt_ms = 0;

// True once this screen session has successfully brought its unit up, so
// teardown knows whether it owes the chip a field_off()/poller_end().
static bool s_unit_ready = false;

// Interval between detection attempts. Both units' single detect pass is
// bounded (RFID2: the donor library's own 36 ms transceive deadline, normally
// cut short by the chip's 25 ms timer IRQ when nothing is in the field; NFC:
// St25r3916::nfca_detect()'s own ~25 ms budget), so a tick never approaches
// the project's ~50 ms poll() ceiling. The gap exists to leave the rest of
// loop() -- LVGL, the C2 link, RF433 -- plenty of room, not because a single
// attempt is expensive.
static constexpr uint32_t kScanIntervalMs = 250;

// The RFID2 unit's PICC-level helper. Construction is trivial (it only stores
// the address, reset pin and TwoWire pointer -- MFRC522_I2C.cpp:21-28), so a
// file-scope instance touches no hardware before bring-up runs.
static MFRC522_I2C s_mfrc(TAB5_RFID2_I2C_ADDR, -1, &Wire1);

static void set_status(const char *text) {
    if (s_status_label != nullptr) {
        lv_label_set_text(s_status_label, text);
    }
}

// --- Bring-up ---------------------------------------------------------------

// One-shot, deliberately NOT inside the Scan click handler and deliberately
// bounded to a single poll() tick.
//
// DISCLOSED POLL() BUDGET EXCEPTION (the project's ~50 ms rule allows one if
// it is commented): this single tick can run long -- for RFID2, up to ~205 ms
// (Ws1850sDriver::init()'s 50 ms settle + up to 3x50 ms soft-reset poll + nine
// register writes + 4 ms post-init settle); for the NFC unit, ~30 ms
// (St25r3916::nfca_poller_begin()'s ~35 register writes at 100 kHz, a 10 ms
// oscillator wait and the 5 ms NFC-A guard time). It happens at most ONCE per
// screen session, and its result latches either way, which is the whole point:
// the version this replaced called Ws1850sDriver::init() from inside every
// scan attempt and so paid that cost four times a second, forever, whenever no
// unit was attached.
static void run_bring_up() {
    bool ok = false;

    if (s_target == TargetUnit::RFID2) {
        // Antenna field enable is part of init(); field_on() is called
        // explicitly so teardown's field_off() is the obvious counterpart.
        ok = Ws1850sDriver::init() && Ws1850sDriver::field_on();
    } else {
        ok = St25r3916::nfca_poller_begin();
    }

    if (!ok) {
        s_state = ScanState::kFailed;
        s_unit_ready = false;
        const char *msg = (s_target == TargetUnit::RFID2)
                              ? "RFID2 unit not responding"
                              : "NFC unit not responding";
        set_status(msg);
        if (s_result_label != nullptr) {
            lv_label_set_text(s_result_label,
                              "Check that the unit is plugged into PORT.A.\n"
                              "Leave and re-enter this screen to retry.");
        }
        Serial.printf("quarky-tab5: [nfc-read] %s -- bring-up failed, latched "
                      "(no 4 Hz retry)\n", msg);
        return;
    }

    s_unit_ready = true;
    s_state = ScanState::kScanning;
    s_last_attempt_ms = millis();
    set_status("Scanning...");
}

// --- Per-unit detection -----------------------------------------------------

// RFID2 / WS1850S. The PICC call sequence below is the donor libraries' own
// and is deliberately unchanged.
//
// ORDERING DEPENDENCY, DO NOT REORDER: correctness here rests on
// Ws1850sDriver::init() having already applied PCD_Init()'s register programme
// (soft reset, TX baud rates, timer prescaler/reload, 100% ASK, CRC preset,
// antenna on) to this same chip. PCD_Init() is deliberately never called on
// the MFRC522_I2C instance itself -- this project owns the register level, the
// donor library is used only from PICC_* upwards -- so the PICC calls would be
// talking to an unconfigured chip if bring-up had not run first.
static bool try_read_rfid2_uid(NfcCommon::TagInfo *out) {
    if (out == nullptr) {
        return false;
    }

    // MFRC522 library contract: PICC_IsNewCardPresent() (which sends REQA and
    // collects the ATQA) must precede PICC_ReadCardSerial() (the cascade-level
    // anticollision loop that yields the UID and SAK).
    if (!s_mfrc.PICC_IsNewCardPresent()) {
        return false;
    }
    if (!s_mfrc.PICC_ReadCardSerial()) {
        return false;
    }

    const uint8_t uid_len = s_mfrc.uid.size; // 4, 7 or 10 per the library
    if (uid_len == 0 || uid_len > sizeof(out->uid)) {
        return false;
    }

    out->uid_len = uid_len;
    for (uint8_t i = 0; i < uid_len; i++) {
        out->uid[i] = s_mfrc.uid.uidByte[i];
    }

    // Real PICC type from the SAK, not a guess from the UID length. The
    // returned string lives in flash; on ESP32 __FlashStringHelper* is a plain
    // mapped pointer, so a normal "%s" read is safe. TagInfo::type_name is 24
    // bytes (the interface shape this task's brief specified), so the longest
    // names -- "MIFARE Ultralight or Ultralight C" -- truncate.
    const uint8_t picc_type = s_mfrc.PICC_GetType(s_mfrc.uid.sak);
    std::snprintf(out->type_name, sizeof(out->type_name), "%s",
                  reinterpret_cast<const char *>(s_mfrc.PICC_GetTypeName(picc_type)));

    // Real SAK (Task 24 / nfc_common.h). ATQA is deliberately left at its
    // {0,0} "not captured" sentinel -- MFRC522_I2C's Uid struct (see that
    // library's header) never surfaces the ATQA bytes PICC_RequestA() saw,
    // only UID/SAK survive past PICC_ReadCardSerial().
    out->sak = s_mfrc.uid.sak;
    out->atqa[0] = 0;
    out->atqa[1] = 0;

    // Leave the tag halted and crypto state clean for the next scan.
    s_mfrc.PICC_HaltA();
    s_mfrc.PCD_StopCrypto1();
    return true;
}

// NFC unit / ST25R3916. Real ISO14443-3A: REQA -> anticollision -> SELECT
// across as many cascade levels as the UID needs, driven by polling the
// chip's IRQ status registers (this hardware has no IRQ line). Implemented in
// St25r3916::nfca_detect(); see st25r3916_driver.cpp for the citations and for
// what is deliberately not implemented (multi-tag collision resolution).
// SAK 0x00 is the real NFC Forum Type 2 Tag (MIFARE Ultralight / NTAG21x
// family) marker, not a guess: the vendored MFRC522_I2C library's own
// PICC_GetType() has `case 0x00: return PICC_TYPE_MIFARE_UL;`, and
// nfc_amiibo.cpp already dispatches on exactly that value (via
// PICC_TYPE_MIFARE_UL) for its own real page-read loop on the RFID2 unit.
// The real NTAG this project has in hand reports exactly this SAK.
static constexpr uint8_t kSakMifareUltralight = 0x00U;

static bool try_read_nfc_uid(NfcCommon::TagInfo *out) {
    if (out == nullptr) {
        return false;
    }

    St25r3916::Iso14443aTag tag{};
    // keep_active=true, ALWAYS -- the SAK that decides whether page content is
    // worth reading is only known after this call returns, and a tag HALTed by
    // the default auto-SLP_REQ answers nothing afterwards. Every path below
    // therefore owes the tag an explicit HALT: t2t_read_pages() sends one
    // itself (documented in its own header comment), and the non-Type-2 path
    // calls nfca_halt() directly. Without that, a card left sitting on the
    // antenna would stay ACTIVE, ignore the next pass's WUPA, and read exactly
    // once per physical presentation -- the precise bug nfca_detect()'s own
    // SLP_REQ step exists to prevent.
    const St25r3916::NfcaResult res = St25r3916::nfca_detect(&tag, /*keep_active=*/true);

    switch (res) {
        case St25r3916::NfcaResult::kFound:
            break;
        case St25r3916::NfcaResult::kNoTag:
            return false;
        case St25r3916::NfcaResult::kCollision:
            set_status("Multiple tags -- present one at a time");
            // A tag may well have got part-way through selection before this
            // was reported; HALT so the next pass's WUPA still reaches it.
            // (kNoTag needs none -- nothing answered at all -- and
            // kHardwareError is a dead bus, where another exchange is
            // pointless.)
            St25r3916::nfca_halt();
            return false;
        case St25r3916::NfcaResult::kProtocolError:
            set_status("Tag answered but the exchange failed");
            St25r3916::nfca_halt();
            return false;
        case St25r3916::NfcaResult::kHardwareError:
        default:
            set_status("NFC unit stopped responding");
            s_state = ScanState::kFailed;
            Serial.println("quarky-tab5: [nfc-read] ST25R3916 I2C failure mid-scan "
                           "-- scanning latched off");
            return false;
    }

    if (tag.uid_len == 0 || tag.uid_len > sizeof(out->uid)) {
        St25r3916::nfca_halt(); // see the keep_active=true note above
        return false;
    }
    out->uid_len = tag.uid_len;
    for (uint8_t i = 0; i < tag.uid_len; i++) {
        out->uid[i] = tag.uid[i];
    }
    // No SAK->name table is ported for this path (the RFID2 path gets one for
    // free from the donor library; writing a second, independent one here
    // would be inventing a mapping rather than citing one). Report the two
    // real ISO14443-3 identity bytes instead of a made-up product name.
    std::snprintf(out->type_name, sizeof(out->type_name), "ISO14443A SAK %02X",
                  (unsigned)tag.sak);
    Serial.printf("quarky-tab5: [nfc-read] ATQA=%02X%02X SAK=%02X UID len=%u\n",
                  tag.atqa[1], tag.atqa[0], tag.sak, (unsigned)tag.uid_len);
    // Real SAK/ATQA (Task 24 / nfc_common.h) -- both already sitting in `tag`
    // from St25r3916::nfca_detect(), unlike the RFID2 path above.
    out->sak = tag.sak;
    out->atqa[0] = tag.atqa[0];
    out->atqa[1] = tag.atqa[1];

    // --- Real page-content capture (2026-08-24, Task 24's content-emulation
    // extension) -----------------------------------------------------------
    // Why it lives HERE rather than in a new screen: this is already the one
    // place in the project where an NFC-unit scan becomes a TagInfo that the
    // "Save to Library" button then writes to SD, and the Emulate screen
    // consumes that exact record. Capturing the pages anywhere else would
    // mean a second scan of the same tag, and a saved record that may or may
    // not have content depending on which screen the user happened to use.
    //
    // COST, disclosed: t2t_read_pages() runs one T2T READ per 4 pages
    // (~4 ms each on this 100 kHz bus), so this branch adds ~50 ms for an
    // NTAG213, ~140 ms for an NTAG215 and ~240 ms for an NTAG216 to the ONE
    // poll() tick that finds such a tag. That is the same class of disclosed,
    // one-shot budget exception as run_bring_up()'s own ~205 ms tick
    // documented above -- it happens once per successful scan, not at the
    // 4 Hz scanning cadence, and is nowhere near the ~5 s task watchdog.
    //
    // Non-Type-2 tags (EMV cards at SAK 0x20, MIFARE Classic at 0x08/0x18,
    // ...) skip this entirely and keep page_count == 0, i.e. exactly the
    // UID/SAK/ATQA-only record Task 10 originally scoped.
    out->page_count = 0;
    if (tag.sak == kSakMifareUltralight) {
        uint8_t pages = 0;
        // t2t_read_pages() sends the trailing HALT itself, success or
        // failure -- see its header comment.
        const bool ok = St25r3916::t2t_read_pages(
            &out->pages[0][0], sizeof(out->pages), &pages);
        out->page_count = pages;
        if (pages > 0) {
            // Deliberately states the MEASURED page count rather than mapping
            // it to a product name. nfc_amiibo.cpp's page_count_tag_name()
            // does have a real, cited 45/135/231 -> NTAG213/215/216 mapping,
            // but it also documents (from real hardware, 2026-08-21) that a
            // genuine amiibo-programmed tag defeats the companion CC-byte
            // detection -- so a bare measured count is the claim that is
            // always true. If the read failed outright, the original
            // "ISO14443A SAK 00" name is left alone rather than replaced with
            // a misleading "0 pg".
            std::snprintf(out->type_name, sizeof(out->type_name),
                          "Ultralight/NTAG %u pg", (unsigned)pages);
        }
        Serial.printf("quarky-tab5: [nfc-read] Type 2 tag (SAK 00): captured %u "
                      "pages (%u bytes)%s\n",
                      (unsigned)pages, (unsigned)(pages * NfcCommon::kT2tPageLen),
                      ok ? "" : " -- read failed, saving UID/SAK/ATQA only");
    } else {
        St25r3916::nfca_halt(); // see the keep_active=true note above
    }
    return true;
}

static bool try_read_uid(NfcCommon::TagInfo *out) {
    return (s_target == TargetUnit::RFID2) ? try_read_rfid2_uid(out)
                                           : try_read_nfc_uid(out);
}

// --- Teardown ---------------------------------------------------------------

// Called from LV_EVENT_DELETE and from nothing else. Every widget pointer is
// nulled BEFORE any driver call, so a driver that logs or takes a moment can
// never race a poll() tick into a dangling label.
static void teardown() {
    s_status_label = nullptr;
    s_result_label = nullptr;
    s_save_label = nullptr;

    s_state = ScanState::kIdle;
    s_last_attempt_ms = 0;

    if (s_unit_ready) {
        // Leaving the screen must not leave the RF field energised.
        if (s_target == TargetUnit::RFID2) {
            Ws1850sDriver::field_off();
        } else {
            St25r3916::nfca_poller_end();
        }
    }
    s_unit_ready = false;

    // This screen's NFC/RFID2 "session" is over -- release the GPIO53 claim
    // (Gpio53Arbiter::Owner::kExternalI2c) so RF433 can claim the pin
    // afterward. Unconditional, not gated on s_unit_ready: the claim happens
    // inside ensureExternalI2CBegun(), which any bring-up attempt above may
    // have already triggered even if it never got as far as setting
    // s_unit_ready. nfc_release_external_i2c() is a safe no-op if this
    // screen's session never actually claimed the pin.
    nfc_release_external_i2c();
}

// --- Screen -----------------------------------------------------------------

static lv_obj_t *build_screen(TargetUnit target) {
    s_target = target;
    s_state = ScanState::kIdle;
    s_unit_ready = false;
    s_last_attempt_ms = 0;

    const char *title = (target == TargetUnit::RFID2) ? "RFID2 Tag Read"
                                                      : "NFC Tag Read";

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(title, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Idle");

    lv_obj_t *scan_btn = lv_button_create(content);
    lv_obj_t *scan_lbl = lv_label_create(scan_btn);
    lv_label_set_text(scan_lbl, "Scan");
    lv_obj_add_event_cb(scan_btn, [](lv_event_t *) {
        // Click handlers stay non-blocking: this only arms the state machine.
        // The one-shot chip bring-up happens on the next poll() tick, so the
        // "Bringing up..." label is painted before the chip is touched.
        if (s_state == ScanState::kFailed) {
            return; // latched; re-enter the screen to retry
        }
        if (s_unit_ready) {
            s_state = ScanState::kScanning;
            set_status("Scanning...");
        } else {
            s_state = ScanState::kBringUp;
            set_status("Bringing up unit...");
        }
        if (s_save_label != nullptr) {
            lv_label_set_text(s_save_label, ""); // clear any prior save result
        }
    }, LV_EVENT_CLICKED, nullptr);

    s_result_label = lv_label_create(content);
    lv_label_set_text(s_result_label, "No tag yet");

    lv_obj_t *save_btn = lv_button_create(content);
    lv_obj_t *save_lbl = lv_label_create(save_btn);
    lv_label_set_text(save_lbl, "Save to Library");
    lv_obj_add_event_cb(save_btn, [](lv_event_t *) {
        // Re-check s_state at click time rather than trusting that a tag is
        // still current -- the same defensive pattern the Scan button above
        // uses against kFailed. s_last_found_tag is only meaningful while
        // still in kFound (a fresh Scan press moves to kBringUp/kScanning
        // and would go on to overwrite it with a DIFFERENT tag once found).
        if (s_state != ScanState::kFound) {
            if (s_save_label != nullptr) {
                lv_label_set_text(s_save_label, "Scan a tag first.");
            }
            return;
        }
        bool ok = NfcTagLibrary::save(storage, s_last_found_tag);
        if (s_save_label != nullptr) {
            lv_label_set_text(s_save_label, ok ? "Saved to library." : "Save failed.");
        }
    }, LV_EVENT_CLICKED, nullptr);

    s_save_label = lv_label_create(content);
    lv_label_set_text(s_save_label, "");

    lv_obj_add_event_cb(content, [](lv_event_t *) { teardown(); },
                        LV_EVENT_DELETE, nullptr);

    return screen;
}

static void start_nfc_unit() {
    ScreenStack::push(build_screen(TargetUnit::NFC));
}

static void start_rfid2_unit() {
    ScreenStack::push(build_screen(TargetUnit::RFID2));
}

void register_module_nfc_unit() {
    g_registry.register_module({"nfc_read_nfc", "NFC: Tag Read",
                                Category::NFC, Affinity::TAB5_NATIVE,
                                start_nfc_unit, nullptr});
}

void register_module_rfid2_unit() {
    g_registry.register_module({"nfc_read_rfid2", "RFID2: Tag Read",
                                Category::NFC, Affinity::TAB5_NATIVE,
                                start_rfid2_unit, nullptr});
}

void poll() {
    // No screen -> nothing to do. This is also the guard that makes poll()
    // free for every other feature's ticks.
    if (s_status_label == nullptr || s_result_label == nullptr) {
        return;
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

    NfcCommon::TagInfo info{};
    if (!try_read_uid(&info)) {
        // try_read_*_uid() may have set a more specific status (collision,
        // protocol error, hardware failure); only overwrite it while still
        // actually scanning.
        if (s_state == ScanState::kScanning) {
            set_status("Scanning...");
        }
        return;
    }

    char uid_str[64];
    uid_str[0] = '\0';
    NfcCommon::format_uid(info.uid, info.uid_len, uid_str, sizeof(uid_str));

    char result[192];
    if (info.page_count > 0) {
        // Real captured page content -- worth surfacing, since it is what
        // decides whether emulating this saved tag can answer a reader's READ
        // commands or only its anticollision.
        std::snprintf(result, sizeof(result),
                      "%s\nUID (%u bytes): %s\n%u pages captured (%u bytes) -- "
                      "Emulate can answer READs",
                      info.type_name, (unsigned)info.uid_len, uid_str,
                      (unsigned)info.page_count,
                      (unsigned)(info.page_count * NfcCommon::kT2tPageLen));
    } else {
        std::snprintf(result, sizeof(result), "%s\nUID (%u bytes): %s",
                      info.type_name, (unsigned)info.uid_len, uid_str);
    }
    lv_label_set_text(s_result_label, result);

    s_last_found_tag = info;
    if (s_save_label != nullptr) {
        lv_label_set_text(s_save_label, ""); // clear any earlier save result
    }

    Serial.printf("quarky-tab5: [nfc-read] %s tag: %s  UID %s\n",
                  (s_target == TargetUnit::RFID2) ? "RFID2" : "NFC",
                  info.type_name, uid_str);

    s_state = ScanState::kFound;
    set_status("Tag found");
}

} // namespace NfcRead
