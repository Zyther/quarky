#include "nfc_amiibo.h"

#include "nfc_common.h"

#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include "ws1850s_driver.h"
#include "../../hal/nfc_pn532.h" // nfc_release_external_i2c() -- GPIO53 arbiter

#include "../../../boards/tab5/pins_config.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include <Wire.h>

// Vendored at lib/MFRC522_I2C/ (Task 4, commit 8152dddc). Used the same way
// nfc_read.cpp/nfc_mifare_crack.cpp already use it: PICC-level calls only,
// never PCD_Init() -- Ws1850sDriver::init() already owns the register level.
#include <MFRC522_I2C.h>

// ===========================================================================
// SOURCES / PORT CITATIONS
//
// Donor: ~/src/firmware/src/modules/rfid/RFID2.cpp (Bruce). NOT
// ~/src/firmware/src/modules/rfid/amiibo.cpp / ESP-Amiibolink -- confirmed by
// fetching bmorcelli/ESP-Amiibolink's real source directly: it is a NimBLE
// client (Nordic UART Service UUIDs, searchDevice()/connectToDevice()/
// cmdUploadDumpData()) for a separate physical commercial device, with zero
// NFC/ISO14443 tag I/O. Not referenced anywhere in this file.
//
//   * Tag-type dispatch (Ultralight/NTAG21x vs MIFARE Classic):
//     RFID2::read_data_blocks(), RFID2.cpp:248-273 -- switches on
//     mfrc522.PICC_GetType(sak); PICC_TYPE_MIFARE_UL is the only branch this
//     module needs (Classic is nfc_mifare_crack.cpp's territory).
//   * Real read loop: RFID2::read_mifare_ultralight_data_blocks(),
//     RFID2.cpp:420-462 -- `for (byte page = 0; page <= 252; page += 4)`,
//     each iteration one mfrc522.MIFARE_Read(page, buffer, &byteCount) call
//     (16 bytes = 4 pages), looping until STATUS_MIFARE_NACK signals
//     end-of-memory (the real, standard NFC Forum Type 2 Tag "read past end"
//     behavior -- a natural loop terminator, not a special error). While
//     iterating, page 3 offset 2 (buffer[4*offset+2] when page+offset==3,
//     RFID2.cpp:434-447) is the Capability Container byte: 0x12 -> NTAG213
//     (45 pages), 0x3E -> NTAG215 (135 pages), 0x6D -> NTAG216 (231 pages).
//   * The caller's own post-read adjustment, RFID2.cpp:264-265 --
//     `dataPages = (readStatus == SUCCESS && dataPages > 0) ? dataPages - 1
//     : dataPages;` then `if (totalPages == 0) totalPages = dataPages;`.
//     Ported verbatim below (see finish_read()): a Type 2 tag's MIFARE_Read
//     near the end of memory returns the last valid page's data repeated to
//     fill the 16-byte response rather than NACKing immediately, so the very
//     last successfully-read group over-counts by one page; RFID2.cpp trims
//     that empirically rather than trying to detect the repeat.
//   * Real write, single 4-byte page via PICC_CMD_UL_WRITE:
//     RFID2::write_mifare_ultralight_data_block(), RFID2.cpp:530-543 --
//     mfrc522.MIFARE_Ultralight_Write((byte)block, buffer, size). No
//     authentication anywhere in this path (Ultralight/NTAG21x has no MIFARE
//     Classic-style crypto) -- matches nfc_mifare_crack.cpp's explicit
//     Classic-only framing for why THAT module needs auth and this one does
//     not.
//   * The write loop's page range, RFID2::write_data_blocks()'s Ultralight
//     branch, RFID2.cpp:497-499 -- `if (pageIndex < 4 || pageIndex >=
//     dataPages - 5) continue;`. Ported verbatim below: pages 0-3 are UID/
//     internal/lock/CC (not user data -- rewriting them risks bricking the
//     tag's own identity or locking bits), and the last 5 pages of an NTAG21x
//     tag are its dynamic-lock/config/password/PACK pages (real for NTAG215:
//     page 130 dynamic lock, 131/132 CFG0/CFG1, 133 PWD, 134 PACK+RFUI),
//     which the donor deliberately never rewrites either.
//
// Composition pattern: same as nfc_read.cpp/nfc_mifare_crack.cpp -- a
// file-scope MFRC522_I2C instance, PCD_Init() never called on it,
// Ws1850sDriver::init()/field_on() own the register level, GPIO53 arbitration
// via nfc_release_external_i2c() on teardown (Ws1850sDriver's own calls take
// the claim internally through nfc_ensure_external_i2c_begun()).
// ===========================================================================

extern FeatureRegistry g_registry;

namespace NfcAmiibo {

// Real donor loop bound (RFID2.cpp:428): pages 0..252 in groups of 4, i.e.
// page indices 0..255 inclusive across the last group.
static constexpr int kDonorLastPageStart = 252;
static constexpr int kMaxPages = 256;

enum class State : uint8_t {
    kIdle,
    kBringUp,          // one-shot chip bring-up
    kFailed,           // bring-up failed; latched until screen re-entered
    kWaitTagForRead,   // polling for a tag to read
    kReadingPages,      // read loop in progress
    kReadDone,          // a dump is held in s_dump[], ready to inspect/write
    kWaitTagForWrite,   // polling for a (possibly different) tag to write onto
    kWritingPages,      // write loop in progress
    kWriteDone,         // write completed (success or reported failure)
};

static State s_state = State::kIdle;

static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_result_label = nullptr;
static lv_obj_t *s_write_status_label = nullptr;

static uint32_t s_last_attempt_ms = 0;
static constexpr uint32_t kPollIntervalMs = 100; // tag-wait poll cadence;
                                                  // page read/write happens
                                                  // every tick once in
                                                  // progress (bounded by the
                                                  // chip's own 25 ms timer,
                                                  // MFRC522_I2C.cpp:221-222 --
                                                  // same budget class as
                                                  // nfc_read.cpp's detect
                                                  // calls, no new budget
                                                  // exception needed)

static bool s_unit_ready = false; // owes teardown a field_off()

// The in-memory dump. 4 bytes/page, up to kMaxPages pages -- 1 KiB, trivial
// on this MCU. No SD persistence (the plan's own interface line only
// requires register_module(); the controller notes explicitly call out not
// over-building a save/load flow beyond what's asked).
static uint8_t s_dump[kMaxPages][4];
static int s_data_pages = 0;      // pages actually captured by the last read
static uint8_t s_cc_byte = 0;     // Capability Container byte (page 3 offset 2)
static int s_cc_total_pages = 0;  // CC-derived tag size, 0 if unrecognised

static uint8_t s_uid[10];
static uint8_t s_uid_len = 0;

static int s_read_page = 0;   // next page-group start for the read loop
static int s_write_page = 0;  // next page index for the write loop

static MFRC522_I2C s_mfrc(TAB5_RFID2_I2C_ADDR, -1, &Wire1);

static void set_status(const char *text) {
    if (s_status_label != nullptr) {
        lv_label_set_text(s_status_label, text);
    }
}

static void set_write_status(const char *text) {
    if (s_write_status_label != nullptr) {
        lv_label_set_text(s_write_status_label, text);
    }
}

// --- Bring-up ---------------------------------------------------------------
// Same one-shot-per-screen-session model as nfc_read.cpp's run_bring_up():
// Ws1850sDriver::init() applies the full MFRC522 register programme
// (soft reset, TX baud, timer prescaler/reload, 100% ASK, CRC preset) plus
// antenna-on; it is deliberately NOT re-run per scan attempt.
static void run_bring_up() {
    bool ok = Ws1850sDriver::init() && Ws1850sDriver::field_on();
    if (!ok) {
        s_state = State::kFailed;
        s_unit_ready = false;
        set_status("RFID2 unit not responding");
        if (s_result_label != nullptr) {
            lv_label_set_text(s_result_label,
                              "Check that the unit is plugged into PORT.A.\n"
                              "Leave and re-enter this screen to retry.");
        }
        Serial.println("quarky-tab5: [nfc-amiibo] bring-up failed, latched "
                       "(no retry)");
        return;
    }
    s_unit_ready = true;
    s_state = State::kWaitTagForRead;
    set_status("Present tag to read...");
}

// --- Tag selection -----------------------------------------------------------
// PICC_IsNewCardPresent() (REQA + ATQA) then PICC_ReadCardSerial()
// (anticollision -> UID + SAK), the same MFRC522 library contract nfc_read.cpp
// documents and depends on Ws1850sDriver::init() having already programmed
// the chip (PCD_Init() is never called on s_mfrc itself).
//
// Returns true only for a real NFC Forum Type 2 (Ultralight-family) tag --
// dispatch mirrors RFID2::read_data_blocks()'s switch (RFID2.cpp:255-269),
// this module only ever wants the PICC_TYPE_MIFARE_UL branch.
static bool try_select_ultralight(const char **reject_reason_out) {
    *reject_reason_out = nullptr;
    if (!s_mfrc.PICC_IsNewCardPresent()) {
        return false;
    }
    if (!s_mfrc.PICC_ReadCardSerial()) {
        return false;
    }
    const uint8_t picc_type = s_mfrc.PICC_GetType(s_mfrc.uid.sak);
    if (picc_type != MFRC522_I2C::PICC_TYPE_MIFARE_UL) {
        // Not this module's tag family -- release it and let the caller
        // report and keep waiting rather than latching a failure.
        s_mfrc.PICC_HaltA();
        *reject_reason_out =
            reinterpret_cast<const char *>(s_mfrc.PICC_GetTypeName(picc_type));
        return false;
    }
    s_uid_len = s_mfrc.uid.size;
    if (s_uid_len > sizeof(s_uid)) {
        s_uid_len = sizeof(s_uid);
    }
    for (uint8_t i = 0; i < s_uid_len; i++) {
        s_uid[i] = s_mfrc.uid.uidByte[i];
    }
    return true;
}

// --- Read --------------------------------------------------------------------

static const char *cc_tag_name(uint8_t cc) {
    switch (cc) {
        case 0x12: return "NTAG213";
        case 0x3E: return "NTAG215";
        case 0x6D: return "NTAG216";
        default: return "Unknown Ultralight/NTAG21x (unrecognised CC byte)";
    }
}

static void render_read_result() {
    if (s_result_label == nullptr) {
        return;
    }
    char uid_str[64];
    uid_str[0] = '\0';
    NfcCommon::format_uid(s_uid, s_uid_len, uid_str, sizeof(uid_str));

    char buf[256];
    if (s_cc_total_pages > 0) {
        std::snprintf(buf, sizeof(buf),
                      "UID: %s\n%s (CC=0x%02X, %d pages)\nCaptured %d pages "
                      "(%d bytes)",
                      uid_str, cc_tag_name(s_cc_byte), s_cc_byte,
                      s_cc_total_pages, s_data_pages, s_data_pages * 4);
    } else {
        std::snprintf(buf, sizeof(buf),
                      "UID: %s\nUltralight-family tag, CC byte not "
                      "recognised (0x%02X)\nCaptured %d pages (%d bytes)",
                      uid_str, s_cc_byte, s_data_pages, s_data_pages * 4);
    }
    lv_label_set_text(s_result_label, buf);
}

// One page-group (4 pages / 16 bytes) per call -- RFID2.cpp:420-462 ported
// directly, including the page-3 Capability Container capture.
static void read_tick() {
    uint8_t buffer[18];
    uint8_t byte_count = sizeof(buffer);
    uint8_t status = s_mfrc.MIFARE_Read(static_cast<uint8_t>(s_read_page),
                                        buffer, &byte_count);

    if (status != MFRC522_I2C::STATUS_OK) {
        // STATUS_MIFARE_NACK == real end-of-memory terminator (RFID2.cpp:
        // 431-432), not a failure.
        const bool ok = (status == MFRC522_I2C::STATUS_MIFARE_NACK);
        s_mfrc.PICC_HaltA();

        if (ok) {
            // Donor's own post-read adjustment (RFID2.cpp:264-265).
            if (s_data_pages > 0) {
                s_data_pages--;
            }
            if (s_cc_total_pages == 0) {
                s_cc_total_pages = s_data_pages;
            }
            s_state = State::kReadDone;
            render_read_result();
            set_status("Tag read.");
            Serial.printf("quarky-tab5: [nfc-amiibo] read complete: %d "
                          "pages, CC=0x%02X\n", s_data_pages, s_cc_byte);
        } else {
            s_state = State::kWaitTagForRead;
            set_status("Read failed -- re-present tag");
            Serial.printf("quarky-tab5: [nfc-amiibo] MIFARE_Read failed, "
                          "status=%u\n", (unsigned)status);
        }
        return;
    }

    for (uint8_t offset = 0; offset < 4; offset++) {
        const int page = s_read_page + offset;
        if (page >= kMaxPages) {
            // Safety cap -- should not happen given the donor's own loop
            // bound below, but never write past the buffer.
            s_mfrc.PICC_HaltA();
            if (s_cc_total_pages == 0) {
                s_cc_total_pages = s_data_pages;
            }
            s_state = State::kReadDone;
            render_read_result();
            set_status("Tag read (capacity cap reached).");
            return;
        }
        std::memcpy(s_dump[page], &buffer[4 * offset], 4);
        if (page == 3) {
            s_cc_byte = buffer[4 * offset + 2];
            switch (s_cc_byte) {
                case 0x12: s_cc_total_pages = 45; break;
                case 0x3E: s_cc_total_pages = 135; break;
                case 0x6D: s_cc_total_pages = 231; break;
                default: break;
            }
        }
        s_data_pages++;
    }

    s_read_page += 4;
    if (s_read_page > kDonorLastPageStart) {
        // Donor's own loop bound (RFID2.cpp:428) reached without a NACK --
        // treat as end-of-memory too, same as the NACK branch above.
        s_mfrc.PICC_HaltA();
        if (s_data_pages > 0) {
            s_data_pages--;
        }
        if (s_cc_total_pages == 0) {
            s_cc_total_pages = s_data_pages;
        }
        s_state = State::kReadDone;
        render_read_result();
        set_status("Tag read.");
        return;
    }

    char progress[48];
    std::snprintf(progress, sizeof(progress), "Reading... %d pages so far",
                  s_data_pages);
    set_status(progress);
}

// --- Write -------------------------------------------------------------------

// One page (4 bytes) per call -- RFID2.cpp:530-543 ported directly. Page
// range enforced exactly as RFID2::write_data_blocks()'s Ultralight branch
// (RFID2.cpp:497-499): skip UID/internal/lock/CC (pages 0-3) and the tag's
// own dynamic-lock/config/password/PACK pages (the last 5 pages of whatever
// was actually captured).
static void write_tick() {
    const int last_writable = s_data_pages - 5; // exclusive upper bound

    if (s_write_page >= last_writable) {
        s_mfrc.PICC_HaltA();
        s_state = State::kWriteDone;
        char msg[64];
        std::snprintf(msg, sizeof(msg), "Write complete: %d pages written.",
                      last_writable - 4);
        set_write_status(msg);
        Serial.printf("quarky-tab5: [nfc-amiibo] write complete: %d pages\n",
                      last_writable - 4);
        return;
    }

    uint8_t page_buf[4];
    std::memcpy(page_buf, s_dump[s_write_page], 4);
    uint8_t status = s_mfrc.MIFARE_Ultralight_Write(
        static_cast<uint8_t>(s_write_page), page_buf, sizeof(page_buf));

    if (status != MFRC522_I2C::STATUS_OK) {
        s_mfrc.PICC_HaltA();
        s_state = State::kWriteDone;
        char msg[64];
        std::snprintf(msg, sizeof(msg), "Write failed at page %d (status %u)",
                      s_write_page, (unsigned)status);
        set_write_status(msg);
        Serial.printf("quarky-tab5: [nfc-amiibo] MIFARE_Ultralight_Write "
                      "failed at page %d, status=%u\n", s_write_page,
                      (unsigned)status);
        return;
    }

    s_write_page++;
    char progress[48];
    std::snprintf(progress, sizeof(progress), "Writing... page %d/%d",
                  s_write_page - 4, last_writable - 4);
    set_write_status(progress);
}

// --- Teardown ----------------------------------------------------------------

static void teardown() {
    s_status_label = nullptr;
    s_result_label = nullptr;
    s_write_status_label = nullptr;

    s_state = State::kIdle;
    s_last_attempt_ms = 0;
    s_data_pages = 0;
    s_cc_byte = 0;
    s_cc_total_pages = 0;
    s_uid_len = 0;
    s_read_page = 0;
    s_write_page = 0;

    if (s_unit_ready) {
        Ws1850sDriver::field_off();
    }
    s_unit_ready = false;

    // Same GPIO53 arbiter release nfc_read.cpp's teardown() performs -- safe
    // no-op if this screen's session never actually claimed the pin.
    nfc_release_external_i2c();
}

// --- Screen ------------------------------------------------------------------

static lv_obj_t *build_screen() {
    s_state = State::kIdle;
    s_unit_ready = false;
    s_last_attempt_ms = 0;
    s_data_pages = 0;
    s_cc_byte = 0;
    s_cc_total_pages = 0;
    s_uid_len = 0;

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Amiibo / NTAG21x (RFID2)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Idle");

    lv_obj_t *read_btn = lv_button_create(content);
    lv_obj_t *read_lbl = lv_label_create(read_btn);
    lv_label_set_text(read_lbl, "Read Tag");
    lv_obj_add_event_cb(read_btn, [](lv_event_t *) {
        if (s_state == State::kFailed) {
            return; // latched; re-enter the screen to retry
        }
        if (s_unit_ready) {
            s_state = State::kWaitTagForRead;
            set_status("Present tag to read...");
        } else {
            s_state = State::kBringUp;
            set_status("Bringing up unit...");
        }
        s_data_pages = 0;
        s_cc_byte = 0;
        s_cc_total_pages = 0;
        s_read_page = 0;
        if (s_result_label != nullptr) {
            lv_label_set_text(s_result_label, "No tag yet");
        }
        set_write_status("");
    }, LV_EVENT_CLICKED, nullptr);

    s_result_label = lv_label_create(content);
    lv_label_set_text(s_result_label, "No tag yet");

    lv_obj_t *write_btn = lv_button_create(content);
    lv_obj_t *write_lbl = lv_label_create(write_btn);
    lv_label_set_text(write_lbl, "Write Dump Back");
    lv_obj_add_event_cb(write_btn, [](lv_event_t *) {
        // Re-check state at click time, same defensive pattern nfc_read.cpp's
        // Save button uses against a stale/absent result.
        if (s_state != State::kReadDone && s_state != State::kWriteDone) {
            set_write_status("Read a tag first.");
            return;
        }
        if (s_data_pages <= 9) {
            // Guarantees last_writable (= s_data_pages - 5) > 4, i.e. at
            // least one real writable page exists between the reserved head
            // (0-3) and tail (last 5) regions.
            set_write_status("Dump too small to write safely.");
            return;
        }
        s_state = State::kWaitTagForWrite;
        s_write_page = 4;
        set_write_status("Present tag to write (re-tap if it was just read)...");
    }, LV_EVENT_CLICKED, nullptr);

    s_write_status_label = lv_label_create(content);
    lv_label_set_text(s_write_status_label, "");

    lv_obj_add_event_cb(content, [](lv_event_t *) { teardown(); },
                        LV_EVENT_DELETE, nullptr);

    return screen;
}

static void start() {
    ScreenStack::push(build_screen());
}

void register_module() {
    g_registry.register_module({"nfc_amiibo", "RFID2: Amiibo (NTAG21x)",
                                Category::NFC, Affinity::TAB5_NATIVE,
                                start, nullptr});
}

void poll() {
    if (s_status_label == nullptr) {
        return; // no screen open
    }

    if (s_state == State::kBringUp) {
        run_bring_up();
        return;
    }

    if (s_state == State::kReadingPages || s_state == State::kWritingPages) {
        // Page I/O ticks run every poll() call once started (no wait
        // interval) -- each is a single bounded MFRC522 transceive (see the
        // kPollIntervalMs comment above), same class of per-tick cost as
        // nfc_read.cpp's detect calls, not a new budget exception.
        if (s_state == State::kReadingPages) {
            read_tick();
        } else {
            write_tick();
        }
        return;
    }

    if (s_state != State::kWaitTagForRead && s_state != State::kWaitTagForWrite) {
        return;
    }

    const uint32_t now = millis();
    if (now - s_last_attempt_ms < kPollIntervalMs) {
        return;
    }
    s_last_attempt_ms = now;

    const char *reject_reason = nullptr;
    if (!try_select_ultralight(&reject_reason)) {
        if (reject_reason != nullptr) {
            char msg[80];
            std::snprintf(msg, sizeof(msg), "Not an Ultralight/NTAG21x tag (%s)",
                          reject_reason);
            if (s_state == State::kWaitTagForRead) {
                set_status(msg);
            } else {
                set_write_status(msg);
            }
        }
        return;
    }

    if (s_state == State::kWaitTagForRead) {
        s_state = State::kReadingPages;
        s_read_page = 0;
        set_status("Reading...");
    } else {
        s_state = State::kWritingPages;
        set_write_status("Writing...");
    }
}

} // namespace NfcAmiibo
