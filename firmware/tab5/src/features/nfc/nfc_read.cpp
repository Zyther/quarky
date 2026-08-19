#include "nfc_read.h"

#include "nfc_common.h"

#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include "ws1850s_driver.h"
#include "st25r3916_driver.h"

#include "../../../boards/tab5/pins_config.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#include <Wire.h>

// ST RFAL-based NFC discovery (used to extract ISO14443A/NFC-A UID via
// technology detection + activation state machine instead of guessing raw
// register-level anticollision sequences).
#include <rfal_nfc.h>
#include <rfal_rfst25r3916.h>
#include <st25r3916_config.h>
#include <st_errno.h>

// UniGeek’s cached donor library (via PlatformIO lib_extra_dirs):
// provides PICC_IsNewCardPresent() + PICC_ReadCardSerial(), and stores
// the UID bytes in the instance's public `uid` member.
#include <MFRC522_I2C.h>

extern FeatureRegistry g_registry;

namespace NfcRead {

enum class TargetUnit { NFC, RFID2 };

static TargetUnit s_target = TargetUnit::RFID2;

static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_result_label = nullptr;

static bool s_scan_armed = false;
static bool s_tag_found = false;
static uint32_t s_last_attempt_ms = 0;

// --- ST RFAL state (NFC unit only) -------------------------------------------
static bool s_rfal_initialized = false;
static bool s_rfal_discovery_started = false;
static bool s_nfc_uid_ready = false;
static uint8_t s_nfc_uid[10] = {0};
static uint8_t s_nfc_uid_len = 0;

// Create the RFAL reader bound to our I2C bus. The Tab5 NFC connector has no
// wired IRQ line, so we pass -1 and rely on RFAL's worker-based polling.
static RfalRfST25R3916Class s_rfal_reader(&Wire1, -1);
static RfalNfcClass s_rfal_nfc(&s_rfal_reader);

static void on_nfc_state_change(rfalNfcState state) {
    if (state != RFAL_NFC_STATE_ACTIVATED) return;

    rfalNfcDevice *device = nullptr;
    if (s_rfal_nfc.rfalNfcGetActiveDevice(&device) != ERR_NONE || device == nullptr) {
        return;
    }

    if (device->nfcidLen == 0 || device->nfcidLen > 10) return;
    if (device->nfcid == nullptr) return;

    s_nfc_uid_len = device->nfcidLen;
    std::memcpy(s_nfc_uid, device->nfcid, s_nfc_uid_len);
    s_nfc_uid_ready = true;
}

static void set_scan_state(bool armed, bool found) {
    s_scan_armed = armed;
    s_tag_found = found;
    if (s_status_label) {
        if (!armed) {
            lv_label_set_text(s_status_label, "Idle");
        } else if (found) {
            lv_label_set_text(s_status_label, "Tag found");
        } else {
            lv_label_set_text(s_status_label, "Scanning...");
        }
    }
}

// RFID2: WS1850S is MFRC522/PN512-family silicon. We use UniGeek's proven
// MFRC522_I2C PICC flow to get UID bytes for Task-4 "baseline tag read".
static bool try_read_rfid2_uid(NfcCommon::TagInfo *out) {
    if (out == nullptr) return false;

    if (!Ws1850sDriver::init()) return false;
    // Antenna field enable is part of init(), but keep it explicit and cheap.
    Ws1850sDriver::field_on();

    // Create the MFRC522 helper for the real chip at 0x28 on Wire1.
    // We keep it static so its internal UID struct persists across polls.
    static MFRC522_I2C s_mfrc(TAB5_RFID2_I2C_ADDR, -1, &Wire1);

    // Ensure RF field for this attempt.
    s_mfrc.PCD_AntennaOn();

    // MFRC522 library contract: call PICC_IsNewCardPresent() before
    // PICC_ReadCardSerial().
    if (!s_mfrc.PICC_IsNewCardPresent()) return false;
    if (!s_mfrc.PICC_ReadCardSerial()) return false;

    const uint8_t uid_len = s_mfrc.uid.size; // 4, 7, or 10 (per library)
    if (uid_len == 0 || uid_len > 10) return false;

    out->uid_len = uid_len;
    for (uint8_t i = 0; i < uid_len; i++) {
        out->uid[i] = s_mfrc.uid.uidByte[i];
    }

    // Minimal, honest type name: we can refine later once we port more
    // PICC/MIFARE classification work. For DoD baseline UID read, length
    // is a meaningful invariant.
    std::snprintf(out->type_name, sizeof(out->type_name),
                  "ISO14443A UID (%uB)", (unsigned)uid_len);

    // Leave the chip in a clean state for the next scan.
    s_mfrc.PICC_HaltA();
    s_mfrc.PCD_StopCrypto1();
    return true;
}

// NFC: ST25R3916 is an RF front-end; tag discovery/anticollision should be
// done via ST's RFAL state machine, not raw register guessing. This Task-4
// scaffolding intentionally compiles and does RFID2 baseline UID read, but
// the ST UID extraction is a dedicated porting task to be completed.
static bool try_read_nfc_uid(NfcCommon::TagInfo *out) {
    (void)out; // currently UID is returned via shared RFAL callback state
    // RFAL runs as a worker-driven state machine: we must call
    // rfalNfcWorker() repeatedly until the activation callback fires.

    // Satisfy Task-4's dependency on the project's own ST25R3916 bring-up,
    // mainly to ensure the PORT.A I2C bus + chip are usable.
    if (!St25r3916::init()) return false;
    St25r3916::field_on();

    if (!s_rfal_initialized) {
        const ReturnCode err = s_rfal_nfc.rfalNfcInitialize();
        if (err != ERR_NONE) {
            Serial.printf("quarky-tab5: [nfc-read] rfalNfcInitialize failed: %d\n", (int)err);
            return false;
        }
        s_rfal_initialized = true;
    }

    if (!s_rfal_discovery_started) {
        rfalNfcDiscoverParam discover;
        std::memset(&discover, 0, sizeof(discover));

        discover.compMode = RFAL_COMPLIANCE_MODE_NFC;
        discover.devLimit = RFAL_ESP32_DEFAULT_DEVICE_LIMIT;
        discover.techs2Find = RFAL_NFC_POLL_TECH_A;
        discover.totalDuration = RFAL_ESP32_DEFAULT_DISCOVERY_DURATION_MS;
        discover.notifyCb = on_nfc_state_change;

        s_nfc_uid_ready = false;
        s_nfc_uid_len = 0;

        const ReturnCode err = s_rfal_nfc.rfalNfcDiscover(&discover);
        if (err != ERR_NONE) {
            Serial.printf("quarky-tab5: [nfc-read] rfalNfcDiscover failed: %d\n", (int)err);
            return false;
        }
        s_rfal_discovery_started = true;
    }

    // Drive RFAL one step.
    s_rfal_nfc.rfalNfcWorker();

    if (!s_nfc_uid_ready) return false;
    return true; // caller will pull s_nfc_uid via shared state below
}

static bool try_read_uid(NfcCommon::TagInfo *out) {
    if (out == nullptr) return false;
    switch (s_target) {
        case TargetUnit::RFID2:
            return try_read_rfid2_uid(out);
        case TargetUnit::NFC:
            // NFC path: try_read_nfc_uid updates shared s_nfc_uid/s_len and
            // returns true when a UID has been captured by the RFAL callback.
            if (!try_read_nfc_uid(out)) return false;
            out->uid_len = s_nfc_uid_len;
            for (uint8_t i = 0; i < out->uid_len; i++) out->uid[i] = s_nfc_uid[i];
            std::snprintf(out->type_name, sizeof(out->type_name), "ISO14443A UID (%uB)",
                          (unsigned)out->uid_len);
            St25r3916::field_off();
            s_rfal_nfc.rfalNfcDeactivate(true);
            s_rfal_discovery_started = false;
            s_nfc_uid_ready = false;
            return true;
    }
    return false;
}

static lv_obj_t *build_screen(TargetUnit target) {
    s_target = target;
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
        set_scan_state(true /*armed*/, false /*found*/);
    }, LV_EVENT_CLICKED, nullptr);

    s_result_label = lv_label_create(content);
    lv_label_set_text(s_result_label, "No tag yet");

    // Teardown: clear UI pointers + in-flight poll state.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        s_status_label = nullptr;
        s_result_label = nullptr;
        s_scan_armed = false;
        s_tag_found = false;
        s_last_attempt_ms = 0;
    }, LV_EVENT_DELETE, nullptr);

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
    if (!s_scan_armed || s_tag_found) return;
    if (!s_status_label || !s_result_label) return;

    const uint32_t now = millis();
    if (now - s_last_attempt_ms < 250) return;
    s_last_attempt_ms = now;

    NfcCommon::TagInfo info{};
    std::memset(info.uid, 0, sizeof(info.uid));
    info.uid_len = 0;
    info.type_name[0] = '\0';

    if (!try_read_uid(&info)) {
        lv_label_set_text(s_status_label, "Scanning...");
        return;
    }

    char uid_str[64];
    uid_str[0] = '\0';
    NfcCommon::format_uid(info.uid, info.uid_len, uid_str, sizeof(uid_str));

    char result[96];
    std::snprintf(result, sizeof(result), "%s\nUID: %s", info.type_name, uid_str);
    lv_label_set_text(s_result_label, result);

    s_tag_found = true;
    lv_label_set_text(s_status_label, "Tag found");
}

} // namespace NfcRead

