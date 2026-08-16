#include "ble_karma.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis() -- needed the same way ble_scan.cpp/
                      // ble_finder.cpp/ble_sniffer.cpp/ble_clone.cpp explicitly
                      // pull this in; nothing else in this file's include list
                      // drags it in transitively
#include <esp_random.h> // esp_random() -- first use of this call in the tab5
                         // tree (grepped: no other firmware/tab5/src file calls
                         // it), and it is NOT transitively pulled in by
                         // Arduino.h/esp32-hal.h/WString.h/Esp.h in this
                         // framework version, so it needs its own include or
                         // this does not compile
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleKarmaFeature {

// NOTE, disclosed per donor research: this rotates identity whenever ANY
// nearby BLE advertisement is seen, not specifically in response to a
// scan-request PDU targeted at this device -- NimBLE's scan callback
// doesn't expose that distinction (confirmed against Poseidon's own
// ble_karma.cpp, which has the same limitation despite its file header's
// "listening for incoming scan requests" description). This is "rotate
// identity while nearby BLE traffic exists," not per-target-matched karma.
static const char *kNames[] = {
    "AirPods Pro", "AirPods Max", "Galaxy Buds Pro", "Galaxy Buds2",
    "Samsung TV", "Sony WH-1000XM4", "JBL Flip 6", "Beats Fit Pro",
    "Pixel Buds Pro", "Bose QC45", "LG TV", "Echo Dot",
    "Fitbit Charge 5", "Garmin Watch", "MX Master 3", "Magic Mouse",
};
static constexpr int kNameCount = sizeof(kNames) / sizeof(kNames[0]);

static volatile bool s_traffic_seen = false;
static bool s_active = false;
static int s_name_idx = 0;
static uint32_t s_last_rotate_ms = 0;
static lv_obj_t *s_status_label = nullptr;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) s_traffic_seen = true;
    return 0;
}

static void rotate_identity() {
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = (uint8_t)esp_random();
    addr[5] |= 0xC0;

    // IMPORTANT, real cross-feature side effect (not a bug to fix silently):
    // ble_hs_id_set_rnd() sets NimBLE's host-wide random *identity* address --
    // it is NOT scoped to this screen or to the ble_gap_adv_start() call
    // below. There is no public "unset" API in this project's actual NimBLE
    // (same finding Task 9's ble_clone.cpp already made and documented against
    // the real installed header, ~/Library/Arduino15/packages/esp32/tools/
    // esp32p4-libs/3.3.10/include/bt/host/nimble/nimble/nimble/host/include/
    // host/ble_hs_id.h -- it declares only ble_hs_id_gen_rnd/ble_hs_id_set_rnd/
    // ble_hs_id_copy_addr/ble_hs_id_infer_auto; the private ble_hs_id_reset()
    // in the underlying .c file is not exposed publicly, so there is no
    // supported way to clear the identity back to "unset"). The value set
    // here persists until either (a) the device reboots, or (b) some other
    // feature calls ble_hs_id_set_rnd() again with a different address --
    // including any subsequent call this same function makes. Meanwhile,
    // ble_hs_id_infer_auto(0, &out_addr_type) will keep returning
    // BLE_OWN_ADDR_RANDOM bound to whatever address was set last, for any
    // other feature that calls it. The currently-known affected feature is
    // firmware/tab5/src/features/ble/ble_hid_spike.cpp, which calls
    // ble_hs_id_infer_auto() in its own start() and advertises with the
    // inferred type/address.
    //
    // What is DIFFERENT here vs. Clone's version of this same issue: Clone
    // sets the identity ONCE, to one specific captured target's real MAC, so
    // the left-over address is a predictable, attacker-chosen value tied to
    // whatever was cloned. Karma instead calls ble_hs_id_set_rnd() repeatedly,
    // every 2 seconds, with a FRESH RANDOM address generated fresh each call
    // -- so whichever random address happened to be current at the exact
    // moment this screen was closed (mid-rotation, effectively arbitrary) is
    // what's left behind for ble_hid_spike.cpp or anything else calling
    // ble_hs_id_infer_auto() afterward, not a value anyone chose or can
    // predict. Same underlying persistence problem, different (non-)shape to
    // the leftover value. Disclosed rather than worked around: Karma
    // genuinely needs ble_hs_id_set_rnd() to rotate its advertised identity,
    // and inventing a fake reset mechanism not backed by a real NimBLE API
    // would be worse than leaving this documented. Only a device reboot, or
    // another feature's own ble_hs_id_set_rnd() call, clears it.
    int rc = ble_hs_id_set_rnd(addr);
    Serial.printf("quarky-tab5: [ble-karma] ble_hs_id_set_rnd rc=%d\n", rc);

    ble_gap_adv_stop();
    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = kNames[s_name_idx];
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    Serial.printf("quarky-tab5: [ble-karma] ble_gap_adv_set_fields rc=%d\n", rc);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    Serial.printf("quarky-tab5: [ble-karma] advertising as '%s' rc=%d\n", name, rc);

    s_name_idx = (s_name_idx + 1) % kNameCount;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Karma", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Starting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        ble_gap_disc_cancel();
        ble_gap_adv_stop();
        s_active = false;
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_label_set_text(s_status_label, "BLE host not ready yet, try again shortly");
        return screen;
    }

    struct ble_gap_disc_params params{};
    params.passive = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-karma] ble_gap_disc rc=%d\n", rc);
    s_active = (rc == 0);
    s_name_idx = 0;
    s_last_rotate_ms = 0;

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_karma", "BLE Karma", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    if (!s_traffic_seen) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < 2000) return;
    rotate_identity();
    s_traffic_seen = false;
    s_last_rotate_ms = now;
    if (s_status_label) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Rotating identity (%s next)", kNames[s_name_idx]);
        lv_label_set_text(s_status_label, buf);
    }
}

} // namespace BleKarmaFeature
