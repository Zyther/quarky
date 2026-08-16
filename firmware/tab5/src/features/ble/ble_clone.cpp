#include "ble_clone.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis(), portMUX_TYPE/portENTER_CRITICAL -- needed the
                      // same way ble_scan.cpp/ble_finder.cpp/ble_sniffer.cpp explicitly
                      // pull this in; nothing else in this file's include list drags it
                      // in transitively (the brief's own header list omitted this, same
                      // gap as ble_sniffer.cpp's task -- would not compile otherwise)
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleCloneFeature {

static constexpr int kMaxTargets = 16;
static BleDeviceInfo s_targets[kMaxTargets];
static int s_target_count = 0;
static portMUX_TYPE s_targets_mux = portMUX_INITIALIZER_UNLOCKED;
static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;
static bool s_cloning = false;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;
    if (event->disc.addr.type != BLE_ADDR_RANDOM) return 0; // public addrs unclonable, see file comment

    BleDeviceInfo d{};
    memcpy(d.addr, event->disc.addr.val, 6);
    ble_addr_to_str(d.addr, d.addr_str);
    d.rssi = event->disc.rssi;
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
        if (fields.name != nullptr && fields.name_len > 0) {
            int len = fields.name_len < (int)sizeof(d.name) - 1 ? fields.name_len : (int)sizeof(d.name) - 1;
            memcpy(d.name, fields.name, len);
            d.name[len] = '\0';
        }
    }
    if (d.name[0] == '\0') return 0; // only offer named, identifiable targets to clone

    portENTER_CRITICAL(&s_targets_mux);
    bool dup = false;
    for (int i = 0; i < s_target_count; i++) {
        if (memcmp(s_targets[i].addr, d.addr, 6) == 0) { dup = true; break; }
    }
    if (!dup && s_target_count < kMaxTargets) s_targets[s_target_count++] = d;
    portEXIT_CRITICAL(&s_targets_mux);
    return 0;
}

static void clone_target(int index) {
    portENTER_CRITICAL(&s_targets_mux);
    BleDeviceInfo target = s_targets[index];
    portEXIT_CRITICAL(&s_targets_mux);

    uint8_t clone_addr[6];
    memcpy(clone_addr, target.addr, 6);
    clone_addr[5] |= 0xC0; // force static-random flag bits, same as ble_clone.cpp's donor reasoning

    ble_addr_t addr{};
    addr.type = BLE_ADDR_RANDOM;
    memcpy(addr.val, clone_addr, 6);

    // IMPORTANT, real cross-feature side effect (not a bug to fix silently):
    // ble_hs_id_set_rnd() sets NimBLE's host-wide random *identity* address --
    // it is NOT scoped to this screen or to the ble_gap_adv_start() call
    // below. There is no public "unset" API in this project's actual NimBLE
    // (verified against the exact headers this firmware links against:
    // ~/Library/Arduino15/packages/esp32/tools/esp32p4-libs/3.3.10/include/
    // bt/host/nimble/nimble/nimble/host/include/host/ble_hs_id.h declares only
    // ble_hs_id_gen_rnd/ble_hs_id_set_rnd/ble_hs_id_copy_addr/
    // ble_hs_id_infer_auto -- the underlying ble_hs_id.c has a private
    // ble_hs_id_rnd_reset()/ble_hs_id_reset(), but neither is exposed in the
    // public header, so there is no supported way to clear the identity back
    // to "unset" once this call succeeds).
    //
    // The value set here persists until either (a) the device reboots (NimBLE
    // re-initializes its identity state from scratch), or (b) some other
    // feature calls ble_hs_id_set_rnd() again with a different address.
    // Meanwhile, ble_hs_id_infer_auto(0, &out_addr_type) -- documented in the
    // same header as preferring "random static address" over "public address"
    // whenever a random static address has been set -- will keep returning
    // BLE_OWN_ADDR_RANDOM bound to THIS cloned target's MAC for any other
    // feature that calls it. The currently-known affected feature is
    // firmware/tab5/src/features/ble/ble_hid_spike.cpp, which calls
    // ble_hs_id_infer_auto(0, &s_own_addr_type) in its own start() and then
    // advertises with that inferred type: if BLE Clone is used on some target
    // (e.g. a smartwatch) and the BLE HID/Bad-KB spike is triggered later --
    // same session or a later one, minutes or reboots apart, as long as no
    // reboot happened in between -- the HID peripheral will silently
    // advertise as "QuarkyKB" under the LEFT-OVER CLONED TARGET'S MAC instead
    // of whatever address it would otherwise have used. This is disclosed
    // here rather than worked around: BLE Clone genuinely needs
    // ble_hs_id_set_rnd() to impersonate a target's identity, and inventing a
    // fake reset mechanism not backed by a real NimBLE API would be worse
    // than leaving this documented. Only a device reboot, or another
    // feature's own ble_hs_id_set_rnd() call, clears it.
    int rc = ble_hs_id_set_rnd(addr.val);
    Serial.printf("quarky-tab5: [ble-clone] ble_hs_id_set_rnd rc=%d\n", rc);

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)target.name;
    fields.name_len = strlen(target.name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable, matches a real cloned peripheral
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    Serial.printf("quarky-tab5: [ble-clone] cloning '%s' rc=%d\n", target.name, rc);
    s_cloning = true;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Clone", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
        if (s_cloning) { ble_gap_adv_stop(); s_cloning = false; }
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_target_count = 0;
    struct ble_gap_disc_params params{};
    params.passive = 0;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-clone] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_clone", "BLE Clone", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning || !s_list) return;
    // Refresh the pick-list every 500ms while scanning, same throttle
    // shape as ble_scan.cpp/ble_finder.cpp -- rebuilding kMaxTargets rows
    // is cheap and this list is short-lived (10s scan window).
    static uint32_t last_refresh = 0;
    if (millis() - last_refresh < 500) return;
    last_refresh = millis();

    portENTER_CRITICAL(&s_targets_mux);
    BleDeviceInfo snapshot[kMaxTargets];
    int count = s_target_count;
    memcpy(snapshot, s_targets, sizeof(BleDeviceInfo) * count);
    portEXIT_CRITICAL(&s_targets_mux);

    lv_obj_clean(s_list);
    for (int i = 0; i < count; i++) {
        char row[48];
        snprintf(row, sizeof(row), "%s  %s", snapshot[i].name, snapshot[i].addr_str);
        // Tapping a row clones that target -- index stashed as user_data.
        lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_COPY, row);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            int idx = (int)(intptr_t)lv_event_get_user_data(e);
            if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
            clone_target(idx);
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

} // namespace BleCloneFeature
