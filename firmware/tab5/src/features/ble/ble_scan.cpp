#include "ble_scan.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial (spike-verification logging)
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>
#include <cstdio>

extern FeatureRegistry g_registry;

// -----------------------------------------------------------------------------
// Task 7 spike: this is the first Tab5 code to call ble_gap_disc() (BLE
// central/observer role) over the raw ESP-IDF NimBLE host hal/c2link_ble.cpp
// (Task 13) already brought up. Task 13's role is peripheral/GATT-server only
// (advertise + accept connections + serve characteristics) -- whether the C6
// controller supports concurrent scan-while-advertising, since c2link_ble's
// C2 GATT server is always advertising once booted, is a real, previously
// unverified question (see task-7-brief.md's framing, same risk class as
// Phase 4's flagged "Tab5-as-BLE-central is unproven" note). This reuses the
// SAME NimBLE host c2link_ble.cpp owns -- per this project's global
// constraint (raw ESP-IDF NimBLE only, confirmed incompatible with the P4 as
// NimBLE-Arduino), a second host is never initialized here.
// -----------------------------------------------------------------------------
namespace BleScanFeature {

static constexpr int kMaxDevices = 32;
static BleDeviceInfo s_devices[kMaxDevices];
static int s_device_count = 0;
static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;

static void add_or_update(const BleDeviceInfo &d) {
    for (int i = 0; i < s_device_count; i++) {
        if (memcmp(s_devices[i].addr, d.addr, 6) == 0) {
            s_devices[i] = d;
            return;
        }
    }
    if (s_device_count < kMaxDevices) {
        s_devices[s_device_count++] = d;
    }
}

// Runs on the NimBLE host task (not the main/LVGL task) -- only touches
// s_devices/s_device_count, which poll() (main task) reads back on its next
// tick. No locking here mirrors c2link_ble.cpp's own comment about which
// pieces of shared state actually need a portMUX: this array is written here
// and read from refresh_list_ui() on the main task with no ordering
// requirement finer than "eventually visible next poll", same shape as
// wifi_spectrum.cpp's s_chart updates.
static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

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

    add_or_update(d);
    return 0;
}

static void refresh_list_ui() {
    if (!s_list) return;
    lv_obj_clean(s_list);
    for (int i = 0; i < s_device_count; i++) {
        char row[64];
        const char *label = s_devices[i].name[0] ? s_devices[i].name : s_devices[i].addr_str;
        snprintf(row, sizeof(row), "%s  %ddBm", label, s_devices[i].rssi);
        lv_list_add_button(s_list, LV_SYMBOL_BLUETOOTH, row);
    }
}

static lv_obj_t *build_screen() {
    // Menu-bar Back button + flex content area, same as every other
    // sub-screen -- see ui/screen_scaffold.cpp. Task 5's amendment note
    // (reiterated for this task in task-7-brief.md) is why this does NOT
    // hand-build its own lv_obj_create(nullptr) screen + Back button.
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Scan", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    // s_list/s_scanning must not outlive the screen -- the scaffold's Back
    // button pops the screen and deletes it (and everything parented under
    // it, including this list) via ScreenStack::pop(). Clearing this state
    // from the list's own LV_EVENT_DELETE, rather than a Back-button click
    // handler, covers every path that can destroy it (not just a tap on
    // Back), the same pattern wifi_scan.cpp and wifi_spectrum.cpp both use.
    // Cancelling an in-progress scan here (rather than leaving it running
    // with nothing consuming its results) is the same cleanup the brief's
    // own Back-button handler did -- just moved to fire on every destruction
    // path instead of only a click.
    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_device_count = 0;
    struct ble_gap_disc_params params{};
    params.passive = 0;         // active scan, matches Cardputer-ADV's Task 17 fix
                                  // (setActiveScan(true)) that was needed to see
                                  // scan-response-only fields like device name
    params.itvl = 0x0050;       // 50ms
    params.window = 0x0030;     // 30ms
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000 /* 10s */, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-scan-spike] ble_gap_disc rc=%d (%s)\n", rc, rc == 0 ? "OK" : "FAILED");
    s_scanning = (rc == 0);
    if (!s_scanning) {
        lv_list_add_text(s_list, "Scan failed to start (see serial log)");
    }

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_scan", "BLE Scan", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning) return;
    refresh_list_ui();
}

} // namespace BleScanFeature
