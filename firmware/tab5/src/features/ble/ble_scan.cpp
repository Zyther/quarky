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

// Task review finding (2026-08-12): s_devices/s_device_count are written by
// gap_scan_event_cb() on the NimBLE host task and read by refresh_list_ui()
// on the main task -- a genuine cross-task case, structurally identical to
// c2link_ble.cpp's s_rx_queue/s_rx_head/s_rx_tail (which DOES get a
// portMUX), not its s_last_recv_ms (which doesn't need one specifically
// because that one word is only ever touched from a single task). An
// earlier version of this file's comment claimed the opposite and was
// wrong -- corrected here, with the same portMUX_TYPE/portENTER_CRITICAL/
// portEXIT_CRITICAL primitive c2link_ble.cpp already uses for its own
// analogous cross-task state.
static portMUX_TYPE s_devices_mux = portMUX_INITIALIZER_UNLOCKED;
// Set (under the lock) whenever a discovery event actually adds or updates
// an entry; cleared by poll() after a refresh. Lets poll() skip rebuilding
// the list on loop() ticks where nothing changed -- see refresh throttling
// below.
static volatile bool s_devices_dirty = false;

static void add_or_update(const BleDeviceInfo &d) {
    portENTER_CRITICAL(&s_devices_mux);
    for (int i = 0; i < s_device_count; i++) {
        if (memcmp(s_devices[i].addr, d.addr, 6) == 0) {
            s_devices[i] = d;
            s_devices_dirty = true;
            portEXIT_CRITICAL(&s_devices_mux);
            return;
        }
    }
    if (s_device_count < kMaxDevices) {
        s_devices[s_device_count++] = d;
        s_devices_dirty = true;
    }
    portEXIT_CRITICAL(&s_devices_mux);
}

// Runs on the NimBLE host task (not the main/LVGL task) -- see the
// s_devices_mux comment above for why its writes to s_devices/
// s_device_count (via add_or_update()) are locked.
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

// Snapshots s_devices/s_device_count under the lock (a fast, bounded memcpy)
// and does the actual LVGL rebuild -- lv_obj_clean() plus up to kMaxDevices
// lv_list_add_button() calls, each allocating a sub-hierarchy -- outside it,
// so the NimBLE host task is never blocked for longer than a plain array
// copy. Same "copy under the lock, do the real work outside it" shape
// c2link_ble.cpp's poll() already uses when draining s_rx_queue.
static void refresh_list_ui() {
    if (!s_list) return;

    static BleDeviceInfo snapshot[kMaxDevices];
    int count;
    portENTER_CRITICAL(&s_devices_mux);
    count = s_device_count;
    memcpy(snapshot, s_devices, sizeof(BleDeviceInfo) * count);
    portEXIT_CRITICAL(&s_devices_mux);

    lv_obj_clean(s_list);
    for (int i = 0; i < count; i++) {
        char row[64];
        const char *label = snapshot[i].name[0] ? snapshot[i].name : snapshot[i].addr_str;
        snprintf(row, sizeof(row), "%s  %ddBm", label, snapshot[i].rssi);
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
            int rc = ble_gap_disc_cancel();
            // Minor task-review finding: log this like every other NimBLE
            // call in this codebase (c2link_ble.cpp is consistently
            // disciplined about it). A non-zero rc here is a normal,
            // harmless no-op (e.g. the 10s scan already finished on its
            // own before Back was tapped), not a functional problem --
            // logged for the same diagnostic-completeness reason, not
            // because failure needs handling.
            Serial.printf("quarky-tab5: [ble-scan] ble_gap_disc_cancel rc=%d\n", rc);
            s_scanning = false;
        }
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_device_count = 0;
    s_devices_dirty = false;
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

// Task review finding (2026-08-12): this used to call refresh_list_ui()
// unconditionally on every loop() tick (~5-10ms cadence from main.cpp's own
// delay(5)) for the whole 10s scan -- a full lv_obj_clean() + up to
// kMaxDevices lv_list_add_button() rebuild roughly 100-200 times/sec,
// almost all of them pure churn since the underlying data only changes on
// an actual BLE discovery event. Gated on s_devices_dirty (set only when
// add_or_update() adds/updates an entry) plus a 250ms floor, matching
// wifi_scan.cpp's own polling cadence for the same class of screen.
static uint32_t s_last_refresh_ms = 0;
static constexpr uint32_t kRefreshIntervalMs = 250;

void poll() {
    if (!s_scanning) return;
    if (!s_devices_dirty) return;
    uint32_t now = millis();
    if (now - s_last_refresh_ms < kRefreshIntervalMs) return;
    refresh_list_ui();
    s_devices_dirty = false;
    s_last_refresh_ms = now;
}

} // namespace BleScanFeature
