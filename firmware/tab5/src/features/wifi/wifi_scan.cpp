#include "wifi_scan.h"
#include "wifi_common.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <cstdio>

extern FeatureRegistry g_registry;

namespace WifiScanFeature {

static lv_obj_t *s_list = nullptr;
// Task-review finding (2026-08-11, Phase 2 Task 3 hotfix review): owning the
// timer handle here (rather than letting scan_poll_timer_cb notice s_list==
// nullptr on its own next 250ms tick) closes a real race -- Back-then-reopen
// inside that window used to leave the old timer alive polling a NEW list
// (s_list had already been reassigned, not nulled), with two timers driving
// one shared wifi_scan_begin()/s_scan_started_ms state. Deleting the timer
// synchronously from the list's own LV_EVENT_DELETE makes "the list is gone"
// and "the timer is gone" the same event, not a race between two.
static lv_timer_t *s_poll_timer = nullptr;

static lv_obj_t *build_screen() {
    // Menu-bar Back button + flex content area, same as every other
    // sub-screen -- see ui/screen_scaffold.cpp.
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WiFi Scan", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));
    lv_list_add_text(s_list, "Scanning...");
    // The list dies with the screen when Back is tapped (ScreenStack::pop()
    // deletes it). Clearing s_list (and the poll timer, if any) from the
    // widget's own delete event, rather than from the Back button's click
    // handler, covers every path that can destroy it -- including a pop
    // triggered from somewhere else -- so run_scan_and_populate() can never
    // write through a dangling pointer, and no stale timer can outlive it.
    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        s_list = nullptr;
        if (s_poll_timer) {
            lv_timer_delete(s_poll_timer);
            s_poll_timer = nullptr;
        }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

// Polled from an LVGL timer rather than run inline: see wifi_common.h for why
// the blocking scan this replaced locked the device solid. Everything here
// runs on the LVGL task between lv_timer_handler() iterations, so the screen
// stays responsive -- Back included -- for the whole scan.
static void scan_poll_timer_cb(lv_timer_t *timer) {
    if (!s_list) { // Back was tapped; the list (and screen) are gone
        s_poll_timer = nullptr; // already being deleted by the DELETE handler above
        return;
    }

    static WifiApInfo aps[32];
    int n = wifi_scan_poll(aps, 32);
    if (n == -1) {
        return; // still scanning
    }
    lv_timer_delete(timer);
    s_poll_timer = nullptr;

    lv_obj_clean(s_list);
    if (n == -2) {
        lv_list_add_text(s_list, "Scan failed or timed out");
        return;
    }
    if (n == 0) {
        lv_list_add_text(s_list, "No networks found");
        return;
    }
    for (int i = 0; i < n; i++) {
        char bssid_str[18];
        wifi_bssid_to_str(aps[i].bssid, bssid_str);
        char row[80];
        snprintf(row, sizeof(row), "%s  ch%d  %ddBm  %s", aps[i].ssid,
                 aps[i].channel, aps[i].rssi, aps[i].open ? "OPEN" : "");
        lv_list_add_button(s_list, LV_SYMBOL_WIFI, row);
    }
}

void register_module() {
    g_registry.register_module({"wifi_scan", "WiFi Scan", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

// -----------------------------------------------------------------------------
// The scan itself is DISABLED pending an ESP32-C6 co-processor firmware update.
//
// Measured on real hardware 2026-08-10. Opening this screen and starting a scan
// wedges the entire device: serial output stops dead, touch stops, LVGL stops,
// and only a hardware reset recovers it. Established by bisection with serial
// probes around every call in wifi_scan_begin():
//
//   * WiFi.scanDelete(), WiFi.mode(...) and WiFi.scanNetworks(true) all RETURN
//     normally -- scanNetworks(true) returns WIFI_SCAN_RUNNING as designed.
//     The device dies some time AFTER the scan is handed to the radio, which
//     rules out the UI, the LVGL task and this file's own logic.
//   * It is not AP/STA coexistence. Tried again in STA-only mode, with
//     c2link_wifi's SoftAP torn down: identical wedge.
//
// So: any esp-hosted WiFi scan hangs this board. WiFi here is proxied over SDIO
// to the onboard ESP32-C6, and every boot logs a version skew that is the prime
// suspect:
//
//   hostedHasUpdate(): Host firmware version: 2.12.11
//   hostedHasUpdate(): Slave firmware version: 1.4.1
//   hostedHasUpdate(): Version on Host is NEWER than version on co-processor
//   hostedHasUpdate(): Update URL:
//       https://espressif.github.io/arduino-esp32/hosted/esp32c6-v2.12.11.bin
//
// TO RE-ENABLE: flash the C6 with the firmware at that URL, flip kScanEnabled
// to true, and re-test. The scan path below is complete and correct -- it is
// gated, not stubbed -- so that is the only change needed. Until then a tap on
// the "WiFi Scan" tile must not brick the device, which is what this guard
// buys.
// -----------------------------------------------------------------------------
static constexpr bool kScanEnabled = false;

void start() {
    ScreenStack::push(build_screen());

    if (!kScanEnabled) {
        lv_obj_clean(s_list);
        lv_list_add_text(s_list, "WiFi scan unavailable");
        lv_list_add_text(s_list, "The ESP32-C6 radio co-processor needs a firmware");
        lv_list_add_text(s_list, "update before it can scan without hanging the device.");
        lv_list_add_text(s_list, "See the note in features/wifi/wifi_scan.cpp.");
        return;
    }

    if (!wifi_scan_begin()) {
        lv_obj_clean(s_list);
        lv_list_add_text(s_list, "Could not start scan (radio unavailable)");
        return;
    }
    s_poll_timer = lv_timer_create(scan_poll_timer_cb, 250, nullptr);
}

} // namespace WifiScanFeature
