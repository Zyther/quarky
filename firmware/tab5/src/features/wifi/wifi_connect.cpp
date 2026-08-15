#include "wifi_connect.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../hal/radio_esp_hosted.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <IPAddress.h> // formats radio.local_ip()'s packed uint32_t correctly --
                        // see poll()'s comment for why
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern FeatureRegistry g_registry;
extern RadioEspHosted radio; // defined in main.cpp (Phase 1 Task 9)

namespace WifiConnectFeature {

static lv_obj_t *s_ssid_input = nullptr;
static lv_obj_t *s_pass_input = nullptr;
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_keyboard = nullptr;

// Real-hardware finding (2026-08-14): RadioEspHosted::connect_wifi() blocks
// for up to 15s (a `while (WiFi.status() != WL_CONNECTED && ...) delay(200);`
// loop, radio_esp_hosted.cpp) and feeds no watchdog. Calling it directly from
// this button's click handler -- which runs on the main/LVGL task, since
// LVGL event callbacks fire synchronously inside lv_timer_handler() during
// loop() -- blocked the whole main task for that entire window. A wrong
// password reliably runs long enough to trip it: WiFi auth failure produces
// a real 4WAY_HANDSHAKE_TIMEOUT only after several retries, comfortably
// past enableLoopWDT()'s ~5s budget, so the device panicked and rebooted on
// the single most common real user mistake this screen exists to handle.
// Confirmed on real hardware: task_wdt abort naming "loopTask (CPU 1)" as
// the task that failed to feed the watchdog, immediately after a real
// 4WAY_HANDSHAKE_TIMEOUT in the serial log.
//
// Fixed by moving the blocking call to its own FreeRTOS task, matching this
// project's established pattern for anything that can't complete inside the
// main task's ~50ms per-tick budget (the NimBLE host task is the other
// example) -- poll() (now wired into main.cpp's loop()) drains the result
// once the background task finishes, the same "do the blocking work
// elsewhere, hand the result back through a few cross-task scalars" shape
// c2link_ble.cpp/wifi_pmkid.cpp already establish.
struct ConnectArgs {
    char ssid[33]; // 32 + NUL, matches the real 802.11 SSID length limit
    char pass[64]; // 63 + NUL, matches WPA2's real max passphrase length
};

static volatile bool s_connecting = false;
static volatile bool s_connect_done = false;
static volatile bool s_connect_result = false;

static void connect_task(void *arg) {
    ConnectArgs *args = static_cast<ConnectArgs *>(arg);
    bool ok = radio.connect_wifi(args->ssid, args->pass);
    delete args;
    s_connect_result = ok;
    s_connect_done = true; // publish result before exiting; poll() reads both
    vTaskDelete(nullptr);
}

static void connect_click_cb(lv_event_t *e) {
    if (s_connecting) return; // a connect attempt is already in flight

    ConnectArgs *args = new ConnectArgs();
    strncpy(args->ssid, lv_textarea_get_text(s_ssid_input), sizeof(args->ssid) - 1);
    args->ssid[sizeof(args->ssid) - 1] = '\0';
    strncpy(args->pass, lv_textarea_get_text(s_pass_input), sizeof(args->pass) - 1);
    args->pass[sizeof(args->pass) - 1] = '\0';

    if (s_status_label) lv_label_set_text(s_status_label, "Connecting...");
    s_connecting = true;
    s_connect_done = false;

    // Stack size matches this codebase's other background-task precedent
    // (c2link_ble.cpp's NimBLE host_task) rather than the FreeRTOS default,
    // since WiFi.begin()/status() polling runs through several driver
    // layers. Priority 1 (same as loopTask) -- this has no reason to
    // preempt the UI.
    xTaskCreate(connect_task, "wifi_connect", 4096, args, 1, nullptr);
}

static void input_focus_cb(lv_event_t *e) {
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    lv_keyboard_set_textarea(s_keyboard, ta);
    lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WiFi Connect", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_ssid_input = lv_textarea_create(content);
    lv_textarea_set_one_line(s_ssid_input, true);
    lv_textarea_set_placeholder_text(s_ssid_input, "SSID");
    lv_obj_add_event_cb(s_ssid_input, input_focus_cb, LV_EVENT_FOCUSED, nullptr);

    s_pass_input = lv_textarea_create(content);
    lv_textarea_set_one_line(s_pass_input, true);
    lv_textarea_set_password_mode(s_pass_input, true);
    lv_textarea_set_placeholder_text(s_pass_input, "Password");
    lv_obj_add_event_cb(s_pass_input, input_focus_cb, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *connect_btn = lv_button_create(content);
    lv_obj_t *connect_label = lv_label_create(connect_btn);
    lv_label_set_text(connect_label, "Connect");
    lv_obj_add_event_cb(connect_btn, connect_click_cb, LV_EVENT_CLICKED, nullptr);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Not connected");

    s_keyboard = lv_keyboard_create(screen); // parented to screen, not content, so it overlays
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);

    // s_ssid_input/s_pass_input/s_status_label/s_keyboard must not outlive
    // the screen -- see ui/screen_scaffold.cpp for why every sub-screen
    // clears its own widget pointers from LV_EVENT_DELETE, matching
    // wifi_scan.cpp/ble_scan.cpp's established pattern. Note this does NOT
    // (and cannot) cancel a connect_task() already in flight -- it's a
    // detached FreeRTOS task with no cancellation hook, and radio_esp_hosted's
    // own blocking loop has no interrupt point. poll() below tolerates that:
    // it only touches s_status_label after checking it's non-null, so a
    // connect that finishes after the screen closes just updates s_connecting/
    // s_connect_done and is silently discarded -- no dangling pointer, no
    // crash, matching this project's "must survive being popped via any
    // path" screen-teardown discipline.
    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        s_ssid_input = nullptr;
        s_pass_input = nullptr;
        s_status_label = nullptr;
        s_keyboard = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_connect", "WiFi Connect", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_connecting) return;
    if (!s_connect_done) return;
    s_connecting = false;

    if (s_status_label) {
        char buf[64];
        if (s_connect_result) {
            // Real-hardware finding (2026-08-14): radio.local_ip() returns
            // the address packed into a uint32_t (IPAddress's own
            // operator uint32_t()), not a value meant to be printed with
            // %u directly -- doing so showed a raw 32-bit integer instead
            // of a dotted-quad address. IPAddress's own constructor/
            // toString() round-trips that same packed value correctly
            // without this file needing to know or assume its byte order.
            IPAddress ip(radio.local_ip());
            snprintf(buf, sizeof(buf), "Connected (ip=%s)", ip.toString().c_str());
        } else {
            snprintf(buf, sizeof(buf), "Connect failed");
        }
        lv_label_set_text(s_status_label, buf);
    }
}

} // namespace WifiConnectFeature
