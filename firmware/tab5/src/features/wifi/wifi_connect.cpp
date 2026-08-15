#include "wifi_connect.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../hal/radio_esp_hosted.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <cstdio>

extern FeatureRegistry g_registry;
extern RadioEspHosted radio; // defined in main.cpp (Phase 1 Task 9)

namespace WifiConnectFeature {

static lv_obj_t *s_ssid_input = nullptr;
static lv_obj_t *s_pass_input = nullptr;
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_keyboard = nullptr;

static void connect_click_cb(lv_event_t *e) {
    const char *ssid = lv_textarea_get_text(s_ssid_input);
    const char *pass = lv_textarea_get_text(s_pass_input);
    lv_label_set_text(s_status_label, "Connecting...");
    bool ok = radio.connect_wifi(ssid, pass);
    char buf[64];
    snprintf(buf, sizeof(buf), ok ? "Connected (ip=%u)" : "Connect failed", radio.local_ip());
    lv_label_set_text(s_status_label, buf);
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
    // wifi_scan.cpp/ble_scan.cpp's established pattern.
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

} // namespace WifiConnectFeature
