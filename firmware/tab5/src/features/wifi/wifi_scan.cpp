#include "wifi_scan.h"
#include "wifi_common.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <cstdio>

extern FeatureRegistry g_registry;

namespace WifiScanFeature {

static lv_obj_t *build_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *list = lv_list_create(screen);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(85));

    static WifiApInfo aps[32];
    int n = wifi_scan_aps(aps, 32);
    if (n == 0) {
        lv_list_add_text(list, "No networks found");
    }
    for (int i = 0; i < n; i++) {
        char bssid_str[18];
        wifi_bssid_to_str(aps[i].bssid, bssid_str);
        char row[80];
        snprintf(row, sizeof(row), "%s  ch%d  %ddBm  %s", aps[i].ssid,
                 aps[i].channel, aps[i].rssi, aps[i].open ? "OPEN" : "");
        lv_list_add_button(list, LV_SYMBOL_WIFI, row);
    }

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_scan", "WiFi Scan", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace WifiScanFeature
