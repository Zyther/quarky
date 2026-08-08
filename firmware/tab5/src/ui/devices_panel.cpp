#include "devices_panel.h"
#include "shell.h"
#include <lvgl.h>
#include <cstdio>
#include <cstring>
#include <Arduino.h> // Serial -- debug-log-on-change below, see comment

namespace DevicesPanel {

void update(bool wifi_connected, bool ble_connected, int32_t last_rtt_ms) {
    lv_obj_t *status_bar = Shell::status_bar();
    if (!status_bar) return; // shell not built yet
    // Child 0 is the battery label, child 1 is the link label -- see
    // Shell::build() in shell.cpp (Task 7).
    lv_obj_t *link_label = lv_obj_get_child(status_bar, 1);
    if (!link_label) return;

    char buf[64];
    if (wifi_connected) {
        snprintf(buf, sizeof(buf), "Cardputer-ADV: connected via WiFi (%ldms)", (long)last_rtt_ms);
    } else if (ble_connected) {
        snprintf(buf, sizeof(buf), "Cardputer-ADV: connected via BLE (%ldms)", (long)last_rtt_ms);
    } else {
        snprintf(buf, sizeof(buf), "Cardputer-ADV: disconnected");
    }
    lv_label_set_text(link_label, buf);

    // Task 19 hardware verification: this UI has no other real-hardware-
    // testable signal (no satellite exists yet to actually connect, and this
    // agent has no visual access to the LVGL framebuffer to read the label
    // directly). Log only on a state transition -- not every 500ms poll --
    // so this stays cheap to leave in rather than needing to be stripped
    // before commit.
    static char s_last_logged[64] = {0};
    if (strcmp(buf, s_last_logged) != 0) {
        Serial.printf("quarky-tab5: devices_panel status bar -> \"%s\"\n", buf);
        strncpy(s_last_logged, buf, sizeof(s_last_logged) - 1);
    }
}

} // namespace DevicesPanel
