#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_gt911.h"
#include "hal/radio_esp_hosted.h"
#include "ui/lvgl_port.h"
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include <feature_registry.h>

DisplayTab5 display;
TouchGT911 touch;
RadioEspHosted radio;
FeatureRegistry g_registry; // populated further in Task 15

void setup() {
    Serial.begin(115200);
    delay(500);

    // Placeholder-credential connect test for Task 9 (IRadio via esp-hosted).
    // Deliberately run before display/touch/LVGL/shell init: WiFi.begin()'s
    // connect loop blocks for up to 15s, and running it before the UI exists
    // avoids a boot sequence where the shell appears then freezes for up to
    // 15s before loop()/lvgl_port_tick() start. Substitute a real test
    // SSID/password locally to verify on hardware, then revert to these
    // placeholders before committing -- see
    // .superpowers/sdd/2026-08-06-tab5-foundation-plan/task-9-report.md.
    Serial.println("quarky-tab5: connecting wifi via esp-hosted...");
    bool ok = radio.connect_wifi("YOUR_TEST_SSID", "YOUR_TEST_PASSWORD");
    Serial.printf("quarky-tab5: wifi connect %s, ip=%u\n", ok ? "OK" : "FAILED", radio.local_ip());

    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    lv_obj_t *root = Shell::build(g_registry);
    ScreenStack::push(root);

    Serial.println("quarky-tab5: lvgl ready");
}

void loop() {
    lvgl_port_tick();
    delay(5);
}
