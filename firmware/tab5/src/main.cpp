#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_gt911.h"
#include "hal/radio_esp_hosted.h"
#include "hal/storage_sd.h"
#include "hal/c2link_espnow.h"
#include "ui/lvgl_port.h"
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include <feature_registry.h>

DisplayTab5 display;
TouchGT911 touch;
RadioEspHosted radio;
StorageSD storage;
C2LinkEspNow c2link;
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

    // Task 10: SD mount + write test while WiFi is (attempted) active, run
    // immediately after the WiFi connect test above and before display/touch
    // init -- deliberately placed here (not deferred to later in setup()) so
    // that a future hardware pass can observe, with both peripherals live at
    // once, whether touching the SD card disturbs the WiFi/C6 link or vice
    // versa. Research (see hal/storage_sd.cpp and task-10-report.md)
    // resolved from source that the SD card and the C6 co-processor sit on
    // two separate ESP32-P4 SDMMC host slots (slot 0 vs slot 1) with
    // disjoint GPIO pins -- no electrical bus contention -- but this
    // instrumentation is what lets Step 3 of the brief's hardware pass
    // (DEFERRED, not attempted here) confirm that empirically too.
    Serial.println("quarky-tab5: mounting sd card...");
    bool sd_ok = storage.mount() && storage.write_test_file();
    Serial.printf("quarky-tab5: sd mount+write while wifi active: %s\n", sd_ok ? "OK" : "FAILED");
    Serial.printf("quarky-tab5: wifi still connected after sd write: %s\n", radio.is_connected() ? "YES" : "NO");

    // Task 11: C2LinkEspNow init-only smoke test. This is a placeholder PSK
    // (all-zero) and a placeholder peer MAC -- neither is a real credential.
    // The real PSK comes from Task 12's pairing flow; the real Cardputer-ADV
    // MAC address comes from Task 15's bring-up. Full send/receive can only
    // be verified once that satellite device exists and with real hardware
    // (DEFERRED here, software-only pass) -- see task-11-report.md.
    uint8_t test_psk[16] = {0};
    uint8_t placeholder_peer_mac[6] = {0x24, 0x0A, 0xC4, 0x00, 0x00, 0x00};
    bool c2_ok = c2link.init(test_psk, placeholder_peer_mac);
    Serial.printf("quarky-tab5: c2link init %s\n", c2_ok ? "OK" : "FAILED");

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
