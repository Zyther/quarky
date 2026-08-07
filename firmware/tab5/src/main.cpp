#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_gt911.h"
#include "hal/radio_esp_hosted.h"
#include "hal/storage_sd.h"
#include "hal/c2link_wifi.h"
#include "hal/c2link_ble.h"
#include "ui/lvgl_port.h"
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include <feature_registry.h>

DisplayTab5 display;
TouchGT911 touch;
RadioEspHosted radio;
StorageSD storage;
C2LinkWifi c2link_wifi;
C2LinkBle c2link_ble;
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

    // Task 11 (amended 2026-08-07, was ESP-NOW -- see task-11-report.md for
    // why that was replaced): C2LinkWifi init-only smoke test. Starts the
    // Tab5's self-contained WiFi AP + TCP server; joining as a station and
    // opening a connection is Cardputer-ADV's side, which doesn't exist yet.
    // This is a placeholder PSK (all-zero) and placeholder AP credentials --
    // none of this is a real credential. The real PSK and AP-credential
    // derivation come from Task 12's pairing flow. Full send/receive can
    // only be verified once Cardputer-ADV's matching side exists and with
    // real hardware (DEFERRED here, software-only pass).
    uint8_t test_psk[16] = {0};
    bool c2_wifi_ok = c2link_wifi.init(test_psk, "Quarky-Tab5-Test", "quarkytest123", 7777);
    Serial.printf("quarky-tab5: c2link_wifi init %s\n", c2_wifi_ok ? "OK" : "FAILED");

    // Task 13: C2LinkBle init-only smoke test -- the second C2 transport,
    // used when the WiFi radio is busy with an active feature. Coexists
    // with C2LinkWifi above (both transports are brought up here; feature
    // code picks which one to actually use at runtime, per the foundation
    // spec). Same placeholder PSK as the WiFi transport until Task 12's
    // pairing flow wires the real one. A known, already-flagged, deferred
    // concern from Task 11 is that WiFi.mode(WIFI_AP) (above) and BLE
    // together touch the same C6 co-processor radio at runtime -- not
    // re-litigated here, and nothing NEW at compile/link time was found
    // (see task-13-report.md). Full advertise/connect verification needs
    // a real BLE central (a scanner app, or Cardputer-ADV's future BLE
    // client from Task 17) -- DEFERRED, software-only pass here.
    bool c2_ble_ok = c2link_ble.init(test_psk, "Quarky-Tab5");
    Serial.printf("quarky-tab5: c2link_ble init %s\n", c2_ble_ok ? "OK" : "FAILED");

    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    lv_obj_t *root = Shell::build(g_registry);
    ScreenStack::push(root);

    Serial.println("quarky-tab5: lvgl ready");
}

void loop() {
    lvgl_port_tick();
    c2link_wifi.poll();
    c2link_ble.poll(); // drains BLE frames received on the NimBLE host task
    delay(5);
}
