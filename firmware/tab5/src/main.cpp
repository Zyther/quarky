#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_tab5.h"
#include "hal/hosted_link.h"
#include "hal/radio_esp_hosted.h"
#include "hal/storage_sd.h"
#include "hal/c2link_wifi.h"
#include "hal/c2link_ble.h"
#include "hal/nfc_pn532.h"
#include "hal/rf433_gpio.h"
#include "ui/lvgl_port.h"
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include "ui/devices_panel.h"
#include "../boards/tab5/pins_config.h"
#include <feature_registry.h>

DisplayTab5 display;
TouchTab5 touch;
RadioEspHosted radio;
StorageSD storage;
C2LinkWifi c2link_wifi;
C2LinkBle c2link_ble;
FeatureRegistry g_registry; // populated further in Task 15

// Task 18: HY2.0 peripheral detection (NFC, RFID2, RF433R/T). See
// hal/nfc_pn532.cpp and boards/tab5/pins_config.h for the real-hardware
// address/pin research behind these constructor arguments.
//
// Task 18's census found NOTHING at all on GPIO 53/54 with a unit plugged in,
// which it correctly suspected was a power problem rather than a wrong
// address. The HY2.0 port-power hotfix (hotfix-hy20-port-report.md) confirmed
// exactly that: PORT.A's 5V rail is gated by EXT_5V_EN on IO-expander 0x43
// P2, which nothing was asserting. With the gate asserted the census returns
// 0x50, so the NFC unit's address is now CONFIRMED on hardware and the 0x24
// PN532 guess is retired.
NfcPN532 nfc_unit(TAB5_NFC_I2C_ADDR);     // 0x50, confirmed (ST25R3916 Unit NFC)
NfcPN532 rfid2_unit(TAB5_RFID2_I2C_ADDR); // 0x28, per docs (WS1850S); no such
                                          // unit was plugged in during the
                                          // hotfix's hardware run, so this
                                          // address remains doc-only.
Rf433Gpio rf433;

// Task 9's WiFi STA smoke test still ships with placeholder credentials.
// Attempting them costs a guaranteed-to-fail 15s connect timeout on every
// single boot, so the test is skipped until someone substitutes a real
// SSID/password here. Keeping the guard (rather than deleting the call) means
// the code path stays exercised the moment real credentials are filled in.
static const char *kTestSsid = "YOUR_TEST_SSID";
static const char *kTestPassword = "YOUR_TEST_PASSWORD";
static bool test_credentials_configured() {
    return strcmp(kTestSsid, "YOUR_TEST_SSID") != 0;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // ---- UI first, radios second -------------------------------------------
    // Deliberate ordering, changed as part of the C6 SDIO hotfix. Radio
    // bring-up talks to a separate ESP32-C6 chip over SDIO and is the single
    // most failure-prone thing in setup(): when its pins were wrong the board
    // crash-looped into the brownout detector and the display never appeared,
    // making every other subsystem un-debuggable. Bringing display/touch/LVGL
    // up first guarantees the screen is alive before anything can go wrong on
    // the radio, so a radio failure degrades to "device works, no comms"
    // instead of "device is a brick". See hosted_link.h and
    // hotfix-c6-sdio-pins-report.md.
    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    lv_obj_t *root = Shell::build(g_registry);
    ScreenStack::push(root);
    Serial.println("quarky-tab5: lvgl ready");

    // Task 10: SD mount + write test. The SD card is on SDMMC slot 0 with
    // silicon-fixed IOMUX pins, entirely disjoint from the C6's slot-1 link
    // (see hal/storage_sd.cpp), so it is exercised independently of, and
    // before, any radio bring-up.
    Serial.println("quarky-tab5: mounting sd card...");
    bool sd_ok = storage.mount() && storage.write_test_file();
    Serial.printf("quarky-tab5: sd mount+write: %s\n", sd_ok ? "OK" : "FAILED");

    // Task 18: HY2.0 peripheral detection (NFC, RFID2, RF433R/T). Detection
    // only -- no read/write/clone/replay logic yet (Phase 3 scope). The NFC
    // and RFID2 units are both I2C, on the EXTERNAL bus (HY2.0 PORT.A, GPIO
    // 53/54) -- a different bus from the internal one display/touch/SD use.
    // The census runs first and unconditionally so a wrong constructor
    // address shows up as an actionable fact in the log, not just a bare
    // "not found".
    nfc_scan_external_i2c_bus();
    nfc_unit.detect("NFC");
    rfid2_unit.detect("RFID2");
    rf433.init();

    // ---- ESP32-C6 radio co-processor ---------------------------------------
    // One bounded attempt at the shared esp-hosted SDIO link, with the result
    // latched. Every radio subsystem below re-checks it and no-ops if it is
    // down, so a broken/absent/mismatched C6 costs exactly one logged failure
    // rather than an unbounded retry storm.
    if (!hosted_link::begin()) {
        Serial.println("quarky-tab5: radios DISABLED for this boot "
                       "(WiFi + BLE + C2 transports unavailable); "
                       "UI and SD remain functional");
    } else {
        if (test_credentials_configured()) {
            Serial.println("quarky-tab5: connecting wifi via esp-hosted...");
            bool ok = radio.connect_wifi(kTestSsid, kTestPassword);
            Serial.printf("quarky-tab5: wifi connect %s, ip=%u\n",
                          ok ? "OK" : "FAILED", radio.local_ip());
            Serial.printf("quarky-tab5: wifi still connected after sd write: %s\n",
                          radio.is_connected() ? "YES" : "NO");
        } else {
            Serial.println("quarky-tab5: wifi STA connect test skipped "
                           "(placeholder credentials in main.cpp)");
        }

        // Task 11 (amended 2026-08-07, was ESP-NOW -- see task-11-report.md for
        // why that was replaced): C2LinkWifi init-only smoke test. Starts the
        // Tab5's self-contained WiFi AP + TCP server; joining as a station and
        // opening a connection is Cardputer-ADV's side, which doesn't exist yet.
        // This is a placeholder PSK (all-zero) and placeholder AP credentials --
        // none of this is a real credential. The real PSK and AP-credential
        // derivation come from Task 12's pairing flow.
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
        // together touch the same C6 co-processor radio at runtime. Full
        // advertise/connect verification needs a real BLE central (a scanner
        // app, or Cardputer-ADV's future BLE client from Task 17).
        bool c2_ble_ok = c2link_ble.init(test_psk, "Quarky-Tab5");
        Serial.printf("quarky-tab5: c2link_ble init %s\n", c2_ble_ok ? "OK" : "FAILED");
    }

    Serial.println("quarky-tab5: setup complete");
}

void loop() {
    lvgl_port_tick();
    c2link_wifi.poll(); // no-ops when the AP never came up
    c2link_ble.poll();  // drains BLE frames received on the NimBLE host task

    // Task 19: derive the shell status bar's link label from how recently
    // each C2 transport last received a frame. Neither transport has a peer
    // yet (Cardputer-ADV, Tasks 14-17, doesn't exist as of this task) so this
    // is expected to read "disconnected" for now -- verified end-to-end once
    // Task 20's ping feature generates real traffic on both sides. Polled at
    // a fixed interval rather than every loop() iteration since the label
    // only needs to be roughly live, not per-frame-accurate.
    static uint32_t s_last_devices_poll_ms = 0;
    uint32_t now = millis();
    if (now - s_last_devices_poll_ms > 500) {
        s_last_devices_poll_ms = now;
        // A last-recv value of 0 means "never received a frame" (both
        // statics' un-set default). Without this guard, `now - 0` is just
        // `now`, which is < 5000ms for the first few seconds after boot --
        // that reads as freshly "connected" before a single real frame ever
        // arrived. Excluding the 0 sentinel keeps the label honestly
        // "disconnected" until an actual frame shows up.
        uint32_t wifi_last = c2link_wifi_last_recv_ms();
        uint32_t ble_last = c2link_ble_last_recv_ms();
        uint32_t wifi_age = now - wifi_last;
        uint32_t ble_age = now - ble_last;
        bool wifi_connected = wifi_last != 0 && wifi_age < 5000;
        bool ble_connected = ble_last != 0 && ble_age < 5000;
        int32_t freshest_age = wifi_connected ? (int32_t)wifi_age : (int32_t)ble_age;
        DevicesPanel::update(wifi_connected, ble_connected, freshest_age);
    }

    delay(5);
}
