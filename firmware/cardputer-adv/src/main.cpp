#include <Arduino.h>
#include "hal/device.h"
#include "hal/c2link_wifi.h"
#include "hal/c2link_ble.h"
#include "remote/command_dispatcher.h"
#include "local_menu.h"
#include <feature_registry.h>

static C2LinkWifi c2link_wifi;
static C2LinkBle c2link_ble;
static FeatureRegistry g_registry; // populated in Task 20

// Placeholder PSK/AP credentials -- the real PSK comes from Task 12's
// Tab5-side QR/typed provisioning; "Quarky-Tab5-Test"/port 7777 match Task
// 11's placeholder AP/port on the Tab5 side. "Quarky-Tab5" (BLE target name)
// matches Task 13's advertised device name.
static uint8_t s_test_psk[16] = {0};

void setup() {
    Serial.begin(115200);
    delay(500);
    Device::instance().init();
    Serial.printf("quarky-cardputer-adv: display=%s keyboard=%s\n",
                  Device::instance().display_ready() ? "OK" : "FAIL",
                  Device::instance().keyboard_ready() ? "OK" : "FAIL");

    bool c2_wifi_ok = c2link_wifi.init(s_test_psk, "Quarky-Tab5-Test", "quarkytest123", "192.168.4.1", 7777);
    Serial.printf("quarky-cardputer-adv: c2link_wifi init %s\n", c2_wifi_ok ? "OK" : "FAILED");

    c2link_wifi.set_receive_handler([](const c2proto::Frame &frame) {
        CommandDispatcher::handle(frame, c2link_wifi, g_registry);
    });

    // Second transport (Task 17), coexists with c2link_wifi above -- radio
    // selection between the two (Task 15/Tab5's c2link_wifi.h) is a later
    // concern; both are simply initialized and polled here for now.
    bool c2_ble_ok = c2link_ble.init(s_test_psk, "Quarky-Tab5");
    Serial.printf("quarky-cardputer-adv: c2link_ble init %s\n", c2_ble_ok ? "OK" : "FAILED");

    c2link_ble.set_receive_handler([](const c2proto::Frame &frame) {
        CommandDispatcher::handle(frame, c2link_ble, g_registry);
    });

    LocalMenu::init();
}

void loop() {
    LocalMenu::tick();
    c2link_wifi.poll();
    c2link_ble.poll();
    delay(20);
}
