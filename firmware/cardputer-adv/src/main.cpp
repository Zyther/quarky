#include <Arduino.h>
#include "hal/device.h"
#include "hal/c2link_wifi.h"
#include "hal/c2link_ble.h"
#include "remote/command_dispatcher.h"
#include "features/ping_feature.h"
#include "local_menu.h"
#include <feature_registry.h>

static C2LinkWifi c2link_wifi;
static C2LinkBle c2link_ble;
FeatureRegistry g_registry; // Task 20: populated by PingFeature::register_module();
                             // non-static so features/ping_feature.cpp's `extern
                             // FeatureRegistry g_registry;` can see it.

// Real provisioned PSK (Task 20 pairing): read off Tab5's serial log (the
// pairing_screen.cpp hex dump added this task) after tapping "Pair
// Satellite" on Tab5, and hardcoded here in place of the Task 15 all-zero
// placeholder. Hex dump was "E534F14EBF828BE9F9EE855169431BA5" -- this is
// Tab5's real NVS-persisted PSK, from real hardware, not a synthetic value.
// "Quarky-Tab5-Test"/port 7777 match Task 11's placeholder AP/port on the
// Tab5 side. "Quarky-Tab5" (BLE target name) matches Task 13's advertised
// device name.
static uint8_t s_test_psk[16] = {
    0xE5, 0x34, 0xF1, 0x4E, 0xBF, 0x82, 0x8B, 0xE9,
    0xF9, 0xEE, 0x85, 0x51, 0x69, 0x43, 0x1B, 0xA5,
};

void setup() {
    Serial.begin(115200);
    delay(500);
    Device::instance().init();
    Serial.printf("quarky-cardputer-adv: display=%s keyboard=%s\n",
                  Device::instance().display_ready() ? "OK" : "FAIL",
                  Device::instance().keyboard_ready() ? "OK" : "FAIL");

    bool c2_wifi_ok = c2link_wifi.init(s_test_psk, "Quarky-Tab5-Test", "quarkytest123", "192.168.4.1", 7777);
    Serial.printf("quarky-cardputer-adv: c2link_wifi init %s\n", c2_wifi_ok ? "OK" : "FAILED");

    PingFeature::register_module(); // first (and so far only) registered module; makes
                                     // "ping" resolvable by CommandDispatcher/find_by_id

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
