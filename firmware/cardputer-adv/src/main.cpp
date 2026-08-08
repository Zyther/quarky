#include <Arduino.h>
#include "hal/device.h"
#include "hal/c2link_wifi.h"
#include "remote/command_dispatcher.h"
#include <feature_registry.h>

static C2LinkWifi c2link_wifi;
static FeatureRegistry g_registry; // populated in Task 20

// Placeholder PSK/AP credentials -- the real PSK comes from Task 12's
// Tab5-side QR/typed provisioning; "Quarky-Tab5-Test"/port 7777 match Task
// 11's placeholder AP/port on the Tab5 side.
static uint8_t s_test_psk[16] = {0};

void setup() {
    Serial.begin(115200);
    delay(500);
    Device::instance().init();
    Serial.printf("quarky-cardputer-adv: display=%s keyboard=%s\n",
                  Device::instance().display_ready() ? "OK" : "FAIL",
                  Device::instance().keyboard_ready() ? "OK" : "FAIL");

    bool c2_ok = c2link_wifi.init(s_test_psk, "Quarky-Tab5-Test", "quarkytest123", "192.168.4.1", 7777);
    Serial.printf("quarky-cardputer-adv: c2link_wifi init %s\n", c2_ok ? "OK" : "FAILED");

    c2link_wifi.set_receive_handler([](const c2proto::Frame &frame) {
        CommandDispatcher::handle(frame, c2link_wifi, g_registry);
    });
}

void loop() {
    c2link_wifi.poll();
    delay(10);
}
