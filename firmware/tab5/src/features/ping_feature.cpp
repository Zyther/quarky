#include "ping_feature.h"
#include "../hal/c2link_wifi.h"
#include "../hal/c2link_ble.h"
#include <feature_registry.h>
#include <Arduino.h>
#include <cstring>

extern FeatureRegistry g_registry;   // defined in main.cpp
extern C2LinkWifi c2link_wifi;       // defined in main.cpp
extern C2LinkBle c2link_ble;         // defined in main.cpp

static uint16_t s_seq = 0;

namespace PingFeature {

void register_module() {
    g_registry.register_module({"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV});
}

void send_ping() {
    c2proto::Frame frame{};
    frame.version = 1;
    frame.type = c2proto::MsgType::CMD_START_FEATURE;
    frame.seq = s_seq++;
    const char *id = "ping";
    memcpy(frame.payload, id, strlen(id));
    frame.payload_len = (uint16_t)strlen(id);

    // Send over whichever transport is actually connected, preferring WiFi --
    // this is the foundation-phase stand-in for real radio-aware selection
    // (deferred to Phase 2+ per the spec's §4.5 amendment, since that needs
    // live FeatureRegistry state about which radio a running feature holds,
    // which doesn't exist until Phase 2+ features are real).
    bool ok = false;
    if (c2link_wifi.is_connected()) {
        ok = c2link_wifi.send(frame);
        Serial.printf("quarky-tab5: ping sent via WiFi, %s\n", ok ? "OK" : "FAILED");
    } else if (c2link_ble.is_connected()) {
        ok = c2link_ble.send(frame);
        Serial.printf("quarky-tab5: ping sent via BLE, %s\n", ok ? "OK" : "FAILED");
    } else {
        Serial.println("quarky-tab5: ping not sent, no transport connected");
    }
}

} // namespace PingFeature
