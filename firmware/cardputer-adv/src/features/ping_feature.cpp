#include "ping_feature.h"
#include <feature_registry.h>
#include <Arduino.h>
#include <cstring>

extern FeatureRegistry g_registry; // defined in main.cpp

namespace PingFeature {

void register_module() {
    g_registry.register_module({"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV});
}

void handle_start(IC2Link &link, uint16_t seq) {
    c2proto::Frame resp{};
    resp.version = 1;
    resp.type = c2proto::MsgType::RESP_TELEMETRY;
    resp.seq = seq;

    uint32_t uptime_s = millis() / 1000;
    char msg[64];
    int n = snprintf(msg, sizeof(msg), "uptime=%us", uptime_s);
    memcpy(resp.payload, msg, n);
    resp.payload_len = (uint16_t)n;

    link.send(resp); // replies over whichever link (WiFi or BLE) delivered the command --
                      // CommandDispatcher::handle already passes in the right IC2Link&
    Serial.printf("quarky-cardputer-adv: ping handled, replied '%s'\n", msg);
}

} // namespace PingFeature
