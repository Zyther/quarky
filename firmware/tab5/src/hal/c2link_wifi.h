#pragma once
#include "ic2link.h"

// WiFi socket backend for the Tab5 <-> Cardputer-ADV control channel.
//
// Supersedes the original ESP-NOW-based design (see git history and
// task-11-report.md): ESP-NOW has no linkable implementation on the ESP32-P4
// in this project's installed Arduino-ESP32 framework, because WiFi on this
// chip is proxied entirely to the onboard ESP32-C6 co-processor via
// esp-hosted/esp_wifi_remote (see Task 9's radio_esp_hosted.h), and that RPC
// layer doesn't proxy the ESP-NOW API surface.
//
// Instead, Tab5 hosts a self-contained WiFi AP (no external network/router
// required, matching the "personal kit" pairing design) and a single TCP
// server socket. Cardputer-ADV joins as a station and opens one persistent
// TCP connection once paired; that connection carries both control messages
// AND bulk data (pcap/handshake files, .sub captures) -- with no ESP-NOW
// payload ceiling to work around, there's no need for a separate bulk
// channel.
//
// This is the WiFi half of Tab5's radio-selected C2 transport pair: BLE is
// the other half (Task 13), used when a WiFi feature is active on the
// device and the WiFi radio isn't free for C2.
class C2LinkWifi : public IC2Link {
public:
    bool init(const uint8_t psk[16], const char *ap_ssid, const char *ap_password, uint16_t port);
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
    bool is_connected() override;
    void poll(); // call every loop() iteration -- accepts a client, reads/dispatches incoming frames
};
