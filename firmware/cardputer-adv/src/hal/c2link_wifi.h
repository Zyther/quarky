#pragma once
#include "ic2link.h"

// WiFi socket backend for the Cardputer-ADV <-> Tab5 control channel --
// client-side counterpart to Tab5's C2LinkWifi (Task 11), which hosts a
// self-contained WiFi AP and TCP server. Cardputer-ADV's own radio (a plain
// ESP32-S3, no esp-hosted proxying involved) joins that AP as a station and
// opens one persistent TCP connection once paired; that connection carries
// both control messages AND bulk data, matching Tab5's side.
//
// This is the WiFi half of the radio-selected C2 transport pair described in
// Tab5's c2link_wifi.h: BLE is the other half (Task 17, mirroring Task 13),
// used when a WiFi feature is active on this device and the WiFi radio isn't
// free for C2.
class C2LinkWifi : public IC2Link {
public:
    bool init(const uint8_t psk[16], const char *ap_ssid, const char *ap_password,
              const char *server_ip, uint16_t port);
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
    bool is_connected() override;
    void poll(); // call every loop() -- maintains the WiFi/socket connection, reads incoming frames
};

// millis() timestamp of the last frame successfully decoded off this
// transport (0 if none yet). Free function rather than a method, matching
// Tab5's c2link_wifi_last_recv_ms() -- kept for parity in case a future
// Cardputer-ADV status UI wants link-freshness the same way Tab5's
// devices_panel.cpp does.
uint32_t c2link_wifi_last_recv_ms();
