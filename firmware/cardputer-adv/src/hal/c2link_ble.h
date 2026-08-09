#pragma once
#include "ic2link.h"

// BLE GATT client backend for the Cardputer-ADV <-> Tab5 control channel --
// client-side counterpart to Tab5's C2LinkBle (Task 13), which hosts a
// Nordic-UART-Service-shaped GATT server. Cardputer-ADV's own radio (a plain
// ESP32-S3, fully native BLE, no esp-hosted proxying involved -- unlike
// Tab5's P4, see Task 13's report) scans for and connects to that server,
// writing outbound frames to its Rx characteristic and subscribing to
// notifications on its Tx characteristic.
//
// This is the BLE half of the radio-selected C2 transport pair described in
// Tab5's c2link_wifi.h: WiFi is the other half (Task 15/C2LinkWifi on this
// device), used when a WiFi feature is active on this device and the WiFi
// radio isn't free for C2. Both transports can coexist in main.cpp; only one
// is expected to be "live" (paired + carrying frames) at a time in practice,
// but nothing here prevents both from being initialized simultaneously.
class C2LinkBle : public IC2Link {
public:
    bool init(const uint8_t psk[16], const char *target_device_name);
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
    bool is_connected() override;
    void poll(); // call every loop() -- scans for and (re)connects to the target if not connected
};

// millis() timestamp of the last frame successfully decoded off this
// transport (0 if none yet). Free function rather than a method, matching
// c2link_wifi_last_recv_ms() and Tab5's c2link_ble_last_recv_ms() -- kept for
// parity in case a future Cardputer-ADV status UI wants link-freshness the
// same way Tab5's devices_panel.cpp does.
uint32_t c2link_ble_last_recv_ms();
