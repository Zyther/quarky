#pragma once
#include "ic2link.h"

// BLE GATT backend for the Tab5 <-> Cardputer-ADV control channel.
//
// This is the second half of Tab5's radio-selected C2 transport pair (see
// c2link_wifi.h for the first half): used when a WiFi feature is active on
// the device and the WiFi radio isn't free for C2. Control-message use only
// in this phase (CMD_*/RESP_STATUS/RESP_TELEMETRY) -- bulk transfers wait
// for the WiFi transport, per the foundation spec's §4.5 amendment.
//
// Exposes a Nordic-UART-Service-shaped GATT server: one write characteristic
// for inbound frames (peer -> Tab5), one notify characteristic for outbound
// frames (Tab5 -> peer). NUS is a de facto standard for exactly this
// "bidirectional byte pipe over GATT" shape, reused rather than inventing a
// custom service so any BLE debugging tool that already knows NUS works
// against this link for free.
class C2LinkBle : public IC2Link {
public:
    bool init(const uint8_t psk[16], const char *device_name);
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
    bool is_connected() override;
    // Call every loop() iteration. Inbound frames are received on NimBLE's own
    // host task (in the GATT write callback), queued, and dispatched to the
    // receive handler from here -- so the handler runs on the main task, the
    // same threading contract as C2LinkWifi::poll(). See c2link_ble.cpp.
    void poll();
};

// millis() timestamp of the last frame successfully dequeued and dispatched
// by poll() (0 if none yet). Free function, mirroring
// c2link_wifi_last_recv_ms() -- see Task 19 (devices_panel.cpp polls this to
// derive link freshness for the shell's status bar).
uint32_t c2link_ble_last_recv_ms();

// True once the NimBLE host and (remote C6) controller have synced and
// on_sync() has run. Task 7: features that also need the BLE host for a
// central/observer-role call (e.g. ble_gap_disc() scanning) check this
// first -- calling into NimBLE before sync leaves the host in an undefined
// state.
//
// Final whole-branch review finding M7 (2026-08-13): this used to
// parenthetically claim sync also means "this device is actively
// advertising as a GATT server" -- true only until Task 8 (BLE Spam) runs,
// which stops that advertisement in favor of its own (see ble_spam.cpp's
// file-level comment for the single-advertisement-instance constraint).
// The actual, still-correct contract this function's two real callers rely
// on is narrower: the host is synced, so central/observer calls are safe.
// Dropped the parenthetical rather than qualify it, since nothing consumes
// the advertising-state claim.
bool c2link_ble_host_synced();
