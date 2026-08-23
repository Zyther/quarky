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
// LAZY BY DESIGN as of 2026-08-23 (real hardware finding -- see the SDD
// ledger's "Task 18: real-hardware crash investigation" section): bringing
// this transport up costs ~146KB of this board's real ~187KB total
// DMA-capable internal memory pool (hostedInitWiFi() alone: ~128KB; AP mode
// + softAP + the TCP server on top of that: ~18KB more) -- confirmed via
// real per-step heap measurement on real hardware, not estimated. That is
// not reducible from this project's own build (the two esp-hosted Kconfig
// options that would let its mempool prefer PSRAM are compiled OFF in the
// framework's own prebuilt libs) and not movable to PSRAM (DMA transfers
// cannot target PSRAM on this SoC -- proven directly: forcing PSRAM-first
// for every allocation recovered only ~4KB).
//
// init() is therefore NOT called unconditionally at boot any more (it used
// to be) -- BLE C2 (hal/c2link_ble.h) alone handles pairing/control by
// default, leaving this board's small DMA budget free for SD-heavy
// Tab5-native features (RF433 saves, NFC tag library, IR Clone's real
// Flipper-IRDB browsing, etc.) that were silently failing SD reads whenever
// WiFi C2 happened to be up. ui/pairing_screen.cpp's "Enable WiFi Link"
// button is the one real place init() is called from now, so a user who
// actually wants the (faster, higher-throughput) WiFi transport can opt
// into paying its real memory cost explicitly, rather than paying it on
// every boot whether or not anything ends up using it.
class C2LinkWifi : public IC2Link {
public:
    bool init(const uint8_t psk[16], const char *ap_ssid, const char *ap_password, uint16_t port);
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
    bool is_connected() override;
    void poll(); // call every loop() iteration -- accepts a client, reads/dispatches incoming frames

    // True once init() has successfully brought up the AP + TCP server (even
    // if no client has connected yet -- distinct from is_connected(), which
    // additionally requires a live client). Added alongside the lazy-init
    // change above so a caller (ui/pairing_screen.cpp's button) can avoid
    // calling init() a second time -- doing so would leak the previous
    // WiFiServer (see c2link_wifi.cpp's own init() -- it unconditionally
    // `new`s a fresh one with no matching delete of any prior instance).
    bool is_initialized();
};

// millis() timestamp of the last frame successfully decoded off this
// transport (0 if none yet). Free function rather than a method, matching
// the file-scope-static storage the rest of this translation unit already
// uses -- see Task 19 (devices_panel.cpp polls this to derive link freshness
// for the shell's status bar).
uint32_t c2link_wifi_last_recv_ms();
