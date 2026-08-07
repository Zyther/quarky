#pragma once
#include "ic2link.h"

// ESP-NOW backend for the Tab5 <-> Cardputer-ADV control channel.
//
// Rides on the WiFi STA interface without associating to an access point
// (WiFi.mode(WIFI_STA) + esp_now_init(), no WiFi.begin()). The PSK passed to
// init() doubles as the ESP-NOW peer's LMK (local master key), so payload
// encryption is handled by the ESP-NOW/WiFi driver itself, not by
// shared/c2proto -- c2proto only frames/parses the wire format.
//
// Real message delivery (this device <-> a matching Cardputer-ADV instance)
// can only be verified once that satellite side exists (a later task) and
// with real hardware; see task-11-report.md for what was and wasn't
// verified here.
class C2LinkEspNow : public IC2Link {
public:
    bool init(const uint8_t psk[16], const uint8_t peer_mac[6]) override;
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
};
