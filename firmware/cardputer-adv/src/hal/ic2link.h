#pragma once
#include <cstdint>
#include <proto.h>

// Cardputer-ADV side of the Tab5 <-> Cardputer-ADV control channel.
//
// Same shape as Tab5's IC2Link (Task 11/13): no per-message peer addressing.
// That was ESP-NOW-specific (see Task 11's amendment) -- a single persistent
// client connection to one known server (Tab5's WiFi AP + TCP socket, or its
// BLE GATT server per Task 13/17) doesn't need per-frame peer identification.
using C2LinkReceiveHandler = void (*)(const c2proto::Frame &);

class IC2Link {
public:
    virtual ~IC2Link() = default;
    virtual bool send(const c2proto::Frame &frame) = 0;
    virtual void set_receive_handler(C2LinkReceiveHandler handler) = 0;
    virtual bool is_connected() = 0;
};
