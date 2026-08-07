#pragma once
#include <cstdint>
#include <proto.h>

using C2LinkReceiveHandler = void (*)(const c2proto::Frame &);

class IC2Link {
public:
    virtual ~IC2Link() = default;
    virtual bool init(const uint8_t psk[16], const uint8_t peer_mac[6]) = 0;
    virtual bool send(const c2proto::Frame &frame) = 0;
    virtual void set_receive_handler(C2LinkReceiveHandler handler) = 0;
};
