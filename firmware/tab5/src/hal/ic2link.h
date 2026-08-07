#pragma once
#include <cstdint>
#include <proto.h>

using C2LinkReceiveHandler = void (*)(const c2proto::Frame &);

class IC2Link {
public:
    virtual ~IC2Link() = default;
    virtual bool send(const c2proto::Frame &frame) = 0;
    virtual void set_receive_handler(C2LinkReceiveHandler handler) = 0;
    virtual bool is_connected() = 0;
};
