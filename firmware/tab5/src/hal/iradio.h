#pragma once
#include <cstdint>

class IRadio {
public:
    virtual ~IRadio() = default;
    virtual bool connect_wifi(const char *ssid, const char *pass) = 0;
    virtual bool is_connected() = 0;
    virtual uint32_t local_ip() = 0;
};
