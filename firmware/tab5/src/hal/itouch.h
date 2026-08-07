#pragma once
#include <cstdint>

class ITouch {
public:
    virtual ~ITouch() = default;
    virtual void init() = 0;
    virtual void read(int16_t &x, int16_t &y, bool &pressed) = 0;
};
