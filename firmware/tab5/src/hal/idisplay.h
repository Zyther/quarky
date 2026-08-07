#pragma once
#include <cstdint>

class IDisplay {
public:
    virtual ~IDisplay() = default;
    virtual void init() = 0;
    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual void flush(int x1, int y1, int x2, int y2, const uint16_t *colors) = 0;
};
