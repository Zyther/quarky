#pragma once
#include "itouch.h"

class TouchGT911 : public ITouch {
public:
    void init() override;
    void read(int16_t &x, int16_t &y, bool &pressed) override;
};
