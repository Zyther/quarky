#pragma once
#include <cstdint>

class Device {
public:
    static Device &instance();
    void init();
    bool display_ready() const { return display_ready_; }
    bool keyboard_ready() const { return keyboard_ready_; }

private:
    bool display_ready_ = false;
    bool keyboard_ready_ = false;
};
