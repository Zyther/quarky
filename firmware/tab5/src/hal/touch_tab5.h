#pragma once
#include "itouch.h"
#include <stdint.h>

// Tab5 touch input.
//
// This board ships in two hardware revisions with *completely different* touch
// hardware, exactly mirroring the display split documented in
// hal/display_tab5.h:
//
//   * ILI9881C panel  -> standalone GT911 touch IC at I2C 0x14 (or 0x5D).
//   * ST7121/ST7123   -> NO separate touch IC at all. These are TDDI (Touch
//                        and Display Driver Integration) parts: the touch
//                        engine lives inside the same silicon as the display
//                        controller and is read over I2C at the panel's own
//                        address, 0x55, through a vendor register range.
//
// Which one is present is decided at runtime by probing, not by a build flag.
// See touch_tab5.cpp for the full citation trail.
class TouchTab5 : public ITouch {
public:
    enum class Backend : uint8_t {
        None = 0,  // nothing answered; touch disabled (UI still works)
        StTddi,    // ST7121/ST7123 integrated touch engine @ 0x55
        Gt911,     // standalone GT911 @ 0x14 / 0x5D
    };

    void init() override;
    void read(int16_t &x, int16_t &y, bool &pressed) override;

    // Diagnostics; mirrors DisplayTab5::controller()/ready().
    Backend backend() const;
    bool available() const;
};
