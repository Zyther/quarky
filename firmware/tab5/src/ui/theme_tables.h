#pragma once
#include <cstdint>

enum class ThemeId : uint8_t {
    ModernLight = 0,
    ModernDark = 1,
    ConsoleGreen = 2,
    ConsoleRed = 3,
    Custom = 255,
};

enum class FontId : uint8_t { Montserrat14 = 0, Unscii16 = 1 };

struct ThemeDesc {
    uint32_t bg;
    uint32_t surface;
    uint32_t text;
    uint32_t muted;
    uint32_t accent;
    uint32_t danger;
    uint32_t chart;
    FontId font;
    int32_t button_radius;
    int32_t button_border_width;
    bool button_shadow;
    bool button_grow;
};

const ThemeDesc *theme_table_lookup(ThemeId id);
ThemeId theme_sanitize_id(uint8_t raw);
