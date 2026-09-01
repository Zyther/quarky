#include "theme_tables.h"

static const ThemeDesc kModernLight = {
    0xEAEEF3, 0xFFFFFF, 0x3B3E42, 0x6B6E73, 0x2196F3, 0xF44336, 0x2196F3,
    FontId::Montserrat14, 8, 0, true, true};
static const ThemeDesc kModernDark = {
    0x15171A, 0x2A2D32, 0xEFEFEF, 0xA0A3A8, 0x2196F3, 0xF44336, 0x2196F3,
    FontId::Montserrat14, 8, 0, true, true};
static const ThemeDesc kConsoleGreen = {
    0x000000, 0x001100, 0x00FF00, 0x00AA00, 0x00FF00, 0xFF3333, 0x00FF00,
    FontId::Unscii16, 0, 2, false, false};
static const ThemeDesc kConsoleRed = {
    0x000000, 0x110000, 0xFF3333, 0xAA0000, 0xFF0000, 0xFFCC00, 0xFF0000,
    FontId::Unscii16, 0, 2, false, false};

const ThemeDesc *theme_table_lookup(ThemeId id) {
    switch (id) {
    case ThemeId::ModernLight: return &kModernLight;
    case ThemeId::ModernDark: return &kModernDark;
    case ThemeId::ConsoleGreen: return &kConsoleGreen;
    case ThemeId::ConsoleRed: return &kConsoleRed;
    default: return nullptr;
    }
}

ThemeId theme_sanitize_id(uint8_t raw) {
    if (raw == 0) return ThemeId::ModernLight;
    if (raw == 1) return ThemeId::ModernDark;
    if (raw == 2) return ThemeId::ConsoleGreen;
    if (raw == 3) return ThemeId::ConsoleRed;
    if (raw == 255) return ThemeId::Custom;
    return ThemeId::ModernLight;
}
