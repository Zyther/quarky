#include <unity.h>
#include "ui/theme_tables.h"

void test_lookup_four_baked() {
    TEST_ASSERT_NOT_NULL(theme_table_lookup(ThemeId::ModernLight));
    TEST_ASSERT_NOT_NULL(theme_table_lookup(ThemeId::ModernDark));
    TEST_ASSERT_NOT_NULL(theme_table_lookup(ThemeId::ConsoleGreen));
    TEST_ASSERT_NOT_NULL(theme_table_lookup(ThemeId::ConsoleRed));
    TEST_ASSERT_NULL(theme_table_lookup(ThemeId::Custom));
    TEST_ASSERT_NULL(theme_table_lookup(static_cast<ThemeId>(4)));
}

void test_sanitize_id() {
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::ModernLight),
                      static_cast<uint8_t>(theme_sanitize_id(0)));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::ModernDark),
                      static_cast<uint8_t>(theme_sanitize_id(1)));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::ConsoleGreen),
                      static_cast<uint8_t>(theme_sanitize_id(2)));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::ConsoleRed),
                      static_cast<uint8_t>(theme_sanitize_id(3)));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::Custom),
                      static_cast<uint8_t>(theme_sanitize_id(255)));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::ModernLight),
                      static_cast<uint8_t>(theme_sanitize_id(4)));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(ThemeId::ModernLight),
                      static_cast<uint8_t>(theme_sanitize_id(99)));
}

void test_console_danger_distinct_from_accent() {
    const ThemeDesc *g = theme_table_lookup(ThemeId::ConsoleGreen);
    const ThemeDesc *r = theme_table_lookup(ThemeId::ConsoleRed);
    TEST_ASSERT_NOT_NULL(g);
    TEST_ASSERT_NOT_NULL(r);
    TEST_ASSERT_NOT_EQUAL(g->danger, g->accent);
    TEST_ASSERT_EQUAL_HEX32(0xFF3333u, g->danger);
    TEST_ASSERT_NOT_EQUAL(r->danger, r->accent);
    TEST_ASSERT_EQUAL_HEX32(0xFFCC00u, r->danger);
}

void test_button_look() {
    const ThemeDesc *light = theme_table_lookup(ThemeId::ModernLight);
    const ThemeDesc *dark = theme_table_lookup(ThemeId::ModernDark);
    const ThemeDesc *green = theme_table_lookup(ThemeId::ConsoleGreen);
    TEST_ASSERT_TRUE(light->button_radius > 0);
    TEST_ASSERT_EQUAL(0, light->button_border_width);
    TEST_ASSERT_TRUE(light->button_grow);
    TEST_ASSERT_EQUAL(light->button_radius, dark->button_radius);
    TEST_ASSERT_EQUAL(0, green->button_radius);
    TEST_ASSERT_EQUAL(2, green->button_border_width);
    TEST_ASSERT_FALSE(green->button_grow);
    TEST_ASSERT_FALSE(green->button_shadow);
}

void test_fonts_and_modern_danger() {
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(FontId::Montserrat14),
                      static_cast<uint8_t>(theme_table_lookup(ThemeId::ModernLight)->font));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(FontId::Montserrat14),
                      static_cast<uint8_t>(theme_table_lookup(ThemeId::ModernDark)->font));
    // Console themes use the generated 12px monospace face (merged LVGL
    // symbols), no longer UNSCII_16.
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(FontId::QuarkyMono12),
                      static_cast<uint8_t>(theme_table_lookup(ThemeId::ConsoleGreen)->font));
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(FontId::QuarkyMono12),
                      static_cast<uint8_t>(theme_table_lookup(ThemeId::ConsoleRed)->font));
    TEST_ASSERT_EQUAL_HEX32(0xF44336u, theme_table_lookup(ThemeId::ModernLight)->danger);
    TEST_ASSERT_EQUAL_HEX32(0x2196F3u, theme_table_lookup(ThemeId::ModernLight)->chart);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_lookup_four_baked);
    RUN_TEST(test_sanitize_id);
    RUN_TEST(test_console_danger_distinct_from_accent);
    RUN_TEST(test_button_look);
    RUN_TEST(test_fonts_and_modern_danger);
    return UNITY_END();
}
