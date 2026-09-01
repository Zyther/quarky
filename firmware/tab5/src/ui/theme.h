#pragma once
#include "theme_tables.h"
#include <lvgl.h>

enum class Token { Bg, Surface, Text, Muted, Accent, Danger, Chart };

namespace Theme {
void init();
ThemeId id();
void set(ThemeId id);
lv_color_t color(Token t);
const lv_font_t *font();
const ThemeDesc &desc();
void add_listener(void (*fn)());
void remove_listener(void (*fn)());
void apply_tree(lv_obj_t *root);
}