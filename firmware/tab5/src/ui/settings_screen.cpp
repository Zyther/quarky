#include "settings_screen.h"
#include "screen_scaffold.h"
#include "screen_stack.h"
#include "theme.h"
#include <lvgl.h>

static lv_obj_t *s_theme_buttons[4] = {};

static void refresh_theme_checked() {
    ThemeId current = Theme::id();
    for (int i = 0; i < 4; ++i) {
        if (s_theme_buttons[i]) {
            lv_obj_clear_state(s_theme_buttons[i], LV_STATE_CHECKED);
        }
    }
    int active = -1;
    switch (current) {
        case ThemeId::ModernLight: active = 0; break;
        case ThemeId::ModernDark: active = 1; break;
        case ThemeId::ConsoleGreen: active = 2; break;
        case ThemeId::ConsoleRed: active = 3; break;
        default: break;
    }
    if (active >= 0 && s_theme_buttons[active]) {
        lv_obj_add_state(s_theme_buttons[active], LV_STATE_CHECKED);
    }
}

lv_obj_t *build_settings_hub() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Settings", &content);

    lv_obj_t *theme_tile = lv_button_create(content);
    lv_obj_set_size(theme_tile, 200, 100);
    lv_obj_t *theme_label = lv_label_create(theme_tile);
    lv_label_set_text(theme_label, "Theme");
    lv_obj_add_event_cb(theme_tile, [](lv_event_t *) {
        ScreenStack::push(build_theme_picker());
    }, LV_EVENT_CLICKED, nullptr);

    return screen;
}

lv_obj_t *build_theme_picker() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Theme", &content);

    struct ThemeOption {
        const char *label;
        ThemeId id;
    };
    static const ThemeOption kOptions[] = {
        {"Modern Light", ThemeId::ModernLight},
        {"Modern Dark", ThemeId::ModernDark},
        {"Console Green", ThemeId::ConsoleGreen},
        {"Console Red", ThemeId::ConsoleRed},
    };

    for (int i = 0; i < 4; ++i) {
        lv_obj_t *btn = lv_button_create(content);
        lv_obj_set_size(btn, 200, 100);
        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, kOptions[i].label);
        s_theme_buttons[i] = btn;
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            auto *opt = (ThemeOption *)lv_event_get_user_data(e);
            Theme::set(opt->id);
            refresh_theme_checked();
        }, LV_EVENT_CLICKED, (void *)&kOptions[i]);
    }

    refresh_theme_checked();

    return screen;
}
