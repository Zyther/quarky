#include "theme.h"
#include "screen_scaffold.h"
#include "screen_stack.h"
#include <Preferences.h>
#include <Arduino.h>

static const char *kNamespace = "quarky-ui";
static const char *kThemeIdKey = "theme_id";
static const char *kCustomKey = "theme_custom";

static ThemeId s_current_id = ThemeId::ModernLight;
static const ThemeDesc *s_current_desc = nullptr;
static ThemeDesc s_custom_desc;
static lv_theme_t *s_theme = nullptr;

static void (*s_listeners[8])() = {nullptr};
static uint8_t s_listener_count = 0;

static void install_theme();
static void apply_cb(lv_theme_t *th, lv_obj_t *obj);
static const lv_font_t *map_font(FontId font_id);
static void notify_listeners();
static void apply_live();
static bool load_custom_blob(Preferences &prefs);
static void use_light_fallback(bool log_missing);
static void use_baked(ThemeId id);
static void persist_id(ThemeId id);

void Theme::init() {
    Preferences prefs;
    prefs.begin(kNamespace, true);

    uint8_t raw_id = prefs.getUChar(kThemeIdKey, 0);
    ThemeId sanitized_id = theme_sanitize_id(raw_id);

    if (sanitized_id == ThemeId::Custom) {
        if (load_custom_blob(prefs)) {
            s_current_id = ThemeId::Custom;
            s_current_desc = &s_custom_desc;
        } else {
            use_light_fallback(true);
        }
    } else {
        use_baked(sanitized_id);
    }

    prefs.end();
    install_theme();
}

ThemeId Theme::id() {
    return s_current_id;
}

void Theme::set(ThemeId id) {
    if (id == ThemeId::Custom) {
        Preferences prefs;
        prefs.begin(kNamespace, true);
        bool blob_ok = load_custom_blob(prefs);
        prefs.end();

        if (!blob_ok) {
            use_light_fallback(true);
            apply_live();
            return;
        }

        persist_id(ThemeId::Custom);
        s_current_id = ThemeId::Custom;
        s_current_desc = &s_custom_desc;
        apply_live();
        return;
    }

    persist_id(id);
    use_baked(id);
    apply_live();
}

lv_color_t Theme::color(Token t) {
    if (!s_current_desc) return lv_color_hex(0x000000);

    switch (t) {
        case Token::Bg: return lv_color_hex(s_current_desc->bg);
        case Token::Surface: return lv_color_hex(s_current_desc->surface);
        case Token::Text: return lv_color_hex(s_current_desc->text);
        case Token::Muted: return lv_color_hex(s_current_desc->muted);
        case Token::Accent: return lv_color_hex(s_current_desc->accent);
        case Token::Danger: return lv_color_hex(s_current_desc->danger);
        case Token::Chart: return lv_color_hex(s_current_desc->chart);
        default: return lv_color_hex(0x000000);
    }
}

const lv_font_t *Theme::font() {
    if (!s_current_desc) return lv_font_get_default();
    return map_font(s_current_desc->font);
}

const ThemeDesc &Theme::desc() {
    static ThemeDesc fallback = {};
    if (!s_current_desc) return fallback;
    return *s_current_desc;
}

void Theme::add_listener(void (*fn)()) {
    if (!fn) return;

    for (uint8_t i = 0; i < s_listener_count; i++) {
        if (s_listeners[i] == fn) return;
    }

    if (s_listener_count >= 8) {
        Serial.println("quarky-tab5: [theme] listener overflow");
        return;
    }

    s_listeners[s_listener_count++] = fn;
}

void Theme::remove_listener(void (*fn)()) {
    for (uint8_t i = 0; i < s_listener_count; i++) {
        if (s_listeners[i] == fn) {
            for (uint8_t j = i; j < s_listener_count - 1; j++) {
                s_listeners[j] = s_listeners[j + 1];
            }
            s_listeners[--s_listener_count] = nullptr;
            return;
        }
    }
}

void Theme::apply_tree(lv_obj_t *root) {
    if (root == nullptr) return;

    lv_theme_apply(root);

    for (uint32_t i = 0; i < lv_obj_get_child_count(root); i++) {
        lv_obj_t *child = lv_obj_get_child(root, i);
        apply_tree(child);
    }
}

static bool load_custom_blob(Preferences &prefs) {
    size_t loaded = prefs.getBytes(kCustomKey, &s_custom_desc, sizeof(s_custom_desc));
    return loaded == sizeof(s_custom_desc);
}

static void use_light_fallback(bool log_missing) {
    if (log_missing) {
        Serial.println("quarky-tab5: [theme] custom blob missing — falling back to Modern Light");
    }
    s_current_id = ThemeId::ModernLight;
    s_current_desc = theme_table_lookup(ThemeId::ModernLight);
}

static void use_baked(ThemeId id) {
    s_current_id = id;
    s_current_desc = theme_table_lookup(id);
    if (!s_current_desc) {
        use_light_fallback(false);
    }
}

static void persist_id(ThemeId id) {
    Preferences prefs;
    prefs.begin(kNamespace, false);
    bool write_success = prefs.putUChar(kThemeIdKey, static_cast<uint8_t>(id));
    prefs.end();
    if (!write_success) {
        Serial.printf("quarky-tab5: [theme] persist failed\n");
    }
}

static void notify_listeners() {
    for (uint8_t i = 0; i < s_listener_count; i++) {
        if (s_listeners[i]) {
            s_listeners[i]();
        }
    }
}

static void apply_live() {
    install_theme();
    ScreenStack::for_each([](lv_obj_t *screen) { Theme::apply_tree(screen); });
    notify_listeners();
}

static void install_theme() {
    if (!s_current_desc) return;

    lv_display_t *disp = lv_display_get_default();
    const lv_font_t *mapped = map_font(s_current_desc->font);
    const bool dark = (s_current_id != ThemeId::ModernLight);

    lv_theme_t *parent = lv_theme_default_init(
        disp,
        lv_color_hex(s_current_desc->accent),
        lv_color_hex(s_current_desc->danger),
        dark,
        mapped);
    if (!parent) return;

    if (!s_theme) {
        s_theme = lv_theme_create();
        if (!s_theme) return;
    }

    lv_theme_copy(s_theme, parent);
    lv_theme_set_parent(s_theme, parent);
    lv_theme_set_apply_cb(s_theme, apply_cb);
    lv_display_set_theme(disp, s_theme);
}

static void apply_cb(lv_theme_t *th, lv_obj_t *obj) {
    LV_UNUSED(th);
    if (!obj || !s_current_desc) return;

    const ThemeDesc &d = *s_current_desc;

    if (lv_obj_check_type(obj, &lv_obj_class)) {
        // After lv_theme_apply wipe, parent default paints a CARD on generic
        // lv_obj. Scaffold chrome must restamp; skip leaves that card.
        if (lv_obj_has_flag(obj, LV_OBJ_FLAG_USER_1)) {
            lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(obj, kContentPad, LV_PART_MAIN);
            lv_obj_set_style_pad_row(obj, kContentPad, LV_PART_MAIN);
            lv_obj_set_style_pad_column(obj, kContentPad, LV_PART_MAIN);
            return;
        }

        if (lv_obj_has_flag(obj, LV_OBJ_FLAG_USER_2)) {
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
            lv_obj_set_style_pad_all(obj, kMenuBarPad, LV_PART_MAIN);
        }

        const bool is_screen = (lv_obj_get_parent(obj) == nullptr);
        if (is_screen) {
            lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
            lv_obj_set_style_pad_row(obj, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_color(obj, lv_color_hex(d.bg), LV_PART_MAIN);
            lv_obj_set_style_text_font(obj, map_font(d.font), LV_PART_MAIN);
        } else if (d.button_border_width > 0) {
            lv_obj_set_style_bg_color(obj, lv_color_hex(d.surface), LV_PART_MAIN);
            lv_obj_set_style_text_font(obj, map_font(d.font), LV_PART_MAIN);
        }
    } else if (lv_obj_check_type(obj, &lv_button_class)) {
        // Console only. Modern leaves radius/border/shadow/grow/bg to parent default.
        if (d.button_border_width > 0) {
            lv_obj_set_style_radius(obj, d.button_radius, LV_PART_MAIN);
            lv_obj_set_style_border_width(obj, d.button_border_width, LV_PART_MAIN);
            lv_obj_set_style_bg_color(obj, lv_color_hex(d.bg), LV_PART_MAIN);
            lv_obj_set_style_border_color(obj, lv_color_hex(d.accent), LV_PART_MAIN);
            lv_obj_set_style_bg_color(obj, lv_color_lighten(lv_color_hex(d.bg), LV_OPA_20),
                                      LV_PART_MAIN | LV_STATE_PRESSED);
            lv_obj_set_style_text_color(obj, lv_color_hex(d.text), LV_PART_MAIN);

            if (d.button_shadow) {
                lv_obj_set_style_shadow_width(obj, 4, LV_PART_MAIN);
                lv_obj_set_style_shadow_opa(obj, LV_OPA_30, LV_PART_MAIN);
            } else {
                lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
                lv_obj_set_style_shadow_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
            }

            lv_obj_set_style_transform_scale(obj, LV_SCALE_NONE, LV_PART_MAIN | LV_STATE_PRESSED);
            if (d.button_grow) {
                const int32_t grow = lv_dpx(3);
                lv_obj_set_style_transform_width(obj, grow, LV_PART_MAIN | LV_STATE_PRESSED);
                lv_obj_set_style_transform_height(obj, grow, LV_PART_MAIN | LV_STATE_PRESSED);
            } else {
                lv_obj_set_style_transform_width(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
                lv_obj_set_style_transform_height(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
            }
        }
    } else if (lv_obj_check_type(obj, &lv_label_class)) {
        lv_obj_set_style_text_font(obj, map_font(d.font), LV_PART_MAIN);

        lv_style_value_t existing;
        const bool has_local_color =
            lv_obj_get_local_style_prop(obj, LV_STYLE_TEXT_COLOR, &existing, LV_PART_MAIN)
            != LV_STYLE_RES_NOT_FOUND;
        lv_obj_t *parent = lv_obj_get_parent(obj);
        const bool on_button = parent && lv_obj_check_type(parent, &lv_button_class);
        if (!has_local_color && !on_button) {
            lv_obj_set_style_text_color(obj, lv_color_hex(d.text), LV_PART_MAIN);
        }
    } else if (lv_obj_check_type(obj, &lv_textarea_class) ||
               lv_obj_check_type(obj, &lv_list_class) ||
               lv_obj_check_type(obj, &lv_keyboard_class)) {
        lv_obj_set_style_bg_color(obj, lv_color_hex(d.surface), LV_PART_MAIN);
        lv_obj_set_style_text_color(obj, lv_color_hex(d.text), LV_PART_MAIN);
        lv_obj_set_style_text_font(obj, map_font(d.font), LV_PART_MAIN);
        if (d.button_border_width > 0) {
            lv_obj_set_style_radius(obj, d.button_radius, LV_PART_MAIN);
            lv_obj_set_style_border_width(obj, d.button_border_width, LV_PART_MAIN);
            lv_obj_set_style_border_color(obj, lv_color_hex(d.accent), LV_PART_MAIN);
            lv_obj_set_style_border_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
        }
    } else if (lv_obj_check_type(obj, &lv_chart_class)) {
        lv_obj_set_style_bg_color(obj, lv_color_hex(d.surface), LV_PART_MAIN);
    }
}

static const lv_font_t *map_font(FontId font_id) {
    switch (font_id) {
        case FontId::Montserrat14:
            return &lv_font_montserrat_14;
        case FontId::Unscii16:
            return &lv_font_unscii_16;
        default:
            return lv_font_get_default();
    }
}
