#include "theme.h"
#include <Preferences.h>
#include <Arduino.h>

static const char *kNamespace = "quarky-ui";
static const char *kThemeIdKey = "theme_id";
static const char *kCustomKey = "theme_custom";

// Static theme state
static ThemeId s_current_id = ThemeId::ModernLight;
static const ThemeDesc *s_current_desc = nullptr;
static lv_theme_t s_theme;

// Listener management
static void (*s_listeners[8])() = {nullptr};
static uint8_t s_listener_count = 0;

// Forward declarations
static void install_theme();
static void apply_cb(lv_theme_t *th, lv_obj_t *obj);
static const lv_font_t *map_font(FontId font_id);

void Theme::init() {
    Preferences prefs;
    prefs.begin(kNamespace, true);
    
    uint8_t raw_id = prefs.getUChar(kThemeIdKey, 0);
    ThemeId sanitized_id = theme_sanitize_id(raw_id);
    
    if (sanitized_id == ThemeId::Custom) {
        // Try to load custom blob
        ThemeDesc custom_desc;
        size_t loaded = prefs.getBytes(kCustomKey, &custom_desc, sizeof(custom_desc));
        
        if (loaded != sizeof(custom_desc)) {
            Serial.println("quarky-tab5: [theme] custom blob missing — falling back to Modern Light");
            s_current_id = ThemeId::ModernLight;
        } else {
            s_current_id = ThemeId::Custom;
            // For Custom, we'd need to store the custom_desc somewhere accessible
            // For now, fall back since custom isn't fully specified in the brief
            Serial.println("quarky-tab5: [theme] custom blob missing — falling back to Modern Light");
            s_current_id = ThemeId::ModernLight;
        }
    } else {
        s_current_id = sanitized_id;
    }
    
    prefs.end();
    
    // Load the theme descriptor
    s_current_desc = theme_table_lookup(s_current_id);
    if (!s_current_desc) {
        s_current_desc = theme_table_lookup(ThemeId::ModernLight);
        s_current_id = ThemeId::ModernLight;
    }
    
    install_theme();
}

ThemeId Theme::id() {
    return s_current_id;
}

void Theme::set(ThemeId id) {
    if (id == ThemeId::Custom) {
        // Check if custom blob exists
        Preferences prefs;
        prefs.begin(kNamespace, true);
        
        ThemeDesc custom_desc;
        size_t loaded = prefs.getBytes(kCustomKey, &custom_desc, sizeof(custom_desc));
        prefs.end();
        
        if (loaded != sizeof(custom_desc)) {
            Serial.println("quarky-tab5: [theme] custom blob missing — falling back to Modern Light");
            // Apply Light in RAM, do not persist Custom
            s_current_id = ThemeId::ModernLight;
            s_current_desc = theme_table_lookup(s_current_id);
            install_theme();
            apply_tree(lv_display_get_screen_active(lv_display_get_default()));
            
            // Invoke listeners
            for (uint8_t i = 0; i < s_listener_count; i++) {
                if (s_listeners[i]) {
                    s_listeners[i]();
                }
            }
            return;
        }
    }
    
    // Persist the theme ID
    Preferences prefs;
    prefs.begin(kNamespace, false);
    bool write_success = prefs.putUChar(kThemeIdKey, static_cast<uint8_t>(id));
    prefs.end();
    
    if (!write_success) {
        Serial.printf("quarky-tab5: [theme] persist failed\n");
    }
    
    // Apply in RAM regardless of persist result
    s_current_id = id;
    s_current_desc = theme_table_lookup(id);
    if (!s_current_desc) {
        s_current_desc = theme_table_lookup(ThemeId::ModernLight);
        s_current_id = ThemeId::ModernLight;
    }
    
    install_theme();
    
    // Walk live screens (Task 3 will expand this to all screens)
    apply_tree(lv_display_get_screen_active(lv_display_get_default()));
    
    // Invoke listeners
    for (uint8_t i = 0; i < s_listener_count; i++) {
        if (s_listeners[i]) {
            s_listeners[i]();
        }
    }
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
    
    // Check for duplicates
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
            // Compact the hole
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
    
    // Recurse through children
    for (uint32_t i = 0; i < lv_obj_get_child_count(root); i++) {
        lv_obj_t *child = lv_obj_get_child(root, i);
        apply_tree(child);
    }
}

// Static helper functions
static void install_theme() {
    if (!s_current_desc) return;
    
    // Initialize theme structure
    lv_memzero(&s_theme, sizeof(s_theme));
    
    s_theme.apply_cb = apply_cb;
    s_theme.font_normal = map_font(s_current_desc->font);
    s_theme.font_small = map_font(s_current_desc->font);
    s_theme.font_large = map_font(s_current_desc->font);
    s_theme.color_primary = lv_color_hex(s_current_desc->accent);
    
    lv_display_set_theme(lv_display_get_default(), &s_theme);
}

static void apply_cb(lv_theme_t *th, lv_obj_t *obj) {
    if (!obj || !s_current_desc) return;
    
    const lv_obj_class_t *class_p = lv_obj_get_class(obj);
    
    if (lv_obj_check_type(obj, &lv_obj_class)) {
        // Base objects (screens/panels)
        lv_color_t bg_color;
        
        // Check if this is a screen (no parent) or a regular panel
        if (lv_obj_get_parent(obj) == nullptr) {
            // This is a screen
            bg_color = lv_color_hex(s_current_desc->bg);
        } else {
            // This is a panel/container, use surface color
            bg_color = lv_color_hex(s_current_desc->surface);
        }
        
        // Skip if already transparent
        if (lv_obj_get_style_bg_opa(obj, LV_PART_MAIN) != LV_OPA_TRANSP) {
            lv_obj_set_style_bg_color(obj, bg_color, LV_PART_MAIN);
            lv_obj_set_style_text_font(obj, map_font(s_current_desc->font), LV_PART_MAIN);
        }
    }
    else if (lv_obj_check_type(obj, &lv_button_class)) {
        // Button styling
        if (s_current_desc->font == FontId::Montserrat14) {
            // Modern theme
            lv_obj_set_style_bg_color(obj, lv_color_hex(s_current_desc->accent), LV_PART_MAIN);
            lv_obj_set_style_radius(obj, s_current_desc->button_radius, LV_PART_MAIN);
            lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
            
            if (s_current_desc->button_shadow) {
                // Add shadow (implementation depends on LVGL shadow support)
                lv_obj_set_style_shadow_width(obj, 4, LV_PART_MAIN);
                lv_obj_set_style_shadow_opa(obj, LV_OPA_30, LV_PART_MAIN);
            }
            
            // Pressed state (slightly darker)
            lv_color_t pressed_color = lv_color_darken(lv_color_hex(s_current_desc->accent), LV_OPA_20);
            lv_obj_set_style_bg_color(obj, pressed_color, LV_PART_MAIN | LV_STATE_PRESSED);
            
            // Transform for grow effect (only if enabled)
            if (s_current_desc->button_grow) {
                lv_obj_set_style_transform_scale(obj, 105, LV_PART_MAIN | LV_STATE_PRESSED);
            }
        } else {
            // Console theme
            lv_obj_set_style_bg_color(obj, lv_color_hex(s_current_desc->bg), LV_PART_MAIN);
            lv_obj_set_style_border_width(obj, s_current_desc->button_border_width, LV_PART_MAIN);
            lv_obj_set_style_border_color(obj, lv_color_hex(s_current_desc->accent), LV_PART_MAIN);
            lv_obj_set_style_radius(obj, 0, LV_PART_MAIN);
            
            // Pressed state (slightly lighter)
            lv_color_t pressed_color = lv_color_lighten(lv_color_hex(s_current_desc->bg), LV_OPA_20);
            lv_obj_set_style_bg_color(obj, pressed_color, LV_PART_MAIN | LV_STATE_PRESSED);
        }
    }
    else if (lv_obj_check_type(obj, &lv_label_class)) {
        // Label styling
        lv_obj_set_style_text_color(obj, lv_color_hex(s_current_desc->text), LV_PART_MAIN);
        lv_obj_set_style_text_font(obj, map_font(s_current_desc->font), LV_PART_MAIN);
    }
    else if (lv_obj_check_type(obj, &lv_textarea_class) ||
             lv_obj_check_type(obj, &lv_list_class) ||
             lv_obj_check_type(obj, &lv_keyboard_class)) {
        // Text input and list styling
        lv_obj_set_style_bg_color(obj, lv_color_hex(s_current_desc->surface), LV_PART_MAIN);
        lv_obj_set_style_text_color(obj, lv_color_hex(s_current_desc->text), LV_PART_MAIN);
        lv_obj_set_style_text_font(obj, map_font(s_current_desc->font), LV_PART_MAIN);
        lv_obj_set_style_radius(obj, s_current_desc->button_radius, LV_PART_MAIN);
        lv_obj_set_style_border_width(obj, s_current_desc->button_border_width, LV_PART_MAIN);
    }
    else if (lv_obj_check_type(obj, &lv_chart_class)) {
        // Chart styling
        lv_obj_set_style_bg_color(obj, lv_color_hex(s_current_desc->surface), LV_PART_MAIN);
        // Series colors are listener-owned
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