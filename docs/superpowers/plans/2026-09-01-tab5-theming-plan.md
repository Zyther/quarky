# Tab5 Theming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship four built-in Tab5 themes (Modern Light, Modern Dark, Console Green, Console Red) with live apply of colors, fonts, and button look, a Settings → Theme picker, NVS persistence, and a reserved Custom hook — covering every existing app page.

**Architecture:** A LVGL-free `theme_tables` module owns baked `ThemeDesc` rows and id sanitizing (host-testable). Device-side `Theme` installs an LVGL display theme, persists `theme_id` in `Preferences` namespace `quarky-ui`, walks `ScreenStack` on change, and notifies listeners for the few widgets that stamp semantic colors locally. Settings is a shell launcher tile plus hub screen, not a `FeatureRegistry` category.

**Tech Stack:** LVGL 9 (`lv_theme_t` / `lv_theme_apply` / `lv_display_set_theme`), `lv_font_montserrat_14` + `lv_font_unscii_16`, Arduino `Preferences` (same as `PskStore`), Unity host-native tests via `firmware/tab5` `[env:native]`.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-09-01-tab5-theming-design.md`. Do not edit existing phase specs, phase plans, `docs/phases/*`, or add a phase number to `CLAUDE.md`.
- Tab5 only. Do not touch Cardputer-ADV.
- Do not add `Category::SETTINGS` to `shared/feature_contract`.
- Modern Light must match today's look (LVGL default light). Modern Dark is LVGL's dark counterpart, not a new palette. Console themes use `Unscii16`, radius 0, 2px border, no shadow/grow.
- Semantic tokens stay distinct: Console Green `danger` is red (`0xFF3333`); Console Red `danger` is amber (`0xFFCC00`).
- Pairing QR canvas stays black-on-white. Evil Portal HTML/CSS is out of scope.
- `Theme::init` / `Theme::set` run on the Arduino loop task only.
- Touch targets stay 200×100 for Back and launcher tiles. Do not shrink them for console fonts.
- No new `lv_palette_main` call sites.

## File structure

| File | Responsibility |
|---|---|
| `firmware/tab5/src/ui/theme_tables.h` / `.cpp` | `ThemeId`, `FontId`, `ThemeDesc`, baked rows, `theme_table_lookup`, `theme_sanitize_id`. No Arduino, no LVGL. |
| `firmware/tab5/src/ui/theme.h` / `.cpp` | Device API: `init`/`set`/`id`/`color`/`font`/`desc`/listeners, NVS, LVGL theme `apply_cb`, live tree walk. |
| `firmware/tab5/src/ui/settings_screen.h` / `.cpp` | Settings hub + Theme picker. |
| `firmware/tab5/src/ui/screen_stack.h` / `.cpp` | Add `for_each`. |
| `firmware/tab5/src/ui/shell.cpp` | Settings launcher tile. |
| `firmware/tab5/src/main.cpp` | `Theme::init()` after `lvgl_port_init`, before `Shell::build`. |
| `firmware/tab5/include/lv_conf.h` | `LV_FONT_UNSCII_16 1`. |
| `firmware/tab5/platformio.ini` | `[env:native]` `build_src_filter` adds `theme_tables.cpp`. |
| `firmware/tab5/test/test_theme/test_theme.cpp` | Host-native table tests. |
| Convert: `cc1101_jammer.cpp`, `cc1101_keeloq.cpp`, `cc1101_spectrum.cpp`, `wifi_spectrum.cpp` | `Theme::color` + listeners. |
| Comment only: `pairing_screen.cpp` | QR stays black/white; document why. |

## Page inventory (every Tab5 UI surface)

Every row must restyle when the theme changes. Most inherit the LVGL theme (no per-file edit). Convert rows get token + listener work in Task 6. Exempt rows keep their functional colors.

### Chrome and shared helpers

| Page / surface | File | How themed |
|---|---|---|
| Launcher + status bar | `ui/shell.cpp` | Inherit. New Settings tile in Task 5. |
| Category screens (Utility, WiFi, BLE, NFC, RF433, IR, Sub-GHz) | `ui/shell.cpp` `build_category_screen` | Inherit. |
| Settings hub | `ui/settings_screen.cpp` (new) | Inherit + built in Task 5. |
| Theme picker | `ui/settings_screen.cpp` (new) | Inherit + selected-state chrome; live-apply target. |
| Sub-screen chrome (Back + title + content) | `ui/screen_scaffold.cpp` | Inherit. Padding/radius/opa locals stay (layout, not palette). |
| Status-bar link text | `ui/devices_panel.cpp` | Inherit (text-only updates). |
| File browser | `ui/file_browser.cpp` | Inherit. |
| Deep file browser | `ui/deep_file_browser.cpp` | Inherit. |
| BLE target picker | `features/ble/ble_target_picker.cpp` | Inherit. |
| Keyboard Test | `ui/keyboard_test_screen.cpp` | Inherit (`lv_textarea` / `lv_keyboard` in `apply_cb`). |
| Pair Satellite | `ui/pairing_screen.cpp` | Chrome inherit. **QR canvas exempt** (white bg / black modules). |
| Ping Satellite | `features/ping_feature.cpp` | **No screen** — tap sends; Utility category tile inherits. |

### Utility / WiFi

| Page | File | How themed |
|---|---|---|
| WiFi Scan | `features/wifi/wifi_scan.cpp` | Inherit. |
| WiFi Spectrum | `features/wifi/wifi_spectrum.cpp` | **Convert** series → `Token::Chart` + listener. |
| WiFi Connect | `features/wifi/wifi_connect.cpp` | Inherit. |
| Evil Portal | `features/wifi/wifi_evil_portal.cpp` | Tab5 chrome inherit. Served HTML/CSS **out of scope**. |
| WiFi PMKID Capture | `features/wifi/wifi_pmkid.cpp` | Inherit. **Not on launcher** (deliberately unregistered); still themed if re-enabled. |

### BLE

| Page | File | How themed |
|---|---|---|
| BLE Scan | `features/ble/ble_scan.cpp` | Inherit. |
| BLE Spam | `features/ble/ble_spam.cpp` | Inherit. |
| BLE Tracker Finder | `features/ble/ble_finder.cpp` | Inherit. |
| BLE Sniffer (CSV) | `features/ble/ble_sniffer.cpp` | Inherit. |
| BLE Clone | `features/ble/ble_clone.cpp` | Inherit. |
| BLE Karma | `features/ble/ble_karma.cpp` | Inherit. |
| Sour Apple | `features/ble/ble_sourapple.cpp` | Inherit. |
| Find My Emulator | `features/ble/ble_findmy.cpp` | Inherit. |
| GATT Explorer | `features/ble/ble_gatt_explorer.cpp` | Inherit. |
| BLE Flood | `features/ble/ble_flood.cpp` | Inherit. |
| BLE Bad-KB | `features/ble/ble_bad_kb.cpp` | Inherit. |
| Fast Pair Exploit | `features/ble/ble_fastpair_exploit.cpp` | Inherit. |
| HFP Exploit | `features/ble/ble_hfp_exploit.cpp` | Inherit. |
| WhisperPair | `features/ble/ble_whisperpair.cpp` | Inherit. |

### NFC / RF433 / IR / Sub-GHz

| Page | File | How themed |
|---|---|---|
| NFC: Tag Read | `features/nfc/nfc_read.cpp` | Inherit. |
| RFID2: Tag Read | `features/nfc/nfc_read.cpp` | Inherit. |
| NFC Tag Library | `features/nfc/nfc_tag_library_ui.cpp` | Inherit. |
| Emulate Tag | `features/nfc/nfc_emulate.cpp` | Inherit (opened from library, no launcher tile). |
| RFID2: MIFARE Keys | `features/nfc/nfc_mifare_crack.cpp` | Inherit. |
| RFID2: Amiibo | `features/nfc/nfc_amiibo.cpp` | Inherit. |
| NFC: EMV Card Read | `features/nfc/nfc_emv_read.cpp` | Inherit. |
| RF433 Scan | `features/rf433/rf433_scan.cpp` | Inherit (pad-only locals). |
| RF433 Bruteforce | `features/rf433/rf433_bruteforce.cpp` | Inherit. |
| TV-B-Gone | `features/ir/ir_tvbgone.cpp` | Inherit. |
| IR Learn | `features/ir/ir_learn.cpp` | Inherit. |
| IR Clone (dir + file screens) | `features/ir/ir_clone.cpp` | Inherit (pad-only locals). |
| IR Jammer | `features/ir/ir_jammer.cpp` | Inherit. |
| CC1101 Scan | `features/cc1101/cc1101_scan.cpp` | Inherit (pad-only locals). |
| CC1101 Spectrum | `features/cc1101/cc1101_spectrum.cpp` | **Convert** series → `Token::Chart` + listener. |
| CC1101 Bruteforce | `features/cc1101/cc1101_bruteforce.cpp` | Inherit (pad-only locals). |
| CC1101 Jammer | `features/cc1101/cc1101_jammer.cpp` | **Convert** banner + stop btn → `Token::Danger` + listener. |
| CC1101 KeeLoq | `features/cc1101/cc1101_keeloq.cpp` | **Convert** replay btn + attack banner → `Token::Danger` + listener. |

Task 7 greps the tree so no page is missed if a new `lv_palette_` / `lv_obj_set_style_bg_color` / `lv_obj_set_style_text_color` appeared after this inventory.

---

### Task 1: Theme tables + host-native tests

**Files:**
- Create: `firmware/tab5/src/ui/theme_tables.h`
- Create: `firmware/tab5/src/ui/theme_tables.cpp`
- Create: `firmware/tab5/test/test_theme/test_theme.cpp`
- Modify: `firmware/tab5/platformio.ini` — add `+<ui/theme_tables.cpp>` to `[env:native]` `build_src_filter`

**Interfaces:**
- Produces:
  - `enum class ThemeId : uint8_t { ModernLight = 0, ModernDark = 1, ConsoleGreen = 2, ConsoleRed = 3, Custom = 255 };`
  - `enum class FontId : uint8_t { Montserrat14 = 0, Unscii16 = 1 };`
  - `struct ThemeDesc { uint32_t bg, surface, text, muted, accent, danger, chart; FontId font; int32_t button_radius; int32_t button_border_width; bool button_shadow; bool button_grow; };`
  - `const ThemeDesc *theme_table_lookup(ThemeId id);` — baked rows only; `Custom` and unknown → `nullptr`
  - `ThemeId theme_sanitize_id(uint8_t raw);` — 0–3 unchanged, 255 stays `Custom`, anything else → `ModernLight`

- [ ] **Step 1: Write the failing tests**

Create `firmware/tab5/test/test_theme/test_theme.cpp`:

```cpp
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
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(FontId::Unscii16),
                      static_cast<uint8_t>(theme_table_lookup(ThemeId::ConsoleGreen)->font));
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
```

- [ ] **Step 2: Run tests — expect compile fail**

Run from `firmware/tab5`:

```bash
pio test -e native --filter test_theme
```

Expected: FAIL — `ui/theme_tables.h` not found (and `theme_tables.cpp` not in `build_src_filter` yet).

- [ ] **Step 3: Implement tables + native filter**

`firmware/tab5/src/ui/theme_tables.h`:

```cpp
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
```

`firmware/tab5/src/ui/theme_tables.cpp` — four rows, exact values:

| Field | ModernLight | ModernDark | ConsoleGreen | ConsoleRed |
|---|---|---|---|---|
| bg | `0xEAEEF3` | `0x15171A` | `0x000000` | `0x000000` |
| surface | `0xFFFFFF` | `0x2A2D32` | `0x001100` | `0x110000` |
| text | `0x3B3E42` | `0xEFEFEF` | `0x00FF00` | `0xFF3333` |
| muted | `0x6B6E73` | `0xA0A3A8` | `0x00AA00` | `0xAA0000` |
| accent | `0x2196F3` | `0x2196F3` | `0x00FF00` | `0xFF0000` |
| danger | `0xF44336` | `0xF44336` | `0xFF3333` | `0xFFCC00` |
| chart | `0x2196F3` | `0x2196F3` | `0x00FF00` | `0xFF0000` |
| font | Montserrat14 | Montserrat14 | Unscii16 | Unscii16 |
| radius / border / shadow / grow | 8 / 0 / true / true | same | 0 / 2 / false / false | same as green |

```cpp
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
```

In `platformio.ini` `[env:native]` `build_src_filter`, append ` +<ui/theme_tables.cpp>`.

- [ ] **Step 4: Run tests — expect pass**

```bash
pio test -e native --filter test_theme
```

Expected: all 5 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/ui/theme_tables.h firmware/tab5/src/ui/theme_tables.cpp \
        firmware/tab5/test/test_theme/test_theme.cpp firmware/tab5/platformio.ini
git commit -m "$(cat <<'EOF'
Add host-testable Tab5 theme tables.

Four baked ThemeDesc rows plus id sanitizing, covered by native Unity tests.
EOF
)"
```

---

### Task 2: Device Theme module (NVS + LVGL apply_cb + listeners)

**Files:**
- Create: `firmware/tab5/src/ui/theme.h`
- Create: `firmware/tab5/src/ui/theme.cpp`
- Modify: `firmware/tab5/include/lv_conf.h` — `#define LV_FONT_UNSCII_16 1`

**Interfaces:**
- Consumes: `theme_table_lookup`, `theme_sanitize_id`, `ThemeDesc`
- Produces:
  - `enum class Token { Bg, Surface, Text, Muted, Accent, Danger, Chart };`
  - `void Theme::init();`
  - `ThemeId Theme::id();`
  - `void Theme::set(ThemeId id);`
  - `lv_color_t Theme::color(Token t);`
  - `const lv_font_t *Theme::font();`
  - `const ThemeDesc &Theme::desc();`
  - `void Theme::add_listener(void (*fn)());`
  - `void Theme::remove_listener(void (*fn)());`
  - `void Theme::apply_tree(lv_obj_t *root);` — recursive `lv_theme_apply` (Task 3 calls this from `ScreenStack::for_each`)

- [ ] **Step 1: Enable UNSCII**

In `firmware/tab5/include/lv_conf.h` change `#define LV_FONT_UNSCII_16 0` to `1`.

- [ ] **Step 2: Write `theme.h`**

```cpp
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
```

- [ ] **Step 3: Write `theme.cpp`**

Requirements (implement exactly; do not skip):

1. Namespace `quarky-ui`, key `theme_id` (`uint8`), reserved key `theme_custom` (read-only for `Custom`).
2. `init()`: `Preferences` read-only begin; `theme_sanitize_id(getUChar("theme_id", 0))`. If result is `Custom`, try `getBytes("theme_custom", ...)`. If blob missing or `getBytes` length != `sizeof` of a persistable color/button subset, log `quarky-tab5: [theme] custom blob missing — falling back to Modern Light`, set session id to `ModernLight`, **do not write NVS**. Else load baked row. Then `install_theme()`.
3. `set(id)`:
   - If `id == Custom` and blob missing: same fallback as init; **do not persist Custom**. Apply Light in RAM. Return.
   - Else `Preferences` write `theme_id` first (`putUChar`). If write fails, `Serial.printf("quarky-tab5: [theme] persist failed\n")` and still apply in RAM.
   - Swap active desc, `install_theme()`, walk live screens (Task 3 will fill the walk; for this task call `apply_tree` on `lv_display_get_screen_active(lv_display_get_default())` only — Task 3 expands it), then invoke listeners.
4. `install_theme()`: fill a static `lv_theme_t` (`apply_cb`, `font_normal`/`font_small`/`font_large` = mapped font, `color_primary` = accent). `lv_display_set_theme(lv_display_get_default(), &s_theme)`.
5. `apply_cb` styles by `lv_obj_check_type`:
   - `lv_obj_class` (screens/panels): `bg` = `desc.bg` (or `surface` if the object is not a screen — `lv_obj_get_parent(obj) != nullptr` and not transparent-by-caller). Set text font. Skip objects with `LV_OPA_TRANSP` bg already (scaffold content).
   - `lv_button_class`: modern — `bg` = accent, radius from desc, border 0, shadow if `button_shadow`; console — `bg` = bg, border width 2, border color = accent, radius 0, no shadow. Pressed: slightly lighten/darken. Grow: only if `button_grow` (do not add a transform if false — transforms allocate layer buffers).
   - `lv_label_class`: text color = `text`, font = mapped font. Do not overwrite a label that already has a local text color if you cannot tell — listeners own semantic labels.
   - `lv_textarea_class`, `lv_list_class`, `lv_keyboard_class`: bg `surface`, text `text`, font mapped, radius/border from button look.
   - `lv_chart_class`: bg `surface`, series colors are listener-owned.
6. Font map: `FontId::Montserrat14` → `&lv_font_montserrat_14`; `Unscii16` → `&lv_font_unscii_16`.
7. `color(Token)`: `lv_color_hex` of the matching `ThemeDesc` field.
8. Listeners: fixed array of 8 `void (*)()`. `add_listener`: skip duplicates; if full, log `quarky-tab5: [theme] listener overflow` and drop. `remove_listener`: compact the hole.
9. `apply_tree(root)`: if `root == nullptr` return; `lv_theme_apply(root)`; recurse `lv_obj_get_child`.

Include `<Preferences.h>`, `<Arduino.h>`, `"theme.h"`.

- [ ] **Step 4: Compile firmware**

```bash
cd firmware/tab5 && pio run -e tab5
```

Expected: SUCCESS. `theme.cpp` is compiled because `src/` is in the firmware build even before `main.cpp` includes it — if the linker drops unused objects that is fine; Task 4 will reference `Theme::init`.

If the unused `.cpp` is dropped and you need a compile check, `#include "ui/theme.h"` from a throwaway in `main.cpp` is deferred to Task 4.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/ui/theme.h firmware/tab5/src/ui/theme.cpp firmware/tab5/include/lv_conf.h
git commit -m "$(cat <<'EOF'
Add Tab5 Theme module with LVGL apply_cb and NVS.

Live install of baked themes plus a reserved Custom blob read; UNSCII_16 enabled.
EOF
)"
```

---

### Task 3: `ScreenStack::for_each` + full live walk

**Files:**
- Modify: `firmware/tab5/src/ui/screen_stack.h`
- Modify: `firmware/tab5/src/ui/screen_stack.cpp`
- Modify: `firmware/tab5/src/ui/theme.cpp` — `set()` walks every stacked screen

**Interfaces:**
- Produces: `static void ScreenStack::for_each(void (*fn)(lv_obj_t *));` visits `stack_[0..depth_)` in order.

- [ ] **Step 1: Add `for_each`**

In `screen_stack.h` public section, after `pop()`:

```cpp
    static void for_each(void (*fn)(lv_obj_t *));
```

In `screen_stack.cpp`:

```cpp
void ScreenStack::for_each(void (*fn)(lv_obj_t *)) {
    if (fn == nullptr) return;
    for (int i = 0; i < depth_; i++) {
        if (stack_[i]) fn(stack_[i]);
    }
}
```

- [ ] **Step 2: Walk the stack from `Theme::set`**

Replace the Task 2 single-active-screen walk with:

```cpp
ScreenStack::for_each([](lv_obj_t *screen) { Theme::apply_tree(screen); });
```

`theme.cpp` includes `"screen_stack.h"`. After `Shell::build` the root is on the stack, so launcher + status bar restyle too.

- [ ] **Step 3: Commit**

```bash
git add firmware/tab5/src/ui/screen_stack.h firmware/tab5/src/ui/screen_stack.cpp firmware/tab5/src/ui/theme.cpp
git commit -m "$(cat <<'EOF'
Walk ScreenStack on theme change so buried pages restyle.

for_each visits every live screen; apply_tree re-runs the LVGL theme on each.
EOF
)"
```

---

### Task 4: Boot wiring

**Files:**
- Modify: `firmware/tab5/src/main.cpp` — after `lvgl_port_init(display, touch);` and **before** any `register_module` / `Shell::build` that creates widgets. Widgets are created in `Shell::build`, so `Theme::init()` must run after `lvgl_port_init` and before `Shell::build`. Register_module calls do not create widgets; placing init immediately after `lvgl_port_init` is correct.

```cpp
#include "ui/theme.h"
// ...
lvgl_port_init(display, touch);
Theme::init();
```

- [ ] **Step 1: Add the include and `Theme::init()` call** as above.

- [ ] **Step 2: Build**

```bash
cd firmware/tab5 && pio run -e tab5
```

Expected: SUCCESS, image still links with `lv_font_unscii_16`.

- [ ] **Step 3: Commit**

```bash
git add firmware/tab5/src/main.cpp
git commit -m "$(cat <<'EOF'
Install the saved Tab5 theme before the shell is built.

Theme::init loads NVS and sets the LVGL display theme on boot.
EOF
)"
```

---

### Task 5: Settings hub + Theme picker + launcher tile

**Files:**
- Create: `firmware/tab5/src/ui/settings_screen.h`
- Create: `firmware/tab5/src/ui/settings_screen.cpp`
- Modify: `firmware/tab5/src/ui/shell.cpp` — Settings tile, always visible

**Interfaces:**
- Produces: `lv_obj_t *build_settings_hub();` and `lv_obj_t *build_theme_picker();`

- [ ] **Step 1: Settings screens**

`settings_screen.h`:

```cpp
#pragma once
#include <lvgl.h>
lv_obj_t *build_settings_hub();
lv_obj_t *build_theme_picker();
```

`settings_screen.cpp`:

- Hub: `build_sub_screen("Settings", &content)`; one 200×100 button labeled `Theme`; click `ScreenStack::push(build_theme_picker())`.
- Picker: `build_sub_screen("Theme", &content)`. Four buttons, same 200×100 size, labels `Modern Light`, `Modern Dark`, `Console Green`, `Console Red`. Click calls `Theme::set` with the matching id. Mark the current id: `lv_obj_add_state(btn, LV_STATE_CHECKED)` on the matching tile (or a small `" (current)"` suffix on its label). After `set()`, update checked/suffix on all four — `set()` already restyles the tree.

Do not jump the launcher Settings tile straight to the picker.

- [ ] **Step 2: Launcher tile**

In `shell.cpp` `Shell::build`, after the pairing tile (keep Pair Satellite where it is), add:

```cpp
#include "settings_screen.h"
// ...
lv_obj_t *settings_tile = lv_button_create(launcher);
lv_obj_set_size(settings_tile, 200, 100);
lv_obj_t *settings_label = lv_label_create(settings_tile);
lv_label_set_text(settings_label, "Settings");
lv_obj_add_event_cb(settings_tile, [](lv_event_t *) {
    ScreenStack::push(build_settings_hub());
}, LV_EVENT_CLICKED, nullptr);
```

Do not add a `kCategoryTiles` row. Do not touch `shared/feature_contract`.

- [ ] **Step 3: Build**

```bash
cd firmware/tab5 && pio run -e tab5
```

Expected: SUCCESS.

- [ ] **Step 4: Commit**

```bash
git add firmware/tab5/src/ui/settings_screen.h firmware/tab5/src/ui/settings_screen.cpp firmware/tab5/src/ui/shell.cpp
git commit -m "$(cat <<'EOF'
Add Settings hub and Theme picker on the Tab5 launcher.

Settings is a shell tile, not a FeatureRegistry category, so later prefs have a home.
EOF
)"
```

---

### Task 6: Convert semantic-color pages

**Files:**
- Modify: `firmware/tab5/src/features/cc1101/cc1101_jammer.cpp`
- Modify: `firmware/tab5/src/features/cc1101/cc1101_keeloq.cpp`
- Modify: `firmware/tab5/src/features/cc1101/cc1101_spectrum.cpp`
- Modify: `firmware/tab5/src/features/wifi/wifi_spectrum.cpp`

**Interfaces:**
- Consumes: `Theme::color(Token)`, `Theme::add_listener`, `Theme::remove_listener`
- Chart recolor: `lv_chart_set_series_color(chart, series, Theme::color(Token::Chart))` — confirmed on this tree's LVGL (`firmware/tab5/.pio/libdeps/tab5/lvgl/src/widgets/chart/lv_chart.h`).

- [ ] **Step 1: Jammer**

`#include "../../ui/theme.h"`

Replace the two `lv_palette_main(LV_PALETTE_RED)` lines with a listener:

```cpp
static void jammer_on_theme() {
    if (s_banner) lv_obj_set_style_text_color(s_banner, Theme::color(Token::Danger), 0);
    if (s_stop_btn) lv_obj_set_style_bg_color(s_stop_btn, Theme::color(Token::Danger), 0);
}
```

In `build_screen`, after creating banner/stop: `Theme::add_listener(jammer_on_theme); jammer_on_theme();`

In the existing `LV_EVENT_DELETE` handler: `Theme::remove_listener(jammer_on_theme);` before nulling pointers.

- [ ] **Step 2: KeeLoq**

Same pattern for `s_replay_btn` bg and `s_attack_banner` text → `Token::Danger`. Listener name `keeloq_on_theme`. Register/unregister around the existing delete handler.

- [ ] **Step 3: Both spectrum charts**

After `lv_chart_add_series(...)`, store `s_series` as today, then:

```cpp
static void spectrum_on_theme() {
    if (s_chart && s_series) {
        lv_chart_set_series_color(s_chart, s_series, Theme::color(Token::Chart));
    }
}
```

Add listener after series create; remove on the existing chart `LV_EVENT_DELETE`. Use `Token::Chart` (passive RSSI), not `Danger`.

- [ ] **Step 4: Build**

```bash
cd firmware/tab5 && pio run -e tab5
```

Expected: SUCCESS, no remaining `lv_palette_main` in those four files.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/cc1101/cc1101_jammer.cpp \
        firmware/tab5/src/features/cc1101/cc1101_keeloq.cpp \
        firmware/tab5/src/features/cc1101/cc1101_spectrum.cpp \
        firmware/tab5/src/features/wifi/wifi_spectrum.cpp
git commit -m "$(cat <<'EOF'
Point jammer, KeeLoq, and spectrum accents at theme tokens.

Listeners re-apply danger/chart colors when the theme changes under an open screen.
EOF
)"
```

---

### Task 7: Whole-app leftover sweep + pairing QR note

**Files:**
- Modify: `firmware/tab5/src/ui/pairing_screen.cpp` — comment only above the canvas fill
- Grep (no other edits unless the grep finds a new palette/color site)

- [ ] **Step 1: Grep leftover palette / local color**

From repo root:

```bash
rg -n "lv_palette_|lv_obj_set_style_bg_color|lv_obj_set_style_text_color|lv_color_white|lv_color_black" firmware/tab5/src --glob '*.{h,cpp}'
```

Allowed leftovers:
- `pairing_screen.cpp` QR `lv_color_white` / `lv_color_black`
- `theme.cpp` itself
- Pad/radius/opa/border-width layout locals (no color) in scaffold, grids, `cc1101_spectrum` freq_row

If any **new** `lv_palette_` or color `set_style_*` appears in a page from the inventory, convert it the same way as Task 6 (token + listener) in this task and list the file in the commit.

- [ ] **Step 2: Pairing QR comment**

Above `lv_canvas_fill_bg(canvas, lv_color_white(), LV_OPA_COVER);`:

```cpp
    // Theme-exempt on purpose (theming spec §3.6): a themed QR can fail to
    // scan. Chrome around this canvas still follows the LVGL theme.
```

- [ ] **Step 3: Commit**

```bash
git add firmware/tab5/src/ui/pairing_screen.cpp
# plus any conversion files from Step 1
git commit -m "$(cat <<'EOF'
Document pairing QR as theme-exempt and clear leftover palette sites.

Every launcher page either inherits the LVGL theme or uses Theme tokens.
EOF
)"
```

---

### Task 8: Verify host tests, firmware link, on-device page walk

**Files:** none required unless a test or apply_cb bug shows up.

- [ ] **Step 1: Host-native (includes existing suites)**

```bash
cd firmware/tab5 && pio test -e native
```

Expected: existing suites still pass **and** `test_theme` passes.

- [ ] **Step 2: Firmware build**

```bash
cd firmware/tab5 && pio run -e tab5
```

Expected: SUCCESS.

- [ ] **Step 3: On-device Definition of Done**

Flash (`pio run -e tab5 -t upload`, sandbox disabled, re-check `/dev/cu.usbmodem*` after reset). Walk on the physical Tab5:

1. Fresh NVS (or erase `quarky-ui`): boots Modern Light — launcher matches today's chrome.
2. Settings → Theme → each of the four: picker buttons change radius/font/colors immediately.
3. Reboot after Console Green and after Modern Dark; that theme is still on the launcher.
4. With a theme other than Light active, open **one page per launcher category** and confirm chrome/buttons/labels follow it:
   - Utility: category screen (Ping tile)
   - WiFi: Scan **and** Spectrum (chart series = theme chart token)
   - BLE: Scan
   - NFC: Tag Read
   - RF433: Scan
   - IR: TV-B-Gone
   - Sub-GHz: Jammer (danger stays red on Console Green) **and** KeeLoq (amber danger on Console Red)
5. Pair Satellite: QR still black-on-white; Back/title follow the theme.
6. Keyboard Test: keyboard + textarea follow the theme.
7. File-driven page: IR Clone or NFC Tag Library — list/buttons inherit.
8. Open Jammer, switch theme from Settings (Jammer still under the stack), pop back: banner/stop used the new `Danger` token.

If a specific inherit page ignores the theme because of a local style, fix it in this task (token or remove the local color) and re-walk that page.

- [ ] **Step 4: Commit any Task 8 fixes** (skip if none)

```bash
git commit -m "$(cat <<'EOF'
Fix theme apply gaps found on the Tab5 page walk.

Leftover local styles were blocking live restyle on real hardware.
EOF
)"
```

---

## Spec coverage (self-review)

| Spec requirement | Task |
|---|---|
| Four baked themes + token table | 1 |
| Custom reserved / missing blob → Light, do not persist Custom | 2 |
| LVGL theme + button look live | 2, 3 |
| `ScreenStack::for_each` walk | 3 |
| Settings hub, not FeatureRegistry | 5 |
| NVS `quarky-ui` / `theme_id` | 2, 4 |
| Boot `Theme::init` before shell | 4 |
| Hardcoded palette conversions + listeners | 6 |
| Pairing QR exempt | 7 |
| Evil Portal HTML out of scope | inventory (no task) |
| Host-native tests | 1, 8 |
| On-device DoD + every category | 8 |
| No phase-plan/spec edits | Global Constraints |
| Every app page | inventory + Task 7 grep + Task 8 walk |
