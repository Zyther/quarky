# Tab5 Theming — Design

**Status:** Draft for review
**Date:** 2026-09-01
**Depends on:** Phase 1 Foundation — LVGL shell, `ScreenStack`, `build_sub_screen`, `Preferences`/NVS (`PskStore` as the persist precedent).
**Scope:** A Tab5-only theming system: four built-in themes (colors, font, button look), a Settings launcher category to pick one, live apply including already-visible widgets, NVS persistence, and a reserved Custom slot so a later editor can reuse the same token struct. **Does not** renumber, edit, or block any existing phase spec or plan.

## 1. Why This Exists

The Tab5 UI is LVGL 9's default light theme (`LV_THEME_DEFAULT_DARK 0` in `include/lv_conf.h`), Montserrat 14 only, with almost no per-screen styling. The few exceptions hardcode `lv_palette_main(LV_PALETTE_RED)` / `LV_PALETTE_BLUE` (CC1101 jammer/KeeLoq banners and buttons, WiFi/CC1101 spectrum series). There is no user-facing way to change look, and no token layer for a future custom theme.

This is a UI add-on, not a numbered program phase. Cardputer-ADV's display is still a HAL stub (Phase 1 deferred work); theming it is out of scope.

## 2. What the User Gets

Four named themes, picked from **Settings → Theme**:

| Id | Name | Colors | Font | Buttons |
|---|---|---|---|---|
| `ModernLight` (0) | Modern Light | Today's LVGL default light palette. This *is* the existing look, not a redesign. | Montserrat 14 (`LV_FONT_DEFAULT`) | Rounded, filled, grow-on-press (today's default) |
| `ModernDark` (1) | Modern Dark | LVGL default *dark* counterpart of the same theme (inverted chrome, same accent family). | Montserrat 14 | Same modern button look |
| `ConsoleGreen` (2) | Console Green | Black background, green text/accent/surface. | `lv_font_unscii_16` | TUI: radius 0, 2px green border, no shadow, no grow |
| `ConsoleRed` (3) | Console Red | Black background, red text/accent/surface. | `lv_font_unscii_16` | TUI: radius 0, 2px red border, no shadow, no grow |

`Custom` (255) is reserved. This spec does not ship an editor or write a custom blob. `Theme::set(Custom)` is valid API: if a blob exists it applies; if not, fall back to Modern Light and stay on that fallback id for the session (do not persist `Custom` with an empty blob).

Default on first boot (no NVS key): **Modern Light**.

Enable `LV_FONT_UNSCII_16` in `lv_conf.h`. Do not enable other Montserrat sizes. Do not add a third-party font.

## 3. Architecture

### 3.1 Token table

`FontId` is `Montserrat14` or `Unscii16`. One `ThemeDesc` is the whole look:

```
struct ThemeDesc {
    uint32_t bg;        // screen / root
    uint32_t surface;   // status bar, menu bar, cards
    uint32_t text;
    uint32_t muted;     // secondary labels
    uint32_t accent;    // default button fill (modern) / border (console)
    uint32_t danger;    // attack/live/stop — always distinct from chrome
    uint32_t chart;     // spectrum series
    FontId font;        // Montserrat14 | Unscii16 — theme.cpp maps to lv_font_t *
    int32_t button_radius;
    int32_t button_border_width;
    bool button_shadow;
    bool button_grow;
};
```

Colors are RGB888 (`0xRRGGBB`). Device-side `Theme::color(Token)` returns `lv_color_hex(...)`.

**Semantic tokens stay semantic.** Chrome follows the theme; meaning colors do not collapse into the accent:

| Theme | `danger` | Why |
|---|---|---|
| Modern Light / Dark | LVGL red (`0xF44336`) | Same as today's hardcoded `LV_PALETTE_RED` |
| Console Green | `0xFF3333` | Must not be green-on-green |
| Console Red | `0xFFCC00` (amber) | Must not be red-on-red |

`chart` is the theme accent on console themes and LVGL blue (`0x2196F3`) on modern themes (today's WiFi spectrum). KeeLoq/jammer charts that used red become `Token::Chart` or `Token::Danger` by meaning: a live-attack series is `Danger`; a passive RSSI series is `Chart`.

### 3.2 Modules

```
firmware/tab5/src/ui/
├── theme.h / theme.cpp          # public API + LVGL theme apply_cb + NVS + notify
├── theme_tables.cpp             # baked ThemeDesc rows; no Arduino, no LVGL
├── theme_tables.h
├── settings_screen.cpp/.h       # Settings hub + Theme picker
├── shell.cpp                    # new Settings launcher tile
└── screen_stack.h/.cpp          # add for_each() so apply can walk live screens
```

Host-native tests compile `theme_tables.cpp` only (same `[env:native]` `build_src_filter` pattern as `ir_nec_encode.cpp`). `theme.cpp` includes LVGL/`Preferences` and stays off the native env.

**Public API (`theme.h`):**

- `void Theme::init()` — load NVS, resolve desc, install LVGL theme. Call after `lvgl_port_init`, before `Shell::build`.
- `ThemeId Theme::id()`
- `void Theme::set(ThemeId)` — persist, swap desc, reinstall theme, walk live tree, notify listeners.
- `lv_color_t Theme::color(Token)`
- `const lv_font_t *Theme::font()`
- `const ThemeDesc &Theme::desc()`
- `void Theme::add_listener(void (*fn)())` / `remove_listener(...)` — for screens that stamped semantic colors at create time.

`Theme` is the only code that reads or writes the theme NVS keys.

### 3.3 LVGL theme shim

One `lv_theme_t` installed on the display via `lv_display_set_theme`. Its `apply_cb` styles, from the active `ThemeDesc`:

- screens / generic `lv_obj` (bg, text font)
- `lv_button` (bg or border, radius, shadow, grow)
- `lv_label` (text color + font)
- `lv_textarea`, `lv_list`, `lv_keyboard` (chrome + font)

Modern Light/Dark must match LVGL's default light/dark palettes, not a new "designer" modern. Implementation may wrap `lv_theme_default_init(..., dark)` for the two modern ids and only override tokens we own (`danger` is unused by the default theme). Console ids use this project's `apply_cb` fully.

**Live apply includes button design.** `Theme::set()`:

1. Write `theme_id` to NVS first (crash mid-apply still boots into the new choice).
2. Point the active desc at the baked table (or Custom blob).
3. Reinstall the LVGL theme so new widgets and the `apply_cb` closure see the new font/radius/border.
4. Walk the shell root plus every screen `ScreenStack` still holds; recursively `lv_theme_apply` so existing buttons change shape in place.
5. Call listeners so feature-local semantic styles (jammer banner, KeeLoq replay, chart series) re-read `Theme::color(...)`.

No full `Shell::build()` rebuild. Navigation stack stays.

`ScreenStack` grows a public `for_each(void (*fn)(lv_obj_t *))` that visits `stack_[0..depth_)`. Do not invent a second screen list.

### 3.4 Settings category — not a `FeatureRegistry` Category

Do **not** add `Category::SETTINGS` to `shared/feature_contract`. Settings is not a radio/feature domain; adding it would force Cardputer-ADV and every registry test to know about UI chrome.

`shell.cpp` adds a launcher tile **"Settings"** (always visible, same 200×100 size as Utility/WiFi/…, sibling to the existing "Pair Satellite" tile). Tap opens a Settings hub (`build_sub_screen("Settings")`) with tiles. First (and, in this spec, only) tile: **Theme**. Theme opens the picker: four tiles named as in Section 2, current id visually selected. Tap calls `Theme::set(id)`; the picker and the hub under it restyle in the same apply pass.

Later prefs (out of this spec) add more tiles to the hub. Do not make the Settings launcher tile jump straight to the four-theme picker — that leaves no place for the next setting.

### 3.5 Persistence

`Preferences` namespace `quarky-ui` (separate from `PskStore`'s `quarky-c2`):

| Key | Type | This spec |
|---|---|---|
| `theme_id` | `uint8` | Written by `set()`, read by `init()` |
| `theme_custom` | blob (`ThemeDesc` color/button fields, no font pointer) | Reserved. Neither read (except the Custom fallback path) nor written here. |

Missing key, read failure, or out-of-range id (anything other than 0–3 or 255) → Modern Light. Log once (`quarky-tab5: [theme] ...`). Do not crash.

`set()` NVS write failure: still apply in RAM, log the persist failure, next boot may revert. Honest, not a retry loop.

### 3.6 Call-site conversions

Replace hardcoded palette use with tokens. Known sites at spec time:

| File | Today | Token |
|---|---|---|
| `features/cc1101/cc1101_jammer.cpp` | red banner text, red stop button | `Danger` |
| `features/cc1101/cc1101_keeloq.cpp` | red replay button, red attack banner | `Danger` |
| `features/cc1101/cc1101_spectrum.cpp` | red chart series | `Chart` |
| `features/wifi/wifi_spectrum.cpp` | blue chart series | `Chart` |

Each of those screens registers a listener (or re-applies in an existing poll) so a theme change under an already-open screen updates the accent. Unregister on screen destroy (the same `ScreenStack::pop()` delete that already tears the widgets down).

**Pairing QR is exempt.** `pairing_screen.cpp` canvas stays white background / black modules. A themed QR can fail to scan; contrast here is a function, not chrome.

**Evil Portal HTML/CSS** (`wifi_evil_portal.cpp`) is served to a client, not Tab5 chrome. Out of scope.

New feature screens written after this spec must use `Theme::color` / inherit the LVGL theme. Do not add new `lv_palette_main` call sites.

### 3.7 What this does not change

- No edits to `docs/superpowers/plans/*`, `docs/superpowers/specs/2026-08-06-*`, `docs/superpowers/specs/2026-08-09-*`, or `docs/phases/*`.
- No new phase number in `CLAUDE.md`.
- No Cardputer-ADV work.
- No pairing-screen UX debt (QR + overlapping Back) — still the Phase 1 leftover.
- No color picker, no per-widget overrides, no extra font sizes.

## 4. Data Flow

```
boot
  lvgl_port_init
  Theme::init          # NVS → desc → lv_display_set_theme
  Shell::build         # widgets inherit the theme
  ScreenStack::push(root)

Settings → Theme → tap
  Theme::set(id)
    Preferences.putUChar(theme_id)
    active desc = table[id]
    reinstall lv_theme
    ScreenStack::for_each → lv_theme_apply recursive
    notify listeners
```

`Theme::init` / `Theme::set` run on the Arduino loop task only. Not from radio, NimBLE, or GPIO ISRs.

## 5. Error Handling

- Corrupt or missing NVS → Modern Light, one log line.
- `Custom` with missing/undersized blob → Modern Light for the session; do not persist `Custom`.
- Widget class with no `apply_cb` rule → leave LVGL's previous style. No assert.
- Apply walk must not introduce the layer-buffer allocations that have locked this device (opacity/transform/mask on full-width rows). Console buttons are opaque, radius 0, no shadow — cheaper than modern, not riskier. Do not add per-object opacity to force a restyle.
- Listener list is a small fixed array. Overflow: log and drop the new registration; do not heap-grow.

## 6. Testing

**Host-native** (`firmware/tab5/test/test_theme/`, `pio test -e native`): resolve id → table row; out-of-range → Light; `Custom` without blob → Light; Console Green `danger` ≠ accent; Console Red `danger` ≠ accent; button radius 0 on console and >0 on modern. No LVGL, no NVS mock required for the table tests.

**On-device Definition of Done** (visual on the Tab5; no screenshot-diff harness):

1. Fresh flash (no `quarky-ui` key) boots Modern Light — indistinguishable from today's chrome.
2. Settings → Theme → each of the four: colors, font, and button shape change immediately on the picker itself.
3. Reboot after each of the four; that theme is still active on the launcher.
4. Open CC1101 Jammer or KeeLoq, switch theme from Settings (stack still under the picker), pop back: danger banner/button followed the new `Danger` token; on Console Green it is still red, not green.
5. Pairing QR is still black-on-white under every theme.

## 7. Risks

- **LVGL 9 live `lv_theme_apply` on an existing tree** may not restyle every local `lv_obj_set_style_*` (those have higher precedence than the theme). The conversion list in 3.6 plus listeners is the mitigation; leftover local styles are a find-and-fix during the on-device pass, not a reason to rebuild the shell.
- **UNSCII_16 metrics** are wider/shorter than Montserrat 14. Labels that barely fit today may wrap or clip on console themes. Do not shrink touch targets (Back stays 200×100). If a specific label clips, shorten the string or allow wrap on that widget — do not drop console fonts.
- **Flash cost** of `LV_FONT_UNSCII_16` is small (bitmap, ASCII). Still verify the Tab5 image still links after enabling it.

## 8. Definition of Done

1. Four baked themes selectable from Settings → Theme; live apply includes button radius/border/font.
2. Choice persists across reboot; first boot is Modern Light.
3. Hardcoded palette sites in Section 3.6 use tokens and update via listeners.
4. Pairing QR remains black-on-white.
5. Host-native table tests pass (`pio test -e native` includes `test_theme`).
6. Section 6 on-device list walked on real Tab5 hardware.
7. Custom id exists as API/NVS reserved slot; no editor UI.
8. No existing phase spec or plan file modified.
