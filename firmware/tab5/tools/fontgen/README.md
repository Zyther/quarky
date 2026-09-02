# Console font generation

Regenerates `src/ui/fonts/lv_font_quarky_mono_12.c` — the console themes' 12px
monospace face with LVGL's symbol glyphs merged in (so the Back chevron and the
on-screen keyboard control keys render instead of showing missing-glyph
rectangles, which is what happened with UNSCII).

## Sources (both freely redistributable)

- `JetBrainsMono-Regular.ttf` — JetBrains Mono, SIL OFL 1.1
  (https://github.com/JetBrains/JetBrainsMono). Body/ASCII glyphs.
- `FontAwesome5-Solid+Brands+Regular.woff` — Font Awesome Free 5, SIL OFL 1.1,
  copied verbatim from LVGL's `scripts/built_in_font/`. Provides the exact same
  symbol set LVGL bakes into its Montserrat fonts.

## Regenerate

Requires Node (`npx`). The symbol codepoint list is LVGL's own, taken from
`.pio/libdeps/tab5/lvgl/scripts/built_in_font/built_in_font_gen.py`.

```bash
cd firmware/tab5/tools/fontgen
SYMS="61441,61448,61451,61452,61452,61453,61457,61459,61461,61465,61468,61473,61478,61479,61480,61502,61507,61512,61515,61516,61517,61521,61522,61523,61524,61543,61544,61550,61552,61553,61556,61559,61560,61561,61563,61587,61589,61636,61637,61639,61641,61664,61671,61674,61683,61724,61732,61787,61931,62016,62017,62018,62019,62020,62087,62099,62212,62189,62810,63426,63650"
npx lv_font_conv --no-compress --no-prefilter --bpp 4 --size 12 \
  --font JetBrainsMono-Regular.ttf -r 0x20-0x7F,0xB0,0x2022 \
  --font "FontAwesome5-Solid+Brands+Regular.woff" -r "$SYMS" \
  --format lvgl -o lv_font_quarky_mono_12.c --force-fast-kern-format
```

Then move `lv_font_quarky_mono_12.c` to `../../src/ui/fonts/` and restore the
file header + replace the emitted `#ifdef LV_LVGL_H_INCLUDE_SIMPLE ...` include
block with a plain `#include "lvgl.h"` (the emitted relative include does not
resolve from `src/ui/fonts/`).
