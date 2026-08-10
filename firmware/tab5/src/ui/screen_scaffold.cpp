#include "screen_scaffold.h"
#include "screen_stack.h"

// ---------------------------------------------------------------------------
// Why sub-screens get a real top menu bar instead of a free-floating button
//
// Measured on the physical Tab5 (2026-08-10) two ways: a recursive geometry
// dump of every live LVGL object, and a readback of the DPI panel's own frame
// buffer -- i.e. the pixels actually on the glass, after LVGL rendering, after
// the PPA rotation, after the DMA2D copy.
//
//   * Keyboard test screen: Back at (10,10)-(85,47), 76x38 px, sitting ON TOP
//     of the text area at (64,20)-(1215,61). They overlap by 22 px
//     horizontally and 28 px vertically, and the frame-buffer readback shows
//     the text area's left border vanishing behind the button. That is the
//     reported "the input collides with the back button".
//   * WiFi category screen: Back at (0,0)-(79,39), jammed into the extreme
//     corner with no margin at all, one sixth the area of the 200x100 tiles
//     next to it.
//   * Pairing screen: same hand-aligned (10,10) Back button over a
//     hand-aligned QR canvas.
//
// None of this was a rendering fault. The frame buffer contains a clean,
// correctly rotated, correctly positioned, properly antialiased rounded
// button every time -- LVGL drew exactly what it was told and the panel showed
// exactly what LVGL drew. The fault is that each sub-screen placed its Back
// button by hand in absolute coordinates, over content also placed by hand in
// absolute coordinates, with nothing reconciling the two, and let the button
// fall back to LVGL's content-based auto-size.
//
// Physical size is the other half of it. This panel is 1280x720 over a 5.0"
// diagonal: ~294 px/inch, ~11.6 px/mm. LVGL's default metrics assume
// LV_DPI_DEF = 130 px/inch, so anything it auto-sizes lands at 130/294 = 44%
// of its intended physical size. 76x38 px is 6.5 x 3.3 mm -- about a third of
// the ~9 mm minimum comfortable touch target -- which is why a perfectly drawn
// button read as "tiny/sliced" and "untouchable".
//
// Hence: one shared scaffold. A flex-managed bar owns the top of the screen,
// the Back button is sized like the launcher tiles (200x100 px = 17 x 8.6 mm,
// the one part of this UI confirmed comfortable on the real device), and
// caller content lives in a separate flex-managed container underneath. Screen
// content can no longer overlap the Back button structurally, not just by
// convention.
// ---------------------------------------------------------------------------

namespace {

constexpr int32_t kMenuBarHeight = 120;
constexpr int32_t kBackButtonWidth = 200;
constexpr int32_t kBackButtonHeight = 100;
constexpr int32_t kMenuBarPad = 10;
constexpr int32_t kContentPad = 20;

} // namespace

int32_t sub_screen_menu_bar_height() { return kMenuBarHeight; }

lv_obj_t *build_sub_screen(const char *title, lv_obj_t **content_out) {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(screen, 0, LV_PART_MAIN);
    // The screen itself must not scroll: the menu bar is a fixed piece of
    // chrome, and a screen-level scroll would carry it off the top.
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *bar = lv_obj_create(screen);
    lv_obj_set_size(bar, LV_PCT(100), kMenuBarHeight);
    lv_obj_set_style_pad_all(bar, kMenuBarPad, LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);
    lv_obj_remove_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(bar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *back = lv_button_create(bar);
    lv_obj_set_size(back, kBackButtonWidth, kBackButtonHeight);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, LV_SYMBOL_LEFT "  Back");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    if (title != nullptr && title[0] != '\0') {
        lv_obj_t *title_label = lv_label_create(bar);
        lv_label_set_text(title_label, title);
        // A left margin so the title never abuts the Back button; flex owns
        // the position, so this is a gap, not an absolute coordinate.
        lv_obj_set_style_margin_left(title_label, kContentPad, LV_PART_MAIN);
    }

    lv_obj_t *content = lv_obj_create(screen);
    lv_obj_set_width(content, LV_PCT(100));
    lv_obj_set_flex_grow(content, 1); // everything the menu bar does not use
    lv_obj_set_style_pad_all(content, kContentPad, LV_PART_MAIN);
    lv_obj_set_style_radius(content, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(content, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_layout(content, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_row(content, kContentPad, LV_PART_MAIN);
    lv_obj_set_style_pad_column(content, kContentPad, LV_PART_MAIN);

    if (content_out != nullptr) {
        *content_out = content;
    }
    return screen;
}
