#pragma once
#include <lvgl.h>

// The standard chrome for every non-root screen: a top menu bar carrying a
// Back button (and an optional title), plus a content area beneath it.
//
// Every sub-screen must build itself through this rather than creating and
// positioning its own Back button -- see screen_scaffold.cpp for the real
// hardware measurements that made hand-positioned Back buttons unusable.
//
// Returns the screen object (hand it to ScreenStack::push). *content_out
// receives the container to parent the screen's own widgets to; it is
// flex-managed (LV_FLEX_FLOW_ROW_WRAP by default -- callers are free to change
// the flow) and occupies all the space the menu bar does not, so nothing a
// caller adds can ever land on top of the Back button.
lv_obj_t *build_sub_screen(const char *title, lv_obj_t **content_out);

// Height of the menu bar, in logical pixels. Exposed so callers that need to
// size themselves against the remaining space (e.g. a keyboard pinned to the
// bottom) can do the arithmetic without duplicating the constant.
int32_t sub_screen_menu_bar_height();
