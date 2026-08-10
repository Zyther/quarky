#include "keyboard_test_screen.h"
#include "screen_scaffold.h"
#include <Arduino.h>

lv_obj_t *build_keyboard_test_screen() {
    // Was: a bare lv_obj_create(nullptr) with the text area aligned TOP_MID at
    // y=20 and the Back button aligned TOP_LEFT at (10,10). On the real panel
    // those two overlapped -- text area (64,20)-(1215,61) against Back button
    // (10,10)-(85,47) -- which is the reported "the input collides with the
    // back button". Both are now flex-managed by the shared scaffold, so they
    // cannot occupy the same space. See screen_scaffold.cpp.
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Keyboard Test", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *ta = lv_textarea_create(content);
    lv_obj_set_width(ta, LV_PCT(100));
    lv_textarea_set_one_line(ta, true);

    // A direct child of the screen, not of `content`: the screen's flex column
    // is menu bar / content / keyboard, so the keyboard takes the bottom band
    // and `content` (flex-grow) shrinks to whatever is left. Parenting it to
    // `content` instead would make it compete with the text area for the
    // content area's space.
    lv_obj_t *kb = lv_keyboard_create(screen);
    lv_keyboard_set_textarea(kb, ta);

    lv_obj_add_event_cb(ta, [](lv_event_t *e) {
        lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
        Serial.printf("quarky-tab5: textarea now '%s'\n", lv_textarea_get_text(ta));
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    return screen;
}
