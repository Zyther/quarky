#include "keyboard_test_screen.h"
#include "screen_stack.h"
#include <Arduino.h>

lv_obj_t *build_keyboard_test_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);

    lv_obj_t *ta = lv_textarea_create(screen);
    lv_obj_set_size(ta, LV_PCT(90), 60);
    lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 20);
    lv_textarea_set_one_line(ta, true);

    lv_obj_t *kb = lv_keyboard_create(screen);
    lv_keyboard_set_textarea(kb, ta);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(ta, [](lv_event_t *e) {
        lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
        Serial.printf("quarky-tab5: textarea now '%s'\n", lv_textarea_get_text(ta));
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    return screen;
}
