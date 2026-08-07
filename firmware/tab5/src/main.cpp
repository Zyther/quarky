#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_gt911.h"
#include "ui/lvgl_port.h"

DisplayTab5 display;
TouchGT911 touch;

void setup() {
    Serial.begin(115200);
    delay(500);
    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Touch anywhere");
    lv_obj_center(label);

    lv_obj_t *btn = lv_button_create(lv_screen_active());
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Tap me");
    lv_obj_add_event_cb(btn, [](lv_event_t *e) {
        Serial.println("quarky-tab5: button tapped");
    }, LV_EVENT_CLICKED, nullptr);

    Serial.println("quarky-tab5: lvgl ready");
}

void loop() {
    lvgl_port_tick();
    delay(5);
}
