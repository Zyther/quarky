#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_gt911.h"
#include "ui/lvgl_port.h"
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include <feature_registry.h>

DisplayTab5 display;
TouchGT911 touch;
FeatureRegistry g_registry; // populated further in Task 15

void setup() {
    Serial.begin(115200);
    delay(500);
    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    lv_obj_t *root = Shell::build(g_registry);
    ScreenStack::push(root);

    Serial.println("quarky-tab5: lvgl ready");
}

void loop() {
    lvgl_port_tick();
    delay(5);
}
