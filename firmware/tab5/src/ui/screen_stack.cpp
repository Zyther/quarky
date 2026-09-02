#include "screen_stack.h"
#include <Arduino.h>

lv_obj_t *ScreenStack::stack_[ScreenStack::kMaxDepth];
int ScreenStack::depth_ = 0;

void ScreenStack::push(lv_obj_t *screen) {
    if (depth_ >= kMaxDepth) {
        // Real fix (see kMaxDepth's own comment for the bug this closes):
        // refuse rather than display a screen we can't track. The caller
        // already fully built `screen` before calling push() (every real
        // caller in this codebase does), so it must be deleted here, not
        // just dropped, or it leaks.
        Serial.println("quarky-tab5: [screen-stack] push() REFUSED -- max "
                       "depth reached, staying on current screen");
        lv_obj_delete(screen);
        return;
    }
    stack_[depth_++] = screen;
    lv_screen_load(screen);
}

void ScreenStack::pop() {
    if (depth_ <= 1) return; // never pop the root shell screen
    lv_obj_t *top = stack_[--depth_];
    lv_screen_load(stack_[depth_ - 1]);
    lv_obj_delete(top);
}

void ScreenStack::for_each(void (*fn)(lv_obj_t *)) {
    if (fn == nullptr) return;
    for (int i = 0; i < depth_; i++) {
        if (stack_[i]) fn(stack_[i]);
    }
}
