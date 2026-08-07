#include "screen_stack.h"

lv_obj_t *ScreenStack::stack_[ScreenStack::kMaxDepth];
int ScreenStack::depth_ = 0;

void ScreenStack::push(lv_obj_t *screen) {
    if (depth_ < kMaxDepth) {
        stack_[depth_++] = screen;
    }
    lv_screen_load(screen);
}

void ScreenStack::pop() {
    if (depth_ <= 1) return; // never pop the root shell screen
    lv_obj_t *top = stack_[--depth_];
    lv_screen_load(stack_[depth_ - 1]);
    lv_obj_delete(top);
}
