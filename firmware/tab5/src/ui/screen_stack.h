#pragma once
#include <lvgl.h>

class ScreenStack {
public:
    static void push(lv_obj_t *screen);
    static void pop();

private:
    static constexpr int kMaxDepth = 8;
    static lv_obj_t *stack_[kMaxDepth];
    static int depth_;
};
