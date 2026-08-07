#pragma once
// NOTE: the brief's sample only included <feature_registry.h> here, but this
// header declares lv_obj_t-typed API (build()'s return type, status_bar_'s
// storage type) -- lv_obj_t comes from LVGL, not from feature_registry.h.
// Any translation unit that includes "shell.h" without having already
// included <lvgl.h> itself (e.g. shell.cpp, if it included this header
// first) would fail with "'lv_obj_t' was not declared in this scope".
// Including <lvgl.h> directly makes this header self-contained regardless
// of include order in consumers, matching Task 6's lvgl_port.h precedent.
#include <lvgl.h>
#include <feature_registry.h>

class Shell {
public:
    static lv_obj_t *build(FeatureRegistry &registry);
    static lv_obj_t *status_bar() { return status_bar_; }

private:
    static lv_obj_t *status_bar_;
};
