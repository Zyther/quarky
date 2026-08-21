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

    // Task 23: real periodic battery-percentage update, replacing the
    // permanent "Battery: --%" stub build() used to leave in place forever.
    // `ok` false (I2C failure, or the HAL never detected the INA226) shows
    // "--%" again rather than a stale or fabricated number; `percent` is
    // ignored in that case. No-ops if build() hasn't run yet. Reaches the
    // label the same way devices_panel.cpp's update() already does for the
    // link label -- by child index off status_bar_ (child 0 is the battery
    // label, child 1 is the link label; see build() below) -- rather than
    // adding a second stored lv_obj_t* static, so both status-bar labels are
    // retrieved the same, already-established way.
    static void update_battery_label(bool ok, int percent);

private:
    static lv_obj_t *status_bar_;
};
