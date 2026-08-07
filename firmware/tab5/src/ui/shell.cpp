#include "shell.h"
#include "screen_stack.h"
#include <lvgl.h>

lv_obj_t *Shell::status_bar_ = nullptr;

lv_obj_t *Shell::build(FeatureRegistry &registry) {
    lv_obj_t *root = lv_obj_create(nullptr);
    lv_obj_set_layout(root, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);

    status_bar_ = lv_obj_create(root);
    lv_obj_set_size(status_bar_, LV_PCT(100), 40);
    lv_obj_t *battery_label = lv_label_create(status_bar_);
    lv_label_set_text(battery_label, "Battery: --%");
    lv_obj_t *link_label = lv_label_create(status_bar_);
    lv_label_set_text(link_label, "Cardputer-ADV: disconnected");
    lv_obj_align(link_label, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *launcher = lv_obj_create(root);
    lv_obj_set_size(launcher, LV_PCT(100), LV_PCT(100));
    lv_obj_set_layout(launcher, LV_LAYOUT_GRID);

    // Populate one tile per registered feature module. In this phase only
    // the Task 15 "ping" module exists, so the grid will show a single tile.
    registry.for_each_in_category(Category::UTILITY, [launcher](const FeatureModule &m) {
        lv_obj_t *tile = lv_button_create(launcher);
        lv_obj_set_size(tile, 200, 100);
        lv_obj_t *label = lv_label_create(tile);
        lv_label_set_text(label, m.name);
    });

    return root;
}
