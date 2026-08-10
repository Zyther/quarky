#include "shell.h"
#include "screen_stack.h"
#include "keyboard_test_screen.h"
#include "pairing_screen.h"
#include "../features/ping_feature.h"
#include <lvgl.h>
#include <cstring>
#include <utility>

lv_obj_t *Shell::status_bar_ = nullptr;

static const struct { Category cat; const char *label; } kCategoryTiles[] = {
    {Category::UTILITY, "Utility"},
    {Category::WIFI, "WiFi"},
    {Category::BLE, "BLE"},
};

// Registry captured by reference in the click handler's user_data (a pointer
// to it, since FeatureRegistry outlives every screen -- it's a global in
// main.cpp) so the category screen can be built lazily, on tap, rather than
// pre-building all three up front.
lv_obj_t *build_category_screen(FeatureRegistry &registry, Category cat) {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_GRID);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    registry.for_each_in_category(cat, [screen](const FeatureModule &m) {
        lv_obj_t *tile = lv_button_create(screen);
        lv_obj_set_size(tile, 200, 100);
        lv_obj_t *label = lv_label_create(tile);
        lv_label_set_text(label, m.name);

        // Generic dispatch: every registered module (this plan onward) is
        // required (Global Constraints) to have a real on_start -- store the
        // function pointer itself as the event's user_data so the click
        // handler needs no per-id branching.
        lv_obj_add_event_cb(tile, [](lv_event_t *e) {
            FeatureStartFn fn = (FeatureStartFn)lv_event_get_user_data(e);
            if (fn) fn();
        }, LV_EVENT_CLICKED, (void *)m.on_start);
    });

    return screen;
}

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

    // One tile per category that has at least one registered module --
    // empty categories are skipped so e.g. BLE doesn't show as a dead end
    // before this plan's BLE tasks land.
    for (const auto &entry : kCategoryTiles) {
        if (registry.count_in_category(entry.cat) == 0) continue;
        lv_obj_t *tile = lv_button_create(launcher);
        lv_obj_set_size(tile, 200, 100);
        lv_obj_t *label = lv_label_create(tile);
        lv_label_set_text(label, entry.label);
        lv_obj_add_event_cb(tile, [](lv_event_t *e) {
            auto *ctx = (std::pair<FeatureRegistry *, Category> *)lv_event_get_user_data(e);
            ScreenStack::push(build_category_screen(*ctx->first, ctx->second));
        }, LV_EVENT_CLICKED, new std::pair<FeatureRegistry *, Category>(&registry, entry.cat));
    }

    // Debug launcher tile for testing lv_keyboard
    lv_obj_t *kb_test_tile = lv_button_create(launcher);
    lv_obj_set_size(kb_test_tile, 200, 100);
    lv_obj_t *kb_test_label = lv_label_create(kb_test_tile);
    lv_label_set_text(kb_test_label, "[debug] Keyboard Test");
    lv_obj_add_event_cb(kb_test_tile, [](lv_event_t *e) {
        ScreenStack::push(build_keyboard_test_screen());
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *pairing_tile = lv_button_create(launcher);
    lv_obj_set_size(pairing_tile, 200, 100);
    lv_obj_t *pairing_label = lv_label_create(pairing_tile);
    lv_label_set_text(pairing_label, "Pair Satellite");
    lv_obj_add_event_cb(pairing_tile, [](lv_event_t *e) {
        ScreenStack::push(build_pairing_screen());
    }, LV_EVENT_CLICKED, nullptr);

    return root;
}
