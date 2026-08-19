#include "shell.h"
#include "screen_stack.h"
#include "screen_scaffold.h"
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
    {Category::NFC, "NFC"},
    {Category::RF433, "RF433"},
    {Category::IR, "IR"},
    // Sub-GHz / NRF24 / LoRa tiles are deliberately absent: those categories
    // belong to Phases 5 and 6 (Cardputer-ADV CC1101/nRF24, GNSS/SX1262) and
    // no module registers under them yet. They were added here by Phase 3
    // Task 4 and removed in its fix round as out-of-scope. Add each one back
    // in the phase that ships its first module, so the launcher never shows a
    // category the firmware cannot do anything with.
};

// The category screen's menu-bar title. Same table as the launcher tiles, so
// tapping "WiFi" lands on a screen that says "WiFi" -- looked up rather than
// duplicated so the two can never drift apart.
static const char *category_title(Category cat) {
    for (const auto &entry : kCategoryTiles) {
        if (entry.cat == cat) return entry.label;
    }
    return "";
}

// Registry captured by reference in the click handler's user_data (a pointer
// to it, since FeatureRegistry outlives every screen -- it's a global in
// main.cpp) so the category screen can be built lazily, on tap, rather than
// pre-building all three up front.
lv_obj_t *build_category_screen(FeatureRegistry &registry, Category cat) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(category_title(cat), &content);

    registry.for_each_in_category(cat, [content](const FeatureModule &m) {
        lv_obj_t *tile = lv_button_create(content);
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
    // Was LV_LAYOUT_GRID. LVGL's grid layout does nothing at all without a
    // track descriptor: lv_grid.c's calc_rows() logs "No row descriptor found
    // even on the parent" and returns LV_RESULT_INVALID, so grid_update()
    // bails before repositioning anything -- and lv_obj_refr_pos() also skips
    // children whose parent has any layout set, so they are left stacked on
    // top of one another at the container origin. Neither
    // lv_obj_set_grid_dsc_array() nor lv_obj_set_grid_cell() was ever called
    // here. FLEX with wrapping needs no track/cell bookkeeping for a
    // variable-length tile list, so use that rather than adding the missing
    // grid plumbing. Do not switch back to LV_LAYOUT_GRID without also adding
    // a real dsc array plus explicit per-child cell assignments.
    lv_obj_set_layout(launcher, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(launcher, LV_FLEX_FLOW_ROW_WRAP);

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
