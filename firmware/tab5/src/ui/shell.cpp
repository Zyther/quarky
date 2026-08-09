#include "shell.h"
#include "screen_stack.h"
#include "keyboard_test_screen.h"
#include "pairing_screen.h"
#include "../features/ping_feature.h"
#include <lvgl.h>
#include <cstring>

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

        // Task 20: only "ping" exists today, so this is the sole tile wired
        // to a real click handler; a Phase 2+ feature would need its own
        // dispatch (e.g. a table keyed by m.id) rather than this hardcoded
        // check, once more than one UTILITY module is registered.
        if (strcmp(m.id, "ping") == 0) {
            lv_obj_add_event_cb(tile, [](lv_event_t *e) {
                PingFeature::send_ping();
            }, LV_EVENT_CLICKED, nullptr);
        }
    });

    // Debug launcher tile for testing lv_keyboard
    lv_obj_t *kb_test_tile = lv_button_create(launcher);
    lv_obj_set_size(kb_test_tile, 200, 100);
    lv_obj_t *kb_test_label = lv_label_create(kb_test_tile);
    lv_label_set_text(kb_test_label, "[debug] Keyboard Test");
    lv_obj_add_event_cb(kb_test_tile, [](lv_event_t *e) {
        ScreenStack::push(build_keyboard_test_screen());
    }, LV_EVENT_CLICKED, nullptr);

    // Launcher tile for Task 12's pairing screen: generates/loads the PSK
    // that authenticates both C2 transports (WiFi Task 11, BLE Task 13) and
    // displays it as a QR code + hex string for Cardputer-ADV to be paired
    // with.
    lv_obj_t *pairing_tile = lv_button_create(launcher);
    lv_obj_set_size(pairing_tile, 200, 100);
    lv_obj_t *pairing_label = lv_label_create(pairing_tile);
    lv_label_set_text(pairing_label, "Pair Satellite");
    lv_obj_add_event_cb(pairing_tile, [](lv_event_t *e) {
        ScreenStack::push(build_pairing_screen());
    }, LV_EVENT_CLICKED, nullptr);

    return root;
}
