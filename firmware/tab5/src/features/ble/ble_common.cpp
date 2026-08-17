#include "ble_common.h"
#include "../../hal/c2link_ble.h" // c2link_ble_host_synced()
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <cstdio>

void ble_addr_to_str(const uint8_t addr[6], char out[18]) {
    // NimBLE addresses are little-endian on the wire; print most-significant
    // byte first to match how BLE addresses are conventionally displayed
    // (matches nRF Connect / other standard tools), i.e. reversed from the
    // raw ble_addr_t byte order.
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

// Kept as the exact wording the eight files that already had their own inline
// copy of this guard used, so the migration to the shared helper changed no
// on-screen text anywhere (finding C1).
const char *const kBleHostNotReadyMsg = "BLE host not ready yet, try again shortly";

const char *const kBleNoScannedTargetMsg =
    "No scanned device available -- run BLE Scan first, then reopen this screen.";

bool ble_require_host_ready(lv_obj_t *status_label, const char *not_ready_msg) {
    if (c2link_ble_host_synced()) return true;
    if (status_label != nullptr) lv_label_set_text(status_label, not_ready_msg);
    return false;
}

bool ble_require_host_ready_list(lv_obj_t *list, const char *not_ready_msg) {
    if (c2link_ble_host_synced()) return true;
    if (list != nullptr) lv_list_add_text(list, not_ready_msg);
    return false;
}

void ble_push_message_screen(const char *title, const char *message) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(title, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_t *label = lv_label_create(content);
    // Wrap rather than run off the edge: these messages are full sentences,
    // and the scaffold's content area is narrower than the panel.
    lv_obj_set_width(label, LV_PCT(100));
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label, message);
    ScreenStack::push(screen);
}
