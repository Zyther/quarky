#include "ble_common.h"
#include "../../hal/c2link_ble.h" // c2link_ble_host_synced()
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
