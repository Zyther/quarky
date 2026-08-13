#include "ble_central.h"
#include <Arduino.h>
#include <host/ble_hs.h>

namespace BleCentral {

int connect(const ble_addr_t &target, int32_t timeout_ms, ble_gap_event_fn *event_cb, void *cb_arg) {
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &target, timeout_ms, nullptr, event_cb, cb_arg);
    Serial.printf("quarky-tab5: [ble-central] ble_gap_connect rc=%d\n", rc);
    return rc;
}

int disconnect(uint16_t conn_handle) {
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    Serial.printf("quarky-tab5: [ble-central] ble_gap_terminate rc=%d\n", rc);
    return rc;
}

} // namespace BleCentral
