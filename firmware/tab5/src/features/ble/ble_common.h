#pragma once
#include <cstdint>

// Shared BLE device-list model, reused by the deferred second plan's BLE
// spam/finder/sniffer features (Task 7).
struct BleDeviceInfo {
    uint8_t addr[6];
    // The peer's advertised address type (NimBLE BLE_ADDR_PUBLIC /
    // BLE_ADDR_RANDOM / BLE_ADDR_PUBLIC_ID / BLE_ADDR_RANDOM_ID), straight
    // from event->disc.addr.type. Recorded because ble_gap_connect() needs the
    // real peer type -- assuming BLE_ADDR_PUBLIC fails against the random
    // static / resolvable-private addresses most modern peripherals actually
    // advertise with. Added for the second Phase 2 plan's Task 1
    // central-connect spike.
    uint8_t addr_type;
    char addr_str[18];
    int8_t rssi;
    char name[32]; // empty string if no AD_TYPE_NAME field present
};

void ble_addr_to_str(const uint8_t addr[6], char out[18]);
