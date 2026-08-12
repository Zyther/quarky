#pragma once
#include <cstdint>

// Shared BLE device-list model, reused by the deferred second plan's BLE
// spam/finder/sniffer features (Task 7).
struct BleDeviceInfo {
    uint8_t addr[6];
    char addr_str[18];
    int8_t rssi;
    char name[32]; // empty string if no AD_TYPE_NAME field present
};

void ble_addr_to_str(const uint8_t addr[6], char out[18]);
