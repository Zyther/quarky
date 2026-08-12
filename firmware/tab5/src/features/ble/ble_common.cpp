#include "ble_common.h"
#include <cstdio>

void ble_addr_to_str(const uint8_t addr[6], char out[18]) {
    // NimBLE addresses are little-endian on the wire; print most-significant
    // byte first to match how BLE addresses are conventionally displayed
    // (matches nRF Connect / other standard tools), i.e. reversed from the
    // raw ble_addr_t byte order.
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}
