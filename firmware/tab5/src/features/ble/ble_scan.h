#pragma once
#include <cstdint>

namespace BleScanFeature {
void register_module();
void start();
void poll();

// Returns the address of the first device in the most recent scan's results,
// or nullptr if none. Added for Task 1 of the second Phase 2 plan's
// central-connect spike, which needs a real target to connect to.
const uint8_t *first_device_addr();

// The matching peer address type for first_device_addr() (BLE_ADDR_PUBLIC /
// BLE_ADDR_RANDOM / ...). Meaningless unless first_device_addr() returned
// non-null. Separate from the address itself only to keep
// first_device_addr()'s briefed signature intact; slot 0 of the device list
// is never reordered once populated (see add_or_update()), so reading the two
// back to back is consistent.
uint8_t first_device_addr_type();
}
