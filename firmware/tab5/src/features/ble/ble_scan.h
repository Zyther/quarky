#pragma once
#include <cstdint>

namespace BleScanFeature {
void register_module();
void start();
void poll();

// Returns the address of the first device in the most recent scan's results,
// or nullptr if none. Added for Task 1 of the second Phase 2 plan's
// central-connect spike, which needs a real target to connect to.
//
// SOLE REMAINING CONSUMER (2026-08-17): main.cpp's QUARKY_SERIAL_DEBUG 'c'
// trigger for BleCentralSpike::run(). Four real features (ble_flood.cpp,
// ble_fastpair_exploit.cpp, ble_hfp_exploit.cpp, ble_whisperpair.cpp) used to
// call this too, and that was the bug: "scan slot 0 of a prior, separate BLE
// Scan screen run" is not a target the user chose. They now scan for
// themselves and let the user tap a target (see ble_target_picker.h). These
// two accessors survive only because the spike is a deliberately
// serial-only, one-shot experiment with no screen and therefore no picker to
// put in front of it -- do NOT reach for them from a new user-facing feature.
const uint8_t *first_device_addr();

// The matching peer address type for first_device_addr() (BLE_ADDR_PUBLIC /
// BLE_ADDR_RANDOM / ...). Meaningless unless first_device_addr() returned
// non-null. Separate from the address itself only to keep
// first_device_addr()'s briefed signature intact; slot 0 of the device list
// is never reordered once populated (see add_or_update()), so reading the two
// back to back is consistent.
uint8_t first_device_addr_type();
}
