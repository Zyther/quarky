#pragma once
#include <cstdint>
namespace BleClassify {
// Returns a short, human-readable label for what this advertisement looks
// like ("iBeacon", "AirPods", "Fast Pair", "Windows Swift Pair", or
// nullptr if nothing recognized). adv_data/adv_len are the raw
// advertisement bytes -- same fields event->disc.data/length_data already
// carry in ble_scan.cpp's gap_scan_event_cb.
const char *classify(const uint8_t *adv_data, uint8_t adv_len);
}
