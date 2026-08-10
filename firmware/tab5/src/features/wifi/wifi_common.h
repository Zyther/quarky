#pragma once
#include <cstdint>
#include <cstddef>

struct WifiApInfo {
    char ssid[33];       // 802.11 SSID max is 32 bytes + null terminator
    uint8_t bssid[6];
    int8_t rssi;
    uint8_t channel;
    bool open;            // true if no auth/encryption required
};

// Runs a blocking WiFi.scanNetworks() and fills out[] with up to max_count
// results (WiFi.scanNetworks()'s own default RSSI-descending order).
// Returns the number of APs written (0 on failure/no results found). Safe
// to call repeatedly; each call re-scans and replaces the prior results.
// Blocks for the duration of the scan (typically 2-4s) -- callers running
// this from a UI must not call it directly from a button's click handler on
// the main task without the caller understanding this stalls LVGL for that
// duration; Task 3's own screen accepts this for a first, minimal scan
// button (see Step 3's note), a background/async scan is deferred to the
// second plan if it proves too janky in practice.
int wifi_scan_aps(WifiApInfo *out, int max_count);

void wifi_bssid_to_str(const uint8_t bssid[6], char out[18]);
