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

// Asynchronous AP scan.
//
// Why asynchronous rather than a bare blocking WiFi.scanNetworks():
//
// Responsiveness. Arduino-ESP32's synchronous scan is effectively
// `while(scanning) delay(10);`, which parks the LVGL task for the several
// seconds a full 2.4GHz scan takes over the SDIO link to the ESP32-C6 -- the
// screen and the Back button go dead for the duration. Kicking the scan off,
// polling it from an LVGL timer and giving up after a deadline keeps the UI
// alive and Back usable no matter what the radio does.
//
// HISTORICAL NOTE: this header used to claim the blocking form "HARD LOCKED
// THE DEVICE" because the scan-done event never arrived over esp-hosted. That
// diagnosis was wrong and is retracted (2026-08-12). The scan-done event
// arrives reliably and the scan itself has always worked; the lock-up was
// LVGL exhausting its draw memory while rendering the results and then
// spinning forever, which the async rewrite happened not to fix because it
// was never a scan problem. See ui/lvgl_port.cpp for the real cause and fix.
// The async design is kept on its own merits, per the paragraph above.

// Starts an asynchronous scan. Returns false if it could not be started at all.
bool wifi_scan_begin();

// Poll an in-flight scan. Returns:
//   >= 0  the scan finished; that many APs were written to out[]
//   -1    still running, call again later
//   -2    the scan failed or exceeded wifi_scan_timeout_ms() since
//         wifi_scan_begin()
// Results are in WiFi's own RSSI-descending order.
int wifi_scan_poll(WifiApInfo *out, int max_count);

// Deadline applied by wifi_scan_poll(), in milliseconds.
uint32_t wifi_scan_timeout_ms();

void wifi_bssid_to_str(const uint8_t bssid[6], char out[18]);
