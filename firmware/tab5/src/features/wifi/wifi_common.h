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
// The blocking form this used to expose (a bare WiFi.scanNetworks()) HARD
// LOCKED THE DEVICE on real hardware -- confirmed 2026-08-10: after opening
// the WiFi Scan screen the firmware emitted nothing further on serial, ever,
// and stopped responding to input entirely; only a reset recovered it.
// Arduino-ESP32's synchronous scan is `while(_scanStarted) delay(10);`, which
// never terminates if the scan-done event does not arrive -- and on this board
// WiFi is proxied over SDIO to the ESP32-C6 co-processor which is already
// running c2link_wifi's SoftAP, so that event does not reliably arrive. Since
// delay() feeds the task watchdog, it hangs silently rather than panicking.
//
// Hence: kick the scan off, poll it, and give up after a deadline. The caller
// keeps running lv_timer_handler() throughout, so the UI stays alive and Back
// stays usable no matter what the radio does.

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
