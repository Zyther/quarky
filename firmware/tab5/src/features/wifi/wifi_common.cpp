#include "wifi_common.h"
#include <WiFi.h>
#include <cstdio>
#include <cstring>

namespace {
// Generous: a full active scan of all 2.4GHz channels proxied over SDIO to the
// C6 is slower than a local radio would be. This is a backstop against hanging
// forever, not a performance target.
constexpr uint32_t kScanTimeoutMs = 15000;
uint32_t s_scan_started_ms = 0;
} // namespace

uint32_t wifi_scan_timeout_ms() { return kScanTimeoutMs; }

bool wifi_scan_begin() {
    // Drop any results still held from a previous visit to the scan screen,
    // otherwise scanComplete() would immediately report the stale count and
    // the new scan would look like it finished instantly.
    WiFi.scanDelete();

    // The SoftAP that c2link_wifi brought up at boot must keep running -- it is
    // the WiFi C2 transport. AP_STA leaves it up while giving the scan a
    // station interface to run on. Without this, Arduino's scan would flip the
    // interface to STA-only and silently drop the C2 link.
    if (WiFi.getMode() == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);
    } else if (WiFi.getMode() == WIFI_OFF) {
        WiFi.mode(WIFI_STA);
    }

    s_scan_started_ms = millis();
    // `true` = asynchronous: returns immediately, results collected via
    // scanComplete() below.
    int rc = WiFi.scanNetworks(true);
    return rc != WIFI_SCAN_FAILED;
}

int wifi_scan_poll(WifiApInfo *out, int max_count) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
        if (millis() - s_scan_started_ms > kScanTimeoutMs) {
            WiFi.scanDelete();
            return -2;
        }
        return -1;
    }
    if (n < 0) {
        return -2; // WIFI_SCAN_FAILED
    }

    int written = 0;
    for (int i = 0; i < n && written < max_count; i++) {
        WifiApInfo &info = out[written];
        strncpy(info.ssid, WiFi.SSID(i).c_str(), sizeof(info.ssid) - 1);
        info.ssid[sizeof(info.ssid) - 1] = '\0';
        memcpy(info.bssid, WiFi.BSSID(i), 6);
        info.rssi = (int8_t)WiFi.RSSI(i);
        info.channel = (uint8_t)WiFi.channel(i);
        info.open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
        written++;
    }
    WiFi.scanDelete();
    return written;
}

void wifi_bssid_to_str(const uint8_t bssid[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}
