#include "wifi_common.h"
#include <WiFi.h>
#include <cstdio>
#include <cstring>

int wifi_scan_aps(WifiApInfo *out, int max_count) {
    int n = WiFi.scanNetworks();
    if (n <= 0) return 0;
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
