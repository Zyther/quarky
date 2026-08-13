#include "wifi_tx_spike.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cstring>

namespace WifiTxSpike {

// IEEE 802.11 deauthentication management frame, fixed 26-byte header +
// 2-byte reason code. Addr1 = destination, Addr2 = source, Addr3 = BSSID.
void build_deauth_frame(uint8_t *out, const uint8_t dest[6], const uint8_t src[6], const uint8_t bssid[6]) {
    out[0] = 0xC0; out[1] = 0x00;                 // Frame Control: mgmt, subtype=deauth
    out[2] = 0x00; out[3] = 0x00;                 // Duration
    memcpy(out + 4, dest, 6);                     // Addr1: destination
    memcpy(out + 10, src, 6);                      // Addr2: source
    memcpy(out + 16, bssid, 6);                    // Addr3: BSSID
    out[22] = 0x00; out[23] = 0x00;                // Seq-ctrl
    out[24] = 0x01; out[25] = 0x00;                // Reason code 1 (unspecified)
}

void run(uint8_t channel) {
    uint8_t frame[26];
    uint8_t own_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, own_mac);
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    build_deauth_frame(frame, broadcast, own_mac, own_mac);
    esp_err_t rc1 = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
    Serial.printf("quarky-tab5: [wifi-tx-spike] own-MAC deauth tx rc=0x%x (%s)\n",
                  rc1, rc1 == ESP_OK ? "OK" : "FAILED");

    delay(3000);

    uint8_t spoofed[6] = {0x02, 0x00, 0x00, 0xAA, 0xBB, 0xCC};
    build_deauth_frame(frame, broadcast, spoofed, spoofed);
    esp_err_t rc2 = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
    Serial.printf("quarky-tab5: [wifi-tx-spike] spoofed-MAC deauth tx rc=0x%x (%s)\n",
                  rc2, rc2 == ESP_OK ? "OK" : "FAILED");
    // Real hardware (2026-08-10): both rc1 and rc2 above are
    // ESP_ERR_NOT_SUPPORTED (0x106), always -- the call fails synchronously
    // before any transmission is attempted, so no external monitor-mode
    // confirmation is needed or would change the answer. See wifi_tx_spike.h
    // for the full, current result; this message is stale runtime advice
    // from before that was known and is unreachable since run() is never
    // called (kept for whoever eventually re-wires this on native-radio
    // hardware, where the answer may differ).
    Serial.println("quarky-tab5: [wifi-tx-spike] done -- rc != ESP_OK on this hardware means "
                    "the transport rejected the call before any transmission was attempted "
                    "(see wifi_tx_spike.h); only re-check externally if this target's rc is OK");
}

} // namespace WifiTxSpike
