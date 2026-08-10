#pragma once
#include <cstdint>

namespace WifiTxSpike {
// Sends two raw 802.11 deauthentication frames back-to-back, three seconds
// apart, both targeting a broadcast destination on the given channel:
//   1. Source address = this device's OWN real MAC (esp_wifi_get_mac) --
//      baseline test, should never be rejected by any sanity check since
//      it's not spoofed.
//   2. Source address = a fabricated address (02:00:00:AA:BB:CC, a locally-
//      administered range so it can't collide with a real vendor OUI) --
//      the actually-attack-relevant case.
// Logs the esp_wifi_80211_tx() return code for both. A real answer requires
// an external observer (e.g. a laptop running Wireshark/airodump-ng in
// monitor mode on the same channel, or a WiFi analyzer app showing deauth
// frame counts) confirming which of the two frames, if either, actually
// appeared over the air -- esp_wifi_80211_tx() returning ESP_OK only means
// the RPC call to the C6 succeeded, not that the C6 chose to transmit it.
void run(uint8_t channel);

// Exposed for reuse by real attack features (Task 4 onward) once the spike
// confirms spoofed-source frames reach the air. dest/src/bssid are all
// 6-byte MAC arrays; out must be at least 26 bytes.
void build_deauth_frame(uint8_t *out, const uint8_t dest[6], const uint8_t src[6], const uint8_t bssid[6]);
}
