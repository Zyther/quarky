#pragma once
#include <cstdint>

// -----------------------------------------------------------------------------
// UNREFERENCED BY DESIGN -- REAL-HARDWARE RESULT ALREADY IN, NOT AN OPEN
// SPIKE. Final whole-branch review finding I2 (2026-08-13): this header used
// to read as if the spike's outcome were still pending ("once the spike
// confirms spoofed-source frames reach the air", "a real answer requires an
// external observer"). It isn't pending. Real hardware (2026-08-10):
// esp_wifi_80211_tx() returns ESP_ERR_NOT_SUPPORTED (0x106) synchronously,
// for both the own-MAC and spoofed-MAC frames, before any radio transmission
// is attempted -- no external monitor-mode capture was needed, or would have
// changed the answer, since the call fails at the RPC layer, not on-air.
//
// A later investigation (Task 6, task-6-promiscuous-investigation-report.md)
// disassembled the actual linked code and found the root cause: this
// project's esp_wifi_remote_80211_tx() is Espressif's weak default stub
// (`li a0,262; ret` -- no RPC call, no SDIO round-trip to the C6 at all),
// the same mechanism behind esp_wifi_set_promiscuous()'s identical failure
// (see wifi_pmkid.cpp's file-level comment for the fuller writeup: what
// this rules out, what upstream esp-hosted actually supports, and the one
// real but high-risk/out-of-scope escape hatch found). Raw 802.11 frame
// injection is not usable through esp-hosted WiFiRemote on the Tab5 at all,
// on any co-processor firmware version, with no config fix available.
//
// Per the project owner's explicit direction, WiFi deauth (Task 4) was
// never implemented on Tab5-native and is deferred to a future
// Cardputer-ADV-affinity or C5-sidecar phase with real, non-proxied WiFi
// radio hardware (see the plan document's Task 4 DEFERRED note). This file
// is kept, uncalled, purely as real, considered reference material for
// whoever picks that up: build_deauth_frame() constructs a correct 26-byte
// 802.11 deauth frame and remains directly reusable; only the *host
// device* changes, not the underlying frame-building approach.
// -----------------------------------------------------------------------------
namespace WifiTxSpike {
// Sends two raw 802.11 deauthentication frames back-to-back, three seconds
// apart, both targeting a broadcast destination on the given channel:
//   1. Source address = this device's OWN real MAC (esp_wifi_get_mac) --
//      baseline test, should never be rejected by any sanity check since
//      it's not spoofed.
//   2. Source address = a fabricated address (02:00:00:AA:BB:CC, a locally-
//      administered range so it can't collide with a real vendor OUI) --
//      the actually-attack-relevant case.
// Logs the esp_wifi_80211_tx() return code for both -- on the Tab5, both are
// ESP_ERR_NOT_SUPPORTED (0x106), always, per the finding above. NOTE: if
// this is ever re-wired to a real UI path on a target where the call
// actually reaches the radio, run()'s internal delay(3000) between the two
// frames would violate this project's ~50ms loop() budget constraint --
// harmless today only because this function is never called.
void run(uint8_t channel);

// Reusable frame-building logic, independent of the esp-hosted transport
// limitation above -- correct on any target. dest/src/bssid are all 6-byte
// MAC arrays; out must be at least 26 bytes.
void build_deauth_frame(uint8_t *out, const uint8_t dest[6], const uint8_t src[6], const uint8_t bssid[6]);
}
