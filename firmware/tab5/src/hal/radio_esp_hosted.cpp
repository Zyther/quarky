#include "radio_esp_hosted.h"
#include "hosted_link.h"
#include <Arduino.h>
#include <WiFi.h> // backed transparently by esp-hosted's WiFiRemote on the C6

bool RadioEspHosted::connect_wifi(const char *ssid, const char *pass) {
    // Everything below needs the ESP32-C6 co-processor. If its SDIO link never
    // came up, return immediately: without this guard each WiFi call re-enters
    // wifiLowLevelInit(), which re-runs the whole failing esp-hosted bring-up
    // (arduino-esp32 clears its `lowLevelInitDone` latch on failure), and the
    // 15s poll loop below turns that into ~75 back-to-back C6 reset attempts.
    // That retry storm -- not any single failure -- is what used to brown the
    // rail out and reset the chip in a loop. See hosted_link.h.
    if (!hosted_link::begin()) {
        Serial.println("quarky-tab5: radio connect_wifi skipped, C6 link down");
        return false;
    }

    // Mode-preserving station bring-up (whole-branch review finding I1,
    // 2026-08-17). This used to be a bare WiFi.mode(WIFI_STA), which drops any
    // existing WIFI_MODE_AP -- i.e. it silently killed c2link_wifi's SoftAP,
    // the Cardputer-ADV's WiFi C2 transport. That was harmless while this
    // function's only caller ran in setup() before c2link_wifi.init(), but the
    // second Phase 2 plan's Task 3 turned it into a real launcher-tile action
    // ("WiFi Connect"), so tapping Connect mid-session took the C2 link down
    // with nothing to bring it back -- a direct violation of this plan's own
    // Global Constraint ("never a bare WIFI_STA/WIFI_AP that drops the
    // existing mode").
    //
    // Same idiom wifi_common.cpp, wifi_spectrum.cpp and wifi_pmkid.cpp all
    // already use: only ever widen WIFI_AP to WIFI_AP_STA, and only set a bare
    // WIFI_STA when there is no AP to preserve.
    //
    // The return value is checked because WiFi.mode() returns false when the
    // low-level radio bring-up fails. Ignoring it (as this originally did)
    // meant a dead radio still cost the full 15s timeout below, and kept
    // re-triggering bring-up from inside WiFi.status().
    wifi_mode_t current = WiFi.getMode();
    wifi_mode_t wanted = (current == WIFI_AP || current == WIFI_AP_STA) ? WIFI_AP_STA : WIFI_STA;
    if (!WiFi.mode(wanted)) {
        Serial.printf("quarky-tab5: radio WiFi.mode(%s) failed\n",
                      wanted == WIFI_AP_STA ? "WIFI_AP_STA" : "WIFI_STA");
        return false;
    }
    if (WiFi.begin(ssid, pass) == WL_CONNECT_FAILED) {
        Serial.println("quarky-tab5: radio WiFi.begin failed");
        return false;
    }
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(200);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool RadioEspHosted::is_connected() {
    // Guarded for the same reason as connect_wifi(): WiFi.status() on an
    // uninitialised stack can re-enter low-level init, so a caller polling this
    // from loop() must not be able to restart the retry storm.
    if (!hosted_link::available()) return false;
    return WiFi.status() == WL_CONNECTED;
}

uint32_t RadioEspHosted::local_ip() {
    if (!hosted_link::available()) return 0;
    return (uint32_t)WiFi.localIP();
}
