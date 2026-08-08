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

    // WiFi.mode() returns false when the low-level radio bring-up fails.
    // Previously ignored, which meant a dead radio still cost the full 15s
    // timeout below (and kept re-triggering bring-up from inside WiFi.status()).
    if (!WiFi.mode(WIFI_STA)) {
        Serial.println("quarky-tab5: radio WiFi.mode(WIFI_STA) failed");
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
