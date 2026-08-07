#include "radio_esp_hosted.h"
#include <WiFi.h> // backed transparently by esp-hosted's WiFiRemote on the C6

bool RadioEspHosted::connect_wifi(const char *ssid, const char *pass) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(200);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool RadioEspHosted::is_connected() {
    return WiFi.status() == WL_CONNECTED;
}

uint32_t RadioEspHosted::local_ip() {
    return (uint32_t)WiFi.localIP();
}
