#include "c2link_espnow.h"
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>

// esp_now_recv_info_t, esp_now_peer_info_t (peer_addr/lmk/channel/ifidx/
// encrypt/priv), esp_now_init(), esp_now_register_recv_cb(), and
// esp_now_add_peer() were checked directly against the esp32p4 headers
// shipped in this project's pioarduino/Arduino-ESP32 v3.3.11 framework
// (framework-arduinoespressif32-libs/esp32p4/include/esp_wifi/include/
// esp_now.h, and the identical injected esp_wifi_remote copy used when
// CONFIG_ESP_WIFI_REMOTE_ENABLED is set -- see Task 9's radio_esp_hosted.h
// for why that config applies on this target). All signatures used below
// match the brief as written; no API adaptation was needed.

static C2LinkReceiveHandler s_handler = nullptr;
static uint8_t s_peer_mac[6];

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    c2proto::Frame frame{};
    if (c2proto::decode(data, (size_t)len, frame) && s_handler) {
        s_handler(frame);
    }
}

bool C2LinkEspNow::init(const uint8_t psk[16], const uint8_t peer_mac[6]) {
    memcpy(s_peer_mac, peer_mac, 6);
    WiFi.mode(WIFI_STA); // ESP-NOW rides on the STA interface without associating
    if (esp_now_init() != ESP_OK) return false;

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = 0;
    peer.encrypt = true;
    memcpy(peer.lmk, psk, 16); // PSK doubles as the ESP-NOW LMK for this link
    if (esp_now_add_peer(&peer) != ESP_OK) return false;

    esp_now_register_recv_cb(on_recv);
    return true;
}

bool C2LinkEspNow::send(const c2proto::Frame &frame) {
    uint8_t buf[300];
    int n = c2proto::encode(frame, buf, sizeof(buf));
    if (n < 0) return false;
    return esp_now_send(s_peer_mac, buf, (size_t)n) == ESP_OK;
}

void C2LinkEspNow::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}
