#include "c2link_wifi.h"
#include "hosted_link.h"
#include <Arduino.h>
#include <WiFi.h>
#include <crypto.h>
#include <cstring>

// Wire format on this transport: [c2proto WireHeader+payload bytes][32-byte HMAC over those bytes].
// Unlike ESP-NOW there's no hard payload ceiling here, but frames still respect
// c2proto::kMaxPayload -- this transport doesn't need larger frames, it needs
// an always-open connection, which a TCP socket already gives it.

static WiFiServer *s_server = nullptr;
static WiFiClient s_client;
static uint8_t s_psk[16];
static C2LinkReceiveHandler s_handler = nullptr;

// Incremental frame-reassembly state for poll(). NetworkClient (this
// framework's WiFiClient, arduino-esp32 v3.3.11) has no peekBytes() --
// unlike e.g. ESP8266's WiFiClient or WiFiClientSecure, it only offers
// peek() (single byte, non-consuming) alongside read()/readBytes(), both of
// which consume. Verified against the installed
// framework-arduinoespressif32/libraries/Network/src/NetworkClient.h: no
// peekBytes member anywhere in its Stream/Client/ESPLwIPClient ancestry.
// So instead of peeking the header to decide how much more to wait for
// (the brief's original approach), bytes are consumed into a persistent
// buffer as they arrive and reassembly state carries across poll() calls --
// same wire format and HMAC-verify-then-decode behavior, just without
// requiring a peek-without-consuming primitive that doesn't exist here.
static uint8_t s_rx_buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload + 32];
static size_t s_rx_have = 0;                                  // bytes accumulated into s_rx_buf so far
static size_t s_rx_need = sizeof(c2proto::WireHeader);        // bytes needed before the next decision point
static bool s_rx_header_parsed = false;
static size_t s_rx_frame_len = 0;                             // WireHeader + payload, once header is known

static void reset_rx_state() {
    s_rx_have = 0;
    s_rx_need = sizeof(c2proto::WireHeader);
    s_rx_header_parsed = false;
    s_rx_frame_len = 0;
}

bool C2LinkWifi::init(const uint8_t psk[16], const char *ap_ssid, const char *ap_password, uint16_t port) {
    memcpy(s_psk, psk, 16);

    // This whole transport rides the ESP32-C6 co-processor. Bail out before
    // touching any WiFi API if its SDIO link is down -- otherwise WiFi.mode()
    // re-enters the failing esp-hosted bring-up on every call, which is what
    // used to crash-loop the board via the brownout detector. See hosted_link.h.
    if (!hosted_link::begin()) {
        Serial.println("quarky-tab5: c2link_wifi init skipped, C6 link down");
        return false;
    }

    // WiFi.mode()'s return value was previously discarded; it is false when the
    // radio failed to come up, and continuing past that used to leave s_server
    // null while poll() dereferenced it every loop() iteration.
    if (!WiFi.mode(WIFI_AP)) {
        Serial.println("quarky-tab5: c2link_wifi WiFi.mode(WIFI_AP) failed");
        return false;
    }
    if (!WiFi.softAP(ap_ssid, ap_password)) {
        Serial.println("quarky-tab5: c2link_wifi softAP failed");
        return false;
    }
    s_server = new WiFiServer(port);
    s_server->begin();
    return true;
}

void C2LinkWifi::poll() {
    // s_server stays null when init() failed or was never called. Without this
    // guard the s_server->accept() below is a null dereference on the very
    // first loop() iteration -- i.e. a failed AP bring-up turned into a crash
    // rather than a degraded-but-running board.
    if (s_server == nullptr) return;

    if (!s_client || !s_client.connected()) {
        WiFiClient incoming = s_server->accept();
        if (incoming) {
            s_client = incoming;
            reset_rx_state();
        }
        return;
    }

    while (s_rx_have < s_rx_need && s_client.available() > 0) {
        int n = s_client.read(s_rx_buf + s_rx_have, s_rx_need - s_rx_have);
        if (n <= 0) break;
        s_rx_have += (size_t)n;
    }
    if (s_rx_have < s_rx_need) return; // wait for the rest to arrive on a later poll()

    if (!s_rx_header_parsed) {
        c2proto::WireHeader hdr{};
        memcpy(&hdr, s_rx_buf, sizeof(hdr));
        if (hdr.payload_len > c2proto::kMaxPayload) { s_client.stop(); reset_rx_state(); return; } // malformed, drop connection

        s_rx_frame_len = sizeof(c2proto::WireHeader) + hdr.payload_len;
        s_rx_need = s_rx_frame_len + 32; // + HMAC trailer
        s_rx_header_parsed = true;
        if (s_rx_have < s_rx_need) return; // header known, still waiting on payload+HMAC
    }

    if (c2proto::hmac_verify(s_psk, 16, s_rx_buf, s_rx_frame_len, s_rx_buf + s_rx_frame_len)) {
        c2proto::Frame frame{};
        if (c2proto::decode(s_rx_buf, s_rx_frame_len, frame) && s_handler) {
            s_handler(frame);
        }
    } // else: drop silently, bad auth

    reset_rx_state();
}

bool C2LinkWifi::send(const c2proto::Frame &frame) {
    if (!s_client || !s_client.connected()) return false;
    uint8_t buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload];
    int n = c2proto::encode(frame, buf, sizeof(buf));
    if (n < 0) return false;
    uint8_t mac[32];
    c2proto::hmac_sha256(s_psk, 16, buf, (size_t)n, mac);
    s_client.write(buf, n);
    s_client.write(mac, 32);
    return true;
}

void C2LinkWifi::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}

bool C2LinkWifi::is_connected() {
    return s_client && s_client.connected();
}
