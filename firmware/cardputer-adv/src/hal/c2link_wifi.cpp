#include "c2link_wifi.h"
#include <Arduino.h>
#include <WiFi.h>
#include <crypto.h>
#include <cstring>

// Wire format on this transport: [c2proto WireHeader+payload bytes][32-byte HMAC over those bytes].
// Matches Tab5's C2LinkWifi (Task 11) byte-for-byte -- this is its client-side
// counterpart, joining Tab5's AP and connecting to its TCP server socket.

static uint8_t s_psk[16];
static char s_server_ip[16];
static uint16_t s_port = 0;
static bool s_initialized = false;
static WiFiClient s_client;
static C2LinkReceiveHandler s_handler = nullptr;
static uint32_t s_last_recv_ms = 0;

// Reconnect throttle: WiFiClient::connect() blocks for its connect timeout
// (several seconds) when the server isn't reachable yet -- e.g. Tab5's AP is
// up but its TCP server hasn't accept()ed yet, or the STA link briefly
// drops. Without a throttle, poll() would re-attempt (and re-block) on every
// loop() iteration while unconnected.
static uint32_t s_last_connect_attempt_ms = 0;
static const uint32_t kReconnectIntervalMs = 2000;

// Incremental frame-reassembly state for poll(). Same fix as Tab5's
// c2link_wifi.cpp (Task 11): NetworkClient (this framework's WiFiClient,
// arduino-esp32 v3.3.11) has no peekBytes() -- confirmed against the
// installed framework-arduinoespressif32/libraries/Network/src/
// NetworkClient.h, whose Stream/Client/ESPLwIPClient ancestry only offers
// peek() (single byte, non-consuming) alongside read()/readBytes() (both
// consume). So bytes are consumed into a persistent buffer as they arrive
// and reassembly state carries across poll() calls, rather than peeking the
// header first to decide how much more to wait for.
static uint8_t s_rx_buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload + 32];
static size_t s_rx_have = 0;                           // bytes accumulated into s_rx_buf so far
static size_t s_rx_need = sizeof(c2proto::WireHeader); // bytes needed before the next decision point
static bool s_rx_header_parsed = false;
static size_t s_rx_frame_len = 0;                      // WireHeader + payload, once header is known

static void reset_rx_state() {
    s_rx_have = 0;
    s_rx_need = sizeof(c2proto::WireHeader);
    s_rx_header_parsed = false;
    s_rx_frame_len = 0;
}

bool C2LinkWifi::init(const uint8_t psk[16], const char *ap_ssid, const char *ap_password,
                       const char *server_ip, uint16_t port) {
    memcpy(s_psk, psk, 16);
    strncpy(s_server_ip, server_ip, sizeof(s_server_ip) - 1);
    s_server_ip[sizeof(s_server_ip) - 1] = '\0';
    s_port = port;
    reset_rx_state();

    if (!WiFi.mode(WIFI_STA)) {
        Serial.println("quarky-cardputer-adv: c2link_wifi WiFi.mode(WIFI_STA) failed");
        return false;
    }
    WiFi.begin(ap_ssid, ap_password);
    s_initialized = true;
    return true; // actual AP association + socket connect happens in poll()
}

void C2LinkWifi::poll() {
    if (!s_initialized) return;

    if (WiFi.status() != WL_CONNECTED) {
        // Not yet (or no longer) associated with Tab5's AP -- drop any stale
        // socket state and wait for the station link to come back.
        if (s_client.connected()) s_client.stop();
        reset_rx_state();
        return;
    }

    if (!s_client.connected()) {
        uint32_t now = millis();
        if (now - s_last_connect_attempt_ms < kReconnectIntervalMs) return;
        s_last_connect_attempt_ms = now;
        reset_rx_state();
        s_client.connect(s_server_ip, s_port);
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
        if (c2proto::decode(s_rx_buf, s_rx_frame_len, frame)) {
            s_last_recv_ms = millis();
            if (s_handler) s_handler(frame);
        }
    } // else: drop silently, bad auth

    reset_rx_state();
}

bool C2LinkWifi::send(const c2proto::Frame &frame) {
    if (!s_client.connected()) return false;
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
    return s_client.connected();
}

uint32_t c2link_wifi_last_recv_ms() {
    return s_last_recv_ms;
}
