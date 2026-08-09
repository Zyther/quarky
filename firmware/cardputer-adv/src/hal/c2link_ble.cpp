#include "c2link_ble.h"
#include <Arduino.h> // Serial, millis(), FreeRTOS portMUX
#include <NimBLEDevice.h>
#include <crypto.h>
#include <cstring>

// Wire format on this transport: [c2proto WireHeader+payload bytes][32-byte
// HMAC over those bytes]. Matches Tab5's C2LinkBle (Task 13) byte-for-byte --
// this is its client-side counterpart, scanning for and connecting to its
// Nordic-UART-Service-shaped GATT server.
//
// -----------------------------------------------------------------------------
// Why this file (unlike Tab5's c2link_ble.cpp) is plain NimBLE-Arduino, not
// the raw ESP-IDF NimBLE C API
// -----------------------------------------------------------------------------
// Tab5 (ESP32-P4) has no onboard BT radio -- BLE is proxied to its C6
// co-processor over esp-hosted, and h2zero/NimBLE-Arduino's
// NimBLEDevice::init() unconditionally requires the classic
// esp_bt_controller_* local-radio bring-up API (esp_bt.h), which doesn't
// exist for esp32p4 in this framework. That's a confirmed, maintainer-
// acknowledged upstream limitation (h2zero/NimBLE-Arduino#906) -- see Task
// 13's report. Cardputer-ADV is a plain ESP32-S3 with a real onboard BT
// radio, so none of that applies here: standard NimBLE-Arduino works
// normally, and this file uses its ordinary client-side C++ API
// (NimBLEClient/NimBLEScan/NimBLERemoteCharacteristic) exactly as intended.
//
// Verified against the actually-installed h2zero/NimBLE-Arduino@2.5.1 (the
// task's suggested ^2.2.1 range resolves to 2.5.1, same as Task 13 found --
// current registry head) rather than assuming the brief's sample code was
// current. Checked the real headers under
// .pio/libdeps/cardputer-adv/NimBLE-Arduino/src. Two real discrepancies
// found:
//   - NimBLEScanResults::begin()/end() are
//     std::vector<NimBLEAdvertisedDevice*>::const_iterator -- iterating
//     yields POINTERS, not NimBLEAdvertisedDevice references. The brief's
//     sketch (`for (const NimBLEAdvertisedDevice &dev : results)`,
//     `dev.haveName()`, `s_client->connect(&dev)`) does not compile as
//     written; fixed below by iterating `const NimBLEAdvertisedDevice *dev`
//     and using `dev->` / `connect(dev)` (no extra `&`, dev is already a
//     pointer).
//   - `NimBLEClient::exchangeMTU()` exists and matches the brief, but
//     `connect()`'s 4th parameter (`exchangeMTU`, default true) already
//     triggers this internally as a side effect of a successful GAP connect
//     (NimBLEClient.cpp's BLE_GAP_EVENT_CONNECT handler calls
//     `pClient->exchangeMTU()` when `m_config.exchangeMTU` is set). Per this
//     task's explicit requirement, exchangeMTU() is still called here
//     explicitly and visibly right after connect() rather than relying
//     solely on that default -- see the comment in poll() below. Everything
//     else in the brief (writeValue/subscribe/getCharacteristic/getService/
//     createClient/getScan/NimBLEDevice::init signatures, and the
//     notify_callback signature) matched the installed 2.5.1 headers as-is.

static const char *kServiceUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *kRxCharUUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // write: Cardputer-ADV -> Tab5
static const char *kTxCharUUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // notify: Tab5 -> Cardputer-ADV

static uint8_t s_psk[16];
static char s_target_name[32];
static C2LinkReceiveHandler s_handler = nullptr;
static NimBLEClient *s_client = nullptr;
static NimBLERemoteCharacteristic *s_rxChar = nullptr; // write end (this device's outbound)
static uint32_t s_last_recv_ms = 0;

// Inbound frames are HMAC-verified and decoded inside on_notify(), which
// NimBLE-Arduino invokes on its own host task -- not the Arduino main task
// that calls poll(). Matching Tab5's hardened c2link_ble.cpp (Task 13,
// commit dce69fa), decoded frames are handed off via a small
// single-producer/single-consumer ring rather than dispatched directly, so
// the receive handler (which may touch shared state such as
// CommandDispatcher's FeatureRegistry, and may call back into this link's
// own send()) only ever runs on the main task, same as C2LinkWifi's
// contract. The portMUX guards the head/tail indices against the concurrent
// host-task producer and main-task consumer.
static constexpr size_t kRxQueueDepth = 4;
static c2proto::Frame s_rx_queue[kRxQueueDepth];
static volatile size_t s_rx_head = 0; // producer writes here (NimBLE host task)
static volatile size_t s_rx_tail = 0; // consumer reads here (main task, via poll())
static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;

static void on_notify(NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
    if (len < 32) return; // must at least hold a HMAC trailer
    size_t frame_len = len - 32;
    if (!c2proto::hmac_verify(s_psk, 16, data, frame_len, data + frame_len)) return;

    c2proto::Frame frame{};
    if (!c2proto::decode(data, frame_len, frame)) return;

    portENTER_CRITICAL(&s_rx_mux);
    size_t next = (s_rx_head + 1) % kRxQueueDepth;
    bool full = (next == s_rx_tail);
    if (!full) {
        s_rx_queue[s_rx_head] = frame;
        s_rx_head = next;
    }
    portEXIT_CRITICAL(&s_rx_mux);
    if (full) {
        Serial.println("quarky-cardputer-adv: c2link_ble rx queue full, frame dropped");
    }
}

bool C2LinkBle::init(const uint8_t psk[16], const char *target_device_name) {
    memcpy(s_psk, psk, 16);
    strncpy(s_target_name, target_device_name, sizeof(s_target_name) - 1);
    s_target_name[sizeof(s_target_name) - 1] = '\0';
    NimBLEDevice::init("");
    return true; // actual scan/connect happens in poll()
}

void C2LinkBle::poll() {
    if (s_client != nullptr && s_client->isConnected()) return;

    NimBLEScan *scan = NimBLEDevice::getScan();
    NimBLEScanResults results = scan->getResults(2000, false); // 2s blocking scan window
    for (const NimBLEAdvertisedDevice *dev : results) {        // 2.5.1: iterator yields pointers, not refs
        if (!dev->haveName() || dev->getName() != s_target_name) continue;

        if (s_client == nullptr) s_client = NimBLEDevice::createClient();
        if (!s_client->connect(dev)) continue; // dev is already a pointer -- no "&" here

        // Task 13's server permanently refuses send() until the ATT MTU is
        // negotiated above the 23-byte default (its send() guard checks
        // ble_att_mtu()). connect()'s default 4th argument already requests
        // this internally on a successful GAP connect, but that's an
        // implementation detail of this NimBLE-Arduino version this code
        // shouldn't depend on silently -- request it explicitly here so the
        // requirement is visible in our own code and survives a future
        // library version changing that default. exchangeMTU() only sends
        // the request (it's async); send()'s own MTU guard below covers the
        // window before the peer's response lands.
        s_client->exchangeMTU();

        NimBLERemoteService *svc = s_client->getService(kServiceUUID);
        if (svc == nullptr) { s_client->disconnect(); continue; }

        s_rxChar = svc->getCharacteristic(kRxCharUUID);
        NimBLERemoteCharacteristic *txChar = svc->getCharacteristic(kTxCharUUID);
        if (s_rxChar == nullptr || txChar == nullptr) { s_rxChar = nullptr; s_client->disconnect(); continue; }

        txChar->subscribe(true, on_notify);
        Serial.printf("quarky-cardputer-adv: c2link_ble connected to \"%s\"\n", s_target_name);
        break;
    }

    // Drain any frames the host task queued while we were scanning/connecting.
    for (;;) {
        c2proto::Frame frame;
        bool have = false;
        portENTER_CRITICAL(&s_rx_mux);
        if (s_rx_tail != s_rx_head) {
            frame = s_rx_queue[s_rx_tail];
            s_rx_tail = (s_rx_tail + 1) % kRxQueueDepth;
            have = true;
        }
        portEXIT_CRITICAL(&s_rx_mux);
        if (!have) break;
        s_last_recv_ms = millis();
        if (s_handler) s_handler(frame);
    }
}

bool C2LinkBle::send(const c2proto::Frame &frame) {
    if (s_client == nullptr || !s_client->isConnected() || s_rxChar == nullptr) return false;

    uint8_t frame_buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload];
    int n = c2proto::encode(frame, frame_buf, sizeof(frame_buf));
    if (n < 0) return false;

    // Mirrors Task 13's server-side send() guard: a single ATT write cannot
    // exceed (MTU - 3) bytes, and the default pre-exchange MTU (23B, 20
    // usable) can't even fit a zero-payload frame (10B header + 32B HMAC).
    // poll()'s exchangeMTU() call is fire-and-forget (request only); this
    // guard covers the window before the peer's response lands, refusing
    // rather than silently truncating a frame while reporting success.
    uint16_t mtu = s_client->getMTU();
    size_t need = (size_t)n + 32;
    if (mtu <= 3 || need > (size_t)(mtu - 3)) {
        Serial.printf("quarky-cardputer-adv: c2link_ble send skipped, frame %d+32 > mtu %u-3\n", n, mtu);
        return false;
    }

    uint8_t mac[32];
    c2proto::hmac_sha256(s_psk, 16, frame_buf, (size_t)n, mac);

    uint8_t out[sizeof(frame_buf) + 32];
    memcpy(out, frame_buf, n);
    memcpy(out + n, mac, 32);

    return s_rxChar->writeValue(out, n + 32, false);
}

void C2LinkBle::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}

bool C2LinkBle::is_connected() {
    // "Ready to carry frames" means connected AND service discovery resolved
    // the Rx characteristic (send()'s other precondition) -- not just a raw
    // GAP connection, which could still be mid-discovery.
    return s_client != nullptr && s_client->isConnected() && s_rxChar != nullptr;
}

uint32_t c2link_ble_last_recv_ms() {
    return s_last_recv_ms;
}
