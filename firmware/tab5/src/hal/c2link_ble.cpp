#include "c2link_ble.h"
#include <NimBLEDevice.h>
#include <crypto.h>
#include <cstring>

// Nordic UART Service UUIDs -- a de facto standard for exactly this
// "bidirectional byte pipe over GATT" shape, reused rather than inventing
// a custom service so any BLE debugging tool that already knows NUS works
// against this link for free.
static const char *kServiceUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *kRxCharUUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // write: peer -> Tab5
static const char *kTxCharUUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // notify: Tab5 -> peer

static NimBLECharacteristic *s_txChar = nullptr;
static uint8_t s_psk[16];
static C2LinkReceiveHandler s_handler = nullptr;
static volatile bool s_connected = false;

// Verified against the actually-installed h2zero/NimBLE-Arduino@2.5.1 (the
// brief suggested ^2.2.1; that version-range resolves to 2.5.1, the current
// registry head -- see task-13-report.md). Checked the real headers under
// .pio/libdeps/tab5/NimBLE-Arduino/src rather than assuming the brief's API
// sketch was current:
//   - NimBLECharacteristicCallbacks::onWrite(NimBLECharacteristic*,
//     NimBLEConnInfo&) and NimBLEServerCallbacks::onConnect/onDisconnect --
//     signatures match the brief exactly (NimBLECharacteristic.h,
//     NimBLEServer.h).
//   - NimBLECharacteristic::getValue() returns a NimBLEAttValue, not a
//     std::string directly, but NimBLEAttValue has an
//     "operator std::string() const" (NimBLEAttValue.h), so
//     "std::string data = chr->getValue();" still compiles and behaves as
//     the brief assumed.
//   - NimBLEService::start() is DEPRECATED in this version
//     (__attribute__((deprecated(...))), a no-op kept only for source
//     compat) -- "Services are started when the server is started."
//     (NimBLEService.h). The brief's code called it; that line is dropped
//     here. It's not needed anyway: NimBLEAdvertising::start() calls
//     pServer->start() internally before it starts advertising
//     (NimBLEAdvertising.cpp: "make sure the GATT server is ready before
//     advertising"), which is also exactly the pattern the library's own
//     NimBLE_Server example follows -- it never calls service->start() or
//     server->start() explicitly either, relying on adv->start() to do it.

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &) override {
        std::string data = chr->getValue();
        if (data.size() < 32) return; // must at least hold a HMAC trailer
        size_t frame_len = data.size() - 32;
        const uint8_t *bytes = reinterpret_cast<const uint8_t *>(data.data());

        if (!c2proto::hmac_verify(s_psk, 16, bytes, frame_len, bytes + frame_len)) return;

        c2proto::Frame frame{};
        if (c2proto::decode(bytes, frame_len, frame) && s_handler) {
            s_handler(frame);
        }
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override { s_connected = true; }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        s_connected = false;
        NimBLEDevice::startAdvertising(); // resume advertising so a dropped link can reconnect
    }
};

// Callback objects are handed to setCallbacks() and must outlive the
// characteristic/server -- static storage, same lifetime as the statics
// above, rather than stack/local objects that would dangle.
static RxCallbacks s_rxCallbacks;
static ServerCallbacks s_serverCallbacks;

bool C2LinkBle::init(const uint8_t psk[16], const char *device_name) {
    memcpy(s_psk, psk, 16);

    NimBLEDevice::init(device_name);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(&s_serverCallbacks);

    NimBLEService *service = server->createService(kServiceUUID);
    NimBLECharacteristic *rxChar = service->createCharacteristic(kRxCharUUID, NIMBLE_PROPERTY::WRITE);
    rxChar->setCallbacks(&s_rxCallbacks);
    s_txChar = service->createCharacteristic(kTxCharUUID, NIMBLE_PROPERTY::NOTIFY);
    // No explicit service->start()/server->start() here -- see the note
    // above. NimBLEAdvertising::start() below brings the GATT server up.

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(kServiceUUID);
    adv->start();

    return true;
}

bool C2LinkBle::send(const c2proto::Frame &frame) {
    if (!s_connected || s_txChar == nullptr) return false;

    uint8_t frame_buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload];
    int n = c2proto::encode(frame, frame_buf, sizeof(frame_buf));
    if (n < 0) return false;

    uint8_t mac[32];
    c2proto::hmac_sha256(s_psk, 16, frame_buf, (size_t)n, mac);

    uint8_t out[sizeof(frame_buf) + 32];
    memcpy(out, frame_buf, n);
    memcpy(out + n, mac, 32);

    s_txChar->setValue(out, n + 32);
    return s_txChar->notify();
}

void C2LinkBle::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}

bool C2LinkBle::is_connected() {
    return s_connected;
}
