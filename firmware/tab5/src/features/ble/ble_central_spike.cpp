#include "ble_central_spike.h"
#include "ble_central.h"
#include <Arduino.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h> // ble_gattc_disc_all_svcs / struct ble_gatt_svc.
                            // NOT <host/ble_gattc.h> -- no such header exists
                            // in the ESP-IDF NimBLE tree the P4 Arduino libs
                            // ship; the client-side ble_gattc_* API is
                            // declared in ble_gatt.h alongside the server-side
                            // ble_gatts_* API.
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <cstring>

namespace BleCentralSpike {

static constexpr int32_t kConnectTimeoutMs = 5000;
static constexpr uint32_t kSpikeTimeoutMs = 8000;

// Written on the NimBLE host task, read on the main/loop task (poll()) --
// same cross-task situation ble_scan.cpp documents for s_devices_dirty /
// s_scan_complete. Single words, so volatile is sufficient here (no portMUX
// needed for the same reason c2link_ble.cpp's s_last_recv_ms doesn't get one).
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile uint32_t s_svc_count = 0;
static volatile uint32_t s_connect_started_ms = 0;
static volatile bool s_active = false;

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr) {
        s_svc_count++;
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&service->uuid.u, uuid_str);
        Serial.printf("quarky-tab5: [ble-central-spike] service found: %s (handles %u-%u)\n",
                      uuid_str, service->start_handle, service->end_handle);
    } else if (error->status == BLE_HS_EDONE) {
        Serial.printf("quarky-tab5: [ble-central-spike] service discovery complete, %lu service(s)\n",
                      (unsigned long)s_svc_count);
        BleCentral::disconnect(conn_handle);
    } else {
        Serial.printf("quarky-tab5: [ble-central-spike] service discovery error status=%d\n", error->status);
    }
    return 0;
}

// Runs on the NimBLE host task. Deliberately the same event-switch shape
// hal/c2link_ble.cpp's gap_event_cb uses (same struct, same field names:
// event->connect.status, event->connect.conn_handle, event->disconnect.reason)
// -- that file is the real-hardware-proven reference for this dispatch.
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            Serial.printf("quarky-tab5: [ble-central-spike] CONNECTED handle=%u -- starting service discovery\n",
                          event->connect.conn_handle);
            s_svc_count = 0;
            int rc = ble_gattc_disc_all_svcs(event->connect.conn_handle, disc_svc_cb, nullptr);
            Serial.printf("quarky-tab5: [ble-central-spike] ble_gattc_disc_all_svcs rc=%d\n", rc);
        } else {
            Serial.printf("quarky-tab5: [ble-central-spike] CONNECT FAILED status=%d\n",
                          event->connect.status);
            s_active = false;
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        Serial.printf("quarky-tab5: [ble-central-spike] disconnected, reason=%d\n",
                      event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_active = false;
        return 0;
    default:
        return 0;
    }
}

void run(const uint8_t addr_val[6], uint8_t addr_type) {
    ble_addr_t target{};
    target.type = addr_type;
    memcpy(target.val, addr_val, 6);
    Serial.printf("quarky-tab5: [ble-central-spike] connecting to "
                  "%02X:%02X:%02X:%02X:%02X:%02X (peer addr type=%u)\n",
                  addr_val[5], addr_val[4], addr_val[3],
                  addr_val[2], addr_val[1], addr_val[0], addr_type);
    s_svc_count = 0;
    s_connect_started_ms = millis();
    s_active = true;
    int rc = BleCentral::connect(target, kConnectTimeoutMs, gap_event_cb, nullptr);
    if (rc != 0) {
        // rc != 0 means the attempt never even started, so no
        // BLE_GAP_EVENT_CONNECT will ever arrive to clear s_active.
        s_active = false;
    }
}

void poll() {
    if (!s_active) return;
    if (millis() - s_connect_started_ms < kSpikeTimeoutMs) return;

    uint16_t handle = s_conn_handle;
    if (handle != BLE_HS_CONN_HANDLE_NONE) {
        Serial.printf("quarky-tab5: [ble-central-spike] spike timeout (%lums) with connection still "
                      "open -- forcing disconnect\n", (unsigned long)kSpikeTimeoutMs);
        BleCentral::disconnect(handle);
    } else {
        Serial.println("quarky-tab5: [ble-central-spike] spike timeout -- no connection ever "
                       "established (ble_gap_connect started, BLE_GAP_EVENT_CONNECT never fired)");
    }
    s_active = false;
}

} // namespace BleCentralSpike
