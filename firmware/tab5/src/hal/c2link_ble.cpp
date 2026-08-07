#include "c2link_ble.h"
#include <crypto.h>
#include <cstring>

// -----------------------------------------------------------------------------
// Why this is written against the ESP-IDF NimBLE C API, not NimBLE-Arduino
// -----------------------------------------------------------------------------
// The ESP32-P4 has no on-chip BT radio; BLE (like WiFi, see Task 9) is proxied
// to the onboard ESP32-C6 co-processor via esp-hosted over SDIO. The standard
// h2zero/NimBLE-Arduino PlatformIO library CANNOT compile for esp32p4: its
// NimBLEDevice::init() unconditionally calls the classic esp_bt_controller_*
// local-radio bring-up API from esp_bt.h, a header that does not exist for
// this chip. That was confirmed upstream (h2zero/NimBLE-Arduino#906, maintainer
// says "use esp-nimble-cpp and esp-idf") and is documented in the first
// (BLOCKED) section of task-13-report.md.
//
// The resolution found in this pass: the pioarduino Arduino-ESP32 v3.3.11
// framework is itself built as an ESP-IDF component, and for esp32p4 it already
// ships the full ESP-IDF NimBLE host stack, prebuilt and linkable:
//   * sdkconfig has CONFIG_BT_NIMBLE_ENABLED=y and CONFIG_BT_CONTROLLER_DISABLED=y
//     (host on, local controller off -- exactly the P4/esp-hosted config).
//   * libbt.a exports nimble_port_init / ble_hs_init / ble_gatts_* / ble_gap_*
//     (verified via nm), and its include tree is already on the compiler
//     CPPPATH (pioarduino-build.py lines ~360-387).
//   * libespressif__esp_hosted.a provides ble_transport_ll_init / _deinit --
//     NimBLE's lower-layer transport hooks -- routing HCI to the C6.
//   * -lbt and -lespressif__esp_hosted are already in the framework's ld_libs.
// So nimble_port_init() brings up the host and, through the esp-hosted LL
// transport, the remote C6 controller -- WITHOUT ever touching the classic
// esp_bt_controller_* API that breaks NimBLE-Arduino. This is the same
// initialization path esp-nimble-cpp uses for the P4; we call the underlying
// ESP-IDF C API directly rather than pulling in a second wrapper library that
// would conflict with the Arduino core. See task-13-report.md (2026-08-07).

extern "C" {
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_id.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
}

// Nordic UART Service UUIDs -- a de facto standard for exactly this
// "bidirectional byte pipe over GATT" shape, reused rather than inventing a
// custom service so any BLE debugging tool that already knows NUS works against
// this link for free. NimBLE stores 128-bit UUIDs least-significant-byte
// first, so these byte arrays are the reverse of the textual UUID:
//   service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   rx      6E400002-... (write:  peer -> Tab5)
//   tx      6E400003-... (notify: Tab5 -> peer)
static const ble_uuid128_t kServiceUUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t kRxCharUUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t kTxCharUUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint8_t s_psk[16];
static char s_device_name[32];
static C2LinkReceiveHandler s_handler = nullptr;
static volatile bool s_connected = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle = 0;
static uint8_t s_own_addr_type = 0;

static void start_advertising();

// GATT access callback for the write (Rx) characteristic: peer -> Tab5.
// Payload is [c2proto frame][32-byte HMAC-SHA256 trailer], mirroring the WiFi
// transport's on-wire format (Task 11).
static int rx_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload + 32];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (len < 32) return 0; // must at least hold a HMAC trailer

    size_t frame_len = (size_t)len - 32;
    if (!c2proto::hmac_verify(s_psk, 16, buf, frame_len, buf + frame_len)) return 0;

    c2proto::Frame frame{};
    if (c2proto::decode(buf, frame_len, frame) && s_handler) {
        s_handler(frame);
    }
    return 0;
}

// The Tx characteristic is notify-only; its access_cb is never invoked for
// read/write (no READ/WRITE flag set), but NimBLE wants a non-null pointer.
static int tx_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *, void *) {
    return 0;
}

static const struct ble_gatt_chr_def s_chrs[] = {
    {
        .uuid = &kRxCharUUID.u,
        .access_cb = rx_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
    },
    {
        .uuid = &kTxCharUUID.u,
        .access_cb = tx_access_cb,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_val_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kServiceUUID.u,
        .characteristics = s_chrs,
    },
    {0},
};

static int gap_event_cb(struct ble_gap_event *event, void *) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
        } else {
            start_advertising(); // failed connection attempt -- keep advertising
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        start_advertising(); // resume so a dropped link can reconnect
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;
    default:
        return 0;
    }
}

static void start_advertising() {
    // Advertisement payload: flags (3B) + the 128-bit service UUID (18B) = 21B,
    // fits inside the 31-byte limit. The device name is put in the scan
    // response instead, since name + 128-bit UUID together would overflow.
    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128 = &kServiceUUID;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;
    if (ble_gap_adv_set_fields(&adv_fields) != 0) return;

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (const uint8_t *)s_device_name;
    rsp_fields.name_len = (uint8_t)strlen(s_device_name);
    rsp_fields.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp_fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_cb, NULL);
}

// Fires once the host and (remote C6) controller are synced and ready.
static void on_sync() {
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) return;
    start_advertising();
}

static void host_task(void *) {
    nimble_port_run(); // blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

bool C2LinkBle::init(const uint8_t psk[16], const char *device_name) {
    memcpy(s_psk, psk, 16);
    strncpy(s_device_name, device_name, sizeof(s_device_name) - 1);
    s_device_name[sizeof(s_device_name) - 1] = '\0';

    if (nimble_port_init() != ESP_OK) return false;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_svc_gap_device_name_set(s_device_name) != 0) return false;

    if (ble_gatts_count_cfg(s_svcs) != 0) return false;
    if (ble_gatts_add_svcs(s_svcs) != 0) return false;

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);
    return true;
}

bool C2LinkBle::send(const c2proto::Frame &frame) {
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;

    uint8_t frame_buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload];
    int n = c2proto::encode(frame, frame_buf, sizeof(frame_buf));
    if (n < 0) return false;

    uint8_t mac[32];
    c2proto::hmac_sha256(s_psk, 16, frame_buf, (size_t)n, mac);

    uint8_t out[sizeof(frame_buf) + 32];
    memcpy(out, frame_buf, n);
    memcpy(out + n, mac, 32);

    // ble_gatts_notify_custom takes ownership of the mbuf and frees it.
    struct os_mbuf *om = ble_hs_mbuf_from_flat(out, (uint16_t)(n + 32));
    if (om == nullptr) return false;

    return ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om) == 0;
}

void C2LinkBle::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}

bool C2LinkBle::is_connected() {
    return s_connected;
}
