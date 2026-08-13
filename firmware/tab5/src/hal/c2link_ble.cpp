#include "c2link_ble.h"
#include "hosted_link.h"
#include <Arduino.h> // Serial (boot-path logging) + FreeRTOS portMUX
#include <crypto.h>
#include <cstring>

extern "C" {
#include "esp32-hal-hosted.h" // hostedInitBLE(): enables the C6's BT controller
}

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
#include "host/ble_att.h"
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
// The central has to enable notifications on the Tx CCCD before
// ble_gatts_notify_custom actually delivers anything -- frames sent in the
// connect->subscribe window are silently dropped by the stack. Track that here
// (set from BLE_GAP_EVENT_SUBSCRIBE) so send()/is_connected() only report ready
// once notifications are actually flowing.
static volatile bool s_subscribed = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle = 0;
static uint8_t s_own_addr_type = 0;
// Task 19: millis() of the last frame dequeued and dispatched by poll() (main
// task). Set there rather than in rx_access_cb (the NimBLE host task, which
// runs on the other core) so it's only ever written from the same task that
// c2link_ble_last_recv_ms() is read from in main.cpp's loop() -- no portMUX
// needed for this one word, unlike s_rx_head/s_rx_tail above.
static uint32_t s_last_recv_ms = 0;
// Task 7: set from on_sync() below (NimBLE host task) and read by
// c2link_ble_host_synced() (main task) so central/observer-role features
// (BLE scan) know it's safe to call into the NimBLE host. volatile per the
// house rule below -- final whole-branch review finding M5 (2026-08-13)
// caught this one missing it, inconsistent with s_connected/s_subscribed
// just above in this same block.
static volatile bool s_host_synced = false;

// House rule this file has followed since Task 13 but never stated
// (whole-branch review finding M5, 2026-08-13, written here once so it
// stops drifting across features that copy this file's patterns): any
// scalar written on one task and read on another is `volatile`
// (s_connected, s_subscribed, s_host_synced above; s_rx_head/s_rx_tail
// below are also volatile for the same reason, on top of the portMUX).
// Anything multi-word or multi-field crossing that same boundary needs the
// portMUX_TYPE/portENTER_CRITICAL/portEXIT_CRITICAL pattern below, not just
// volatile -- volatile alone doesn't make a multi-field struct copy atomic
// (see ble_scan.cpp's s_devices for the real bug this distinction caught).

// Inbound frames are decoded on NimBLE's host task (in rx_access_cb) and handed
// to the main task via this single-producer/single-consumer ring, drained by
// poll(). This matches C2LinkWifi's contract: the receive handler runs on the
// main task, not a radio-stack task. The portMUX guards the head/tail indices
// against the concurrent host-task producer and main-task consumer (they run on
// different cores: NimBLE is pinned to core 0, Arduino loop to the other).
static constexpr size_t kRxQueueDepth = 4;
static c2proto::Frame s_rx_queue[kRxQueueDepth];
static volatile size_t s_rx_head = 0; // producer writes here (host task)
static volatile size_t s_rx_tail = 0; // consumer reads here (main task)
static portMUX_TYPE s_rx_mux = portMUX_INITIALIZER_UNLOCKED;

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

    // HMAC + decode happen here on the host task (both are pure functions), but
    // the decoded frame is queued for the main task rather than dispatched
    // directly -- the handler must run on the main task (see poll()).
    c2proto::Frame frame{};
    if (!c2proto::decode(buf, frame_len, frame)) return 0;

    portENTER_CRITICAL(&s_rx_mux);
    size_t next = (s_rx_head + 1) % kRxQueueDepth;
    bool full = (next == s_rx_tail);
    if (!full) {
        s_rx_queue[s_rx_head] = frame;
        s_rx_head = next;
    }
    portEXIT_CRITICAL(&s_rx_mux);
    if (full) {
        Serial.println("quarky-tab5: c2link_ble rx queue full, frame dropped");
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
        // Declare BOTH write-with-response (Write Request) and
        // write-without-response (Write Command). The Cardputer-ADV client's
        // C2LinkBle::send() issues an ATT Write Command (writeValue(...,
        // response=false)) for its reply direction; NimBLE-Arduino's client does
        // NOT verify the peer characteristic advertises WRITE_NO_RSP before
        // firing ble_gattc_write_no_rsp_flat(), and a GATT server silently drops
        // a Write Command to a characteristic missing this flag (Task 20 BLE
        // bug: RESP_TELEMETRY never arrived). Advertising WRITE_NO_RSP here makes
        // the server accept those Commands (rx_access_cb still fires with
        // BLE_GATT_ACCESS_OP_WRITE_CHR for both write kinds), fixing the reply
        // path without any client-side change. See task-20-report.md.
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
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
            s_subscribed = false; // central hasn't enabled the Tx CCCD yet
            Serial.printf("quarky-tab5: c2link_ble connected (handle=%u)\n",
                          event->connect.conn_handle);
        } else {
            Serial.printf("quarky-tab5: c2link_ble connect failed (status=%d)\n",
                          event->connect.status);
            start_advertising(); // failed connection attempt -- keep advertising
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        Serial.printf("quarky-tab5: c2link_ble disconnected (reason=%d)\n",
                      event->disconnect.reason);
        s_connected = false;
        s_subscribed = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        start_advertising(); // resume so a dropped link can reconnect
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        // Fires when the central writes the Tx characteristic's CCCD. Only the
        // Tx characteristic's notify subscription gates our send readiness.
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            s_subscribed = event->subscribe.cur_notify;
            Serial.printf("quarky-tab5: c2link_ble notify %s\n",
                          s_subscribed ? "subscribed" : "unsubscribed");
        }
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
    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble adv_set_fields failed (rc=%d)\n", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (const uint8_t *)s_device_name;
    rsp_fields.name_len = (uint8_t)strlen(s_device_name);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble adv_rsp_set_fields failed (rc=%d)\n", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble adv_start failed (rc=%d)\n", rc);
        return;
    }
    Serial.printf("quarky-tab5: c2link_ble advertising as \"%s\"\n", s_device_name);
}

// Fires once the host and (remote C6) controller are synced and ready. This is
// the real "BLE came up" signal -- init() returns before this runs, so the
// serial log here is what the deferred hardware-verification step looks for.
static void on_sync() {
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble id_infer_auto failed (rc=%d)\n", rc);
        return;
    }
    start_advertising();
    s_host_synced = true;
}

static void host_task(void *) {
    nimble_port_run(); // blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

bool C2LinkBle::init(const uint8_t psk[16], const char *device_name) {
    memcpy(s_psk, psk, 16);
    strncpy(s_device_name, device_name, sizeof(s_device_name) - 1);
    s_device_name[sizeof(s_device_name) - 1] = '\0';

    // BLE on this board is proxied to the ESP32-C6 over the same esp-hosted
    // SDIO transport WiFi uses, so it cannot come up if that link is down.
    // Bail out here rather than letting nimble_port_init() discover it: the
    // NimBLE stack retries HCI bring-up internally and would reproduce the
    // reset storm this guard exists to stop. See hosted_link.h.
    if (!hosted_link::begin()) {
        Serial.println("quarky-tab5: c2link_ble init skipped, C6 link down");
        return false;
    }

    // Explicitly enable the co-processor's BT controller BEFORE nimble_port_init().
    // This mirrors what arduino-esp32's own BLEDevice::init() does under
    // CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE (libraries/BLE/src/BLEDevice.cpp), and
    // was missing here. Previously this file relied on the esp-hosted LL
    // transport self-initialising from inside NimBLE -- which works only if
    // something else (the WiFi path) had already brought esp-hosted up first,
    // and would otherwise initialise the transport from the sdkconfig defaults
    // (CONFIG_ESP_HOSTED_SDIO_PIN_* = the generic EV-board's WRONG pins,
    // 18/19/14/15/16/17/54) instead of the Tab5 pins. hostedInitBLE() is
    // idempotent w.r.t. the shared transport: hosted_link::begin() already
    // established it, so this only adds the BT-controller enable on top.
    if (!hostedInitBLE()) {
        Serial.println("quarky-tab5: c2link_ble hostedInitBLE failed "
                       "(C6 BT controller did not come up)");
        return false;
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        Serial.printf("quarky-tab5: c2link_ble nimble_port_init failed (err=0x%x)\n", err);
        return false;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_svc_gap_device_name_set(s_device_name);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble device_name_set failed (rc=%d)\n", rc);
        return false;
    }

    rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble gatts_count_cfg failed (rc=%d)\n", rc);
        return false;
    }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        Serial.printf("quarky-tab5: c2link_ble gatts_add_svcs failed (rc=%d)\n", rc);
        return false;
    }

    ble_hs_cfg.sync_cb = on_sync;

    nimble_port_freertos_init(host_task);
    // NOTE: this returns as soon as the host task is created. Actual BLE
    // bring-up (controller sync, advertising start) happens asynchronously in
    // on_sync() -- watch the serial log for "advertising as ..." to confirm it,
    // since a true return here only means the host task was started.
    return true;
}

bool C2LinkBle::send(const c2proto::Frame &frame) {
    if (!s_connected || !s_subscribed || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return false;
    }

    uint8_t frame_buf[sizeof(c2proto::WireHeader) + c2proto::kMaxPayload];
    int n = c2proto::encode(frame, frame_buf, sizeof(frame_buf));
    if (n < 0) return false;

    // A single notification cannot exceed (ATT_MTU - 3) bytes; the stack
    // silently truncates anything larger AND still returns success, so the peer
    // would receive a mangled frame while we reported OK. Before MTU exchange
    // the MTU is the 23-byte default (20 usable), which even a zero-payload
    // frame (10B header + 32B HMAC) overruns. Refuse rather than lie.
    uint16_t mtu = ble_att_mtu(s_conn_handle);
    if (mtu == 0 || (size_t)(n + 32) > (size_t)(mtu - 3)) {
        Serial.printf("quarky-tab5: c2link_ble send skipped, frame %d+32 > mtu %u-3\n",
                      n, mtu);
        return false;
    }

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
    // "Ready to carry frames" means both connected AND the peer has enabled
    // notifications -- otherwise send() can't actually deliver anything.
    return s_connected && s_subscribed;
}

void C2LinkBle::poll() {
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

uint32_t c2link_ble_last_recv_ms() {
    return s_last_recv_ms;
}

bool c2link_ble_host_synced() {
    return s_host_synced;
}
