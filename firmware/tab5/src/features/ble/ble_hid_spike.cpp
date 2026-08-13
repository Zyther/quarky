#include "ble_hid_spike.h"
#include "../../hal/c2link_ble.h" // c2link_ble_host_synced()
#include <Arduino.h>              // Serial + delay()
#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h> // ble_gatts_* (server-side) live here, alongside the
                            // ble_gattc_* client API -- see ble_central_spike.cpp
#include <host/ble_att.h>  // BLE_ATT_F_READ (descriptor permissions)
#include <host/ble_hs_mbuf.h>
#include <host/ble_store.h> // ble_store_util_delete_peer() (repeat-pairing path)
#include <host/ble_uuid.h>
#include <os/os_mbuf.h> // os_mbuf_append()
#include <services/gap/ble_svc_gap.h>
#include <cstring>

// -----------------------------------------------------------------------------
// Reuses the ONE NimBLE host hal/c2link_ble.cpp brought up at boot (Global
// Constraint: raw ESP-IDF NimBLE, never NimBLE-Arduino; and never a second
// host). This file only adds its own GATT service and its own advertisement on
// top of that already-running host -- it never calls nimble_port_init().
//
// SECOND DISCLOSED SIDE EFFECT (beyond the advertisement takeover documented in
// ble_hid_spike.h): NimBLE's ble_gatts_add_svcs() only QUEUES a service; queued
// services are not actually in the ATT database until ble_gatts_start() runs.
// The host already started at boot, so this spike must call ble_gatts_start()
// itself -- and that call resets and re-registers the WHOLE local GATT server,
// c2link_ble's Nordic-UART service included. That is safe here (every service
// def in this project is a file-scope static whose val_handle pointers are
// simply refilled by the re-registration) but it is a real, global action, it
// requires no peer to be connected and no GAP procedure in flight, and it is
// the reason start() stops advertising before touching the server.
//
// The brief's sketch for this task called ble_gatts_count_cfg() +
// ble_gatts_add_svcs() with no ble_gatts_start(), which would have left the HID
// service permanently absent from the ATT database -- a guaranteed FALSE
// NEGATIVE for a spike whose entire question is "does a host see our HID
// service". Deviation made deliberately; see task-2-report.md.
//
// Also note ESP-IDF's own NimBLE HID service helper (services/hid/ble_svc_hid.h)
// is NOT usable here: the header ships, but libbt.a's ble_svc_hid.c.obj exports
// no symbols at all in the esp32p4 build (MYNEWT_VAL(BLE_SVC_HID) is off), so
// the service is hand-rolled below.
// -----------------------------------------------------------------------------

namespace BleHidSpike {

// Standard USB HID boot-keyboard report descriptor. This is not
// project-specific data -- it is the boot-keyboard report format from the USB
// HID Usage Tables spec, byte-for-byte the descriptor every generic BLE
// keyboard uses. It defines an 8-byte input report
// ([modifier, reserved, key1..key6]) and a 1-byte LED output report.
static const uint8_t kReportMap[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
    0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07,
    0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0
};

// HID Information (0x2A4A): bcdHID = 0x0111 (HID 1.11, little-endian),
// bCountryCode = 0 (not localized), flags = 0x02 (NormallyConnectable).
static const uint8_t kHidInfo[4] = {0x11, 0x01, 0x00, 0x02};

// Report Reference (0x2908) for the input report: report ID 0, type 1 = Input.
static const uint8_t kReportRef[2] = {0x00, 0x01};

static const char kDeviceName[] = "QuarkyKB";

// Written on the NimBLE host task, read on the main task -- single-word
// scalars crossing a real task boundary, so volatile, per the house rule
// stated in c2link_ble.cpp (and followed by ble_central_spike.cpp).
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_notify_enabled = false;

// Main-task-only state: both are read/written exclusively from start()/stop(),
// which only ever run from main.cpp's serial-trigger path.
static bool s_svc_registered = false;
static bool s_advertising = false;
static char s_prev_device_name[32] = {0};

// Filled in by ble_gatts_start()'s registration pass.
static uint16_t s_report_val_handle = 0;

// Report Protocol (1) vs Boot Protocol (0). We only implement Report Protocol;
// the characteristic exists because hosts read it, and a write is accepted and
// ignored rather than rejected.
static uint8_t s_protocol_mode = 1;

// ---- GATT access callbacks (all run on the NimBLE host task) ---------------

static int report_map_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    Serial.println("quarky-tab5: [ble-hid-spike] host read the Report Map");
    return os_mbuf_append(ctxt->om, kReportMap, sizeof(kReportMap)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int report_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    // Input Report: the host may read the current report state (always "no
    // keys pressed" at rest); real keystrokes are pushed via notify instead.
    static const uint8_t kIdle[8] = {0};
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, kIdle, sizeof(kIdle)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Without this descriptor a host cannot map the notified 8 bytes back to the
// keyboard collection in the Report Map, and most HID stacks will refuse to
// bind a keyboard driver at all.
static int report_ref_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, kReportRef, sizeof(kReportRef)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_info_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, kHidInfo, sizeof(kHidInfo)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int protocol_mode_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return os_mbuf_append(ctxt->om, &s_protocol_mode, 1) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t mode = 0;
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, &mode, sizeof(mode), &len) == 0 && len == 1) {
            Serial.printf("quarky-tab5: [ble-hid-spike] host set protocol mode=%u "
                          "(0=boot, 1=report)\n", mode);
            s_protocol_mode = mode;
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// HID Control Point: hosts write 0x00 (suspend) / 0x01 (exit suspend). Nothing
// to do, but the characteristic must exist and accept the write.
static int ctrl_pt_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    return 0;
}

// ---- GATT service definition -----------------------------------------------
// Designated initializers must appear in declaration order in C++, and
// ble_gatt_chr_def declares uuid, access_cb, arg, descriptors, flags,
// min_key_size, val_handle -- so .val_handle comes AFTER .flags, not before it
// (the brief's sketch had those two swapped, which does not compile).
// ble_gatt_dsc_def's `descriptors` field is non-const, so s_report_dscs cannot
// be const either.

// 16-bit SIG UUIDs, as named statics rather than BLE_UUID16_DECLARE(...):
// that macro expands to a C compound literal, which is a GNU extension in C++
// and cannot have its address taken in a static initializer ("taking address of
// rvalue"). c2link_ble.cpp already takes this approach for its 128-bit UUIDs.
static const ble_uuid16_t kUuidHidSvc      = BLE_UUID16_INIT(0x1812); // Human Interface Device
static const ble_uuid16_t kUuidHidInfo     = BLE_UUID16_INIT(0x2A4A); // HID Information
static const ble_uuid16_t kUuidReportMap   = BLE_UUID16_INIT(0x2A4B); // Report Map
static const ble_uuid16_t kUuidCtrlPoint   = BLE_UUID16_INIT(0x2A4C); // HID Control Point
static const ble_uuid16_t kUuidReport      = BLE_UUID16_INIT(0x2A4D); // Report
static const ble_uuid16_t kUuidProtoMode   = BLE_UUID16_INIT(0x2A4E); // Protocol Mode
static const ble_uuid16_t kUuidReportRef   = BLE_UUID16_INIT(0x2908); // Report Reference (descriptor)

static struct ble_gatt_dsc_def s_report_dscs[] = {
    {
        .uuid = &kUuidReportRef.u, // Report Reference
        .att_flags = BLE_ATT_F_READ,
        .min_key_size = 0,
        .access_cb = report_ref_access_cb,
        .arg = nullptr,
    },
    {0},
};

static const struct ble_gatt_chr_def s_hid_chrs[] = {
    {
        .uuid = &kUuidProtoMode.u, // Protocol Mode
        .access_cb = protocol_mode_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &kUuidReportMap.u, // Report Map
        .access_cb = report_map_access_cb,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &kUuidReport.u, // Report (Input)
        .access_cb = report_access_cb,
        .descriptors = s_report_dscs,
        // NOTIFY makes NimBLE add the CCCD automatically (it must NOT be listed
        // in s_report_dscs -- see ble_gatt_chr_def's own doc comment).
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_report_val_handle,
    },
    {
        .uuid = &kUuidHidInfo.u, // HID Information
        .access_cb = hid_info_access_cb,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &kUuidCtrlPoint.u, // HID Control Point
        .access_cb = ctrl_pt_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {0},
};

static const struct ble_gatt_svc_def s_hid_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kUuidHidSvc.u, // Human Interface Device
        .characteristics = s_hid_chrs,
    },
    {0},
};

// ---- Advertising + GAP ------------------------------------------------------

static void start_advertising();

static int gap_event_cb(struct ble_gap_event *event, void *) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_notify_enabled = false; // host has not written the report CCCD yet
            Serial.printf("quarky-tab5: [ble-hid-spike] host CONNECTED (handle=%u)\n",
                          event->connect.conn_handle);
        } else {
            Serial.printf("quarky-tab5: [ble-hid-spike] connect failed (status=%d)\n",
                          event->connect.status);
            start_advertising(); // stay discoverable after a failed attempt
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        Serial.printf("quarky-tab5: [ble-hid-spike] host disconnected (reason=%d)\n",
                      event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        start_advertising(); // let the host reconnect without a re-trigger
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        // The single most informative event in this whole spike: a host that
        // has actually bound a keyboard driver enables notifications on the
        // input report. If this never fires, the OS did not accept us as a HID
        // device, and no keystroke can possibly be delivered.
        if (event->subscribe.attr_handle == s_report_val_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
            Serial.printf("quarky-tab5: [ble-hid-spike] host %s input-report notifications\n",
                          s_notify_enabled ? "ENABLED" : "disabled");
        }
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE:
        // HID hosts encrypt (and usually bond) before using the profile.
        Serial.printf("quarky-tab5: [ble-hid-spike] encryption change status=%d\n",
                      event->enc_change.status);
        return 0;
    case BLE_GAP_EVENT_MTU:
        Serial.printf("quarky-tab5: [ble-hid-spike] MTU now %u\n", event->mtu.value);
        return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // A host that bonded during an earlier run will re-pair on reconnect.
        // Dropping the stale bond and retrying turns an otherwise-confusing
        // silent failure into a normal pairing.
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        Serial.println("quarky-tab5: [ble-hid-spike] repeat pairing -- dropped stale bond, retrying");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    default:
        return 0;
    }
}

static void start_advertising() {
    // Budget check against the 31-byte legacy advertisement limit:
    //   flags 3 + appearance 4 + one 16-bit UUID 4 + "QuarkyKB" 10 = 21 bytes.
    // The 0x1812 UUID is what makes an OS classify the device as HID BEFORE
    // connecting; appearance alone is a hint, not a classification, so both are
    // in the primary advertisement rather than a scan response.
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16 = &kUuidHidSvc;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.appearance = 0x03C1; // Keyboard (Human Interface Device category)
    fields.appearance_is_present = 1;
    fields.name = (const uint8_t *)kDeviceName;
    fields.name_len = (uint8_t)(sizeof(kDeviceName) - 1);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] adv_set_fields rc=%d\n", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable, undirected
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_start rc=%d%s\n", rc,
                  rc == 0 ? " -- advertising as \"QuarkyKB\"" : "");
}

// ---- Public API -------------------------------------------------------------

void start() {
    if (!c2link_ble_host_synced()) {
        Serial.println("quarky-tab5: [ble-hid-spike] NimBLE host not synced "
                       "(C6 link down, or c2link_ble init failed) -- cannot start");
        return;
    }
    if (s_advertising) {
        Serial.println("quarky-tab5: [ble-hid-spike] already started");
        return;
    }
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        Serial.println("quarky-tab5: [ble-hid-spike] a host is already connected");
        return;
    }

    // Stop whatever is currently advertising -- c2link_ble's C2 advertisement
    // at minimum. Required twice over: legacy advertising is single-instance,
    // and ble_gatts_start() below refuses to run (BLE_HS_EBUSY) while any GAP
    // procedure is in flight.
    int rc = ble_gap_adv_stop();
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_stop rc=%d "
                  "(c2link_ble's C2 advertisement is now down for this boot)\n", rc);

    if (!s_svc_registered) {
        // Snapshot the GAP device name so stop() can put it back -- the
        // characteristic is global to the host, shared with c2link_ble.
        const char *prev = ble_svc_gap_device_name();
        if (prev != nullptr) {
            strncpy(s_prev_device_name, prev, sizeof(s_prev_device_name) - 1);
            s_prev_device_name[sizeof(s_prev_device_name) - 1] = '\0';
        }
        rc = ble_svc_gap_device_name_set(kDeviceName);
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-hid-spike] device_name_set rc=%d\n", rc);
        }

        rc = ble_gatts_count_cfg(s_hid_svcs);
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-hid-spike] gatts_count_cfg rc=%d\n", rc);
            return;
        }
        rc = ble_gatts_add_svcs(s_hid_svcs);
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-hid-spike] gatts_add_svcs rc=%d\n", rc);
            return;
        }
        // Queued != registered. This is what actually puts the HID service in
        // the ATT database, and it re-registers every other local service too
        // (see the file-level comment). BLE_HS_EBUSY here means something was
        // still connected or a GAP procedure was still running.
        rc = ble_gatts_start();
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-hid-spike] ble_gatts_start rc=%d "
                          "-- HID service NOT registered%s\n", rc,
                          rc == BLE_HS_EBUSY ? " (BLE_HS_EBUSY: a peer is connected "
                                               "or a GAP procedure is active)" : "");
            return;
        }
        s_svc_registered = true;
        Serial.printf("quarky-tab5: [ble-hid-spike] HID service registered, "
                      "input report value handle=%u (CCCD at %u)\n",
                      s_report_val_handle, (unsigned)(s_report_val_handle + 1));
    }

    start_advertising();
    s_advertising = true;
}

void send_test_keystroke() {
    uint16_t conn = s_conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        Serial.println("quarky-tab5: [ble-hid-spike] no host connected -- pair from the "
                       "host's Bluetooth settings first");
        return;
    }
    if (!s_notify_enabled) {
        // Refuse rather than lie: ble_gatts_notify_custom() returns 0 for an
        // unsubscribed characteristic, so sending here would log rc=0 while
        // delivering nothing at all.
        Serial.println("quarky-tab5: [ble-hid-spike] host has NOT enabled input-report "
                       "notifications -- it did not bind a keyboard driver; not sending "
                       "(a notify here would silently succeed and deliver nothing)");
        return;
    }

    // 'a' = USB HID keyboard usage 0x04. Report layout is the 8-byte boot
    // keyboard report the Report Map above declares:
    // [modifier, reserved, key1..key6].
    const uint8_t down[8] = {0, 0, 0x04, 0, 0, 0, 0, 0};
    const uint8_t up[8] = {0};

    struct os_mbuf *om = ble_hs_mbuf_from_flat(down, sizeof(down));
    if (om == nullptr) {
        Serial.println("quarky-tab5: [ble-hid-spike] mbuf alloc failed (key-down)");
        return;
    }
    int rc = ble_gatts_notify_custom(conn, s_report_val_handle, om);
    Serial.printf("quarky-tab5: [ble-hid-spike] key-down notify rc=%d\n", rc);

    // A real host needs a measurable gap to register a distinct press. 50ms is
    // far under the ~5s loop watchdog budget, and this only ever runs from the
    // serial-debug trigger, not from a polled path.
    delay(50);

    om = ble_hs_mbuf_from_flat(up, sizeof(up));
    if (om == nullptr) {
        Serial.println("quarky-tab5: [ble-hid-spike] mbuf alloc failed (key-up) -- "
                       "the key is now stuck down on the host");
        return;
    }
    rc = ble_gatts_notify_custom(conn, s_report_val_handle, om);
    Serial.printf("quarky-tab5: [ble-hid-spike] key-up notify rc=%d\n", rc);
}

void stop() {
    int rc = ble_gap_adv_stop();
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_stop rc=%d\n", rc);
    s_advertising = false;

    uint16_t conn = s_conn_handle;
    if (conn != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }

    // Put the GAP device name back. The HID service itself stays registered --
    // removing it would mean another full ble_gatts_start() cycle, and leaving
    // it costs nothing while nothing advertises it.
    if (s_prev_device_name[0] != '\0') {
        ble_svc_gap_device_name_set(s_prev_device_name);
    }
}

} // namespace BleHidSpike
