#include "ble_hid_spike.h"
#include "ble_common.h" // ble_addr_to_str() -- real-hardware verification diagnostic
#include "../../hal/c2link_ble.h" // c2link_ble_host_synced()
#include <Arduino.h>              // Serial + delay()
#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h> // ble_gatts_* (server-side) live here, alongside the
                            // ble_gattc_* client API -- see ble_central_spike.cpp
#include <host/ble_att.h>  // BLE_ATT_F_READ (descriptor permissions)
#include <host/ble_hs_mbuf.h>
#include <host/ble_store.h> // ble_store_util_delete_peer() (repeat-pairing path)
#include <host/ble_sm.h>    // BLE_SM_IO_CAP_NO_IO, BLE_SM_ERR_* (real-hardware pairing fix)
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
// WHERE THE SERVICE GETS REGISTERED, and why it is not here in start().
//
// NimBLE's ble_gatts_add_svcs() only QUEUES a service definition; the queue is
// drained exactly once, by the ble_gatts_start() that ble_hs_start() runs
// automatically when the host task comes up. So the brief's sketch (count_cfg +
// add_svcs from a runtime trigger, no start) would have left the HID service
// permanently absent from the ATT database -- a guaranteed FALSE NEGATIVE for a
// spike whose entire question is "does a host see our HID service".
//
// The first fix for that -- calling ble_gatts_start() from start() -- was WORSE,
// and is the mistake this file most wants to stop anyone repeating.
// ble_gatts_start() does not re-register the other services: it drains a queue
// that by then contains only the newly-added one, while ble_att_svr_start() ->
// ble_att_svr_free_start_mem() frees the heap block every already-registered ATT
// attribute (GAP 0x1800, GATT 0x1801, c2link_ble's NUS) lives in. ble_att_svr_list
// is never cleared, so it is left holding ~15 dangling pointers -- which
// ble_gatts_start()'s own CCCD-cache loop then walks and dereferences before it
// even returns. Use-after-free of the live GATT server, verified against the
// shipped esp32p4 libbt.a by disassembly (task-2-review.md finding C1), not
// inferred from headers.
//
// So registration happens at BOOT instead, via register_service() installed as a
// c2link_ble GATT hook from main.cpp -- queued alongside c2link_ble's own service
// and registered by the same single automatic ble_gatts_start(). start() only
// advertises. Nothing in this file calls ble_gatts_start(); nothing should.
//
// Beyond the advertisement takeover documented in ble_hid_spike.h, that leaves
// this spike with no side effects on c2link_ble's GATT server at all.
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
// "The spike is armed", not "a packet is going out right now": true from a
// successful start() until stop(), including while a host is connected (a
// connectable-undirected advertisement ends on connect, and the disconnect
// handler re-arms it). Crosses the boundary the other way -- written on the
// main task in start()/stop(), read on the host task in gap_event_cb to decide
// whether a disconnect should resume advertising -- so volatile for the same
// reason. Without that read, stop() would not stop: ble_gap_terminate()'s
// asynchronous BLE_GAP_EVENT_DISCONNECT used to re-advertise unconditionally,
// putting "QuarkyKB" back on the air milliseconds after stop() returned
// (task-2-review.md finding I1).
static volatile bool s_advertising = false;

// Main-task-only: read/written exclusively from start()/stop(), which only ever
// run from main.cpp's serial-trigger path.
static bool s_svc_queued = false;
static char s_prev_device_name[32] = {0};

// Neither is volatile, unlike the scalars above -- deliberate, safe by
// publication order rather than by luck, but the two run in OPPOSITE
// directions and shouldn't be read as a matched pair:
//   - s_report_val_handle is written on the NimBLE HOST task, during the
//     boot-time GATT registration (ble_gatts_start(), which the automatic
//     ble_hs_start() runs before ble_hs_sync() fires) and read on the MAIN
//     task, in start()/send_test_keystroke(). Safe because every reader
//     gates on c2link_ble_host_synced() (volatile bool, set only after
//     on_sync() runs, which is itself only after ble_gatts_start() returns)
//     -- so a read can never observe the handle before it's been written.
//     It also cannot be volatile even if we wanted it to be:
//     ble_gatt_chr_def::val_handle is a plain uint16_t*, which a
//     volatile uint16_t* does not convert to.
//   - s_own_addr_type is written on the MAIN task, in start(), and read on
//     the HOST task, from gap_event_cb/start_advertising -- which cannot
//     run until after that same start() call has put an advertisement on
//     the air, so the write always precedes any read.
// (task-2-review.md re-review finding N1, 2026-08-14: an earlier version of
// this comment attributed both variables' write to the main task, which was
// true for s_report_val_handle before this file's C1 fix moved GATT
// registration to boot time -- flagged because Task 15's Bad-KB feature is
// expected to copy this file's pattern.)
static uint16_t s_report_val_handle = 0;
static uint8_t s_own_addr_type = 0;

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
        // BLE_GATT_CHR_F_READ_ENC: real-hardware finding (2026-08-14). Without
        // this, nothing in the GATT permission model tells a central it needs
        // to pair before touching this service -- macOS connected, read the
        // Report Map, and enabled notifications entirely over an
        // UNENCRYPTED link (confirmed: no BLE_GAP_EVENT_ENC_CHANGE ever
        // fired), and then silently declined to actually inject the
        // keystroke. The HID-over-GATT profile (HOGP) requires the Report
        // Map and Report characteristics to be read-encrypted for exactly
        // this reason -- without it, a host has no signal to initiate
        // Security Manager pairing at all.
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {
        .uuid = &kUuidReport.u, // Report (Input)
        .access_cb = report_access_cb,
        .descriptors = s_report_dscs,
        // NOTIFY makes NimBLE add the CCCD automatically (it must NOT be listed
        // in s_report_dscs -- see ble_gatt_chr_def's own doc comment).
        // BLE_GATT_CHR_F_READ_ENC (read) + BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC
        // (CCCD write) -- same real-hardware finding as Report Map above.
        // NOTIFY_INDICATE_ENC is the flag that actually matters here: it
        // requires the link to be encrypted before a host can write the
        // auto-added CCCD, i.e. before it can enable notifications at all --
        // forcing pairing to happen before subscribe succeeds, rather than
        // subscribe silently succeeding over plaintext and the keystroke
        // then being dropped somewhere in the host's own HID input pipeline.
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC |
                 BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
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

static int start_advertising();

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
            // Stay discoverable after a failed attempt -- unless stop() has
            // already disarmed the spike (see s_advertising's comment).
            if (s_advertising) start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        Serial.printf("quarky-tab5: [ble-hid-spike] host disconnected (reason=%d)\n",
                      event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        // Let the host reconnect without a re-trigger -- but only if this
        // disconnect wasn't stop()'s own ble_gap_terminate(). stop() clears
        // s_advertising before terminating precisely so this check sees it.
        if (s_advertising) start_advertising();
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
        // Necessary, not merely defensive: this build has bonding ON
        // (MYNEWT_VAL_BLE_SM_BONDING defaults to 1) with NVS persistence
        // (CONFIG_BT_NIMBLE_NVS_PERSIST=1) and the bond store auto-installed by
        // ble_hs_startup_go() -> ble_store_config_init(), so a host that bonded
        // in an earlier run really does come back holding keys, across reboots.
        // Dropping the stale bond and retrying turns an otherwise-confusing
        // silent failure into a normal pairing. Note the store caps at 3 bonds
        // (CONFIG_BT_NIMBLE_MAX_BONDS=3) -- repeated spike runs against several
        // hosts can fill it.
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

// Returns the ble_gap_adv_start() return code (or the earlier failure's), so
// start() can keep s_advertising honest instead of assuming success.
static int start_advertising() {
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
        return rc;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable, undirected
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // s_own_addr_type comes from ble_hs_id_infer_auto() in start(), matching
    // c2link_ble.cpp's on_sync() -- the only advertising path this project has
    // ever proven on hardware. A hardcoded BLE_OWN_ADDR_PUBLIC (what the
    // brief's sketch used) returns BLE_HS_ENOADDR if the C6's controller has no
    // usable public identity address, and the spike would silently never
    // advertise: a false negative with nothing to do with HID, which is exactly
    // the failure class this task exists to avoid (review finding I2).
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_start rc=%d (own addr type=%u)%s\n",
                  rc, s_own_addr_type, rc == 0 ? " -- advertising as \"QuarkyKB\"" : "");

    // Real-hardware verification diagnostic: log the exact address this
    // advertises under, same reasoning as ble_spam.cpp's equivalent -- lets a
    // real scan result be matched to this device by MAC, independent of
    // whether the host OS's own Bluetooth UI chooses to surface it as a
    // keyboard (or at all).
    if (rc == 0) {
        uint8_t addr_val[6];
        if (ble_hs_id_copy_addr(s_own_addr_type == BLE_OWN_ADDR_PUBLIC ? BLE_ADDR_PUBLIC : BLE_ADDR_RANDOM,
                                 addr_val, nullptr) == 0) {
            char addr_str[18];
            ble_addr_to_str(addr_val, addr_str);
            Serial.printf("quarky-tab5: [ble-hid-spike] broadcasting under address %s\n", addr_str);
        }
    }
    return rc;
}

// ---- Public API -------------------------------------------------------------

void register_service() {
    // Real-hardware finding (2026-08-14): with encryption required on the
    // Report/Report-Map characteristics (added above, same finding), macOS
    // DID initiate real pairing -- BLE_GAP_EVENT_ENC_CHANGE fired, where
    // before it never did -- but the pairing itself failed with
    // BLE_HS_ERR_SM_US_BASE + BLE_SM_ERR_DHKEY (a local LE Secure
    // Connections DHKey-check failure). SC pairing does an ECDH public-key
    // exchange this device has no display/keyboard to confirm on anyway, so
    // rather than chase the DHKey failure itself, force legacy pairing
    // (no ECDH, no DHKey check at all) with "Just Works" -- Just Works is
    // the only honest choice given this device's actual IO capability (it
    // *emulates* a keyboard over BLE, it doesn't have a physical one a user
    // can type a passkey on). This is a global ble_hs_cfg change, shared
    // with c2link_ble's own service -- safe for it too, since its peer
    // (Cardputer-ADV) never initiates pairing today and NO_IO/no-SC only
    // changes what happens IF a peer starts a Security Manager procedure.
    // Set here (called before nimble_port_freertos_init(), i.e. before
    // ble_hs_start() and any pairing can possibly occur) for the same
    // "must land before the host task exists" timing reason GATT
    // registration does.
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;

    // Runs from c2link_ble.init(), on the main task, before the NimBLE host
    // task exists. count_cfg + add_svcs ONLY -- see the file-level comment for
    // what calling ble_gatts_start() here (or anywhere) would destroy.
    int rc = ble_gatts_count_cfg(s_hid_svcs);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] gatts_count_cfg rc=%d\n", rc);
        return;
    }
    rc = ble_gatts_add_svcs(s_hid_svcs);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] gatts_add_svcs rc=%d\n", rc);
        return;
    }
    s_svc_queued = true;
    Serial.println("quarky-tab5: [ble-hid-spike] HID service queued for boot-time registration");
}

void start() {
    if (!c2link_ble_host_synced()) {
        Serial.println("quarky-tab5: [ble-hid-spike] NimBLE host not synced "
                       "(C6 link down, or c2link_ble init failed) -- cannot start");
        return;
    }
    if (!s_svc_queued) {
        // register_service() never ran, so the HID service is not in the ATT
        // database and nothing can be registered now. Advertising anyway would
        // manufacture the exact false negative this spike must not produce.
        Serial.println("quarky-tab5: [ble-hid-spike] HID service was never registered "
                       "-- rebuild with -DQUARKY_SERIAL_DEBUG so main.cpp installs the "
                       "boot-time c2link_ble GATT hook; not advertising");
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

    // Legacy BLE advertising is single-instance system-wide, so take over from
    // c2link_ble's C2 advertisement. This is the spike's only remaining side
    // effect on c2link_ble; the GATT server is untouched.
    int rc = ble_gap_adv_stop();
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_stop rc=%d "
                  "(c2link_ble's C2 advertisement is now down for this boot)\n", rc);

    // Same async stop/set-data race documented in ble_sourapple.cpp's
    // send_one() (the original real-hardware discovery: the controller can
    // still be asynchronously processing the previous ble_gap_adv_stop()
    // when a "set advertisement data" call arrives immediately after, and
    // silently fails) and confirmed for real on this project's own hardware
    // in ble_spam.cpp's send_one_advertisement() (vendor-picker dropdown
    // appeared not to change the broadcast payload; root cause was this
    // exact race), then hit a second time as a preventive fix in
    // ble_karma.cpp's rotate_identity() (identical stop-then-set_fields
    // shape). start_advertising() below (~line 384) calls
    // ble_gap_adv_set_fields() after this stop, with only ble_hs_id_infer_auto()
    // and a device-name set in between -- neither is a guaranteed settling
    // delay, just incidental host-call time. This file's own real-hardware
    // pairing test already succeeded end-to-end (a test keystroke registered
    // on a real macOS host) without this delay, so unlike ble_spam.cpp this
    // is NOT a confirmed bug here -- it's a preventive fix closing a race
    // class this codebase has now proven real twice, before Task 15 (Bad-KB)
    // builds more code on top of this start() path where an intermittent
    // failure would be far harder to isolate. start() runs once per
    // user-triggered pairing session, not on a polling cadence, so 5ms here
    // is trivially negligible.
    delay(5);

    // Same derivation c2link_ble.cpp's on_sync() uses. Done here rather than
    // once at registration time because register_service() runs before the
    // controller has synced, when no identity address exists yet.
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] ble_hs_id_infer_auto rc=%d "
                      "-- no usable own address, not advertising\n", rc);
        return;
    }

    // Point the GAP Device Name characteristic at "QuarkyKB" too. Unconditional
    // (not gated on first-run state) so a stop() -> start() cycle cannot leave
    // the device advertising as "QuarkyKB" while 0x1800 still reads
    // c2link_ble's name -- the post-connect mismatch this call exists to
    // prevent (review finding M2). The strcmp guard keeps a repeated start()
    // from snapshotting our own name as the one to restore.
    const char *prev = ble_svc_gap_device_name();
    if (prev != nullptr && strcmp(prev, kDeviceName) != 0) {
        strncpy(s_prev_device_name, prev, sizeof(s_prev_device_name) - 1);
        s_prev_device_name[sizeof(s_prev_device_name) - 1] = '\0';
    }
    rc = ble_svc_gap_device_name_set(kDeviceName);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] device_name_set rc=%d\n", rc);
    }

    Serial.printf("quarky-tab5: [ble-hid-spike] input report value handle=%u (CCCD at %u)\n",
                  s_report_val_handle, (unsigned)(s_report_val_handle + 1));

    // Only claim to be armed if the radio actually accepted the advertisement.
    // Setting this unconditionally would both lie in the log and make the
    // "already started" check above block a legitimate retry (finding M1).
    s_advertising = (start_advertising() == 0);
}

void send_test_keystroke() {
    uint16_t conn = s_conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        Serial.println("quarky-tab5: [ble-hid-spike] no host connected -- pair from the "
                       "host's Bluetooth settings first");
        return;
    }
    if (!s_notify_enabled) {
        // Refuse rather than lie. ble_gatts_notify_custom() performs no CCCD or
        // subscription check whatsoever -- it hands the PDU to
        // ble_att_clt_tx_notify() unconditionally and returns 0, and an
        // unsubscribed host discards it on receipt. So sending here would log
        // rc=0 twice while the keystroke goes nowhere, turning the spike's
        // primary failure mode into an apparent success.
        Serial.println("quarky-tab5: [ble-hid-spike] host has NOT enabled input-report "
                       "notifications -- it did not bind a keyboard driver; not sending "
                       "(the notify would return rc=0 and the host would discard it)");
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

void send_key(uint8_t keycode) {
    uint16_t conn = s_conn_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE || keycode == 0) {
        // No host connected, or keycode_for()'s "unrecognized character"
        // sentinel (ble_bad_kb.cpp) -- nothing to send. Deliberately silent
        // (unlike send_test_keystroke()'s equivalent guard): this can be
        // called once per poll() tick for every character of a real script,
        // and an unrecognized character is an expected, disclosed outcome of
        // this reduced Ducky-script subset, not worth a log line per tick.
        return;
    }
    if (!s_notify_enabled) {
        Serial.println("quarky-tab5: [ble-hid-spike] host has NOT enabled input-report "
                       "notifications -- not sending (see send_test_keystroke()'s comment "
                       "for why this guard exists)");
        return;
    }

    // Report layout matches the 8-byte boot keyboard report the Report Map
    // above declares: [modifier, reserved, key1..key6].
    const uint8_t down[8] = {0, 0, keycode, 0, 0, 0, 0, 0};
    const uint8_t up[8] = {0};

    struct os_mbuf *om = ble_hs_mbuf_from_flat(down, sizeof(down));
    if (om == nullptr) {
        Serial.println("quarky-tab5: [ble-hid-spike] mbuf alloc failed (key-down)");
        return;
    }
    int rc = ble_gatts_notify_custom(conn, s_report_val_handle, om);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] key-down notify rc=%d (keycode=0x%02X)\n",
                      rc, keycode);
    }

    // ~20ms, not send_test_keystroke()'s 50ms -- see this function's header
    // comment (ble_hid_spike.h) for why: this runs once per loop() tick from
    // ble_bad_kb.cpp's poll(), so its total blocking time must stay well
    // under this project's ~50ms no-blocking-call-longer-than-this Global
    // Constraint, leaving headroom for that same tick's other poll() calls
    // and lvgl_port_tick().
    delay(20);

    om = ble_hs_mbuf_from_flat(up, sizeof(up));
    if (om == nullptr) {
        Serial.println("quarky-tab5: [ble-hid-spike] mbuf alloc failed (key-up) -- "
                       "the key is now stuck down on the host");
        return;
    }
    rc = ble_gatts_notify_custom(conn, s_report_val_handle, om);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-hid-spike] key-up notify rc=%d\n", rc);
    }
    delay(20);
}

bool is_connected() {
    // Single-word volatile scalar read on the main task, written on the
    // NimBLE host task in gap_event_cb -- same safe-by-existing-precedent
    // pattern send_test_keystroke() already uses for the same variable (see
    // the file-level comment's discussion of s_conn_handle/s_notify_enabled).
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

void stop() {
    // Disarm FIRST. gap_event_cb's disconnect path checks this flag, and the
    // ble_gap_terminate() below raises that event asynchronously -- clearing
    // the flag afterwards would let the spike re-advertise itself back to life
    // milliseconds after stop() returned (finding I1).
    s_advertising = false;

    int rc = ble_gap_adv_stop();
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_stop rc=%d\n", rc);

    uint16_t conn = s_conn_handle;
    if (conn != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }

    // Put the GAP device name back. The HID service stays in the ATT database
    // for the life of the boot -- there is no safe runtime way to remove it
    // (ble_gatts_reset()/ble_gatts_start() is the same hazard the file-level
    // comment describes). That is not free: ATT service discovery enumerates
    // the whole database regardless of what is being advertised, so any
    // connected central -- a real Cardputer-ADV C2 peer included -- can still
    // discover 0x1812 and subscribe to the input report. Acceptable for a
    // QUARKY_SERIAL_DEBUG-only spike; not something a shipped feature should
    // inherit uncritically.
    if (s_prev_device_name[0] != '\0') {
        ble_svc_gap_device_name_set(s_prev_device_name);
    }
}

} // namespace BleHidSpike
