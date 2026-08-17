#include "ble_whisperpair.h"
#include "ble_central.h"
#include "ble_scan.h" // BleScanFeature::first_device_addr()/first_device_addr_type()
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial / millis() / delay() -- the brief's sketch used
                      // Serial.printf() and millis() without including this,
                      // same as every other BLE feature file needs
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h> // ble_gattc_disc_svc_by_uuid / ble_gattc_disc_all_chrs /
                            // ble_gattc_disc_all_dscs / ble_gattc_write_flat, plus
                            // BLE_GATT_DSC_CLT_CFG_UUID16. NOT <host/ble_gattc.h>
                            // (Bug 1) -- no such header exists in the ESP-IDF
                            // NimBLE tree the P4 Arduino libs ship; the
                            // client-side ble_gattc_* API lives in ble_gatt.h
                            // alongside the server-side ble_gatts_* API. Same
                            // finding Tasks 1, 13, 14, 16 and 17 already made.
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <esp_random.h> // esp_fill_random()
#include <mbedtls/ecdh.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/aes.h>
#include <cstring>
#include <cstdio>

extern FeatureRegistry g_registry;

// -----------------------------------------------------------------------------
// WhisperPair, CVE-2025-36911 detector (second Phase 2 plan, Task 18).
//
// Donors, both read rather than assumed:
//   ~/src/poseidon-tab5/src/features/ble_whisperpair.cpp
//   ~/src/unigeek-main/firmware/src/screens/ble/WhisperPairScreen.cpp
// Both agree on the technique: connect, find Fast Pair 0xFE2C, find the KBP
// characteristic, SUBSCRIBE to its notify, write an encrypted 16-byte KBP
// request while the accessory is not in pairing mode, and wait ~2s. A notify
// means the accessory serviced a KBP request outside pairing mode; silence
// means it did not.
//
// This file ports UniGeek's simpler 16-byte variant (Poseidon's writes an
// 80-byte blob and wants a pre-baked anti-spoofing key file). What it does NOT
// port, disclosed rather than silently dropped: UniGeek ALSO tries to obtain
// the accessory's own 64-byte public key (a GATT read of the KBP
// characteristic, else 64 bytes out of the Fast Pair advertising service data)
// and, when it gets one, computes a genuine ECDH shared secret with
// mbedtls_ecdh_compute_shared(). Without a peer key it falls back to random
// key bytes -- which is functionally where this port always sits. The
// consequence is stated plainly in the UI and in build_probe(): the accessory
// cannot decrypt our probe, so this establishes only whether it RESPONDS to a
// KBP write outside pairing mode. That is the actual CVE question, and it is
// the whole claim this feature makes. Peer-key extraction is real, considered
// future work; it is deliberately out of this task's scope.
//
// THREADING (Bug 3). write_cb, cccd_write_cb, dsc_disc_cb, chr_disc_cb,
// svc_disc_cb and gap_event_cb ALL run on the NimBLE HOST task, not the
// main/LVGL task. This project's LVGL port has no OS/mutex integration
// (LV_USE_OS is LV_OS_NONE, see ui/lvgl_port.cpp), so none of them may call
// any lv_* function -- the brief's log_status() called lv_label_set_text()
// directly from four of them. Fixed with the shape ble_gatt_explorer.cpp
// (Task 13), ble_fastpair_exploit.cpp (Task 16) and ble_hfp_exploit.cpp
// (Task 17) already proved: host-task callbacks only ever write a formatted
// string into a portMUX-guarded buffer and set a dirty flag; poll(), on the
// main task, snapshots it under the lock and does the one
// lv_label_set_text() call. poll() also owns the two other things that must
// not happen on the host task: enabling the Send Probe button once the
// subscription lands, and expiring the notify wait window.
// -----------------------------------------------------------------------------

namespace BleWhisperPairFeature {

// Google Fast Pair service, assigned 16-bit UUID 0xFE2C.
static const ble_uuid16_t kFastPairSvcUuid = BLE_UUID16_INIT(0xFE2C);

// Fast Pair Key-based Pairing characteristic,
// fe2c1234-8366-4814-8eb0-01de32100bea, byte-reversed here because
// BLE_UUID128_INIT takes little-endian order.
//
// UNFLAGGED BRIEF BUG, worth calling out because it would have silently
// disabled the entire feature: the brief's PROSE names this UUID correctly,
// but the code block underneath it pasted Task 16's UUID
// (a92ee202-5501-4e6b-90fb-79a8c1f2e5a8) with a comment asserting the two are
// "confirmed identical across donor implementations". They are not identical
// and the donors do not use a92ee202 at all -- both
// ~/src/poseidon-tab5/src/features/ble_whisperpair.cpp:74 (WP_KBP_UUID) and
// ~/src/unigeek-main/firmware/src/screens/ble/WhisperPairScreen.cpp:15
// (kKbpCharUUID) spell fe2c1234-8366-4814-8eb0-01de32100bea. a92ee202 is the
// ORIGINAL (pre-2021) Fast Pair KBP UUID, which is what Task 16's own donor
// (~/src/firmware/src/modules/ble/BLE_Suite.cpp) targets; current Fast Pair
// accessories expose the fe2c1234 one. Using Task 16's value here would have
// reported "KBP characteristic not found" against every modern accessory --
// indistinguishable from a patched target, i.e. a silent false negative on a
// vulnerability detector. The donors win.
static const ble_uuid128_t kKbpCharUuid =
    BLE_UUID128_INIT(0xea, 0x0b, 0x10, 0x32, 0xde, 0x01, 0xb0, 0x8e,
                     0x14, 0x48, 0x66, 0x83, 0x34, 0x12, 0x2c, 0xfe);

// Client Characteristic Configuration descriptor, 0x2902. Taken from NimBLE's
// own BLE_GATT_DSC_CLT_CFG_UUID16 rather than a literal, so the constant is
// the stack's, not ours (Bug 5 -- confirmed present in the real installed
// host/ble_gatt.h:69).
static const ble_uuid16_t kCccdUuid = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);

// Standard "enable notifications" CCCD value: little-endian uint16_t 1.
static const uint8_t kCccdNotifyEnable[2] = {0x01, 0x00};

// How long after the probe write we keep waiting for a notify before calling
// the target silent. UniGeek waits 2000ms; 3000 here because that donor's wait
// starts after a synchronous blocking write, whereas ours starts when the
// write COMPLETION callback fires, and we would rather over- than under-wait
// on a detector where a false "patched" is the costly error.
static const uint32_t kNotifyWaitMs = 3000;

// ---- Cross-task state -------------------------------------------------------
// Written on the NimBLE host task, read on the main task (button handler,
// poll(), the teardown loop). volatile per the house rule c2link_ble.cpp
// states for single-word scalars crossing a real task boundary -- and
// load-bearing rather than merely tidy for s_conn_handle, whose value the
// LV_EVENT_DELETE teardown loop below re-reads on every iteration expecting to
// observe a host-task write land mid-loop.
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t s_kbp_val_handle = 0;
static volatile uint16_t s_cccd_handle = 0;
static volatile bool s_subscribed = false;
static volatile bool s_probe_sent = false;
static volatile bool s_notify_received = false;
static volatile uint32_t s_notify_wait_start_ms = 0;

// Return code of the BleCentral::connect() call build_screen() makes. 0 means
// an attempt really was started and may still be in flight; nonzero means
// nothing is pending and teardown has no connect to chase. Mirrors
// ble_flood.cpp's s_last_connect_rc and ble_fastpair_exploit.cpp /
// ble_hfp_exploit.cpp's s_connect_rc, used for the same purpose.
static volatile int s_connect_rc = -1;

// Discovery bookkeeping touched ONLY from the NimBLE host task (the discovery
// callbacks are all the same task, so they are sequential with respect to each
// other) plus a reset in build_screen() before any connect attempt can exist --
// hence plain statics.
static bool s_svc_found = false;
static bool s_kbp_found = false;
static uint16_t s_svc_end_handle = 0;
// End of the KBP characteristic's own attribute range: the handle just below
// the NEXT characteristic's declaration, or the service's end handle when KBP
// is the last characteristic. See chr_disc_cb for why this is not just
// s_svc_end_handle.
static uint16_t s_kbp_end_handle = 0;

// Main-task-only: created in build_screen(), cleared in its LV_EVENT_DELETE
// handler, read only by poll(). No NimBLE callback ever touches these -- that
// is the whole point of the status buffer below.
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_send_btn = nullptr;

// ---- Cross-task status buffer (Bug 3) --------------------------------------
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_pending_status[128];
static volatile bool s_status_dirty = false;

// Safe to call from ANY task. Never touches LVGL.
static void log_status(const char *msg) {
    Serial.printf("quarky-tab5: [whisperpair] %s\n", msg);
    portENTER_CRITICAL(&s_status_mux);
    strncpy(s_pending_status, msg, sizeof(s_pending_status) - 1);
    s_pending_status[sizeof(s_pending_status) - 1] = '\0';
    s_status_dirty = true;
    portEXIT_CRITICAL(&s_status_mux);
}

// ---- Probe construction (main task, from the Send Probe button) ------------

// Real ECDH via mbedtls (secp256r1), matching both donors' confirmed
// technique -- not a placeholder. Generates an ephemeral P-256 keypair, takes
// 16 bytes of the resulting public point as AES-128 key material, and
// encrypts a 16-byte KBP request (type=0x00, flags=0x00, then 14 random
// bytes standing in for the provider MAC + salt UniGeek fills in) into `out`.
//
// Bug 6 -- the brief's version DID NOT COMPILE, and the fix is not cosmetic.
// It reached directly for ecdh.grp / ecdh.d / ecdh.Q and passed them to
// mbedtls_ecdh_gen_public() + mbedtls_ecp_point_write_binary(). Against the
// real installed header for this build target
// (~/.platformio/packages/framework-arduinoespressif32-libs/esp32p4/include/
// mbedtls/mbedtls/include/mbedtls/ecdh.h) those members are declared
// MBEDTLS_PRIVATE(grp) etc., which without MBEDTLS_ALLOW_PRIVATE_ACCESS
// expands to a mangled name, so `ecdh.grp` is not even a member. Worse, the
// struct is conditionally a flat legacy layout OR a union of
// mbed_ecdh/everest_ecdh sub-contexts: this build has MBEDTLS_ECP_RESTARTABLE
// off (esp_config.h gates it on CONFIG_MBEDTLS_ECP_RESTARTABLE, absent here)
// and MBEDTLS_ECDH_VARIANT_EVEREST_ENABLED off, and ecdh.h:41-45 selects
// MBEDTLS_ECDH_LEGACY_CONTEXT purely from MBEDTLS_ECP_RESTARTABLE -- so the
// NON-legacy union layout is what is actually compiled, and even with private
// access forced the correct path would have been ecdh.ctx.mbed_ecdh.grp, an
// internal detail that can change between mbedtls versions.
//
// The fix uses mbedtls_ecdh_make_public(), a public documented API taking the
// context opaquely, which generates the keypair AND serialises the public key
// in one call. No struct member is touched.
//
// IMPORTANT CORRECTION to the expected output format. The task instructions
// predicted `buf` would hold a bare uncompressed point (0x04 || X || Y). It
// does not. The real header documents mbedtls_ecdh_make_public() as exporting
// "a TLS ClientKeyExchange payload", which is an RFC 4492 s5.4 TLS ECPoint
// record: a ONE-BYTE LENGTH prefix followed by the point (ecdh.h:347-375 ->
// ecp.h:829-851 mbedtls_ecp_tls_write_point). For P-256 uncompressed that is
// 66 bytes: 0x41, 0x04, X[32], Y[32]. So the brief's "our_pub + 1" would have
// keyed AES with 0x04 || X[0..14] -- one byte of constant framing plus 15 key
// bytes -- rather than the intended X[0..15]. Since the accessory cannot
// decrypt this probe either way it would not have changed the verdict, but it
// would have made the code silently not do what its own comment claimed. The
// prefix is parsed explicitly below rather than assumed away, and the bare
// -point shape is accepted too so a future mbedtls that drops the framing does
// not break this silently.
//
// Bug 7 -- every failure path in the brief's version returned without freeing
// the three contexts it had already initialised, leaking their internal
// allocations once per failed "Send Probe" tap (a button a user troubleshooting
// a target will press repeatedly). Restructured to a single exit point: one
// do/while(0) whose breaks all fall through to the same cleanup block. Calling
// _free() on a context that was init'd but never used is safe by design, so
// all three are freed unconditionally regardless of how far we got.
static bool build_probe(uint8_t out[16]) {
    mbedtls_ecdh_context ecdh;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ecdh_init(&ecdh);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);

    bool ok = false;
    char msg[128];

    do {
        static const char kPers[] = "whisperpair";
        int rc = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                       (const unsigned char *)kPers, sizeof(kPers) - 1);
        if (rc != 0) {
            snprintf(msg, sizeof(msg), "ctr_drbg_seed failed rc=-0x%04x", -rc);
            log_status(msg);
            break;
        }

        rc = mbedtls_ecdh_setup(&ecdh, MBEDTLS_ECP_DP_SECP256R1);
        if (rc != 0) {
            snprintf(msg, sizeof(msg), "ecdh_setup(secp256r1) failed rc=-0x%04x", -rc);
            log_status(msg);
            break;
        }

        // 66 bytes needed for the TLS ECPoint record described above; 80 gives
        // headroom rather than sitting exactly on the boundary.
        uint8_t our_pub[80];
        size_t our_pub_len = 0;
        rc = mbedtls_ecdh_make_public(&ecdh, &our_pub_len, our_pub, sizeof(our_pub),
                                      mbedtls_ctr_drbg_random, &ctr_drbg);
        if (rc != 0) {
            snprintf(msg, sizeof(msg), "ecdh_make_public failed rc=-0x%04x", -rc);
            log_status(msg);
            break;
        }

        // Locate the uncompressed point inside whatever framing we got.
        const uint8_t *point = nullptr;
        size_t point_len = 0;
        if (our_pub_len >= 2 && our_pub[0] == (uint8_t)(our_pub_len - 1) && our_pub[1] == 0x04) {
            point = our_pub + 1;          // TLS ECPoint record: skip the length byte
            point_len = our_pub_len - 1;
        } else if (our_pub_len >= 1 && our_pub[0] == 0x04) {
            point = our_pub;              // bare uncompressed point
            point_len = our_pub_len;
        }
        // 1 prefix byte + at least the 16 X bytes we key AES with.
        if (point == nullptr || point_len < 1 + 16) {
            snprintf(msg, sizeof(msg),
                     "unexpected ECDH pubkey format (len=%u, [0]=0x%02x) -- probe not built",
                     (unsigned)our_pub_len, our_pub_len > 0 ? our_pub[0] : 0);
            log_status(msg);
            break;
        }
        Serial.printf("quarky-tab5: [whisperpair] ecdh pubkey %u bytes, point %u bytes "
                      "(prefix 0x%02x)\n",
                      (unsigned)our_pub_len, (unsigned)point_len, our_pub[0]);

        // KBP request plaintext, UniGeek's layout
        // (WhisperPairScreen.cpp:246-256): [0] type=0x00 (KBP request),
        // [1] flags=0x00 (seeker initiates), [2..7] provider MAC, [8..15] salt.
        // UniGeek fills [2..7] with its own BLE address; this port fills the
        // whole 14-byte tail randomly, which is equivalent for the only thing
        // the probe is used for (the accessory cannot decrypt it -- see the
        // file header's disclosed limitation).
        uint8_t plain[16] = {0x00, 0x00};
        esp_fill_random(plain + 2, sizeof(plain) - 2);

        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        // Bug 8: the brief never checked this return code, so a failed
        // key-set would have fallen straight into crypt_ecb() against an
        // unset key and produced a silently wrong probe.
        rc = mbedtls_aes_setkey_enc(&aes, point + 1, 128); // first 16 bytes of X
        if (rc == 0) {
            rc = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plain, out);
        }
        mbedtls_aes_free(&aes);
        if (rc != 0) {
            snprintf(msg, sizeof(msg), "AES-128-ECB probe encryption failed rc=-0x%04x", -rc);
            log_status(msg);
            break;
        }

        ok = true;
    } while (0);

    // Single cleanup path (Bug 7). Safe on contexts that were only init'd.
    mbedtls_ecdh_free(&ecdh);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
    return ok;
}

// ---- GATT callbacks (all on the NimBLE host task) --------------------------

static int write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                    struct ble_gatt_attr *attr, void *arg) {
    int status = error != nullptr ? error->status : -1;
    char msg[128];
    if (status == 0) {
        snprintf(msg, sizeof(msg), "probe accepted (att status=0) -- waiting %ums for notify...",
                 (unsigned)kNotifyWaitMs);
        // Start the window only on a write the target actually acknowledged.
        // Starting it on a rejected write would count a target that refused
        // the write at all as "silent, therefore patched" -- a different and
        // much weaker finding reported as the strong one.
        s_notify_wait_start_ms = millis();
        s_probe_sent = true;
    } else {
        snprintf(msg, sizeof(msg),
                 "probe write REJECTED (att status=%d) -- inconclusive, not a 'patched' result",
                 status);
        s_probe_sent = false;
    }
    log_status(msg);
    return 0;
}

static int cccd_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg) {
    int status = error != nullptr ? error->status : -1;
    char msg[128];
    if (status == 0) {
        s_subscribed = true;
        snprintf(msg, sizeof(msg),
                 "subscribed to KBP notify -- tap Send Probe (target must NOT be pairing)");
    } else {
        // Without a subscription the accessory has nowhere to send a reply, so
        // a subsequent silence would prove nothing. Say so instead of letting
        // the user probe into a dead channel.
        snprintf(msg, sizeof(msg),
                 "CCCD subscribe FAILED (att status=%d) -- cannot detect a reply", status);
    }
    log_status(msg);
    return 0;
}

// Bug 5. The brief suggested writing the CCCD at a handle "discovered the same
// way chr_disc_cb finds the value handle", and the common shortcut in simple
// central code is to assume val_handle + 1. The BLE spec does not guarantee
// that: any number of other descriptors (a Characteristic User Description,
// a Presentation Format, ...) may sit between the value attribute and the
// CCCD. Both donors sidestep the question by calling NimBLE-Arduino's
// NimBLERemoteCharacteristic::subscribe(), which internally performs REAL
// descriptor discovery. We are on the raw ESP-IDF NimBLE C API, so we do the
// discovery ourselves: enumerate every descriptor of the KBP characteristic
// and match on UUID 0x2902.
static int dsc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg) {
    if (error == nullptr) {
        log_status("descriptor discovery gave no status -- cannot subscribe");
        return 0;
    }

    if (error->status == 0) {
        if (dsc == nullptr) return 0;
        if (s_cccd_handle != 0) return 0; // already found and written
        if (ble_uuid_cmp(&dsc->uuid.u, &kCccdUuid.u) != 0) return 0;

        s_cccd_handle = dsc->handle;
        int rc = ble_gattc_write_flat(conn_handle, dsc->handle, kCccdNotifyEnable,
                                      sizeof(kCccdNotifyEnable), cccd_write_cb, nullptr);
        Serial.printf("quarky-tab5: [whisperpair] CCCD found at handle %u, "
                      "ble_gattc_write_flat rc=%d\n", dsc->handle, rc);
        if (rc != 0) {
            char msg[128];
            snprintf(msg, sizeof(msg), "CCCD write failed to start rc=%d -- cannot subscribe", rc);
            log_status(msg);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (s_cccd_handle == 0) {
            // A KBP characteristic with no CCCD cannot notify at all. Reporting
            // it is the difference between "this target has no notify channel"
            // and the screen simply looking hung with a dead button.
            log_status("KBP characteristic has no CCCD (0x2902) -- cannot subscribe, "
                       "no reply detectable");
        }
    } else {
        char msg[128];
        snprintf(msg, sizeof(msg), "descriptor discovery failed: status=%d", error->status);
        log_status(msg);
    }
    return 0;
}

// Starts descriptor discovery over the KBP characteristic's own attribute
// range. ble_gattc_disc_all_dscs() takes (characteristic value handle,
// last handle in the characteristic definition) per the real header
// (ble_gatt.h:660-677), and enumerates the handles strictly between them --
// so the end handle matters: passing the SERVICE's end handle when KBP is not
// the last characteristic would sweep the following characteristics' descriptors
// too and could latch onto some OTHER characteristic's CCCD, subscribing to the
// wrong thing. s_kbp_end_handle is therefore the next characteristic's
// declaration handle minus one, falling back to the service end only when KBP
// really is last.
static void start_dsc_discovery(uint16_t conn_handle) {
    if (!s_kbp_found || s_kbp_val_handle == 0) return;
    if (s_kbp_end_handle <= s_kbp_val_handle) {
        // No attribute handles exist after the value attribute, so there is no
        // room for a CCCD to live in.
        log_status("KBP characteristic has no descriptor range -- cannot subscribe");
        return;
    }
    int rc = ble_gattc_disc_all_dscs(conn_handle, s_kbp_val_handle, s_kbp_end_handle,
                                     dsc_disc_cb, nullptr);
    Serial.printf("quarky-tab5: [whisperpair] ble_gattc_disc_all_dscs(%u..%u) rc=%d\n",
                  s_kbp_val_handle, s_kbp_end_handle, rc);
    if (rc != 0) {
        char msg[128];
        snprintf(msg, sizeof(msg), "descriptor discovery failed to start rc=%d", rc);
        log_status(msg);
    }
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg) {
    if (error == nullptr) {
        log_status("chr discovery gave no status");
        return 0;
    }

    if (error->status == 0) {
        if (chr == nullptr) return 0;
        if (!s_kbp_found && ble_uuid_cmp(&chr->uuid.u, &kKbpCharUuid.u) == 0) {
            s_kbp_found = true;
            s_kbp_val_handle = chr->val_handle;
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "KBP characteristic found (handle %u) -- discovering CCCD",
                     chr->val_handle);
            log_status(msg);
        } else if (s_kbp_found && s_kbp_end_handle == 0 && chr->def_handle > s_kbp_val_handle) {
            // The first characteristic declared AFTER KBP bounds KBP's own
            // attribute range. See start_dsc_discovery() for why that bound
            // matters rather than just using the service end.
            s_kbp_end_handle = chr->def_handle - 1;
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (!s_kbp_found) {
            log_status("KBP characteristic NOT found in Fast Pair service -- "
                       "not applicable to this target");
            return 0;
        }
        if (s_kbp_end_handle == 0) {
            // KBP was the last characteristic in the service, so its range runs
            // to the service's end handle.
            s_kbp_end_handle = s_svc_end_handle;
        }
        start_dsc_discovery(conn_handle);
    } else {
        // A genuine mid-discovery failure (insufficient authentication/
        // encryption, an unexpected disconnect, an ATT error) must not look
        // like a clean "no KBP characteristic here" -- a materially different
        // finding on a detector.
        char msg[128];
        snprintf(msg, sizeof(msg), "chr discovery failed: status=%d", error->status);
        log_status(msg);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg) {
    if (error == nullptr) {
        log_status("svc discovery gave no status");
        return 0;
    }

    if (error->status == 0) {
        if (service == nullptr) return 0;
        s_svc_found = true;
        s_svc_end_handle = service->end_handle;
        int rc = ble_gattc_disc_all_chrs(conn_handle, service->start_handle,
                                         service->end_handle, chr_disc_cb, nullptr);
        Serial.printf("quarky-tab5: [whisperpair] Fast Pair svc handles %u-%u, "
                      "ble_gattc_disc_all_chrs rc=%d\n",
                      service->start_handle, service->end_handle, rc);
        if (rc != 0) {
            char msg[128];
            snprintf(msg, sizeof(msg), "chr discovery failed to start rc=%d", rc);
            log_status(msg);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (!s_svc_found) {
            log_status("no Fast Pair service (0xFE2C) -- not applicable to this target");
        }
    } else {
        char msg[128];
        snprintf(msg, sizeof(msg), "svc discovery failed: status=%d", error->status);
        log_status(msg);
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_kbp_val_handle = 0;
            s_cccd_handle = 0;
            s_subscribed = false;
            s_probe_sent = false;
            s_notify_received = false;
            s_svc_found = false;
            s_kbp_found = false;
            s_svc_end_handle = 0;
            s_kbp_end_handle = 0;
            Serial.printf("quarky-tab5: [whisperpair] connected handle=%u\n", s_conn_handle);
            log_status("connected -- discovering Fast Pair service (0xFE2C)");
            int rc = ble_gattc_disc_svc_by_uuid(s_conn_handle, &kFastPairSvcUuid.u,
                                                svc_disc_cb, nullptr);
            Serial.printf("quarky-tab5: [whisperpair] ble_gattc_disc_svc_by_uuid rc=%d\n", rc);
            if (rc != 0) {
                char msg[128];
                snprintf(msg, sizeof(msg), "svc discovery failed to start rc=%d", rc);
                log_status(msg);
            }
        } else {
            char msg[64];
            snprintf(msg, sizeof(msg), "connect failed status=%d", event->connect.status);
            log_status(msg);
        }
    } else if (event->type == BLE_GAP_EVENT_NOTIFY_RX) {
        // Attribute-handle filter: a Fast Pair accessory may notify on other
        // characteristics (Passkey, Account Key) for reasons that have nothing
        // to do with our probe. Reporting one of those as the CVE signal would
        // be a false positive on a vulnerability detector, which is the worst
        // failure mode this feature has.
        if (event->notify_rx.attr_handle == s_kbp_val_handle) {
            s_notify_received = true;
            char msg[128];
            snprintf(msg, sizeof(msg),
                     "NOTIFY on KBP -- target responded outside pairing mode "
                     "(CVE-2025-36911 indicated)");
            log_status(msg);
        } else {
            Serial.printf("quarky-tab5: [whisperpair] ignoring notify on handle %u "
                          "(KBP is %u)\n", event->notify_rx.attr_handle, s_kbp_val_handle);
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_kbp_val_handle = 0;
        s_cccd_handle = 0;
        s_subscribed = false;
        s_kbp_found = false;
        s_svc_found = false;
        char msg[64];
        snprintf(msg, sizeof(msg), "disconnected (reason=%d)", event->disconnect.reason);
        log_status(msg);
    }
    return 0;
}

// ---- Probe send (main task, from the LVGL button handler) ------------------

static void send_probe() {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        log_status("not connected -- nothing sent");
        return;
    }
    if (s_kbp_val_handle == 0) {
        log_status("KBP characteristic not discovered -- nothing sent");
        return;
    }
    // The ordering requirement, not a nicety: an accessory cannot notify a
    // client that has not subscribed, so probing first would make every target
    // look silent.
    if (!s_subscribed) {
        log_status("not subscribed to KBP notify yet -- nothing sent");
        return;
    }

    uint8_t probe[16];
    if (!build_probe(probe)) {
        // build_probe() already logged the specific mbedtls failure.
        return;
    }

    s_notify_received = false;
    s_probe_sent = false;
    int rc = ble_gattc_write_flat(s_conn_handle, s_kbp_val_handle, probe, sizeof(probe),
                                  write_cb, nullptr);
    Serial.printf("quarky-tab5: [whisperpair] probe ble_gattc_write_flat rc=%d (16 bytes)\n", rc);
    char msg[128];
    if (rc == 0) {
        snprintf(msg, sizeof(msg), "probe write started (rc=0)");
    } else {
        snprintf(msg, sizeof(msg), "probe write FAILED to start rc=%d", rc);
    }
    log_status(msg);
}

// ---- Screen -----------------------------------------------------------------

static lv_obj_t *build_screen(const uint8_t addr_val[6], uint8_t addr_type) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WhisperPair (CVE-2025-36911)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Connecting...");

    s_send_btn = lv_button_create(content);
    lv_obj_t *send_label = lv_label_create(s_send_btn);
    lv_label_set_text(send_label, "Send Probe");
    // Starts disabled; poll() enables it only once cccd_write_cb has confirmed
    // the subscription (Bug 5's ordering requirement made visible in the UI
    // rather than only enforced in send_probe()).
    lv_obj_add_state(s_send_btn, LV_STATE_DISABLED);
    lv_obj_add_event_cb(s_send_btn, [](lv_event_t *e) { send_probe(); },
                        LV_EVENT_CLICKED, nullptr);

    // Runs on the main task (LVGL fires LV_EVENT_DELETE from whichever task
    // destroys the object, here the main task via ScreenStack::pop()). Clearing
    // s_status_label / s_send_btn is what makes a late host-task callback
    // harmless: it can still write into s_pending_status, but poll() has
    // nothing left to render it into. Attached to the status label rather than
    // to `content`, matching ble_flood.cpp / ble_gatt_explorer.cpp /
    // ble_fastpair_exploit.cpp / ble_hfp_exploit.cpp's teardown shape.
    //
    // Bug 4, the connect-in-flight connection-table leak. The brief's handler
    // only terminated an ESTABLISHED connection and never called
    // ble_gap_conn_cancel(). That leaks on essentially every use of this
    // screen: build_screen() starts a 5-second connect attempt the instant the
    // screen opens (there is no tap-a-target step here), so a Back tap inside
    // that window -- the LIKELY case, since the entire point of opening this
    // screen is to find out whether the scanned target is even a Fast Pair
    // accessory, and most are not -- left the attempt in flight. If it then
    // landed after teardown, gap_event_cb set s_conn_handle to a connection
    // nothing would ever terminate: the peer stays joined, burns a
    // connection-table slot, and the next BLE feature in the session trips
    // conn-table-full (ble_flood.cpp's teardown documents that exact symptom).
    // Same hazard already fixed in Tasks 13, 14, 16 and 17.
    //
    // A single fire-and-hope cancel is not enough, for the same reason
    // ble_flood.cpp loops: one cancel races BLE_GAP_EVENT_CONNECT, so a
    // connection landing between the cancel and this handler returning still
    // leaks. Each iteration cancels anything still in flight AND hangs up
    // anything that landed in the previous 50ms gap (gap_event_cb here, like
    // ble_flood.cpp's, does not itself terminate a connection that arrives
    // after teardown).
    //
    // WHY THIS LOOP HAS NO EARLY EXIT -- do not "optimize" it back. Task 16
    // first tried breaking out as soon as ble_gap_conn_cancel() returned
    // nonzero (BLE_HS_EALREADY, i.e. !ble_gap_conn_active()) AND s_conn_handle
    // still read NONE, on the theory that those two together mean "settled".
    // They do not. Traced through the real NimBLE source (ble_gap.c
    // ble_gap_rx_conn_complete), the MASTER-role path -- ours, we are the
    // central -- resets the master state FIRST (so conn_cancel() already
    // returns BLE_HS_EALREADY), then inserts the connection into the host's
    // table, and only reports it to gap_event_cb after TWO deferred HCI round
    // trips (read remote supported features, then read remote version) over the
    // P4<->C6 esp-hosted transport. In that window both halves of the "settled"
    // test read settled while a connection is already occupying a slot and
    // about to be reported -- the loop exits and leaks it anyway. The fix is to
    // stop trying to detect "settled" and simply burn the deadline, which is
    // what ble_flood.cpp always did and why ble_flood.cpp never had this
    // exposure. No early exit means no early-exit race.
    //
    // The loop only has to survive the narrow "landed concurrently with the
    // cancel" window, NOT the connect's full 5-second timeout: once a cancel
    // has actually taken effect, no connection can subsequently appear.
    //
    // Blocking 500ms would violate this project's ~50ms rule on a polled path,
    // but this is a one-shot, user-triggered teardown -- exactly the
    // justification ble_flood.cpp makes for its own identical loop.
    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        // First: stop poll() from rendering or touching widgets. Everything
        // below can take up to 500ms, during which host-task callbacks keep
        // buffering status text.
        s_status_label = nullptr;
        s_send_btn = nullptr;

        uint16_t conn = s_conn_handle;
        if (conn != BLE_HS_CONN_HANDLE_NONE) {
            int rc = BleCentral::disconnect(conn);
            Serial.printf("quarky-tab5: [whisperpair] teardown disconnect rc=%d\n", rc);
        }

        // The one safe early return: BleCentral::connect() itself failed, so
        // ble_gap_connect() never started an attempt and no connection can ever
        // land. Unlike the rejected "settled" test, this is not a sampled race
        // -- it is a fact fixed at connect time, checked once here and never
        // re-sampled inside the loop. (Still checks conn, in case a connection
        // somehow predates this screen.)
        if (s_connect_rc != 0 && conn == BLE_HS_CONN_HANDLE_NONE) return;

        uint32_t deadline = millis() + 500;
        int cancel_rc;
        do {
            cancel_rc = ble_gap_conn_cancel();

            // A connection that landed in the last gap -- including one that
            // was mid-deferred-HCI-round-trip when we started: hang it up.
            conn = s_conn_handle;
            if (conn != BLE_HS_CONN_HANDLE_NONE) {
                BleCentral::disconnect(conn);
            }

            delay(50);
        } while (millis() < deadline);

        Serial.printf("quarky-tab5: [whisperpair] teardown conn_cancel rc=%d, "
                      "conn_handle=%u (%s)\n",
                      cancel_rc, s_conn_handle,
                      s_conn_handle == BLE_HS_CONN_HANDLE_NONE
                          ? "settled"
                          : "STILL SET -- possible leaked connection slot");
    }, LV_EVENT_DELETE, nullptr);

    s_kbp_val_handle = 0;
    s_cccd_handle = 0;
    s_subscribed = false;
    s_probe_sent = false;
    s_notify_received = false;
    s_notify_wait_start_ms = 0;
    s_svc_found = false;
    s_kbp_found = false;
    s_svc_end_handle = 0;
    s_kbp_end_handle = 0;
    s_connect_rc = -1;
    portENTER_CRITICAL(&s_status_mux);
    s_pending_status[0] = '\0';
    s_status_dirty = false;
    portEXIT_CRITICAL(&s_status_mux);

    ble_addr_t target{};
    target.type = addr_type; // Bug 2 fix: the peer's REAL advertised address
                              // type, not a hardcoded BLE_ADDR_PUBLIC. Most
                              // modern peripherals -- Fast Pair accessories very
                              // much included -- advertise random addresses, so
                              // the hardcoded value the brief used would simply
                              // fail to connect and look like "target not
                              // vulnerable", the exact false negative a detector
                              // must not produce. Same fix already made in Tasks
                              // 1, 9, 13, 14, 16 and 17.
    memcpy(target.val, addr_val, 6);
    // Recorded for the teardown handler above: 0 means an attempt is genuinely
    // in flight and must be cancelled if the user leaves before it resolves.
    int rc = BleCentral::connect(target, 5000, gap_event_cb, nullptr);
    s_connect_rc = rc;
    Serial.printf("quarky-tab5: [whisperpair] BleCentral::connect rc=%d (peer addr type=%u)\n",
                  rc, addr_type);
    if (rc != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "connect could not start rc=%d", rc);
        log_status(msg);
    }

    return screen;
}

void register_module() {
    // No launcher-supplied target: like ble_flood.cpp (Task 14),
    // ble_fastpair_exploit.cpp (Task 16) and ble_hfp_exploit.cpp (Task 17),
    // this runs against the first BLE-scanned device, so BLE Scan must have
    // been run first.
    g_registry.register_module({"ble_whisperpair", "WhisperPair CVE-2025-36911", Category::BLE,
                                Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    const uint8_t *addr = BleScanFeature::first_device_addr();
    if (!addr) {
        Serial.println("quarky-tab5: [whisperpair] no scanned device available -- run BLE Scan first");
        return;
    }
    uint8_t addr_type = BleScanFeature::first_device_addr_type(); // Bug 2 fix
    ScreenStack::push(build_screen(addr, addr_type));
}

void poll() {
    if (s_status_label == nullptr) return; // screen not open

    // Expire the post-probe notify wait window. This runs on the main task
    // because it ends by writing the verdict, and because the host task has no
    // timer of its own here. s_probe_sent is cleared so the verdict is emitted
    // exactly once per probe.
    if (s_probe_sent && !s_notify_received &&
        (millis() - s_notify_wait_start_ms) > kNotifyWaitMs) {
        s_probe_sent = false;
        log_status("no reply within window -- target stayed silent outside pairing mode "
                   "(patched or not applicable)");
    }

    // Gate the Send Probe button on the subscription (Bug 5's ordering
    // requirement). Done here rather than in cccd_write_cb because that
    // callback is on the NimBLE host task and must not touch LVGL.
    if (s_send_btn != nullptr) {
        bool disabled = lv_obj_has_state(s_send_btn, LV_STATE_DISABLED);
        if (s_subscribed && disabled) {
            lv_obj_clear_state(s_send_btn, LV_STATE_DISABLED);
        } else if (!s_subscribed && !disabled) {
            lv_obj_add_state(s_send_btn, LV_STATE_DISABLED);
        }
    }

    if (!s_status_dirty) return;

    char snapshot[sizeof(s_pending_status)];
    portENTER_CRITICAL(&s_status_mux);
    memcpy(snapshot, s_pending_status, sizeof(snapshot));
    s_status_dirty = false;
    portEXIT_CRITICAL(&s_status_mux);

    lv_label_set_text(s_status_label, snapshot);
}

} // namespace BleWhisperPairFeature
