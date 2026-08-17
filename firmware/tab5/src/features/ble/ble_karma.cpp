#include "ble_karma.h"
#include "ble_common.h" // ble_require_host_ready() -- shared host-ready screen guard
#include "../../hal/c2link_ble.h" // c2link_ble_rearm_advertising()
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis() -- needed the same way ble_scan.cpp/
                      // ble_finder.cpp/ble_sniffer.cpp/ble_clone.cpp explicitly
                      // pull this in; nothing else in this file's include list
                      // drags it in transitively
#include <esp_random.h> // esp_random() -- first use of this call in the tab5
                         // tree (grepped: no other firmware/tab5/src file calls
                         // it), and it is NOT transitively pulled in by
                         // Arduino.h/esp32-hal.h/WString.h/Esp.h in this
                         // framework version, so it needs its own include or
                         // this does not compile
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleKarmaFeature {

// NOTE, disclosed per donor research: this rotates identity whenever ANY
// nearby BLE advertisement is seen, not specifically in response to a
// scan-request PDU targeted at this device -- NimBLE's scan callback
// doesn't expose that distinction (confirmed against Poseidon's own
// ble_karma.cpp, which has the same limitation despite its file header's
// "listening for incoming scan requests" description). This is "rotate
// identity while nearby BLE traffic exists," not per-target-matched karma.
static const char *kNames[] = {
    "AirPods Pro", "AirPods Max", "Galaxy Buds Pro", "Galaxy Buds2",
    "Samsung TV", "Sony WH-1000XM4", "JBL Flip 6", "Beats Fit Pro",
    "Pixel Buds Pro", "Bose QC45", "LG TV", "Echo Dot",
    "Fitbit Charge 5", "Garmin Watch", "MX Master 3", "Magic Mouse",
};
static constexpr int kNameCount = sizeof(kNames) / sizeof(kNames[0]);

static volatile bool s_traffic_seen = false;
static bool s_active = false;
// True once rotate_identity() has actually started an advertisement, i.e.
// once this screen has taken over the radio's single legacy-advertising slot
// from c2link_ble. Main-task only (set in rotate_identity(), which poll()
// calls; read in the teardown handler). Finding I6: teardown uses this to
// decide whether there is anything for c2link_ble_rearm_advertising() to
// restore -- Karma only advertises once it has actually seen nearby BLE
// traffic, so a screen opened in a quiet room never disturbed C2 at all.
static bool s_advertised = false;
static int s_name_idx = 0;
static uint32_t s_last_rotate_ms = 0;
static lv_obj_t *s_status_label = nullptr;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) s_traffic_seen = true;
    return 0;
}

static void rotate_identity() {
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = (uint8_t)esp_random();
    addr[5] |= 0xC0;

    // IMPORTANT, real cross-feature side effect (not a bug to fix silently):
    // ble_hs_id_set_rnd() sets NimBLE's host-wide random *identity* address --
    // it is NOT scoped to this screen or to the ble_gap_adv_start() call
    // below. There is no public "unset" API in this project's actual NimBLE
    // (same finding Task 9's ble_clone.cpp already made and documented against
    // the real installed header, ~/Library/Arduino15/packages/esp32/tools/
    // esp32p4-libs/3.3.10/include/bt/host/nimble/nimble/nimble/host/include/
    // host/ble_hs_id.h -- it declares only ble_hs_id_gen_rnd/ble_hs_id_set_rnd/
    // ble_hs_id_copy_addr/ble_hs_id_infer_auto; the private ble_hs_id_reset()
    // in the underlying .c file is not exposed publicly, so there is no
    // supported way to clear the identity back to "unset"). The value set
    // here persists until either (a) the device reboots, or (b) some other
    // feature calls ble_hs_id_set_rnd() again with a different address --
    // including any subsequent call this same function makes. Meanwhile,
    // ble_hs_id_infer_auto(0, &out_addr_type) will keep returning
    // BLE_OWN_ADDR_RANDOM bound to whatever address was set last, for any
    // other feature that calls it. The currently-known affected feature is
    // firmware/tab5/src/features/ble/ble_hid_spike.cpp, which calls
    // ble_hs_id_infer_auto() in its own start() and advertises with the
    // inferred type/address.
    //
    // What is DIFFERENT here vs. Clone's version of this same issue: Clone
    // sets the identity ONCE, to one specific captured target's real MAC, so
    // the left-over address is a predictable, attacker-chosen value tied to
    // whatever was cloned. Karma instead calls ble_hs_id_set_rnd() repeatedly,
    // every 2 seconds, with a FRESH RANDOM address generated fresh each call
    // -- so whichever random address happened to be current at the exact
    // moment this screen was closed (mid-rotation, effectively arbitrary) is
    // what's left behind for ble_hid_spike.cpp or anything else calling
    // ble_hs_id_infer_auto() afterward, not a value anyone chose or can
    // predict. Same underlying persistence problem, different (non-)shape to
    // the leftover value. Disclosed rather than worked around: Karma
    // genuinely needs ble_hs_id_set_rnd() to rotate its advertised identity,
    // and inventing a fake reset mechanism not backed by a real NimBLE API
    // would be worse than leaving this documented. Only a device reboot, or
    // another feature's own ble_hs_id_set_rnd() call, clears it.
    //
    // For the full downstream consequence chain -- leftover identity ->
    // ble_hid_spike.cpp's inferred address -> BLE Bad-KB (ble_bad_kb.cpp)
    // appearing as an unknown device to a host it had already bonded with,
    // and churning a bond store capped at 3 -- see ble_clone.cpp's version of
    // this disclosure, which is where that chain is written out in full
    // (whole-branch review finding I5, 2026-08-17). It applies verbatim here,
    // and arguably more often: Karma re-rolls the identity every 2 seconds
    // for as long as its screen is open, so the odds that a Bad-KB session
    // later in the same boot inherits a stale identity are near-certain
    // rather than incidental.
    int rc = ble_hs_id_set_rnd(addr);
    Serial.printf("quarky-tab5: [ble-karma] ble_hs_id_set_rnd rc=%d\n", rc);

    ble_gap_adv_stop(); // no-op (returns an error this ignores) if nothing is currently advertising

    // Scoped re-review finding (2026-08-17, Minor): this used to only flip
    // s_advertised on a successful ble_gap_adv_start() below, matching
    // ble_hid_spike.cpp's s_took_adv_slot in spirit but not in timing --
    // the C2 advertisement is already stopped and gone the moment the call
    // above returns, regardless of whether the replacement start succeeds.
    // A failed start left s_advertised false, so teardown skipped
    // c2link_ble_rearm_advertising() and C2 stayed down for the rest of the
    // boot even though this function is exactly what took its slot. Set it
    // here, unconditionally, at the point the slot actually changes hands.
    s_advertised = true;

    // Defensive 5ms settle delay between stop and ble_gap_adv_set_fields()
    // below. Same async stop/set-data race fixed in ble_sourapple.cpp's
    // send_one() (donor-documented hardware bug: the BLE controller can
    // still be asynchronously processing the previous ble_gap_adv_stop()
    // when a "set advertisement data" call arrives immediately after, and
    // silently fails, leaving the radio broadcasting the PREVIOUS payload)
    // and confirmed for real on this project's own hardware in ble_spam.cpp's
    // send_one_advertisement() (2026-08-16 finding: the vendor-picker
    // dropdown appeared not to change the broadcast payload -- root cause
    // was this exact race, not a picker bug). rotate_identity() has the
    // identical stop-then-set_fields shape and the identical symptom class
    // this would produce if hit: a rotated name/MAC that appears not to take
    // effect on some rotations. Same fix, same value, for the same reason.
    //
    // This call site is, if anything, less exposed than ble_spam.cpp's: this
    // rotates every 2000ms (poll()'s 2000ms gate below) vs. ble_spam.cpp's
    // 200ms cadence, so a 5ms delay here is more negligible still against
    // both the ~50ms main-loop() budget and the 2000ms rotation cadence.
    delay(5);

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = kNames[s_name_idx];
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    Serial.printf("quarky-tab5: [ble-karma] ble_gap_adv_set_fields rc=%d\n", rc);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // IMPORTANT, same single-legacy-advertising-instance constraint
    // ble_spam.cpp and ble_clone.cpp already disclose for their own
    // ble_gap_adv_start() calls (whole-branch review finding I6, 2026-08-17 --
    // this file did the identical thing with no disclosure at all): this
    // project has not configured NimBLE Extended Advertising, so exactly one
    // advertisement can be live system-wide and starting this one STOPS
    // c2link_ble's C2 advertisement. While BLE Karma is open the Tab5 is not
    // discoverable/connectable as "Quarky-Tab5" for BLE C2 pairing. Unlike
    // when this was first written, that is now temporary: the teardown
    // handler in build_screen() calls c2link_ble_rearm_advertising() after
    // stopping this advertisement, so C2 comes back on Back.
    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    Serial.printf("quarky-tab5: [ble-karma] advertising as '%s' rc=%d\n", name, rc);

    s_name_idx = (s_name_idx + 1) % kNameCount;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Karma", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Starting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        // Finding M1 (2026-08-17): capture and log both return codes, matching
        // every other teardown handler in this plan's files. These are the two
        // calls that decide whether the radio is actually quiet after Back --
        // discarding their rc was the one place in this file where a silent
        // failure would look identical to success.
        int disc_rc = ble_gap_disc_cancel();
        int adv_rc = ble_gap_adv_stop();
        Serial.printf("quarky-tab5: [ble-karma] teardown ble_gap_disc_cancel rc=%d, "
                      "ble_gap_adv_stop rc=%d\n", disc_rc, adv_rc);
        bool was_advertising = s_advertised;
        s_active = false;
        s_advertised = false;
        s_status_label = nullptr;
        // Finding I6 (2026-08-17): hand the radio's single legacy-advertising
        // slot back to the C2 link -- rotate_identity() took it over (see its
        // disclosure comment), and before this call that takeover lasted for
        // the rest of the boot.
        if (was_advertising) c2link_ble_rearm_advertising();
    }, LV_EVENT_DELETE, nullptr);

    // Finding C1 (2026-08-17): migrated from this file's own inline copy of
    // the host-ready guard to the shared helper (see ble_common.h). This is
    // the guard that keeps rotate_identity()'s ble_hs_id_set_rnd() -- which
    // has NO internal host-readiness check and panics on a radios-disabled
    // boot -- from ever being reached with no host behind it.
    if (!ble_require_host_ready(s_status_label)) return screen;

    struct ble_gap_disc_params params{};
    params.passive = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-karma] ble_gap_disc rc=%d\n", rc);
    s_active = (rc == 0);
    s_name_idx = 0;
    s_last_rotate_ms = 0;

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_karma", "BLE Karma", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    if (!s_traffic_seen) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < 2000) return;
    rotate_identity();
    s_traffic_seen = false;
    s_last_rotate_ms = now;
    if (s_status_label) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Rotating identity (%s next)", kNames[s_name_idx]);
        lv_label_set_text(s_status_label, buf);
    }
}

} // namespace BleKarmaFeature
