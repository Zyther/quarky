#include "ble_findmy.h"
#include "ble_common.h" // ble_require_host_ready() -- shared host-ready screen guard
#include "../../hal/c2link_ble.h" // c2link_ble_rearm_advertising()
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis(), delay() -- needed explicitly here,
                      // same as ble_karma.cpp's file-level comment explains:
                      // not transitively pulled in by lvgl.h/feature_registry.h
                      // in this framework version
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <esp_random.h>

extern FeatureRegistry g_registry;

namespace BleFindMyFeature {

// Real 31-byte Apple offline-finding advertisement shape, from donor
// research (2026-08-13/2026-08-16, ~/src/poseidon-tab5/src/features/
// ble_findmy.cpp's build_findmy()):
// [0x1E, 0xFF, 0x4C, 0x00, 0x12, 0x19, status, key[22], hint_byte, 0x00].
// key[22] stands in for a real rotating Curve25519 public key -- iPhones
// don't validate it, they relay blindly, which is the whole point of the
// offline-finding network's design (and the reason this "works" without a
// real Apple account/key infrastructure).
//
// Two deliberate deviations from a naive transcription of the brief, both
// taken from the real donor rather than invented here:
//
//  - status byte = 0xE0 ("OWNED", per the donor's own comment on its
//    build_findmy() call: `build_findmy(pkt, key, 0xE0 /* OWNED */, 0x00)`),
//    not a hardcoded 0x00. Apple's offline-finding status byte is a real
//    protocol field, not filler -- 0x00 (or other status values) are used
//    to signal accessory states such as "not registered to an owner". Since
//    this feature's whole purpose is emulating an accessory a passing
//    iPhone will treat as a legitimate registered Find My tag and relay to
//    iCloud, the "owned" status is the more faithful choice for what we're
//    actually trying to broadcast -- an unregistered-looking status byte
//    risks looking like a different (or invalid) accessory state to any
//    real analysis tooling, even though the donor's own comment confirms
//    iPhones don't gate relaying on this byte either way.
//  - hint byte (index 29, the byte immediately before the final 0x00) is
//    derived from the top 2 bits of the random key's first byte
//    (`(key[0] >> 6) & 0x03`), matching the donor's build_findmy():
//    `pkt[i++] = (key22[0] >> 6) & 0x03; /* hint */`. This is real Apple
//    protocol behavior (used server-side to help match/group reports), not
//    an incidental donor choice -- a hardcoded 0x00 here would silently
//    diverge from what a real accessory (or the donor's own proven
//    implementation) actually broadcasts.
static void build_advert(uint8_t out[31], const uint8_t key[22]) {
    out[0] = 0x1E; out[1] = 0xFF; out[2] = 0x4C; out[3] = 0x00; out[4] = 0x12; out[5] = 0x19;
    out[6] = 0xE0; // status byte: OWNED, per donor's build_findmy() call site
    for (int i = 0; i < 22; i++) out[7 + i] = key[i];
    out[29] = (key[0] >> 6) & 0x03; // hint byte, derived from key[0]'s top 2
                                     // bits -- see comment above
    out[30] = 0x00; // donor's own caller-supplied final byte is always 0x00
                     // at its one real call site; not touched by this fix
}

static bool s_active = false;
static uint32_t s_last_rotate_ms = 0;
static uint32_t s_dwell_ms = 60000; // 60s "single tag" default, per donor
static lv_obj_t *s_status_label = nullptr;

static void rotate() {
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = (uint8_t)esp_random();
    addr[5] |= 0xC0;

    // IMPORTANT, real cross-feature side effect (whole-branch review finding
    // I5, 2026-08-17 -- this file called the identical API with no disclosure
    // at all, while ble_clone.cpp and ble_karma.cpp both documented it):
    // ble_hs_id_set_rnd() sets NimBLE's host-wide random *identity* address,
    // not a per-advertisement one, and this project's NimBLE exposes no
    // supported way to unset it (ble_hs_id.h declares only
    // ble_hs_id_gen_rnd/ble_hs_id_set_rnd/ble_hs_id_copy_addr/
    // ble_hs_id_infer_auto). Whichever tag identity happened to be current
    // when this screen closed is what every later ble_hs_id_infer_auto()
    // caller inherits, until reboot or the next feature that overwrites it.
    //
    // What is DIFFERENT about this file's version: rotation here is SLOW and
    // deliberate -- one identity per s_dwell_ms (60s by default), because
    // that is what makes the broadcast look like a real single Find My
    // accessory rather than a swarm. So unlike Sour Apple's 5-per-second
    // churn, the identity left behind by a Find My session is one specific
    // address that a passing iPhone may well have already relayed to iCloud
    // as a sighting. That is the intended behaviour of the feature, but it
    // does mean the leftover host identity is one that has plausibly been
    // reported to Apple's offline-finding network -- worth knowing before
    // reusing this boot for anything where the device's BLE identity matters.
    //
    // For the full downstream consequence chain -- leftover identity ->
    // ble_hid_spike.cpp's ble_hs_id_infer_auto() -> BLE Bad-KB advertising
    // under an address a previously-bonded macOS/Windows host does not
    // recognise, forcing a re-pair and churning a bond store capped at 3 --
    // see ble_clone.cpp's version of this disclosure, where that chain is
    // written out in full.
    int rc = ble_hs_id_set_rnd(addr);
    Serial.printf("quarky-tab5: [ble-findmy] ble_hs_id_set_rnd rc=%d\n", rc);

    uint8_t key[22];
    for (int i = 0; i < 22; i++) key[i] = (uint8_t)esp_random();

    uint8_t advert[31];
    build_advert(advert, key);

    ble_gap_adv_stop(); // no-op (returns an error this ignores) if nothing is
                         // currently advertising, same convention as
                         // ble_spam.cpp's send_one_advertisement()

    // Defensive 5ms settle delay between stop and set_data. Same async
    // stop/set-data race documented in ble_sourapple.cpp's send_one()
    // (donor-documented hardware bug: the BLE controller can still be
    // asynchronously processing the previous ble_gap_adv_stop() when a
    // "set advertisement data" call arrives immediately after, and silently
    // fails, leaving the radio broadcasting the PREVIOUS payload) and
    // confirmed for real on this project's own hardware in ble_spam.cpp's
    // send_one_advertisement(), then applied preventively in ble_karma.cpp's
    // rotate_identity() and ble_hid_spike.cpp's start(). rotate() here has
    // the identical stop-then-set_data shape and the identical symptom
    // class this would produce if hit: a rotated tag identity that appears
    // not to take effect on some rotations. Same fix, same value, for the
    // same reason. Notably, the real donor's own fm_tick() does NOT have
    // this delay -- this is a latent gap in the donor, not a fix being
    // removed from a correct donor implementation; this project is more
    // careful than its donor here, consistent with how this bug class has
    // been handled everywhere else in this codebase.
    delay(5);

    int data_rc = ble_gap_adv_set_data(advert, sizeof(advert));
    Serial.printf("quarky-tab5: [ble-findmy] ble_gap_adv_set_data rc=%d\n", data_rc);

    int start_rc = -1;
    if (data_rc == 0) {
        struct ble_gap_adv_params adv_params{};
        adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
        adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
        // Deliberately slow advertising interval (1000ms/1200ms in NimBLE's
        // 0.625ms units), matching the real donor's adv->setMinInterval(
        // 0x0640)/setMaxInterval(0x0780) rather than leaving NimBLE's much
        // faster default in place. Real power-constrained Find My
        // accessories (AirTags etc.) advertise slowly to save battery; this
        // is part of what makes the broadcast look realistic to any
        // analysis tooling, not an incidental donor choice.
        adv_params.itvl_min = 0x0640;
        adv_params.itvl_max = 0x0780;
        // IMPORTANT, same single-legacy-advertising-instance constraint
        // ble_spam.cpp and ble_clone.cpp disclose for their own
        // ble_gap_adv_start() calls (whole-branch review finding I6,
        // 2026-08-17 -- this file did the identical thing undisclosed): this
        // project has not configured NimBLE Extended Advertising, so exactly
        // one advertisement is live system-wide and starting this one stops
        // c2link_ble's C2 advertisement. While the Find My emulator is open
        // the Tab5 is not discoverable as "Quarky-Tab5" for BLE C2 pairing.
        // The teardown handler in build_screen() calls
        // c2link_ble_rearm_advertising(), so C2 comes back on Back rather
        // than staying down for the rest of the boot.
        start_rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    }
    Serial.printf("quarky-tab5: [ble-findmy] ble_gap_adv_start rc=%d\n", start_rc);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Find My Emulator", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Broadcasting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        bool was_advertising = s_active;
        s_active = false;
        s_status_label = nullptr;
        int rc = ble_gap_adv_stop();
        Serial.printf("quarky-tab5: [ble-findmy] ble_gap_adv_stop rc=%d\n", rc);
        // Finding I6 (2026-08-17): give the C2 link its advertisement back --
        // rotate() took over the radio's single legacy-advertising slot (see
        // its disclosure above). Stop ours first, then re-arm, or the
        // re-arm's ble_gap_adv_start() returns BLE_HS_EALREADY and leaves the
        // emulated tag broadcasting.
        if (was_advertising) c2link_ble_rearm_advertising();
    }, LV_EVENT_DELETE, nullptr);

    // Finding C1 (2026-08-17). Without this guard, opening this screen on a
    // radios-disabled boot panics the device on its first rotation: rotate()
    // calls ble_hs_id_set_rnd(), which -- unlike ble_gap_adv_start/stop/
    // set_data -- has NO internal ble_hs_is_enabled() check and dereferences
    // a NULL npl_funcs table pointer (LoadProhibited). See ble_common.h for
    // the full disassembly-verified writeup. Must come before s_active is
    // armed, and before the s_last_rotate_ms priming below (which exists
    // specifically to make that first rotation happen immediately).
    if (!ble_require_host_ready(s_status_label)) return screen;

    s_active = true;
    // Force an immediate first rotate on the very next poll() tick.
    //
    // Finding I4 (2026-08-17): this used to be `s_last_rotate_ms = 0;` with
    // the same "force immediate first rotate" comment, which was false for
    // this file specifically. poll()'s gate is
    // `if (now - s_last_rotate_ms < s_dwell_ms) return;` -- with a zero
    // timestamp that reads `millis() < 60000`, i.e. the first advertisement
    // was SUPPRESSED for the whole first minute of device uptime while this
    // screen already said "Broadcasting...". Every other file using the same
    // `= 0` idiom (ble_spam.cpp/ble_sourapple.cpp at 200ms, ble_karma.cpp at
    // 2000ms) has a dwell short enough that the window is unnoticeable; Find
    // My's 60s dwell is the one place it was reachable and user-visible.
    // Subtracting the dwell makes the gate's subtraction come out >= dwell on
    // the first tick regardless of uptime, which is what the comment always
    // claimed. (Unsigned wraparound is fine and intentional here: for
    // millis() < s_dwell_ms this wraps to a huge value, and the gate's own
    // `now - s_last_rotate_ms` wraps back to exactly s_dwell_ms.)
    s_last_rotate_ms = millis() - s_dwell_ms;
    return screen;
}

void register_module() {
    g_registry.register_module({"ble_findmy", "Find My Emulator", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < s_dwell_ms) return;
    rotate();
    s_last_rotate_ms = now;
}

} // namespace BleFindMyFeature
