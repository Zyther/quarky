#include "ble_findmy.h"
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
        s_active = false;
        s_status_label = nullptr;
        int rc = ble_gap_adv_stop();
        Serial.printf("quarky-tab5: [ble-findmy] ble_gap_adv_stop rc=%d\n", rc);
    }, LV_EVENT_DELETE, nullptr);

    s_active = true;
    s_last_rotate_ms = 0; // force immediate first rotate in poll()
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
