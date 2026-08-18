#include "ble_target_picker.h"
#include "ble_common.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis(), portMUX_TYPE/portENTER_CRITICAL -- pulled in
                      // explicitly the same way ble_scan.cpp/ble_clone.cpp/
                      // ble_gatt_explorer.cpp do; nothing else here drags it in
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>
#include <cstdio>

// -----------------------------------------------------------------------------
// See ble_target_picker.h for WHY this component exists. This file is the
// extraction of the scan/lock/tap-to-select half of ble_gatt_explorer.cpp,
// which was (with ble_clone.cpp) the proven reference implementation.
//
// Threading, restated here because it is the one thing a future editor must
// not get wrong: gap_scan_event_cb runs on the NimBLE HOST task, not the
// main/LVGL task. This project's LVGL port has no OS/mutex integration
// (LV_USE_OS is LV_OS_NONE, see ui/lvgl_port.cpp), so that callback may not
// call any lv_* function -- it only ever writes into the portMUX-guarded
// s_targets array. Every LVGL call in this file happens in
// refresh_target_list_ui() or build_screen(), both main-task-only. Same split
// ble_scan.cpp / ble_finder.cpp / ble_gatt_explorer.cpp all use.
// -----------------------------------------------------------------------------
namespace BleTargetPicker {

static constexpr int kMaxTargets = 16;
static BleDeviceInfo s_targets[kMaxTargets];
static int s_target_count = 0;
static portMUX_TYPE s_targets_mux = portMUX_INITIALIZER_UNLOCKED;

static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;

// Main-task-only: written by start(), read by the row click handler. Never
// touched from the host task.
static TargetSelectedFn s_on_selected = nullptr;

// Runs on the NimBLE host task -- see the threading note above.
//
// addr_type is recorded from event->disc.addr.type rather than assumed to be
// BLE_ADDR_PUBLIC. This is the established fix for a real bug class in this
// codebase (ble_central_spike.cpp / ble_clone.cpp / ble_gatt_explorer.cpp /
// ble_flood.cpp / ble_hfp_exploit.cpp all carry their own note about it):
// ble_gap_connect() needs the peer's REAL advertised address type, and most
// modern peripherals advertise random addresses, so a hardcoded PUBLIC type
// silently fails to connect and reads to the user as "target not vulnerable".
// Getting this right ONCE, here, is a large part of the point of sharing the
// picker -- every feature the picker feeds now inherits the correct type.
static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    BleDeviceInfo d{};
    memcpy(d.addr, event->disc.addr.val, 6);
    d.addr_type = event->disc.addr.type;
    ble_addr_to_str(d.addr, d.addr_str);
    d.rssi = event->disc.rssi;
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
        if (fields.name != nullptr && fields.name_len > 0) {
            int len = fields.name_len < (int)sizeof(d.name) - 1 ? fields.name_len : (int)sizeof(d.name) - 1;
            memcpy(d.name, fields.name, len);
            d.name[len] = '\0';
        }
    }

    portENTER_CRITICAL(&s_targets_mux);
    bool dup = false;
    for (int i = 0; i < s_target_count; i++) {
        if (memcmp(s_targets[i].addr, d.addr, 6) == 0) {
            // Refresh RSSI in place so the list reads as live rather than
            // frozen at first sighting. Entries are never reordered or
            // removed, which is what makes the row index stashed in
            // lv_event_get_user_data() stable across refreshes.
            s_targets[i].rssi = d.rssi;
            if (s_targets[i].name[0] == '\0' && d.name[0] != '\0') {
                memcpy(s_targets[i].name, d.name, sizeof(d.name));
            }
            dup = true;
            break;
        }
    }
    if (!dup && s_target_count < kMaxTargets) s_targets[s_target_count++] = d;
    portEXIT_CRITICAL(&s_targets_mux);
    return 0;
}

// Main/LVGL task only (LV_EVENT_CLICKED on a row button, dispatched from
// loop()'s lv_timer_handler()).
static void select_target(int index) {
    BleDeviceInfo chosen{};
    bool valid = false;
    portENTER_CRITICAL(&s_targets_mux);
    if (index >= 0 && index < s_target_count) {
        chosen = s_targets[index];
        valid = true;
    }
    portEXIT_CRITICAL(&s_targets_mux);
    if (!valid) return;

    // Copy the callback out before popping: the pop below destroys this
    // screen and runs its LV_EVENT_DELETE teardown, and the whole point of
    // reading it first is that nothing after the pop may depend on picker
    // state still being live.
    TargetSelectedFn cb = s_on_selected;

    // Stop the scan before handing off -- the feature we are about to enter
    // immediately starts its own connect attempt, and leaving discovery
    // running underneath it wastes radio time and can slow the connect. The
    // teardown handler below will (harmlessly) try again; a second
    // ble_gap_disc_cancel() on an already-stopped scan is a no-op returning a
    // nonzero rc, same window ble_scan.cpp/ble_finder.cpp already document.
    if (s_scanning) {
        int rc = ble_gap_disc_cancel();
        Serial.printf("quarky-tab5: [ble-picker] ble_gap_disc_cancel rc=%d\n", rc);
        s_scanning = false;
    }

    Serial.printf("quarky-tab5: [ble-picker] target selected %s (addr type=%u)\n",
                  chosen.addr_str, chosen.addr_type);

    // Pop FIRST, then hand off: the feature's own screen replaces this one on
    // the stack rather than stacking on top of a dead scan list. See the
    // SCREEN CHOREOGRAPHY note in ble_target_picker.h for why this is safe to
    // do from inside a click handler on one of this screen's own descendants.
    ScreenStack::pop();

    if (cb != nullptr) cb(chosen);
}

// Snapshots s_targets/s_target_count under the lock and does the actual LVGL
// rebuild outside it -- the same "copy under the lock, do the real work
// outside it" shape ble_scan.cpp's refresh_list_ui() and
// ble_gatt_explorer.cpp's refresh_target_list_ui() both use, so the NimBLE
// host task is never blocked for longer than a plain array copy.
static void refresh_target_list_ui() {
    if (!s_list) return;

    static BleDeviceInfo snapshot[kMaxTargets];
    int count;
    portENTER_CRITICAL(&s_targets_mux);
    count = s_target_count;
    memcpy(snapshot, s_targets, sizeof(BleDeviceInfo) * count);
    portEXIT_CRITICAL(&s_targets_mux);

    lv_obj_clean(s_list);

    // Discoverability is the entire reason this component exists, so say what
    // to do rather than presenting a bare list and hoping.
    lv_list_add_text(s_list, count > 0 ? "Tap a target to continue" : "Scanning for targets...");

    for (int i = 0; i < count; i++) {
        char row[56];
        const char *label = snapshot[i].name[0] ? snapshot[i].name : snapshot[i].addr_str;
        snprintf(row, sizeof(row), "%s  %s  %ddBm", label, snapshot[i].addr_str, snapshot[i].rssi);
        lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_BLUETOOTH, row);
        // Row index (not a pointer into s_targets) stashed as user_data, same
        // as ble_gatt_explorer.cpp / ble_clone.cpp: entries are append-only
        // and never reordered, so the index stays valid across refreshes, and
        // select_target() re-reads the entry under the lock anyway.
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            select_target((int)(intptr_t)lv_event_get_user_data(e));
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

static lv_obj_t *build_screen(const char *screen_title) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(screen_title, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    // s_list must not outlive the screen -- same LV_EVENT_DELETE teardown
    // shape every other BLE screen in this tree uses, covering every path
    // that can destroy this screen (a Back tap, and select_target()'s own
    // ScreenStack::pop()), not just Back.
    //
    // Deliberately SIMPLER than ble_gatt_explorer.cpp's three-state teardown:
    // that file's screen can be destroyed while a connect attempt is in
    // flight or a connection is live, which is what its unconditional-500ms
    // ble_gap_conn_cancel() deadline loop exists to close. The picker never
    // connects to anything -- it only ever runs a discovery scan, and hands
    // the target to a feature whose OWN screen (and own already-proven
    // connect-cancel teardown) takes over from there. So the only resource
    // this handler has to release is the scan. Adding a connect-cancel loop
    // here would be cargo-culting: it would burn 500ms on every Back tap
    // cancelling an attempt this file is structurally incapable of starting.
    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) {
            // ble_gap_disc_cancel() only REQUESTS cancellation -- an in-flight
            // discovery event can still be queued and processed on the NimBLE
            // host task after this returns. That is harmless here precisely
            // because gap_scan_event_cb touches nothing but the mux-guarded
            // s_targets array: with s_list nulled below, a late sighting just
            // writes into a buffer nothing will render. Logged for the same
            // diagnostic-completeness reason ble_scan.cpp/ble_finder.cpp log
            // theirs; a nonzero rc here is a normal no-op (scan already ended
            // on its own after its 10s window), not a fault.
            int rc = ble_gap_disc_cancel();
            Serial.printf("quarky-tab5: [ble-picker] ble_gap_disc_cancel rc=%d\n", rc);
            s_scanning = false;
        }
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    // Finding C1 (2026-08-17): the shared host-ready guard, called after the
    // teardown handler is registered and before the first NimBLE call, per
    // ble_common.h's contract.
    //
    // This is now the guard that protects all four features the picker feeds.
    // Previously each of them was only ACCIDENTALLY safe on a radios-disabled
    // boot (their start() bailed because BLE Scan could never have populated
    // slot 0). Their own build_screen() guards are still in place and
    // unchanged; this one simply means the user never even reaches them.
    if (!ble_require_host_ready_list(s_list)) return screen;

    portENTER_CRITICAL(&s_targets_mux);
    s_target_count = 0;
    portEXIT_CRITICAL(&s_targets_mux);

    struct ble_gap_disc_params params{};
    params.passive = 0; // active scan: pull scan-response data so named
                         // devices show a name, not just an address
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-picker] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);
    if (rc != 0) {
        char msg[64];
        snprintf(msg, sizeof(msg), "scan could not start rc=%d", rc);
        lv_list_add_text(s_list, msg);
    } else {
        lv_list_add_text(s_list, "Scanning for targets...");
    }

    return screen;
}

void start(const char *screen_title, TargetSelectedFn on_selected) {
    s_on_selected = on_selected;
    ScreenStack::push(build_screen(screen_title));
}

void poll() {
    // Only relevant while the picker screen is open and its scan is running,
    // matching ble_clone.cpp / ble_gatt_explorer.cpp's own throttled refresh.
    if (!s_scanning || !s_list) return;
    static uint32_t last_refresh = 0;
    if (millis() - last_refresh < 500) return;
    last_refresh = millis();
    refresh_target_list_ui();
}

} // namespace BleTargetPicker
