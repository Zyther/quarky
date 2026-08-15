#include "ble_finder.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis() -- needed the same way ble_scan.cpp/
                      // ble_spam.cpp explicitly pull this in; nothing else in
                      // this file's include list drags it in transitively
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleFinderFeature {

// Tracker signature lookup -- AirTag (Apple mfr ID + subtype 0x12, same
// "Find My" subtype Task 5's classifier also recognizes, but tracker-locked
// here specifically), Samsung SmartTag (mfr ID 0x0075 -- same Samsung ID
// Task 5's general classifier flags, narrowed here since not all Samsung
// devices are trackers), Tile (service UUID 0xFEED/0xFD84).
static bool is_tracker(const uint8_t *adv_data, uint8_t adv_len, const char **kind_out) {
    uint8_t i = 0;
    while (i + 1 < adv_len) {
        uint8_t field_len = adv_data[i];
        if (field_len == 0 || i + 1 + field_len > adv_len) break;
        uint8_t type = adv_data[i + 1];
        const uint8_t *payload = &adv_data[i + 2];
        uint8_t payload_len = field_len - 1;
        if (type == 0xFF && payload_len >= 4) {
            uint16_t company = payload[0] | (payload[1] << 8);
            if (company == 0x004C && payload[2] == 0x12) { *kind_out = "AirTag"; return true; }
            if (company == 0x0075) { *kind_out = "SmartTag"; return true; }
        }
        if (type == 0x16 && payload_len >= 2) {
            uint16_t svc = payload[0] | (payload[1] << 8);
            if (svc == 0xFEED || svc == 0xFD84) { *kind_out = "Tile"; return true; }
        }
        i += 1 + field_len;
    }
    return false;
}

static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;
static bool s_locked = false;
static uint8_t s_locked_addr[6];
static volatile int8_t s_locked_rssi = -128;
static volatile uint32_t s_locked_last_ms = 0;
// Single mux for ALL of this file's cross-task state (locked-target RSSI,
// the unlocked-tracker list below, and the lock/last-seen bookkeeping) --
// deliberately not split into per-state muxes. Contention is negligible
// (one producer task, one consumer task, short critical sections) and one
// mux is simpler to reason about correctly than juggling which lock guards
// which field.
static portMUX_TYPE s_lock_mux = portMUX_INITIALIZER_UNLOCKED;

// Task-review fix (post-merge finding): gap_scan_event_cb runs on the
// NimBLE host task, not the main/LVGL task. This project's LVGL port has no
// OS/mutex integration (LV_USE_OS is LV_OS_NONE, see ui/lvgl_port.cpp) --
// calling lv_list_add_button()/lv_obj_clean() directly from the host task
// (as an earlier version of this file did, transcribed verbatim from the
// brief) is a real thread-safety bug, not just a data race: heap corruption
// or an LVGL assert crash is a real risk, exactly the class of bug
// ble_scan.cpp's s_devices/add_or_update()/refresh_list_ui() split already
// exists to avoid. Mirrors that same pattern here: gap_scan_event_cb only
// ever writes into this plain, mux-guarded array and sets a dirty flag; all
// actual LVGL calls happen in refresh_tracker_list_ui(), called from
// poll() on the main/LVGL task.
static constexpr int kMaxTrackers = 16;
struct TrackerEntry {
    uint8_t addr[6];
    char addr_str[18];
    char kind[16];
    int8_t rssi;
};
static TrackerEntry s_trackers[kMaxTrackers];
static int s_tracker_count = 0;
// Single bool, same class as ble_scan.cpp's s_devices_dirty -- set (under
// the lock) whenever add_or_update_tracker() adds or updates an entry;
// cleared by poll() after a refresh so unchanged loop() ticks are cheap.
static volatile bool s_trackers_dirty = false;

// Same shape as ble_scan.cpp's add_or_update(): dedupe by address under the
// lock (fast, bounded array scan + memcpy), no LVGL involved.
static void add_or_update_tracker(const uint8_t addr[6], const char *addr_str,
                                   const char *kind, int8_t rssi) {
    portENTER_CRITICAL(&s_lock_mux);
    for (int i = 0; i < s_tracker_count; i++) {
        if (memcmp(s_trackers[i].addr, addr, 6) == 0) {
            s_trackers[i].rssi = rssi;
            s_trackers_dirty = true;
            portEXIT_CRITICAL(&s_lock_mux);
            return;
        }
    }
    if (s_tracker_count < kMaxTrackers) {
        TrackerEntry &e = s_trackers[s_tracker_count++];
        memcpy(e.addr, addr, 6);
        strncpy(e.addr_str, addr_str, sizeof(e.addr_str) - 1);
        e.addr_str[sizeof(e.addr_str) - 1] = '\0';
        strncpy(e.kind, kind, sizeof(e.kind) - 1);
        e.kind[sizeof(e.kind) - 1] = '\0';
        e.rssi = rssi;
        s_trackers_dirty = true;
    }
    portEXIT_CRITICAL(&s_lock_mux);
}

// Controller-resolution addition (see ble_finder.h's comment on
// lock_last_seen() and this task's report for the full gap writeup): as
// brief-written, s_locked never transitions to true anywhere in this file or
// main.cpp, which makes update_geiger_ui() permanently unreachable dead code
// -- the brief's own Step 4 and the comment that used to sit at the bottom of
// gap_scan_event_cb() both claimed a serial trigger did this, but no such
// trigger existed. s_last_seen_addr/s_have_last_seen record the most
// recently-seen tracker's address (updated in the same place
// gap_scan_event_cb adds a row to the not-yet-locked list below) so that
// main.cpp's new 'f' serial trigger has a real target to lock onto via
// lock_last_seen(). Written on the NimBLE host task (inside
// gap_scan_event_cb), read on the main/LVGL task (inside lock_last_seen(),
// called from the serial-trigger block in loop()) -- a genuine cross-task
// case, so folded into the same s_lock_mux the brief already uses for
// s_locked_rssi/s_locked_last_ms rather than left as a new unprotected
// access. lock_last_seen() also writes s_locked_addr/s_locked themselves
// while a scan may be actively running on the NimBLE host task (something
// the brief's own code never needed to guard against, since nothing ever
// set s_locked = true during a scan) -- both are folded under the same mux
// for the same reason, on both the write side here and the read side in
// gap_scan_event_cb's lock-comparison below.
static uint8_t s_last_seen_addr[6];
static bool s_have_last_seen = false;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    const char *kind = nullptr;
    if (!is_tracker(event->disc.data, event->disc.length_data, &kind)) return 0;

    portENTER_CRITICAL(&s_lock_mux);
    bool locked = s_locked;
    uint8_t locked_addr[6];
    memcpy(locked_addr, s_locked_addr, 6);
    portEXIT_CRITICAL(&s_lock_mux);

    if (locked && memcmp(event->disc.addr.val, locked_addr, 6) == 0) {
        portENTER_CRITICAL(&s_lock_mux);
        s_locked_rssi = event->disc.rssi;
        s_locked_last_ms = millis();
        portEXIT_CRITICAL(&s_lock_mux);
        return 0;
    }

    if (!locked) {
        char addr_str[18];
        ble_addr_to_str(event->disc.addr.val, addr_str);
        // No LVGL calls here -- see add_or_update_tracker()/TrackerEntry's
        // comment above for why. This just records the sighting; the actual
        // list rebuild happens in refresh_tracker_list_ui() on the main/LVGL
        // task, driven by poll().
        add_or_update_tracker(event->disc.addr.val, addr_str, kind, event->disc.rssi);
        // Real target-lock UI (tap a row to lock onto it) is a reasonable
        // near-future addition using lv_list's per-button click callback
        // with the row's own addr stashed in user_data -- kept out of this
        // task's own real-hardware verification loop, matching the
        // scoped-narrow pattern wifi_pmkid.cpp's brief used for EAPOL
        // filtering. Locking is demonstrated here via main.cpp's 'f' serial
        // trigger (BleFinderFeature::lock_last_seen()) instead, which locks
        // onto whichever tracker was seen most recently -- recorded right
        // here, every time a not-yet-locked sighting is recorded.
        portENTER_CRITICAL(&s_lock_mux);
        memcpy(s_last_seen_addr, event->disc.addr.val, 6);
        s_have_last_seen = true;
        portEXIT_CRITICAL(&s_lock_mux);
    }
    return 0;
}

// Snapshots s_trackers/s_tracker_count under the lock (a fast, bounded
// memcpy) and does the actual LVGL rebuild -- lv_obj_clean() plus up to
// kMaxTrackers lv_list_add_button() calls -- outside it, so the NimBLE host
// task is never blocked for longer than a plain array copy. Same "copy
// under the lock, do the real work outside it" shape ble_scan.cpp's own
// refresh_list_ui() already uses.
static void refresh_tracker_list_ui() {
    if (!s_list) return;

    static TrackerEntry snapshot[kMaxTrackers];
    int count;
    portENTER_CRITICAL(&s_lock_mux);
    count = s_tracker_count;
    memcpy(snapshot, s_trackers, sizeof(TrackerEntry) * count);
    portEXIT_CRITICAL(&s_lock_mux);

    lv_obj_clean(s_list);
    for (int i = 0; i < count; i++) {
        char row[48];
        snprintf(row, sizeof(row), "%s  (%s)  %ddBm", snapshot[i].kind, snapshot[i].addr_str, snapshot[i].rssi);
        lv_list_add_button(s_list, LV_SYMBOL_GPS, row);
    }
}

static void update_geiger_ui() {
    // s_locked read here is unprotected by design, not oversight: every
    // writer of s_locked (build_screen(), the LV_EVENT_DELETE handler, and
    // lock_last_seen()) runs on this same main/LVGL task that
    // update_geiger_ui() runs on (called from poll()) -- a same-task
    // read/write is not a cross-task race. Only gap_scan_event_cb (a
    // different task) reads s_locked, and it does so under s_lock_mux (see
    // above), which is what actually matters for correctness there.
    if (!s_list || !s_locked) return;
    portENTER_CRITICAL(&s_lock_mux);
    int8_t rssi = s_locked_rssi;
    uint32_t last_ms = s_locked_last_ms;
    portEXIT_CRITICAL(&s_lock_mux);

    const char *tier;
    if (millis() - last_ms > 4000) tier = "NO SIGNAL";
    else if (rssi > -45) tier = "RIGHT HERE";
    else if (rssi > -60) tier = "HOT";
    else if (rssi > -72) tier = "WARM";
    else if (rssi > -84) tier = "COOL";
    else tier = "COLD";

    lv_obj_clean(s_list);
    char row[48];
    snprintf(row, sizeof(row), "%s  (%ddBm)", tier, rssi);
    lv_list_add_text(s_list, row);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Tracker Finder", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) {
            // ble_gap_disc_cancel() only REQUESTS cancellation -- an
            // in-flight discovery event can still be queued and processed
            // on the NimBLE host task after this returns (ble_scan.cpp's
            // own ble_gap_disc_cancel() call documents this same window).
            // Logged for the same diagnostic-completeness reason
            // ble_scan.cpp logs its rc: a non-zero return here is a normal,
            // harmless no-op (scan already ended on its own), not a
            // functional problem.
            int rc = ble_gap_disc_cancel();
            Serial.printf("quarky-tab5: [ble-finder] ble_gap_disc_cancel rc=%d\n", rc);
            s_scanning = false;
        }
        // Task-review fix: s_locked is read under s_lock_mux by
        // gap_scan_event_cb on the NimBLE host task (see its snapshot at the
        // top of that function). Because of the in-flight-event window
        // noted above, that callback can still run concurrently with this
        // handler even after ble_gap_disc_cancel() returns -- so this write
        // must take the same mux, not just the ones lock_last_seen() and
        // gap_scan_event_cb already coordinate between themselves. s_list =
        // nullptr doesn't strictly need the lock (only ever touched on the
        // main task), but it's simplest to wrap both assignments in one
        // critical section rather than split hairs over which half needs
        // it.
        portENTER_CRITICAL(&s_lock_mux);
        s_list = nullptr;
        s_locked = false;
        portEXIT_CRITICAL(&s_lock_mux);
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_locked = false;
    // Reset the unlocked-tracker list before (re-)starting a scan -- locked
    // for uniformity with every other touch of s_trackers/s_tracker_count,
    // even though it's safe by construction here too (no producer task
    // exists until ble_gap_disc() below actually runs), same reasoning
    // ble_scan.cpp's own s_device_count reset uses.
    portENTER_CRITICAL(&s_lock_mux);
    s_tracker_count = 0;
    portEXIT_CRITICAL(&s_lock_mux);
    s_trackers_dirty = false;
    struct ble_gap_disc_params params{};
    params.passive = 1; // passive is fine here -- tracker mfr-data/service-data
                          // is in the primary advertisement, not scan-response
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-finder] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_finder", "BLE Tracker Finder", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning) return;
    if (s_locked) {
        update_geiger_ui();
        return;
    }
    // Same dirty-flag gating as ble_scan.cpp's poll(): only rebuild the
    // list on loop() ticks where gap_scan_event_cb actually added or
    // updated an entry, not on every ~5-10ms tick.
    if (!s_trackers_dirty) return;
    refresh_tracker_list_ui();
    s_trackers_dirty = false;
}

// Controller-resolution addition: the serial-trigger-only ('f' in main.cpp,
// mnemonic "finder-lock") entry point that actually exercises geiger mode.
// Runs on the main/LVGL task (called from loop()'s QUARKY_SERIAL_DEBUG
// block), so every touch of s_last_seen_addr/s_have_last_seen/s_locked_addr/
// s_locked below is under s_lock_mux -- see the s_last_seen_addr/
// s_have_last_seen comment above for why this is a genuine cross-task case
// and not just belt-and-suspenders.
void lock_last_seen() {
    portENTER_CRITICAL(&s_lock_mux);
    bool have = s_have_last_seen;
    uint8_t addr[6];
    if (have) memcpy(addr, s_last_seen_addr, 6);
    portEXIT_CRITICAL(&s_lock_mux);

    if (!have) {
        Serial.println("quarky-tab5: [debug] no scanned tracker available -- open BLE Tracker Finder first");
        return;
    }

    char addr_str[18];
    ble_addr_to_str(addr, addr_str);

    portENTER_CRITICAL(&s_lock_mux);
    memcpy(s_locked_addr, addr, 6);
    s_locked = true;
    // -128 + "now" so update_geiger_ui() doesn't immediately report NO
    // SIGNAL before the first real RSSI update arrives from
    // gap_scan_event_cb's locked-target branch.
    s_locked_rssi = -128;
    s_locked_last_ms = millis();
    portEXIT_CRITICAL(&s_lock_mux);

    Serial.printf("quarky-tab5: [debug] BleFinderFeature locked onto %s via serial trigger\n", addr_str);
}

} // namespace BleFinderFeature
