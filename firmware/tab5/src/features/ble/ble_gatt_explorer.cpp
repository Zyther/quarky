#include "ble_gatt_explorer.h"
#include "ble_central.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis() -- same as every other BLE feature file
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h> // ble_gattc_disc_all_svcs/ble_gattc_disc_all_chrs.
                            // NOT <host/ble_gattc.h> -- no such header exists in
                            // the ESP-IDF NimBLE tree the P4 Arduino libs ship;
                            // the client-side ble_gattc_* API lives in
                            // ble_gatt.h alongside the server-side ble_gatts_*
                            // API c2link_ble.cpp already uses. Same finding
                            // Task 1's ble_central_spike.cpp already made.
#include <host/ble_hs.h>
#include <host/ble_uuid.h> // ble_uuid_to_str/BLE_UUID_STR_LEN -- transitively
                            // pulled in by ble_gatt.h too, but included
                            // explicitly for clarity, matching
                            // ble_central_spike.cpp's own include list.
#include <cstring>
#include <cstdio>

extern FeatureRegistry g_registry;

// -----------------------------------------------------------------------------
// GATT explorer (second Phase 2 plan, Task 13): scan for BLE peripherals,
// connect to one via BleCentral::connect() (Task 1), and enumerate its GATT
// services/characteristics on screen.
//
// Threading: gap_scan_event_cb, gap_event_cb, svc_disc_cb, and chr_disc_cb all
// run on the NimBLE host task, not the main/LVGL task -- this project's LVGL
// port has no OS/mutex integration (LV_USE_OS is LV_OS_NONE, see
// ui/lvgl_port.cpp), so none of them may call any lv_* function directly. This
// mirrors the fix ble_finder.cpp's gap_scan_event_cb needed (its task review
// caught the identical mistake): host-task callbacks only ever write into a
// plain, portMUX-guarded buffer plus a dirty flag; the actual LVGL rebuild
// happens in a poll()-driven refresh function on the main task. This file has
// two independent pieces of such state -- the pre-connect target-picker list
// (s_targets, guarded by s_targets_mux, refreshed by refresh_target_list_ui())
// and the post-connect discovery log (s_log_rows, guarded by its own
// s_log_mux, refreshed by refresh_log_ui()) -- kept as two separate mutexes
// rather than one shared lock. This is NOT what ble_scan.cpp does (that file
// has only one cross-task state cluster and one mutex, so it doesn't
// demonstrate a multi-mutex pattern either way); it's closer in spirit to
// ble_finder.cpp choosing a single shared mutex for ITS one tightly-related
// state cluster -- the difference here is that s_targets and s_log_rows are
// two unrelated arrays with non-overlapping lifetimes (the log only starts
// filling once the target list is done being read), so splitting them keeps
// each critical section scoped to the data it actually protects rather than
// forcing an artificial relationship between them.
// -----------------------------------------------------------------------------
namespace BleGattExplorerFeature {

static constexpr int kMaxTargets = 16;
static BleDeviceInfo s_targets[kMaxTargets];
static int s_target_count = 0;
static portMUX_TYPE s_targets_mux = portMUX_INITIALIZER_UNLOCKED;

static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// -----------------------------------------------------------------------------
// Discovery log: an append-only, ordered row buffer covering connect status,
// service discovery, and characteristic discovery (characteristics need to
// stay visually grouped under their own service, so this is one combined log
// rather than three separate pieces of state). Written under s_log_mux by
// svc_disc_cb/chr_disc_cb/gap_event_cb (NimBLE host task); read/cleared under
// the same lock by refresh_log_ui() (main task, via poll()). Bounded and
// drop-and-count on overflow rather than silently corrupting memory or
// blocking the host task -- same "ring buffer with a dropped-count" shape
// ble_sniffer.cpp's ring_push() uses, simplified to a flat array since rows
// here are small and fixed-size rather than variable-length binary data.
// -----------------------------------------------------------------------------
static constexpr int kMaxLogRows = 64;
static char s_log_rows[kMaxLogRows][64];
static int s_log_row_count = 0;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_log_dirty = false;
static volatile uint32_t s_log_dropped_count = 0;

// Called only from NimBLE host-task callbacks (svc_disc_cb, chr_disc_cb,
// gap_event_cb's BLE_GAP_EVENT_CONNECT handling). Never touches LVGL --
// just records the row and marks the log dirty for refresh_log_ui() to pick
// up on the main task.
static void append_log(const char *text) {
    portENTER_CRITICAL(&s_log_mux);
    if (s_log_row_count < kMaxLogRows) {
        strncpy(s_log_rows[s_log_row_count], text, sizeof(s_log_rows[0]) - 1);
        s_log_rows[s_log_row_count][sizeof(s_log_rows[0]) - 1] = '\0';
        s_log_row_count++;
        s_log_dirty = true;
    } else {
        s_log_dropped_count++;
    }
    portEXIT_CRITICAL(&s_log_mux);
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != nullptr) {
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&chr->uuid.u, uuid_str);
        char row[64];
        snprintf(row, sizeof(row), "  chr %s (handle %u)", uuid_str, chr->val_handle);
        append_log(row);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr) {
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&service->uuid.u, uuid_str);
        char row[64];
        snprintf(row, sizeof(row), "svc %s", uuid_str);
        append_log(row);
        int rc = ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, chr_disc_cb, nullptr);
        Serial.printf("quarky-tab5: [gatt-explorer] ble_gattc_disc_all_chrs rc=%d\n", rc);
    } else if (error->status == BLE_HS_EDONE) {
        append_log("-- discovery complete --");
    }
    return 0;
}

// Runs on the NimBLE host task -- see the file-level threading comment above.
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            append_log("-- connected, discovering --");
            int rc = ble_gattc_disc_all_svcs(s_conn_handle, svc_disc_cb, nullptr);
            Serial.printf("quarky-tab5: [gatt-explorer] ble_gattc_disc_all_svcs rc=%d\n", rc);
        } else {
            char row[32];
            snprintf(row, sizeof(row), "connect failed status=%d", event->connect.status);
            append_log(row);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        return 0;
    default:
        return 0;
    }
}

// Runs on the NimBLE host task. addr_type is recorded per-target (Bug 2 fix,
// see the file-level notes in the task brief): ble_gap_connect() needs the
// peer's REAL advertised address type, not a hardcoded BLE_ADDR_PUBLIC
// assumption -- this is the exact false-negative risk BleCentral's own header
// comment warns about, and ble_scan.cpp's gap_scan_event_cb already
// establishes the fix pattern (d.addr_type = event->disc.addr.type).
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
    for (int i = 0; i < s_target_count; i++) if (memcmp(s_targets[i].addr, d.addr, 6) == 0) { dup = true; break; }
    if (!dup && s_target_count < kMaxTargets) s_targets[s_target_count++] = d;
    portEXIT_CRITICAL(&s_targets_mux);
    return 0;
}

// Runs on the main/LVGL task (called from the target-list button's
// LV_EVENT_CLICKED handler, itself dispatched by LVGL from the main task) --
// so lv_obj_clean() here is fine, unlike the callbacks above. Resets the
// discovery log state so a fresh connect attempt starts with an empty log.
static int s_log_rows_rendered = 0; // main-task-only cursor into s_log_rows;
                                     // no lock needed, see refresh_log_ui().

static void connect_to(int index) {
    portENTER_CRITICAL(&s_targets_mux);
    ble_addr_t target{};
    target.type = s_targets[index].addr_type; // Bug 2 fix: real peer address
                                                // type, not a hardcoded
                                                // BLE_ADDR_PUBLIC assumption.
    memcpy(target.val, s_targets[index].addr, 6);
    portEXIT_CRITICAL(&s_targets_mux);

    if (s_list) lv_obj_clean(s_list);

    portENTER_CRITICAL(&s_log_mux);
    s_log_row_count = 0;
    s_log_dirty = false;
    s_log_dropped_count = 0;
    portEXIT_CRITICAL(&s_log_mux);
    s_log_rows_rendered = 0;

    BleCentral::connect(target, 5000, gap_event_cb, nullptr);
}

// Snapshots s_targets/s_target_count under the lock and does the actual LVGL
// rebuild outside it -- same "copy under the lock, do the real work outside
// it" shape ble_scan.cpp's refresh_list_ui() uses. Only runs while scanning
// (gated by poll()); connect_to() switches the screen over to the discovery
// log, at which point this stops being called (s_scanning is cleared by the
// button handler before connect_to() runs).
static void refresh_target_list_ui() {
    if (!s_list) return;

    static BleDeviceInfo snapshot[kMaxTargets];
    int count;
    portENTER_CRITICAL(&s_targets_mux);
    count = s_target_count;
    memcpy(snapshot, s_targets, sizeof(BleDeviceInfo) * count);
    portEXIT_CRITICAL(&s_targets_mux);

    lv_obj_clean(s_list);
    for (int i = 0; i < count; i++) {
        char row[48];
        const char *label = snapshot[i].name[0] ? snapshot[i].name : snapshot[i].addr_str;
        snprintf(row, sizeof(row), "%s  %ddBm", label, snapshot[i].rssi);
        lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_BLUETOOTH, row);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            int idx = (int)(intptr_t)lv_event_get_user_data(e);
            if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
            connect_to(idx);
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

// Snapshots s_log_rows under the lock and appends only the NEW rows (since
// s_log_rows_rendered, a main-task-only cursor) to s_list on the main task --
// append-only rendering matches the append-only growth of the underlying
// log, avoiding an lv_obj_clean()+full-rebuild every tick the way
// refresh_target_list_ui() does for its replace-on-refresh device list.
// Independent of s_scanning: discovery/connect activity continues well after
// scanning itself has stopped, so this must not be gated on s_scanning the
// way refresh_target_list_ui() is.
static void refresh_log_ui() {
    if (!s_list) return;

    static char snapshot[kMaxLogRows][64];
    int count;
    uint32_t dropped;
    portENTER_CRITICAL(&s_log_mux);
    count = s_log_row_count;
    memcpy(snapshot, s_log_rows, sizeof(char) * 64 * count);
    dropped = s_log_dropped_count;
    s_log_dirty = false;
    portEXIT_CRITICAL(&s_log_mux);

    for (int i = s_log_rows_rendered; i < count; i++) {
        lv_list_add_text(s_list, snapshot[i]);
    }
    s_log_rows_rendered = count;

    if (dropped > 0) {
        Serial.printf("quarky-tab5: [gatt-explorer] %lu discovery row(s) dropped, log full\n",
                      (unsigned long)dropped);
    }
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("GATT Explorer", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    // s_list must not outlive the screen -- same LV_EVENT_DELETE teardown
    // shape ble_scan.cpp/ble_finder.cpp use. Cancels an in-progress scan AND
    // disconnects a live connection, covering every path that can destroy
    // this screen (not just a Back tap). Runs on the main task (LVGL always
    // fires LV_EVENT_DELETE from the task that destroys the object, here the
    // main task via ScreenStack::pop()), so this is not a new cross-task
    // hazard -- the lock-guarded log buffer above already ensures no
    // host-task callback ever touches s_list directly, so a callback firing
    // after this handler runs (e.g. a late BLE_GAP_EVENT_DISCONNECT) just
    // writes into s_log_rows/s_conn_handle harmlessly with nothing left to
    // render it.
    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) BleCentral::disconnect(s_conn_handle);
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_target_count = 0;
    s_log_rows_rendered = 0;
    portENTER_CRITICAL(&s_log_mux);
    s_log_row_count = 0;
    s_log_dirty = false;
    s_log_dropped_count = 0;
    portEXIT_CRITICAL(&s_log_mux);

    struct ble_gap_disc_params params{};
    params.passive = 0;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [gatt-explorer] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);
    return screen;
}

void register_module() {
    g_registry.register_module({"ble_gatt_explorer", "GATT Explorer", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    // Target-picker list: only relevant while the initial scan is running,
    // matching ble_scan.cpp's own throttled-refresh shape.
    if (s_scanning && s_list) {
        static uint32_t last_refresh = 0;
        if (millis() - last_refresh >= 500) {
            last_refresh = millis();
            refresh_target_list_ui();
        }
    }

    // Discovery log: independent of s_scanning -- connect/service/
    // characteristic discovery all happen after scanning has already
    // stopped, so this must keep running regardless of s_scanning's state.
    if (s_log_dirty && s_list) {
        refresh_log_ui();
    }
}

} // namespace BleGattExplorerFeature
