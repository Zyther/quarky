#include "ble_sniffer.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial, millis(), portMUX_TYPE/portENTER_CRITICAL -- needed the
                      // same way ble_scan.cpp/ble_finder.cpp explicitly pull this in;
                      // nothing else in this file's include list drags it in
                      // transitively (the brief's own header list omitted this, which
                      // would not compile -- see this task's report for the full note)
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage;

namespace BleSnifferFeature {

// -----------------------------------------------------------------------------
// Task 7 (real-hardware review, 2026-08-15): the brief's own gap_scan_event_cb
// called storage.append_capture_file() -- a synchronous SD_MMC open/write/close
// -- directly on the NimBLE host task. Investigated against this project's two
// existing, real-hardware-established precedents before transcribing that
// verbatim:
//
//   1. wifi_evil_portal.cpp's handle_submit() does the same shape of thing (a
//      synchronous append_capture_file() call from a non-main task, AsyncTCP's)
//      and was confirmed safe -- but specifically for THREAD SAFETY, because
//      this project's FatFs build has FF_FS_REENTRANT=1 (confirmed here again
//      against the actual installed ffconf.h for esp32p4:
//      ~/.platformio/packages/framework-arduinoespressif32-libs/esp32p4/include/
//      fatfs/src/ffconf.h line 326). That answers "is concurrent SD_MMC access
//      from another task a data race" (no), not "is it safe to block THIS
//      particular task for the duration of a card write."
//
//   2. wifi_pmkid.cpp deliberately does NOT write to SD from its own
//      IRAM_ATTR'd promiscuous-mode callback, instead buffering into a
//      portMUX-locked byte ring drained by poll() on the main task -- because
//      blocking an ISR-adjacent driver callback for card-write latency risks
//      dropped packets/watchdog issues, a call-frequency/callback-criticality
//      concern, not a thread-safety one.
//
// This callback is architecturally closer to (2) than (1), for reasons checked
// directly against this project's own code, not assumed:
//
//   - No watchdog is registered against the NimBLE host task. main.cpp only
//     calls enableLoopWDT() (confirmed: cores/esp32/esp32-hal-misc.c's
//     enableLoopWDT() calls esp_task_wdt_add(loopTaskHandle) -- the Arduino
//     loop task ONLY). hal/c2link_ble.cpp's host_task() is never passed to
//     esp_task_wdt_add anywhere in this codebase, and
//     CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 (the installed esp32p4
//     sdkconfig.h) only watches the core-0 idle task. So, unlike Task 3's
//     real-hardware loop-task WDT crash, blocking the NimBLE host task cannot
//     panic-reboot the device via the task watchdog -- that specific failure
//     mode wifi_pmkid.cpp's comment warns about does not apply here the same
//     way.
//   - BUT: this same host task is the ONE AND ONLY NimBLE host in the whole
//     firmware (hal/c2link_ble.cpp's file-level comment: raw ESP-IDF NimBLE,
//     never a second host), and it is not just running this scan -- it is
//     simultaneously running c2link_ble's own GATT server (BLE_GAP_EVENT_
//     CONNECT/DISCONNECT/SUBSCRIBE, ble_gatts_notify_custom() for the C2 Tx
//     characteristic) whenever the Cardputer-ADV C2 link is connected over
//     BLE. NimBLE processes GAP/GATT/HCI events serially on this single task.
//     A passive BLE scan in a real, BLE-dense environment (unlike the evil
//     portal's rare human-triggered form submits) can produce many discovery
//     events per second -- and blocking this task for an SD open+write+close
//     (real SDIO card I/O: FAT metadata updates, potential wear-leveling
//     stalls) on EVERY one of those events risks starving this project's own
//     C2 control link (delayed connection supervision / notify delivery /
//     GAP event processing) exactly while a sniff capture is running. That
//     risk is specific to this callback -- wifi_evil_portal.cpp's AsyncTCP
//     task carries no other project-critical duty, so it had no analogous
//     concern to weigh.
//
// Conclusion: buffer, don't block. Same "host-task-writes-a-locked-buffer /
// poll()-does-the-heavy-work-on-main-task" shape ble_scan.cpp/ble_finder.cpp
// already use for LVGL calls off this same task, and the same byte-oriented
// portMUX ring wifi_pmkid.cpp uses to keep its own driver callback off SD I/O
// -- reused here because CSV rows, like pcap records, have no record-boundary
// requirement: draining an arbitrary byte count and stopping mid-row is fine,
// the row's remainder just sits in the ring for the next poll() call, and the
// file ends up byte-for-byte identical to what synchronous per-event writes
// would have produced.
// -----------------------------------------------------------------------------

constexpr size_t kRingBufferSize = 4096; // CSV rows are <=~100 bytes each (see
                                          // row[128] below); this buffers
                                          // several dozen rows between drains
constexpr size_t kMaxDrainPerPollBytes = 2048; // bounds poll()'s SD write time
                                                 // per call, same reasoning as
                                                 // wifi_pmkid.cpp's own cap

static uint8_t s_ring_buf[kRingBufferSize];
static volatile size_t s_ring_head = 0; // producer writes here (NimBLE host task)
static volatile size_t s_ring_tail = 0; // consumer reads here (main task, via poll())
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_dropped_count = 0; // rows dropped because the ring was full

static char s_path[64];
// s_row_count: rows successfully queued (written only by gap_scan_event_cb on
// the NimBLE host task, read only by poll() on the main task) -- single
// writer, so volatile alone suffices, matching ble_scan.cpp's s_devices_dirty
// precedent. s_dropped_count is the same shape for the same reason.
static volatile uint32_t s_row_count = 0;
static bool s_scanning = false;
static lv_obj_t *s_status_label = nullptr;

// Caller must hold s_ring_mux.
static size_t ring_used_locked() {
    return (s_ring_head - s_ring_tail + kRingBufferSize) % kRingBufferSize;
}

// Appends len bytes to the ring buffer. Bounds-checked, drop-and-count if it
// doesn't fit -- never blocks, never allocates. Called only from
// gap_scan_event_cb (the single producer, on the NimBLE host task).
static void ring_push(const uint8_t *data, size_t len) {
    portENTER_CRITICAL(&s_ring_mux);
    // -1 leaves a one-byte gap so a full ring (head caught up to tail) can
    // never look identical to an empty one (head == tail).
    size_t free_bytes = kRingBufferSize - 1 - ring_used_locked();
    if (len > free_bytes) {
        s_dropped_count++;
        portEXIT_CRITICAL(&s_ring_mux);
        return;
    }
    size_t first_chunk = kRingBufferSize - s_ring_head;
    if (first_chunk > len) first_chunk = len;
    memcpy(&s_ring_buf[s_ring_head], data, first_chunk);
    if (first_chunk < len) {
        memcpy(&s_ring_buf[0], data + first_chunk, len - first_chunk);
    }
    s_ring_head = (s_ring_head + len) % kRingBufferSize;
    portEXIT_CRITICAL(&s_ring_mux);
}

// Runs on the NimBLE host task (not the main/LVGL task) -- see the file-level
// comment above for the full blocking-risk analysis. Only ever touches the
// plain, mux-guarded ring buffer; the actual SD write happens in poll() on
// the main task.
static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    char addr_str[18];
    ble_addr_to_str(event->disc.addr.val, addr_str);

    // adv_hex: the full raw advertisement payload as a hex string, matching
    // Poseidon's own CSV format exactly (ms,mac,rssi,addr_type,name is
    // omitted here since name requires a separate parse pass -- this row
    // keeps the raw bytes, which is the analytically useful part for
    // offline tooling, over a partially-duplicated name field).
    char hex[62]; // up to 31 bytes * 2 hex chars, generous for legacy adv max
    uint8_t len = event->disc.length_data < 31 ? event->disc.length_data : 31;
    for (uint8_t i = 0; i < len; i++) {
        snprintf(hex + i * 2, 3, "%02X", event->disc.data[i]);
    }
    hex[len * 2] = '\0';

    char row[128];
    // Deviation from the brief (real-hardware review finding, same bug class
    // wifi_evil_portal.cpp's handle_submit() was already fixed for on
    // 2026-08-15): snprintf's return value is how many characters WOULD have
    // been written had the buffer been large enough (C99/C11), not how many
    // actually were -- using it directly as ring_push()'s length argument
    // would over-read past row's end on truncation. row is always
    // NUL-terminated by snprintf even on truncation, so strlen() is the real
    // written length. Not expected to truncate in practice (worst case is
    // well under 100 of row's 128 bytes), but cheap to get right rather than
    // copy a known-fixed bug forward.
    snprintf(row, sizeof(row), "%lu,%s,%d,%d,%s\n",
             (unsigned long)millis(), addr_str, (int)event->disc.rssi,
             (int)event->disc.addr.type, hex);
    ring_push((const uint8_t *)row, strlen(row));
    s_row_count++;
    return 0;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Sniffer (CSV)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Starting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        if (s_scanning) {
            int rc = ble_gap_disc_cancel();
            // Logged for the same diagnostic-completeness reason ble_scan.cpp/
            // ble_finder.cpp log this: a non-zero rc here is a normal, harmless
            // no-op (scan already ended on its own), not a functional problem.
            Serial.printf("quarky-tab5: [ble-sniffer] ble_gap_disc_cancel rc=%d\n", rc);
            s_scanning = false;
        }
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_label_set_text(s_status_label, "BLE host not ready yet, try again shortly");
        return screen;
    }

    snprintf(s_path, sizeof(s_path), "/quarky/captures/ble/sniff_%lu.csv", millis());
    s_row_count = 0;
    s_dropped_count = 0;
    s_ring_head = 0;
    s_ring_tail = 0;

    struct ble_gap_disc_params params{};
    params.passive = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-sniffer] ble_gap_disc rc=%d path=%s\n", rc, s_path);
    s_scanning = (rc == 0);
    if (!s_scanning) lv_label_set_text(s_status_label, "Scan failed to start");

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_sniffer", "BLE Sniffer (CSV)", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_status_label) return; // sniffer screen not open

    // Drain the ring buffer into the CSV file, capped per call so this never
    // risks the ~50ms loop() budget (Global Constraint) even if the ring is
    // full. Same shape as wifi_pmkid.cpp's own poll(): a raw byte stream,
    // appended verbatim, so draining a partial row is safe -- the remainder
    // just sits in the ring for the next call.
    static uint8_t s_drain_buf[kMaxDrainPerPollBytes];
    size_t to_drain;
    portENTER_CRITICAL(&s_ring_mux);
    size_t used = ring_used_locked();
    to_drain = used < kMaxDrainPerPollBytes ? used : kMaxDrainPerPollBytes;
    if (to_drain > 0) {
        size_t first_chunk = kRingBufferSize - s_ring_tail;
        if (first_chunk > to_drain) first_chunk = to_drain;
        memcpy(s_drain_buf, &s_ring_buf[s_ring_tail], first_chunk);
        if (first_chunk < to_drain) {
            memcpy(s_drain_buf + first_chunk, &s_ring_buf[0], to_drain - first_chunk);
        }
        s_ring_tail = (s_ring_tail + to_drain) % kRingBufferSize;
    }
    portEXIT_CRITICAL(&s_ring_mux);

    if (to_drain > 0) {
        if (!storage.append_capture_file(s_path, s_drain_buf, to_drain)) {
            Serial.println("quarky-tab5: [ble-sniffer] append_capture_file FAILED");
        }
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Capturing... %lu rows (%lu dropped)",
             (unsigned long)s_row_count, (unsigned long)s_dropped_count);
    lv_label_set_text(s_status_label, buf);
}

} // namespace BleSnifferFeature
