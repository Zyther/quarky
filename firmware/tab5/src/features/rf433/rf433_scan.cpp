#include "rf433_scan.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../hal/storage_sd.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage;

namespace Rf433Scan {

// Session signals buffer
static CapturedSignal s_signals[kMaxCapturedSignals];
static size_t s_signal_count = 0;

// UI elements
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_toggle_btn = nullptr;
static lv_obj_t *s_toggle_label = nullptr;
static lv_obj_t *s_list = nullptr;
static lv_obj_t *s_placeholder = nullptr; // "No signals captured yet" row, if shown

static bool s_active = false;
static uint32_t s_last_edge_time_us = 0;
static size_t s_accum_edge_count = 0;
static Rf433Common::EdgeSample s_accum_edges[kMaxEdgesPerSignal];

// Gap threshold (in microseconds) to delimit end of an RF burst
// Standard OOK remotes repeat bursts separated by 8ms - 15ms of silence
constexpr uint32_t kBurstGapThresholdUs = 10000; // 10ms
constexpr size_t kMinEdgesForSignal = 10;        // Filter spurious single glitch transitions

static void update_status_ui() {
    if (!s_status_label || !s_toggle_label) return;
    if (s_active) {
        lv_label_set_text(s_status_label, "Status: Capturing (GPIO53)...");
        lv_label_set_text(s_toggle_label, "Stop Capture");
    } else {
        lv_label_set_text(s_status_label, "Status: Idle");
        lv_label_set_text(s_toggle_label, "Start Capture");
    }
}

static void set_capture_active(bool active) {
    if (s_active == active) return;
    s_active = active;
    if (active) {
        s_accum_edge_count = 0;
        s_last_edge_time_us = 0;
        Rf433Common::capture_start();
    } else {
        Rf433Common::capture_stop();
    }
    update_status_ui();
}

static void add_signal_to_list(const CapturedSignal &sig, size_t index) {
    if (!s_list) return;

    // Drop the "nothing yet" placeholder the moment a real row exists.
    if (s_placeholder) {
        lv_obj_delete(s_placeholder);
        s_placeholder = nullptr;
    }

    // Keep the widget bounded in lockstep with the s_signals ring (finalize_burst
    // caps the array at kMaxCapturedSignals). Without this the list grew a button
    // per burst forever, leaking heap until LVGL's allocator failed. Delete the
    // oldest row before appending the newest.
    while (lv_obj_get_child_count(s_list) >= kMaxCapturedSignals) {
        lv_obj_t *oldest = lv_obj_get_child(s_list, 0);
        if (!oldest) break;
        lv_obj_delete(oldest);
    }

    // Calculate approx burst duration in milliseconds
    uint32_t duration_us = 0;
    if (sig.edge_count > 1) {
        duration_us = sig.edges[sig.edge_count - 1].timestamp_us - sig.edges[0].timestamp_us;
    }

    char row[80];
    std::snprintf(row, sizeof(row), "Sig #%u: %u edges, ~%lu ms (@%lu s)",
                  (unsigned)(index + 1),
                  (unsigned)sig.edge_count,
                  (unsigned long)(duration_us / 1000),
                  (unsigned long)(sig.captured_at_ms / 1000));

    lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_AUDIO, row);
    lv_obj_add_event_cb(btn, [](lv_event_t *e) {
        size_t idx = (size_t)(uintptr_t)lv_event_get_user_data(e);
        Serial.printf("quarky-tab5: [rf433-scan] Selected signal #%u (%u edges)\n",
                      (unsigned)(idx + 1), (unsigned)s_signals[idx].edge_count);
    }, LV_EVENT_CLICKED, (void *)(uintptr_t)index);
}

static void finalize_burst() {
    if (s_accum_edge_count < kMinEdgesForSignal) {
        // Discard short noise glitch
        s_accum_edge_count = 0;
        return;
    }

    // Shift if buffer full
    if (s_signal_count >= kMaxCapturedSignals) {
        for (size_t i = 1; i < kMaxCapturedSignals; i++) {
            s_signals[i - 1] = s_signals[i];
        }
        s_signal_count = kMaxCapturedSignals - 1;
    }

    size_t new_idx = s_signal_count++;
    s_signals[new_idx].edge_count = s_accum_edge_count;
    s_signals[new_idx].captured_at_ms = millis();
    std::memcpy(s_signals[new_idx].edges, s_accum_edges,
                sizeof(Rf433Common::EdgeSample) * s_accum_edge_count);

    Serial.printf("quarky-tab5: [rf433-scan] Captured burst #%u: %u edges\n",
                  (unsigned)s_signal_count, (unsigned)s_accum_edge_count);

    // Save capture to SD storage if available
    char filename[64];
    std::snprintf(filename, sizeof(filename), "/quarky/captures/rf433/sig_%lu_%u.raw",
                  (unsigned long)millis(), (unsigned)s_accum_edge_count);
    storage.write_capture_file(filename,
                               (const uint8_t *)s_signals[new_idx].edges,
                               sizeof(Rf433Common::EdgeSample) * s_accum_edge_count);

    add_signal_to_list(s_signals[new_idx], new_idx);
    s_accum_edge_count = 0;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("RF433 Scan", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Status: Idle");

    s_toggle_btn = lv_button_create(content);
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_label_set_text(s_toggle_label, "Start Capture");
    lv_obj_add_event_cb(s_toggle_btn, [](lv_event_t *) {
        set_capture_active(!s_active);
    }, LV_EVENT_CLICKED, nullptr);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    // Populate existing session signals if reopening
    if (s_signal_count == 0) {
        s_placeholder = lv_list_add_text(s_list, "No signals captured yet");
    } else {
        for (size_t i = 0; i < s_signal_count; i++) {
            add_signal_to_list(s_signals[i], i);
        }
    }

    // Teardown on delete (e.g. Back button tapped)
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        if (s_active) {
            set_capture_active(false);
        }
        s_status_label = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        s_list = nullptr;
        s_placeholder = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() {
    ScreenStack::push(build_screen());
}

void register_module() {
    g_registry.register_module({"rf433_scan", "RF433 Scan/Capture",
                                 Category::RF433, Affinity::TAB5_NATIVE,
                                 start, nullptr});
}

void poll() {
    if (!s_active) return;

    constexpr size_t kDrainChunk = 64;
    // Hard cap on edges processed per poll() tick. Without it the drain loop
    // below re-reads the ring until it is empty, which under a continuous or
    // noisy 433MHz source (or a floating RF433R pin) never happens within one
    // tick -- poll() then never returns to loop(), starving the task watchdog
    // into a reboot. One signal's worth of edges per tick keeps pace with real
    // OOK traffic while guaranteeing poll() yields; any backlog is drained on
    // the next tick (the ring + gap state persist across ticks).
    constexpr size_t kMaxEdgesPerPoll = kMaxEdgesPerSignal;
    Rf433Common::EdgeSample chunk[kDrainChunk];

    size_t processed = 0;
    while (processed < kMaxEdgesPerPoll) {
        size_t n = Rf433Common::capture_read(chunk, kDrainChunk);
        if (n == 0) break;
        for (size_t i = 0; i < n; i++) {
            uint32_t t = chunk[i].timestamp_us;
            if (s_accum_edge_count > 0 && (t - s_last_edge_time_us) > kBurstGapThresholdUs) {
                // Gap between two real edges exceeded the burst threshold ->
                // the previous burst is complete. Both timestamps come from the
                // ISR clock, so this delta is always well-ordered.
                finalize_burst();
            }

            if (s_accum_edge_count < kMaxEdgesPerSignal) {
                s_accum_edges[s_accum_edge_count++] = chunk[i];
            }
            s_last_edge_time_us = t;
        }
        processed += n;
    }

    // Idle-timeout close-out: if no new edge has arrived for longer than the
    // burst gap, finalize whatever is accumulated. micros() is sampled HERE,
    // after draining -- not before -- so it is always >= the last drained
    // edge's timestamp. Sampling it before the drain (as the original code did)
    // let an edge recorded mid-drain carry a timestamp later than now_us,
    // underflowing this unsigned subtraction to ~4e9 and firing finalize_burst()
    // on almost every tick during active reception.
    if (s_accum_edge_count > 0) {
        const uint32_t now_us = micros();
        if ((now_us - s_last_edge_time_us) > kBurstGapThresholdUs) {
            finalize_burst();
        }
    }
}

size_t signal_count() {
    return s_signal_count;
}

const CapturedSignal *get_signal(size_t index) {
    if (index >= s_signal_count) return nullptr;
    return &s_signals[index];
}

} // namespace Rf433Scan
