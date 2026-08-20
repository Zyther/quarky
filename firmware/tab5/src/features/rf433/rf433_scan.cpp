#include "rf433_scan.h"
#include "rf433_replay.h" // Phase 3 Task 6: Replay action for the scan-result list below
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

// Task 6 (RF433 replay) UI: tap a row to select it (see add_signal_to_list()'s
// click handler), then tap this dedicated Replay button. Two-step rather than
// tap-to-replay-immediately is a deliberate choice -- Replay can trigger a
// real physical device (garage door, doorbell, etc; see this plan's Task 6
// acceptance criterion), so a bare accidental tap on a list row must not fire
// a transmission.
static lv_obj_t *s_replay_selected_label = nullptr;
static lv_obj_t *s_replay_btn = nullptr;
static lv_obj_t *s_replay_btn_label = nullptr;
static lv_obj_t *s_replay_status_label = nullptr;
static uint32_t s_selected_capture_id = 0; // 0 = sentinel "nothing selected" --
                                            // capture_id is 1-based (see s_next_capture_id)

static bool s_active = false;
static uint32_t s_last_edge_time_us = 0;
static size_t s_accum_edge_count = 0;
static bool s_accum_truncated = false;
static Rf433Common::EdgeSample s_accum_edges[kMaxEdgesPerSignal];
static uint32_t s_next_capture_id = 1;

// Gap threshold (in microseconds) to delimit end of an RF burst.
// Standard OOK remotes repeat the same code's burst every 8ms-15ms while a
// button is held/pressed. The threshold must sit clearly OUTSIDE that band --
// a value inside 8-15ms (the original 10ms here) sits squarely in the repeat
// band itself, so jitter around the threshold splits some repeat-gaps into
// separate signals while merging others. 25ms clears the repeat band with a
// wide margin while staying comfortably below the tens-of-milliseconds of
// silence that separates two distinct button presses, so one press's repeats
// still coalesce into a single CapturedSignal.
constexpr uint32_t kBurstGapThresholdUs = 25000; // 25ms
constexpr size_t kMinEdgesForSignal = 10;        // Filter spurious single glitch transitions

// Per-poll()-tick cap on the number of finalize_burst() calls (each one is
// the expensive operation: synchronous SD write + LVGL list churn -- the
// same operation that starved the watchdog before). A noisy/floating pin
// producing many short gap-delimited runs within one tick must not be able
// to fire this an unbounded number of times in a single loop() iteration.
// Any boundary detected beyond the cap is deferred, not lost: the
// accumulator and gap-tracking state persist across ticks (see poll()), so
// an uncapped burst just keeps accumulating (bounded by kMaxEdgesPerSignal /
// the truncated flag) until a later tick's finalize budget resets.
constexpr size_t kMaxFinalizesPerPoll = 2;

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
    if (active) {
        if (!Rf433Common::capture_start()) {
            // Arm failed -- do not flip s_active or the UI into a state that
            // claims we're capturing when we're not.
            Serial.printf("quarky-tab5: [rf433-scan] capture_start() failed -- "
                          "staying Idle\n");
            return;
        }
        s_accum_edge_count = 0;
        s_accum_truncated = false;
        s_last_edge_time_us = 0;
        s_active = true;
    } else {
        // Stopping mid-burst deliberately discards whatever is accumulated
        // in s_accum_edges without finalizing it: a partial burst the user
        // chose to cut off isn't a real captured signal, and the next
        // capture_start() resets the accumulator anyway.
        Rf433Common::capture_stop();
        s_active = false;
    }
    update_status_ui();
}

// Looks up a signal by its stable capture_id rather than its (mutable, ring-
// shifted) array slot. Mirrors get_signal()'s bounds-checked nullptr-on-miss
// contract so callers (the row click handler) never index s_signals directly.
static const CapturedSignal *find_signal_by_id(uint32_t capture_id) {
    for (size_t i = 0; i < s_signal_count; i++) {
        if (s_signals[i].capture_id == capture_id) return &s_signals[i];
    }
    return nullptr;
}

static void add_signal_to_list(const CapturedSignal &sig) {
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

    // Calculate approx burst duration with one decimal place of millisecond
    // precision -- most real bursts are only a few ms long, and truncating to
    // whole milliseconds rendered nearly every short capture as "~0 ms".
    uint32_t duration_us = 0;
    if (sig.edge_count > 1) {
        duration_us = sig.edges[sig.edge_count - 1].timestamp_us - sig.edges[0].timestamp_us;
    }

    char row[96];
    std::snprintf(row, sizeof(row), "Sig #%u: %u edges, ~%lu.%01lu ms (@%lu s)%s",
                  (unsigned)sig.capture_id,
                  (unsigned)sig.edge_count,
                  (unsigned long)(duration_us / 1000),
                  (unsigned long)((duration_us % 1000) / 100),
                  (unsigned long)(sig.captured_at_ms / 1000),
                  sig.truncated ? " [truncated]" : "");

    lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_AUDIO, row);
    lv_obj_add_event_cb(btn, [](lv_event_t *e) {
        uint32_t capture_id = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
        const CapturedSignal *found = find_signal_by_id(capture_id);
        if (!found) {
            // Evicted from the session ring since this row was created.
            Serial.printf("quarky-tab5: [rf433-scan] Selected signal #%u no longer "
                          "available (evicted from ring)\n", (unsigned)capture_id);
            return;
        }
        Serial.printf("quarky-tab5: [rf433-scan] Selected signal #%u (%u edges)\n",
                      (unsigned)capture_id, (unsigned)found->edge_count);

        // Tapping a row only SELECTS it for replay -- it does not transmit.
        // See s_selected_capture_id's declaration comment for why a tap
        // alone must never fire a real transmission.
        s_selected_capture_id = capture_id;
        if (s_replay_selected_label) {
            char sel[48];
            std::snprintf(sel, sizeof(sel), "Selected: Sig #%u%s",
                          (unsigned)capture_id, found->truncated ? " [truncated]" : "");
            lv_label_set_text(s_replay_selected_label, sel);
        }
    }, LV_EVENT_CLICKED, (void *)(uintptr_t)sig.capture_id);
}

// Reflects Rf433Replay's async state onto this screen's replay widgets.
// Called from poll() every tick regardless of capture state (s_active) --
// a replay can be in flight whether or not a capture is also running,
// though never BOTH: rf433_common.cpp's capture_start() refuses while
// Rf433Replay::is_busy(), and Rf433Replay::transmit() refuses while
// Rf433Common::is_capturing() -- an explicit RX/TX check on top of the
// GPIO53 arbiter, added because the arbiter's single Owner::kRf433 token
// (idempotent per-owner) cannot by itself tell RX and TX apart. There is
// still no UI-level lock here; the exclusion is enforced by those two
// modules, not by this screen. No-ops if the screen isn't open (widgets
// null).
static void update_replay_status_ui() {
    if (!s_replay_status_label || !s_replay_btn) return;

    // Suffix appended to the Transmitting/Done text when the in-flight (or
    // just-finished) replay is of a truncated capture -- see
    // Rf433Replay::transmit()'s doc comment for why truncated signals are
    // replayed (their available prefix) rather than refused, and
    // last_transmit_was_truncated()'s doc comment for this flag's lifetime.
    const char *trunc_suffix = Rf433Replay::last_transmit_was_truncated()
                                    ? " [TRUNCATED -- partial burst only]"
                                    : "";

    switch (Rf433Replay::state()) {
        case Rf433Replay::State::kIdle:
            lv_label_set_text(s_replay_status_label, "Replay: Idle");
            break;
        case Rf433Replay::State::kTransmitting: {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Replay: Transmitting...%s", trunc_suffix);
            lv_label_set_text(s_replay_status_label, buf);
            break;
        }
        case Rf433Replay::State::kDone: {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Replay: Done%s", trunc_suffix);
            lv_label_set_text(s_replay_status_label, buf);
            break;
        }
        case Rf433Replay::State::kFailed: {
            char buf[112];
            std::snprintf(buf, sizeof(buf), "Replay: Failed (%s)",
                          Rf433Replay::failure_reason());
            lv_label_set_text(s_replay_status_label, buf);
            break;
        }
    }

    // Disable the button while a transmit is in flight so a second tap can't
    // stack a request transmit() would refuse anyway -- belt-and-suspenders
    // over transmit()'s own busy check, not a substitute for it.
    if (Rf433Replay::is_busy()) {
        lv_obj_add_state(s_replay_btn, LV_STATE_DISABLED);
    } else {
        lv_obj_remove_state(s_replay_btn, LV_STATE_DISABLED);
    }
}

static void finalize_burst() {
    if (s_accum_edge_count < kMinEdgesForSignal) {
        // Discard short noise glitch
        s_accum_edge_count = 0;
        s_accum_truncated = false;
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
    s_signals[new_idx].capture_id = s_next_capture_id++;
    s_signals[new_idx].truncated = s_accum_truncated;
    std::memcpy(s_signals[new_idx].edges, s_accum_edges,
                sizeof(Rf433Common::EdgeSample) * s_accum_edge_count);

    Serial.printf("quarky-tab5: [rf433-scan] Captured burst #%u: %u edges%s\n",
                  (unsigned)s_signals[new_idx].capture_id, (unsigned)s_accum_edge_count,
                  s_accum_truncated ? " (TRUNCATED -- burst exceeded capacity)" : "");

    // Save capture to SD storage if available
    char filename[64];
    std::snprintf(filename, sizeof(filename), "/quarky/captures/rf433/sig_%lu_%u.raw",
                  (unsigned long)millis(), (unsigned)s_accum_edge_count);
    bool wrote = storage.write_capture_file(filename,
                               (const uint8_t *)s_signals[new_idx].edges,
                               sizeof(Rf433Common::EdgeSample) * s_accum_edge_count);
    if (!wrote) {
        Serial.printf("quarky-tab5: [rf433-scan] Failed to write capture file %s "
                      "(SD not mounted or write error) -- signal kept in RAM only\n",
                      filename);
    }

    add_signal_to_list(s_signals[new_idx]);
    s_accum_edge_count = 0;
    s_accum_truncated = false;
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

    // Task 6 (RF433 replay): selection label + Replay button + status label,
    // in that order, ahead of the list -- same idiom as s_status_label/
    // s_toggle_btn above (a small fixed-height label/button pair living in
    // this same COLUMN flex container, ahead of the size-flexed list).
    s_replay_selected_label = lv_label_create(content);
    lv_label_set_text(s_replay_selected_label, "Selected: none");

    s_replay_btn = lv_button_create(content);
    s_replay_btn_label = lv_label_create(s_replay_btn);
    lv_label_set_text(s_replay_btn_label, "Replay Selected");
    lv_obj_add_event_cb(s_replay_btn, [](lv_event_t *) {
        if (s_selected_capture_id == 0) {
            Serial.println("quarky-tab5: [rf433-scan] Replay tapped with nothing "
                            "selected -- tap a signal in the list first");
            return;
        }
        const CapturedSignal *found = find_signal_by_id(s_selected_capture_id);
        if (!found) {
            Serial.printf("quarky-tab5: [rf433-scan] Replay REFUSED -- signal #%u "
                          "no longer available (evicted from ring)\n",
                          (unsigned)s_selected_capture_id);
            if (s_replay_selected_label) {
                lv_label_set_text(s_replay_selected_label, "Selected: none (evicted)");
            }
            s_selected_capture_id = 0;
            return;
        }
        // transmit() itself refuses cleanly (busy / a capture is active /
        // no edges / arbiter-held) and reports the reason via
        // Rf433Replay::state()/failure_reason(), which
        // update_replay_status_ui() (driven from poll()) surfaces here -- no
        // need to duplicate those checks in this click handler. A truncated
        // signal is NOT refused: transmit() replays its captured prefix and
        // update_replay_status_ui() shows a warning instead (see
        // Rf433Replay::last_transmit_was_truncated()).
        Rf433Replay::transmit(*found);
    }, LV_EVENT_CLICKED, nullptr);

    s_replay_status_label = lv_label_create(content);
    lv_label_set_text(s_replay_status_label, "Replay: Idle");

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    // Populate existing session signals if reopening
    if (s_signal_count == 0) {
        s_placeholder = lv_list_add_text(s_list, "No signals captured yet");
    } else {
        for (size_t i = 0; i < s_signal_count; i++) {
            add_signal_to_list(s_signals[i]);
        }
    }

    // Teardown on delete (e.g. Back button tapped). Null the widget pointers
    // FIRST, before set_capture_active(false) -- that call runs
    // update_status_ui(), which would otherwise write to labels this event is
    // in the middle of freeing. update_status_ui() already no-ops when its
    // labels are null, so nulling first removes the ordering dependency on
    // LVGL's event-callback sequencing entirely rather than relying on it.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        bool was_active = s_active;
        s_status_label = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        s_list = nullptr;
        s_placeholder = nullptr;
        s_replay_selected_label = nullptr;
        s_replay_btn = nullptr;
        s_replay_btn_label = nullptr;
        s_replay_status_label = nullptr;
        // Deliberately NOT clearing s_selected_capture_id here: it is only a
        // lookup key (re-validated via find_signal_by_id() on next use), and
        // an in-flight Rf433Replay transmit task (like WifiConnectFeature's
        // connect_task, see wifi_connect.cpp) has no cancellation hook and
        // keeps running after this screen closes -- poll() below tolerates
        // that by checking widget pointers before touching them.
        if (was_active) {
            set_capture_active(false);
        }
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
    // Independent of capture state (s_active): a replay can be in flight
    // whether or not a capture is also running -- but never both at once.
    // See update_replay_status_ui()'s comment for how RX/TX exclusion is
    // actually enforced (an explicit is_capturing()/is_busy() check in each
    // of rf433_common.cpp/rf433_replay.cpp, on top of the GPIO53 arbiter,
    // not a lock in this screen). Must run before the early return below.
    update_replay_status_ui();

    if (!s_active) return;

    // The ISR self-disarms (masks its own interrupt source) once a capture
    // hits the hard edge ceiling, but is_capturing() stays true -- it was
    // never told to stop, it just stopped collecting (see rf433_common.h).
    // Left unchecked, the UI would show "Capturing..." forever with no new
    // edges ever arriving, and the interrupt would sit half-torn-down until
    // the user happened to tap Stop. Check every tick and complete the
    // teardown ourselves the moment it happens.
    if (Rf433Common::overrun()) {
        Serial.printf("quarky-tab5: [rf433-scan] Capture overrun -- ISR hit the edge "
                      "ceiling, stopping\n");
        set_capture_active(false);
        if (s_status_label) {
            lv_label_set_text(s_status_label, "Status: Capture overrun -- check antenna/pin");
        }
        return;
    }

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

    // Bounds finalize_burst() calls (the expensive part: synchronous SD
    // write + LVGL churn), independent of the edge-count cap above -- see
    // kMaxFinalizesPerPoll's comment. A gap boundary detected once the
    // budget is spent is simply not finalized this tick: its edges keep
    // accumulating (bounded by kMaxEdgesPerSignal / the truncated flag) and
    // get finalized on a later tick once the budget resets.
    size_t finalizes_this_tick = 0;

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
                if (finalizes_this_tick < kMaxFinalizesPerPoll) {
                    finalize_burst();
                    finalizes_this_tick++;
                }
                // else: deferred -- see kMaxFinalizesPerPoll's comment.
            }

            if (s_accum_edge_count < kMaxEdgesPerSignal) {
                s_accum_edges[s_accum_edge_count++] = chunk[i];
            } else if (!s_accum_truncated) {
                s_accum_truncated = true;
                Serial.printf("quarky-tab5: [rf433-scan] Burst exceeded %u edges -- "
                              "further edges in this burst are being dropped\n",
                              (unsigned)kMaxEdgesPerSignal);
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
    if (s_accum_edge_count > 0 && finalizes_this_tick < kMaxFinalizesPerPoll) {
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
