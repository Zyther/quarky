#include "rf433_scan.h"
#include "rf433_replay.h" // Phase 3 Task 6: Replay action for the scan-result list below
#include "rf433_sub_format.h" // Phase 3 Task 22: "Load from SD" .sub interop (Task 21's read())
#include "rf433_protocol_decode.h" // Task 7's decode(), wired into a live "Decode
                                    // Selected" button below -- previously tested
                                    // only against a hardcoded fixture, never
                                    // called from any live screen (rf433_bruteforce.h
                                    // 's own comment already flagged this exact gap)
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../ui/file_browser.h" // Phase 3 Task 22: generic SD file picker
#include "../../hal/storage_sd.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <esp_heap_caps.h> // heap_caps_malloc()/MALLOC_CAP_SPIRAM -- see s_signals' allocation comment
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage;

namespace Rf433Scan {

// Session signals buffer -- heap-allocated in PSRAM, NOT a static/global
// array, and NOT optional polish. Real-hardware finding (2026-08-20):
// kMaxCapturedSignals(16) * sizeof(CapturedSignal) is 65792 bytes (~64KB;
// edges[512] * 8B/EdgeSample + bookkeeping), which as a plain `static`
// array lands in internal DRAM. Bisected on real hardware: the last
// commit before this buffer existed had 214302 bytes of internal DRAM
// free (51.88% used); the commit that added it as a static array dropped
// that to 140210 bytes free (68.52% used) -- a 74092-byte whole-commit
// delta (this buffer plus s_accum_edges and other additions in the same
// commit, not this buffer alone) that was enough to intermittently starve
// the BLE controller's own internal-RAM-only allocations (observed as
// continuous "vhci_drv: ... malloc failed" / "NimBLE: ... rc=17" errors)
// or, worse, push some other task's stack into PSRAM entirely, which trips
// a real ESP-IDF safety abort the instant anything disables the flash
// cache: `assert failed: spi_flash_disable_interrupts_caches_and_other_cpu
// cache_utils.c:127 (esp_task_stack_is_sane_cache_disabled())` -- a hard
// crash-and-reboot loop, 100% reproducible on every boot once enough of
// this session's other work piled onto the same internal-DRAM budget.
//
// The "obvious" fix -- EXT_RAM_BSS_ATTR, which is supposed to place a
// static/global variable's .bss in PSRAM at link time -- does NOT work in
// this build: it expands to nothing unless
// CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY is set, and this project's
// sdkconfig (framework-arduinoespressif32-libs/esp32p4/sdkconfig:2022)
// has it explicitly unset. Verified directly against that file rather
// than assumed -- using EXT_RAM_BSS_ATTR here would have silently done
// nothing and shipped the same crash. The real, functional mechanism on
// this build is an explicit runtime allocation with MALLOC_CAP_SPIRAM
// (esp_heap_caps.h), which this project's sdkconfig DOES support
// (CONFIG_SPIRAM=y, CONFIG_SPIRAM_USE_MALLOC=y, both confirmed present).
//
// Allocated once, in register_module() (called once from setup(), before
// any feature use is possible). If the allocation fails -- which would
// mean PSRAM itself is unusable, a much larger problem -- register_module()
// logs loudly and does NOT register the module, so the feature is simply
// absent from the launcher rather than crashing later on a null
// dereference. Every access below still reads as plain array indexing
// (s_signals[i]) since a heap pointer and a static array use identical
// indexing syntax -- only the declaration and the one allocation site
// changed.
static CapturedSignal *s_signals = nullptr;
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
static lv_obj_t *s_clear_btn = nullptr;
static lv_obj_t *s_decode_btn = nullptr;
static lv_obj_t *s_decode_result_label = nullptr;
static lv_obj_t *s_save_sub_btn = nullptr;
static lv_obj_t *s_save_sub_status_label = nullptr;
static uint32_t s_selected_capture_id = 0; // 0 = sentinel "nothing selected" --
                                            // capture_id is 1-based (see s_next_capture_id)

// "Select"/"Combine -> .sub" -- daisy-chains several captured signals into
// one .sub file. A SEPARATE selection mode from the single-selection above
// (s_selected_capture_id): row taps toggle membership in this small ordered
// set instead of replacing a single selection, while Select mode is on.
constexpr int kMaxChainSignals = 8;
static bool s_select_mode = false;
static uint32_t s_chain_ids[kMaxChainSignals] = {0};
static int s_chain_count = 0;
static lv_obj_t *s_select_btn = nullptr;
static lv_obj_t *s_combine_btn = nullptr;
static lv_obj_t *s_chain_status_label = nullptr;
static lv_obj_t *s_combine_status_label = nullptr;

// Phase 3 Task 22: "Load from SD" -- browse to and load a real .sub file
// (Task 21's Rf433SubFormat::read(), Task 22's ui/file_browser.h picker) and
// replay it. Deliberately a SEPARATE slot/UI group from the live-capture
// s_signals ring above, not a synthetic entry merged into it: a file loaded
// from SD is not a live capture, has no ring-eviction/capture_id semantics,
// and (per task-22-controller-notes.md's explicit guidance) shouldn't quietly
// inherit s_signals'/find_signal_by_id()'s indexing scheme, which exists for
// a different purpose.
//
// Directory choice, CORRECTED 2026-08-21 (real-hardware finding): originally
// pointed at the same folder finalize_burst() writes its unconditional
// "sig_<ms>_<edges>.raw" diagnostic dump into (/quarky/captures/rf433/),
// reasoning that ".sub" extension filtering would keep the two file kinds
// from colliding in the picker. That reasoning held locally but missed a real
// interaction with StorageSD::list_files()'s own kMaxEntriesScanned safety
// cap (256, storage_sd.cpp): after enough real capture sessions (this
// project's own ambient-433MHz-traffic stress test alone produced dozens of
// bursts), the raw-dump directory accumulates FAR more than 256 files, and
// the scan exhausts its entry budget entirely on old .raw files before ever
// reaching a .sub file -- confirmed on real hardware via temporary
// diagnostic logging (256 entries visited, all "sig_*.raw", 0 matches, a
// genuine capture_N.sub file present but never reached). Moved to its own
// subdirectory so this picker's scan is never at the mercy of how many raw
// diagnostic dumps the OTHER mechanism has accumulated.
static const char kSubFileDir[] = "/quarky/captures/rf433/sub";
static const char kSubFileExt[] = ".sub";
static lv_obj_t *s_load_sd_btn = nullptr;
static lv_obj_t *s_loaded_status_label = nullptr;
static lv_obj_t *s_loaded_replay_btn = nullptr;
static lv_obj_t *s_loaded_replay_status_label = nullptr;
// Not PSRAM-backed like s_signals[] -- this is a single CapturedSignal
// (~4.1KB: 512 edges * 8 bytes + bookkeeping), not an array of
// kMaxCapturedSignals(16) of them, so it doesn't reproduce the internal-DRAM
// exhaustion s_signals' own allocation comment documents. `static` (not a
// build_screen()-local) so the loaded signal survives Back/reopen, matching
// s_signals'/s_signal_count's own session-persistence, and so it is never a
// dangling stack reference by the time a Replay tap reads it.
static CapturedSignal s_loaded_signal;
static bool s_has_loaded_signal = false;
static char s_loaded_path[160];

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

// An empty-text label still occupies a full row (font-line height plus the
// flex container's own row gap) -- with two of these sitting back to back
// (decode result, save-.sub status) between the live-capture Replay status
// and the loaded-from-SD group, that showed up as a visibly oversized gap on
// real hardware. Hidden instead of just emptied when there's nothing to show;
// shown again the moment there's real text to display.
static void set_decode_result_text(const char *text) {
    if (s_decode_result_label == nullptr) return;
    lv_label_set_text(s_decode_result_label, text);
    if (text[0] == '\0') lv_obj_add_flag(s_decode_result_label, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_remove_flag(s_decode_result_label, LV_OBJ_FLAG_HIDDEN);
}

static void set_save_sub_status_text(const char *text) {
    if (s_save_sub_status_label == nullptr) return;
    lv_label_set_text(s_save_sub_status_label, text);
    if (text[0] == '\0') lv_obj_add_flag(s_save_sub_status_label, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_remove_flag(s_save_sub_status_label, LV_OBJ_FLAG_HIDDEN);
}

static void update_chain_status_label() {
    if (s_chain_status_label == nullptr) return;
    if (!s_select_mode && s_chain_count == 0) {
        lv_obj_add_flag(s_chain_status_label, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_remove_flag(s_chain_status_label, LV_OBJ_FLAG_HIDDEN);
    char buf[128];
    int n = std::snprintf(buf, sizeof(buf), "Chain (%d/%d):", s_chain_count, kMaxChainSignals);
    for (int i = 0; i < s_chain_count && n < (int)sizeof(buf) - 8; i++) {
        n += std::snprintf(buf + n, sizeof(buf) - (size_t)n, " #%u", (unsigned)s_chain_ids[i]);
    }
    lv_label_set_text(s_chain_status_label, buf);
}

// Toggles capture_id's membership in the chain (add if absent and room
// remains, remove if already present -- tapping an already-chained row a
// second time un-chains it, the same "tap again to undo" idiom this project
// uses elsewhere). Only called while s_select_mode is on.
static void toggle_chain_membership(uint32_t capture_id) {
    for (int i = 0; i < s_chain_count; i++) {
        if (s_chain_ids[i] == capture_id) {
            // Remove: shift the rest down, preserving chain ORDER (the order
            // signals were tapped is the order they'll be concatenated in).
            for (int j = i; j < s_chain_count - 1; j++) s_chain_ids[j] = s_chain_ids[j + 1];
            s_chain_count--;
            update_chain_status_label();
            return;
        }
    }
    if (s_chain_count >= kMaxChainSignals) {
        Serial.printf("quarky-tab5: [rf433-scan] Chain full (%d signals) -- "
                      "not adding #%u\n", kMaxChainSignals, (unsigned)capture_id);
        return;
    }
    s_chain_ids[s_chain_count++] = capture_id;
    update_chain_status_label();
}

// Concatenates the chained signals' real edge timing (in tap order) into one
// synthetic CapturedSignal, separated by kBurstGapThresholdUs (the same real
// 25ms burst-boundary constant this project already uses to decide "this is
// a new press, not a continuation" -- rf433_scan.cpp's own capture logic,
// reused here rather than inventing a new gap value). Each segment's OWN
// relative timing (edge[i].timestamp_us - edge[0].timestamp_us) is preserved
// exactly; only the gap between segments is synthetic. No edge object needs
// to be fabricated for the gap itself -- EdgeSample.level is "the level AFTER
// this edge" (rf433_common.h), so simply offsetting the next segment's
// timestamps forward already represents the prior level being held for the
// gap's duration, which is exactly correct silence semantics.
static bool build_chain_signal(CapturedSignal *out) {
    *out = CapturedSignal{};
    uint32_t running_offset = 0;
    bool any = false;
    for (int c = 0; c < s_chain_count; c++) {
        const CapturedSignal *seg = find_signal_by_id(s_chain_ids[c]);
        if (seg == nullptr || seg->edge_count == 0) {
            Serial.printf("quarky-tab5: [rf433-scan] Chain signal #%u no longer "
                          "available (evicted) -- skipped\n", (unsigned)s_chain_ids[c]);
            continue;
        }
        any = true;
        uint32_t seg_base = seg->edges[0].timestamp_us;
        size_t i = 0;
        for (; i < seg->edge_count; i++) {
            if (out->edge_count >= kMaxEdgesPerSignal) {
                out->truncated = true;
                break;
            }
            out->edges[out->edge_count].timestamp_us =
                running_offset + (seg->edges[i].timestamp_us - seg_base);
            out->edges[out->edge_count].level = seg->edges[i].level;
            out->edge_count++;
        }
        if (out->edge_count >= kMaxEdgesPerSignal) break;
        uint32_t seg_duration = seg->edges[seg->edge_count - 1].timestamp_us - seg_base;
        running_offset += seg_duration + kBurstGapThresholdUs;
        if (seg->truncated) out->truncated = true; // a source segment was itself truncated
    }
    out->captured_at_ms = millis();
    return any && out->edge_count > 0;
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
        if (s_select_mode) {
            // Chain mode: tap toggles membership, does not disturb the
            // separate single-selection (s_selected_capture_id) used by
            // Replay/Decode/Save-as-.sub.
            toggle_chain_membership(capture_id);
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
// Formats Rf433Replay's current async state into `label` -- factored out of
// update_replay_status_ui() (Task 22) so BOTH the live-capture replay area and
// the new "loaded from SD" replay area (below) can render the SAME shared
// Rf433Replay engine's state without duplicating the switch. There is only
// ever one Rf433Replay transmit in flight regardless of which UI group
// started it (live-capture Replay Selected or Load-from-SD's Replay Loaded),
// so it is correct -- not merely convenient -- for both labels to show
// identical text at any given moment.
static void format_replay_status_text(lv_obj_t *label) {
    if (!label) return;

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
            lv_label_set_text(label, "Replay: Idle");
            break;
        case Rf433Replay::State::kTransmitting: {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Replay: Transmitting...%s", trunc_suffix);
            lv_label_set_text(label, buf);
            break;
        }
        case Rf433Replay::State::kDone: {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Replay: Done%s", trunc_suffix);
            lv_label_set_text(label, buf);
            break;
        }
        case Rf433Replay::State::kFailed: {
            char buf[112];
            std::snprintf(buf, sizeof(buf), "Replay: Failed (%s)",
                          Rf433Replay::failure_reason());
            lv_label_set_text(label, buf);
            break;
        }
    }
}

// Reflects Rf433Replay's async state onto this screen's replay widgets --
// BOTH the live-capture group (s_replay_status_label/s_replay_btn) and the
// Task 22 loaded-from-SD group (s_loaded_replay_status_label/
// s_loaded_replay_btn). Called from poll() every tick regardless of capture
// state (s_active) -- a replay can be in flight whether or not a capture is
// also running, though never BOTH: rf433_common.cpp's capture_start() refuses
// while Rf433Replay::is_busy(), and Rf433Replay::transmit() refuses while
// Rf433Common::is_capturing() -- an explicit RX/TX check on top of the
// GPIO53 arbiter, added because the arbiter's single Owner::kRf433 token
// (idempotent per-owner) cannot by itself tell RX and TX apart. There is
// still no UI-level lock here; the exclusion is enforced by those two
// modules, not by this screen. No-ops per-widget-group if that group's
// widgets aren't built (screen isn't open, or -- for the loaded group -- Task
// 22 support didn't build its widgets for some other reason).
static void update_replay_status_ui() {
    format_replay_status_text(s_replay_status_label);
    format_replay_status_text(s_loaded_replay_status_label);

    // Disable each button while a transmit is in flight so a second tap can't
    // stack a request transmit() would refuse anyway -- belt-and-suspenders
    // over transmit()'s own busy check, not a substitute for it.
    bool busy = Rf433Replay::is_busy();
    if (s_replay_btn) {
        if (busy) lv_obj_add_state(s_replay_btn, LV_STATE_DISABLED);
        else lv_obj_remove_state(s_replay_btn, LV_STATE_DISABLED);
    }
    if (s_loaded_replay_btn) {
        // Additionally stays disabled with nothing successfully loaded yet --
        // busy is the ONLY thing that can re-disable it once a load succeeds
        // (transmit() itself refuses on 0 edges too, but disabling here saves
        // a doomed tap-and-see-it-fail round trip).
        if (busy || !s_has_loaded_signal) lv_obj_add_state(s_loaded_replay_btn, LV_STATE_DISABLED);
        else lv_obj_remove_state(s_loaded_replay_btn, LV_STATE_DISABLED);
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

// Renders s_loaded_signal/s_has_loaded_signal/s_loaded_path onto
// s_loaded_status_label. Shared by the file-select callback (fires
// immediately after a load attempt) and build_screen() (restores the label's
// text if the screen is reopened after a file was already loaded this
// session -- s_has_loaded_signal/s_loaded_path/s_loaded_signal are `static`,
// not build_screen()-local, so they persist across Back/reopen exactly like
// s_signal_count's own session persistence above).
static void update_loaded_status_label() {
    if (!s_loaded_status_label) return;
    if (!s_has_loaded_signal) {
        lv_label_set_text(s_loaded_status_label, "Loaded: none");
        return;
    }
    char buf[200];
    std::snprintf(buf, sizeof(buf), "Loaded: %s (%u edges)%s", s_loaded_path,
                  (unsigned)s_loaded_signal.edge_count,
                  s_loaded_signal.truncated ? " [truncated]" : "");
    lv_label_set_text(s_loaded_status_label, buf);
}

// FileBrowser::SelectCallback for the "Load from SD" button below. `path` is
// only valid for the duration of this call (see file_browser.h's doc
// comment) -- copied into s_loaded_path before use since it's needed again
// by update_loaded_status_label() on a later reopen.
static void on_sub_file_selected(const char *path, void * /*user_data*/) {
    std::strncpy(s_loaded_path, path, sizeof(s_loaded_path) - 1);
    s_loaded_path[sizeof(s_loaded_path) - 1] = '\0';

    // Rf433SubFormat::read() (Task 21) fully populates *out on success,
    // including out->truncated if the real file on SD was larger than its
    // internal read buffer (see rf433_sub_format.h's read() doc comment) --
    // that's surfaced via update_loaded_status_label()'s "[truncated]" suffix,
    // the same idiom rf433_scan.cpp's own add_signal_to_list() already uses
    // for live-capture truncation.
    bool ok = Rf433SubFormat::read(storage, s_loaded_path, &s_loaded_signal);
    s_has_loaded_signal = ok;
    if (!ok) {
        Serial.printf("quarky-tab5: [rf433-scan] Failed to load .sub file '%s' -- not a "
                      "well-formed RAW .sub file this module supports, or SD read failed\n",
                      s_loaded_path);
    } else {
        Serial.printf("quarky-tab5: [rf433-scan] Loaded '%s': %u edges%s\n", s_loaded_path,
                      (unsigned)s_loaded_signal.edge_count,
                      s_loaded_signal.truncated ? " (TRUNCATED)" : "");
    }
    update_loaded_status_label();
    update_replay_status_ui(); // re-evaluates s_loaded_replay_btn's disabled state
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("RF433 Scan", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Status: Idle");

    // All 7 action buttons live in a 3-column row-wrap grid instead of each
    // taking a full-width row -- with this many actions (capture toggle,
    // clear, replay/decode/save-sub on a selection, load-from-SD, replay
    // loaded) stacking them individually pushed the actual signal list
    // (the thing this screen is for) off the bottom of the visible area.
    // Their result/status labels stay in their own group below the grid
    // (see after the grid's closing brace) rather than immediately under
    // each button, so the grid can stay compact.
    lv_obj_t *btn_grid = lv_obj_create(content);
    lv_obj_set_width(btn_grid, LV_PCT(100));
    lv_obj_set_height(btn_grid, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_all(btn_grid, 2, 0);
    lv_obj_set_style_pad_gap(btn_grid, 4, 0);

    s_toggle_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_toggle_btn, LV_PCT(32));
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_label_set_text(s_toggle_label, "Start Capture");
    lv_obj_add_event_cb(s_toggle_btn, [](lv_event_t *) {
        set_capture_active(!s_active);
    }, LV_EVENT_CLICKED, nullptr);

    // "Clear" -- empties the live-capture list (s_signals/s_signal_count),
    // for when ambient 433 MHz traffic (weather stations, doorbells, other
    // remotes) has filled it with bursts unrelated to whatever the user
    // actually wants to test. Deliberately does NOT touch s_loaded_signal
    // (the SEPARATE Task 22 "Load from SD" slot -- see its own declaration
    // comment for why live captures and a loaded file are kept apart) and
    // does NOT stop an in-progress capture (safe either way: accumulation
    // state is independent of the finalized s_signals list this clears).
    // Safe to run mid-capture -- finalize_burst() only ever APPENDS to
    // s_signals, so clearing it here can't race a concurrent append into an
    // inconsistent state, just an append landing right after a clear.
    s_clear_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_clear_btn, LV_PCT(32));
    lv_obj_t *clear_lbl = lv_label_create(s_clear_btn);
    lv_label_set_text(clear_lbl, "Clear");
    lv_obj_add_event_cb(s_clear_btn, [](lv_event_t *) {
        s_signal_count = 0;
        if (s_list != nullptr) {
            lv_obj_clean(s_list);
            s_placeholder = lv_list_add_text(s_list, "No signals captured yet");
        }
        s_selected_capture_id = 0;
        if (s_replay_selected_label) {
            lv_label_set_text(s_replay_selected_label, "Selected: none");
        }
        set_decode_result_text("");
        set_save_sub_status_text("");
        s_chain_count = 0; // chained signal IDs point into the ring being
                            // cleared above; stale IDs would just get
                            // silently skipped by build_chain_signal()'s own
                            // eviction check, but resetting here is honest
                            // rather than leaving a chain that looks intact
        update_chain_status_label();
        if (s_combine_status_label) {
            lv_label_set_text(s_combine_status_label, "");
            lv_obj_add_flag(s_combine_status_label, LV_OBJ_FLAG_HIDDEN);
        }
        Serial.println("quarky-tab5: [rf433-scan] capture list cleared");
    }, LV_EVENT_CLICKED, nullptr);

    // Task 6 (RF433 replay): Replay button lives in the grid; its selection/
    // status labels move to the label group below the grid (see after the
    // grid's last button) so the grid itself stays a compact 3-column block.
    s_replay_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_replay_btn, LV_PCT(32));
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

    // "Decode Selected" -- runs Task 7's Rf433ProtocolDecode::decode() against
    // whatever's currently selected. Prior to this, decode() was tested only
    // against a hardcoded fixture (its own host-native test) and was never
    // actually called from any live screen -- rf433_bruteforce.h's own
    // comment already disclosed this gap explicitly. Same re-validate-by-id
    // pattern as Replay Selected above (a selection can be evicted from the
    // ring between being tapped and being acted on).
    s_decode_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_decode_btn, LV_PCT(32));
    lv_obj_t *decode_lbl = lv_label_create(s_decode_btn);
    lv_label_set_text(decode_lbl, "Decode Selected");
    lv_obj_add_event_cb(s_decode_btn, [](lv_event_t *) {
        if (s_selected_capture_id == 0) {
            set_decode_result_text("Select a signal first.");
            return;
        }
        const CapturedSignal *found = find_signal_by_id(s_selected_capture_id);
        if (!found) {
            set_decode_result_text("Selected signal evicted -- reselect.");
            return;
        }
        Rf433ProtocolDecode::DecodedCode out{};
        char buf[80];
        if (Rf433ProtocolDecode::decode(*found, &out)) {
            std::snprintf(buf, sizeof(buf), "Decoded: %s  code=0x%llX  (%u bits)",
                          out.protocol_name, (unsigned long long)out.code,
                          (unsigned)out.bit_length);
            Serial.printf("quarky-tab5: [rf433-scan] decode: %s code=0x%llX bits=%u\n",
                          out.protocol_name, (unsigned long long)out.code,
                          (unsigned)out.bit_length);
        } else {
            std::snprintf(buf, sizeof(buf), "No known protocol matched (%u edges)",
                          (unsigned)found->edge_count);
            Serial.printf("quarky-tab5: [rf433-scan] decode: no match (%u edges)\n",
                          (unsigned)found->edge_count);
        }
        set_decode_result_text(buf);
    }, LV_EVENT_CLICKED, nullptr);

    // "Save as .sub" -- Task 21's Rf433SubFormat::write(), previously only
    // exercised by its own host-native test; this is the first live path that
    // actually produces a real .sub file on the SD card from a genuine
    // capture (closing the disclosed gap noted in Task 22's own ledger entry
    // -- "no Save as .sub export is wired in either"). Filename keyed by
    // capture_id, matching this project's existing hex/id-based naming
    // convention (nfc_tag_library.cpp's hex-UID naming, nfc_mifare_crack.cpp's
    // hex-UID naming) rather than inventing a new one.
    s_save_sub_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_save_sub_btn, LV_PCT(32));
    lv_obj_t *save_sub_lbl = lv_label_create(s_save_sub_btn);
    lv_label_set_text(save_sub_lbl, "Save as .sub");
    lv_obj_add_event_cb(s_save_sub_btn, [](lv_event_t *) {
        if (s_selected_capture_id == 0) {
            set_save_sub_status_text("Select a signal first.");
            return;
        }
        const CapturedSignal *found = find_signal_by_id(s_selected_capture_id);
        if (!found) {
            set_save_sub_status_text("Selected signal evicted -- reselect.");
            return;
        }
        char path[96];
        std::snprintf(path, sizeof(path), "%s/capture_%u.sub", kSubFileDir,
                      (unsigned)s_selected_capture_id);
        bool ok = Rf433SubFormat::write(storage, path, *found);
        char buf[128];
        if (ok) {
            std::snprintf(buf, sizeof(buf), "Saved to %s", path);
            Serial.printf("quarky-tab5: [rf433-scan] saved .sub: %s (%u edges)\n",
                          path, (unsigned)found->edge_count);
        } else {
            std::snprintf(buf, sizeof(buf), "Save failed (%s)", path);
            Serial.printf("quarky-tab5: [rf433-scan] failed to save .sub: %s\n", path);
        }
        set_save_sub_status_text(buf);
    }, LV_EVENT_CLICKED, nullptr);

    // "Select" -- toggles chain mode (see s_select_mode's declaration
    // comment). Turning it OFF does not clear an already-built chain (so the
    // user can toggle it off to use Replay/Decode/Save-as-.sub on a single
    // row without losing chain progress, then toggle back on to keep
    // building it) -- only "Combine -> .sub" below and "Clear" reset it.
    s_select_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_select_btn, LV_PCT(32));
    lv_obj_t *select_lbl = lv_label_create(s_select_btn);
    lv_label_set_text(select_lbl, "Select");
    lv_obj_add_event_cb(s_select_btn, [](lv_event_t *e) {
        s_select_mode = !s_select_mode;
        lv_obj_t *btn = static_cast<lv_obj_t *>(lv_event_get_target(e));
        lv_obj_t *lbl = lv_obj_get_child(btn, 0);
        if (lbl) lv_label_set_text(lbl, s_select_mode ? "Select: ON" : "Select");
        update_chain_status_label();
    }, LV_EVENT_CLICKED, nullptr);

    // "Combine -> .sub" -- daisy-chains the tapped-in-order signals above
    // into one file via build_chain_signal() (see its own comment for the
    // real-gap-reuse reasoning) and Task 21's Rf433SubFormat::write().
    s_combine_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_combine_btn, LV_PCT(32));
    lv_obj_t *combine_lbl = lv_label_create(s_combine_btn);
    lv_label_set_text(combine_lbl, "Combine -> .sub");
    lv_obj_add_event_cb(s_combine_btn, [](lv_event_t *) {
        if (s_chain_count == 0) {
            if (s_combine_status_label) {
                lv_label_set_text(s_combine_status_label, "Chain is empty -- tap Select, then tap signals.");
                lv_obj_remove_flag(s_combine_status_label, LV_OBJ_FLAG_HIDDEN);
            }
            return;
        }
        CapturedSignal combined{};
        bool ok = build_chain_signal(&combined);
        char buf[128];
        if (!ok) {
            std::snprintf(buf, sizeof(buf), "All %d chained signals were evicted -- nothing to combine.",
                          s_chain_count);
        } else {
            char path[96];
            std::snprintf(path, sizeof(path), "%s/chain_%u.sub", kSubFileDir,
                          (unsigned)s_chain_ids[0]);
            bool wrote = Rf433SubFormat::write(storage, path, combined);
            if (wrote) {
                std::snprintf(buf, sizeof(buf), "Saved %d-signal chain to %s (%u edges)%s",
                              s_chain_count, path, (unsigned)combined.edge_count,
                              combined.truncated ? " [truncated]" : "");
                Serial.printf("quarky-tab5: [rf433-scan] combined %d signals -> %s (%u edges)%s\n",
                              s_chain_count, path, (unsigned)combined.edge_count,
                              combined.truncated ? " TRUNCATED" : "");
                // Reset the chain on success, same "done, start fresh" idiom
                // Clear uses for the live-capture list.
                s_chain_count = 0;
            } else {
                std::snprintf(buf, sizeof(buf), "Save failed (%s)", path);
                Serial.printf("quarky-tab5: [rf433-scan] failed to save combined .sub: %s\n", path);
            }
        }
        if (s_combine_status_label) {
            lv_label_set_text(s_combine_status_label, buf);
            lv_obj_remove_flag(s_combine_status_label, LV_OBJ_FLAG_HIDDEN);
        }
        update_chain_status_label();
    }, LV_EVENT_CLICKED, nullptr);

    // Task 22: "Load from SD" -- a SEPARATE group from the live-capture
    // replay UI above (see s_loaded_signal's declaration comment for why).
    // Opens ui/file_browser.h's generic picker over kSubFileDir/kSubFileExt;
    // on_sub_file_selected() runs Rf433SubFormat::read() and updates the
    // status label + Replay Loaded button below.
    s_load_sd_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_load_sd_btn, LV_PCT(32));
    lv_obj_t *load_sd_label = lv_label_create(s_load_sd_btn);
    lv_label_set_text(load_sd_label, "Load from SD (.sub)");
    lv_obj_add_event_cb(s_load_sd_btn, [](lv_event_t *) {
        FileBrowser::push(storage, "Load RF433 .sub file", kSubFileDir, kSubFileExt,
                           on_sub_file_selected);
    }, LV_EVENT_CLICKED, nullptr);

    s_loaded_replay_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_loaded_replay_btn, LV_PCT(32));
    lv_obj_t *loaded_replay_label = lv_label_create(s_loaded_replay_btn);
    lv_label_set_text(loaded_replay_label, "Replay Loaded");
    lv_obj_add_event_cb(s_loaded_replay_btn, [](lv_event_t *) {
        if (!s_has_loaded_signal) {
            Serial.println("quarky-tab5: [rf433-scan] Replay Loaded tapped with no file "
                            "loaded -- use Load from SD first");
            return;
        }
        // Same shared transmit() path as the live-capture Replay Selected
        // button -- see rf433_replay.h's doc comment: exactly one
        // safety-reviewed transmit path (GPIO53 arbiter + RX/TX exclusion),
        // never a second one for SD-loaded signals.
        Rf433Replay::transmit(s_loaded_signal);
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_state(s_loaded_replay_btn, LV_STATE_DISABLED); // update_replay_status_ui()
                                                               // (below) re-derives this
                                                               // from s_has_loaded_signal
                                                               // on every poll() tick anyway;
                                                               // set here too so the very
                                                               // first frame (before poll()
                                                               // next runs) isn't briefly
                                                               // tappable with nothing loaded.

    // Status/result labels for the grid above, grouped together (rather than
    // sitting individually under each button) so the button grid itself
    // stays compact -- see btn_grid's own declaration comment.
    s_replay_selected_label = lv_label_create(content);
    lv_label_set_text(s_replay_selected_label, "Selected: none");

    s_replay_status_label = lv_label_create(content);
    lv_label_set_text(s_replay_status_label, "Replay: Idle");

    s_decode_result_label = lv_label_create(content);
    lv_label_set_long_mode(s_decode_result_label, LV_LABEL_LONG_WRAP);
    set_decode_result_text(""); // starts hidden -- see the helper's own comment

    s_save_sub_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_save_sub_status_label, LV_LABEL_LONG_WRAP);
    set_save_sub_status_text(""); // starts hidden

    s_chain_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_chain_status_label, LV_LABEL_LONG_WRAP);
    update_chain_status_label(); // starts hidden (select mode off, chain empty)

    s_combine_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_combine_status_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_combine_status_label, "");
    lv_obj_add_flag(s_combine_status_label, LV_OBJ_FLAG_HIDDEN);

    s_loaded_status_label = lv_label_create(content);
    update_loaded_status_label(); // restores "Loaded: <path>" text if reopening

    s_loaded_replay_status_label = lv_label_create(content);
    lv_label_set_text(s_loaded_replay_status_label, "Replay: Idle");

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
        s_clear_btn = nullptr;
        s_list = nullptr;
        s_placeholder = nullptr;
        s_replay_selected_label = nullptr;
        s_replay_btn = nullptr;
        s_replay_btn_label = nullptr;
        s_replay_status_label = nullptr;
        s_decode_btn = nullptr;
        s_decode_result_label = nullptr;
        s_save_sub_btn = nullptr;
        s_save_sub_status_label = nullptr;
        s_select_btn = nullptr;
        s_combine_btn = nullptr;
        s_chain_status_label = nullptr;
        s_combine_status_label = nullptr;
        s_load_sd_btn = nullptr;
        s_loaded_status_label = nullptr;
        s_loaded_replay_btn = nullptr;
        s_loaded_replay_status_label = nullptr;
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
    // See s_signals' declaration comment for the real-hardware finding this
    // allocation exists to fix. heap_caps_malloc's caps argument is a
    // bitwise AND, not a fallback list -- the returned block must have ALL
    // requested capabilities, there is no "try this, fall back to that"
    // behavior to guard against here. MALLOC_CAP_8BIT is omitted because
    // PSRAM regions already register 8-bit-capable by default (this is a
    // plain byte-addressed struct array, nothing here needs it spelled
    // out); MALLOC_CAP_DMA is omitted because nothing DMAs from this
    // buffer -- transmit() (rf433_replay.cpp) copies it out to its own
    // heap args on the main task before any task/DMA touches the copy. A
    // failed allocation is treated as fatal to this feature only, not
    // papered over with a silent internal-RAM fallback that would
    // reintroduce the exact exhaustion this fix exists to prevent.
    s_signals = static_cast<CapturedSignal *>(
        heap_caps_malloc(sizeof(CapturedSignal) * kMaxCapturedSignals, MALLOC_CAP_SPIRAM));
    if (!s_signals) {
        Serial.printf("quarky-tab5: [rf433-scan] heap_caps_malloc(%u bytes, "
                      "MALLOC_CAP_SPIRAM) FAILED -- PSRAM appears unusable. "
                      "RF433 Scan will not be registered (refusing rather than "
                      "risking the internal-DRAM exhaustion this allocation "
                      "exists to prevent).\n",
                      (unsigned)(sizeof(CapturedSignal) * kMaxCapturedSignals));
        return;
    }
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
