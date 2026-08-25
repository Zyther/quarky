#include "cc1101_scan.h"
#include "cc1101_hw.h"
#include "cc1101_record.h"
#include "cc1101_replay.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../ui/deep_file_browser.h"
#include "../../hal/storage_sd.h"
#include <subghz_protocol_decode.h>
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage;

// ===========================================================================
// ISR/ring-buffer capture -- direct port of rf433_common.cpp's real,
// hardware-hardened shape (runaway-interrupt self-disarm ceiling, oldest-
// first drain, portMUX_TYPE critical sections) onto Cc1101Hw::gdo0_pin()
// instead of TAB5_RF433R_PIN. See cc1101_scan.h's own header comment for the
// donor citation (Poseidon subghz_scan.cpp's gdo0_isr()/capture_now()) and
// why no GPIO53-style arbiter is needed here (CC1101's M-Bus pins are
// dedicated, not shared with the external I2C bus).
// ===========================================================================
namespace Cc1101Scan {
namespace {

constexpr size_t kRingSize = 512;
constexpr uint32_t kMaxEdgesPerCapture = 100000; // same runaway-ISR ceiling
                                                  // reasoning as rf433_common.cpp

EdgeSample s_ring[kRingSize];
volatile size_t s_ring_head = 0;
volatile size_t s_ring_count = 0;
volatile uint32_t s_edges_this_capture = 0;
volatile bool s_overrun = false;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
bool s_isr_armed = false;
int s_gdo0_pin = -1;

void IRAM_ATTR isr_edge() {
    portENTER_CRITICAL_ISR(&s_mux);
    if (s_edges_this_capture >= kMaxEdgesPerCapture) {
        gpio_intr_disable((gpio_num_t)s_gdo0_pin);
        s_overrun = true;
        portEXIT_CRITICAL_ISR(&s_mux);
        return;
    }
    s_edges_this_capture = s_edges_this_capture + 1;
    s_ring[s_ring_head].timestamp_us = micros();
    s_ring[s_ring_head].level = (digitalRead(s_gdo0_pin) != LOW);
    s_ring_head = (s_ring_head + 1) % kRingSize;
    if (s_ring_count < kRingSize) s_ring_count = s_ring_count + 1;
    portEXIT_CRITICAL_ISR(&s_mux);
}

bool isr_capture_start() {
    if (s_isr_armed) return true;
    if (Cc1101Replay::is_busy()) {
        Serial.println("quarky-tab5: [cc1101-scan] capture start REFUSED -- a "
                        "replay is currently transmitting on GDO0");
        return false;
    }
    if (!Cc1101Hw::is_present() && !Cc1101Hw::init()) {
        Serial.println("quarky-tab5: [cc1101-scan] capture start REFUSED -- "
                        "CC1101 not present");
        return false;
    }
    if (!Cc1101Hw::enable_async_rx()) {
        Serial.println("quarky-tab5: [cc1101-scan] capture start REFUSED -- "
                        "enable_async_rx() failed");
        return false;
    }
    s_gdo0_pin = Cc1101Hw::gdo0_pin();
    pinMode(s_gdo0_pin, INPUT);
    portENTER_CRITICAL(&s_mux);
    s_ring_head = 0;
    s_ring_count = 0;
    s_edges_this_capture = 0;
    s_overrun = false;
    portEXIT_CRITICAL(&s_mux);
    attachInterrupt(digitalPinToInterrupt(s_gdo0_pin), isr_edge, CHANGE);
    gpio_intr_enable((gpio_num_t)s_gdo0_pin);
    s_isr_armed = true;
    return true;
}

void isr_capture_stop() {
    if (!s_isr_armed) return;
    detachInterrupt(digitalPinToInterrupt(s_gdo0_pin));
    Cc1101Hw::idle();
    s_isr_armed = false;
}

size_t isr_capture_read(EdgeSample *out, size_t max) {
    if (!out || max == 0) return 0;
    portENTER_CRITICAL(&s_mux);
    size_t n = s_ring_count < max ? s_ring_count : max;
    size_t start = (s_ring_head + kRingSize - s_ring_count) % kRingSize;
    for (size_t i = 0; i < n; i++) out[i] = s_ring[(start + i) % kRingSize];
    s_ring_count = s_ring_count - n;
    portEXIT_CRITICAL(&s_mux);
    return n;
}

} // namespace

// ── Session state ──────────────────────────────────────────────────────────

// Same PSRAM heap_caps_malloc() reasoning as rf433_scan.cpp's s_signals --
// see that file's own allocation comment for the full real-hardware
// internal-DRAM-exhaustion citation this reuses unchanged.
static CapturedSignal *s_signals = nullptr;
static size_t s_signal_count = 0;
static uint32_t s_next_capture_id = 1;

static EdgeSample *s_accum_edges = nullptr; // PSRAM, same reasoning
static size_t s_accum_edge_count = 0;
static bool s_accum_truncated = false;
static uint32_t s_last_edge_time_us = 0;

static bool s_active = false;    // capture mode running
static bool s_hotcold_active = false; // hot/cold RSSI mode running (mutually
                                       // exclusive with s_active -- both want
                                       // the radio in a different mode)

// Common sub-GHz center frequencies -- real list, ported from Poseidon's
// subghz_scan.cpp COMMON_FREQS[] (~/src/poseidon-tab5/src/features/
// subghz_scan.cpp:54-58), trimmed to the module's documented 855-925MHz
// range (spec Section 5 -- do not hard-code the owner's narrower intended
// 868-925MHz tuning as a hard limit).
static const float kCommonFreqsMhz[] = {868.00f, 868.35f, 915.00f, 925.00f};
constexpr int kCommonFreqCount = sizeof(kCommonFreqsMhz) / sizeof(kCommonFreqsMhz[0]);
static int s_freq_idx = 0;

// Same real burst-gap/min-edges constants as rf433_scan.cpp -- identical OOK
// burst-delimiting problem, identical real-hardware-derived reasoning (25ms
// clears the ~8-15ms repeat band with margin).
constexpr uint32_t kBurstGapThresholdUs = 25000;
constexpr size_t kMinEdgesForSignal = 10;
constexpr size_t kMaxFinalizesPerPoll = 2;
constexpr size_t kDrainChunk = 64;

static const char kCaptureDir[] = "/quarky/captures/subghz";

// Root for the deep-browsable bundled Flipper SubGhz signal database
// (/quarky/sub/flipperdb) -- rooted one level above flipperdb/ itself, same
// reasoning as ir_clone.cpp's own kRootDir (/quarky/ir, one level above
// /quarky/ir/flipperdb): a future second bundle/category living alongside
// flipperdb/ is browsable too without hardcoding the flipperdb name here.
// Deliberately separate from kCaptureDir above -- this project's own
// established pattern (see IR: /quarky/ir browse root vs. /quarky/captures/ir
// save target) keeps a bundled-library browse root and this project's own
// new-capture save directory as two distinct real trees, not one.
static const char kSubLibraryRootDir[] = "/quarky/sub";

// ── UI widgets ──────────────────────────────────────────────────────────────
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_freq_label = nullptr;
static lv_obj_t *s_toggle_btn = nullptr;
static lv_obj_t *s_toggle_lbl = nullptr;
static lv_obj_t *s_hotcold_btn = nullptr;
static lv_obj_t *s_hotcold_lbl = nullptr;
static lv_obj_t *s_hotcold_bar = nullptr;
static lv_obj_t *s_hotcold_rssi_label = nullptr;
static lv_obj_t *s_list = nullptr;
static lv_obj_t *s_placeholder = nullptr;
static lv_obj_t *s_decode_result_label = nullptr;
static lv_obj_t *s_save_status_label = nullptr;
static lv_obj_t *s_replay_status_label = nullptr;
static lv_obj_t *s_loaded_status_label = nullptr;
static uint32_t s_selected_capture_id = 0;

static const char kSubFileExt[] = ".sub";
static CapturedSignal *s_loaded_signal = nullptr;
static bool s_has_loaded_signal = false;
static char s_loaded_path[160];

static void set_hidden_label(lv_obj_t *label, const char *text) {
    if (!label) return;
    lv_label_set_text(label, text);
    if (text[0] == '\0') lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_remove_flag(label, LV_OBJ_FLAG_HIDDEN);
}

static const CapturedSignal *find_signal_by_id(uint32_t id) {
    for (size_t i = 0; i < s_signal_count; i++) {
        if (s_signals[i].capture_id == id) return &s_signals[i];
    }
    return nullptr;
}

static void update_freq_label() {
    if (!s_freq_label) return;
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Freq: %.3f MHz", (double)kCommonFreqsMhz[s_freq_idx]);
    lv_label_set_text(s_freq_label, buf);
}

static void update_status_label() {
    if (!s_status_label) return;
    if (s_active) lv_label_set_text(s_status_label, "Status: Capturing...");
    else if (s_hotcold_active) lv_label_set_text(s_status_label, "Status: Hot/Cold running");
    else lv_label_set_text(s_status_label, "Status: Idle");
}

static void add_signal_to_list(const CapturedSignal &sig) {
    if (!s_list) return;
    if (s_placeholder) {
        lv_obj_delete(s_placeholder);
        s_placeholder = nullptr;
    }
    while (lv_obj_get_child_count(s_list) >= kMaxCapturedSignals) {
        lv_obj_t *oldest = lv_obj_get_child(s_list, 0);
        if (!oldest) break;
        lv_obj_delete(oldest);
    }
    uint32_t duration_us = sig.edge_count > 1
        ? sig.edges[sig.edge_count - 1].timestamp_us - sig.edges[0].timestamp_us : 0;
    char row[112];
    std::snprintf(row, sizeof(row), "Sig #%u: %u edges, ~%lu ms @ %.2fMHz%s",
                  (unsigned)sig.capture_id, (unsigned)sig.edge_count,
                  (unsigned long)(duration_us / 1000), (double)(sig.freq_hz / 1000000.0),
                  sig.truncated ? " [truncated]" : "");
    lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_AUDIO, row);
    lv_obj_add_event_cb(btn, [](lv_event_t *e) {
        uint32_t id = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
        if (!find_signal_by_id(id)) return;
        s_selected_capture_id = id;
        Serial.printf("quarky-tab5: [cc1101-scan] selected signal #%u\n", (unsigned)id);
    }, LV_EVENT_CLICKED, (void *)(uintptr_t)sig.capture_id);
}

static void finalize_burst() {
    if (s_accum_edge_count < kMinEdgesForSignal) {
        s_accum_edge_count = 0;
        s_accum_truncated = false;
        return;
    }
    if (s_signal_count >= kMaxCapturedSignals) {
        for (size_t i = 1; i < kMaxCapturedSignals; i++) s_signals[i - 1] = s_signals[i];
        s_signal_count = kMaxCapturedSignals - 1;
    }
    size_t idx = s_signal_count++;
    s_signals[idx].edge_count = s_accum_edge_count;
    s_signals[idx].captured_at_ms = millis();
    s_signals[idx].capture_id = s_next_capture_id++;
    s_signals[idx].truncated = s_accum_truncated;
    s_signals[idx].freq_hz = (uint32_t)(kCommonFreqsMhz[s_freq_idx] * 1000000.0f);
    std::memcpy(s_signals[idx].edges, s_accum_edges, sizeof(EdgeSample) * s_accum_edge_count);

    Serial.printf("quarky-tab5: [cc1101-scan] captured burst #%u: %u edges @ %.3fMHz%s\n",
                  (unsigned)s_signals[idx].capture_id, (unsigned)s_accum_edge_count,
                  (double)kCommonFreqsMhz[s_freq_idx],
                  s_accum_truncated ? " (TRUNCATED)" : "");

    // Decode attempt -- SubghzProto::decode() needs unsigned DURATIONS (gaps
    // between edges), not edge timestamps. Build that array here, same
    // conversion rf433_protocol_decode.cpp's own callers already perform.
    static unsigned int *durs = new unsigned int[kMaxEdgesPerSignal];
    size_t dur_count = 0;
    for (size_t i = 1; i < s_signals[idx].edge_count && dur_count < kMaxEdgesPerSignal; i++) {
        durs[dur_count++] = s_signals[idx].edges[i].timestamp_us - s_signals[idx].edges[i - 1].timestamp_us;
    }
    SubghzProto::Match match{};
    bool decoded = SubghzProto::decode(durs, (uint16_t)dur_count, &match);
    if (decoded) {
        Serial.printf("quarky-tab5: [cc1101-scan] decoded: %s key=0x%llX bits=%u\n",
                      match.name, (unsigned long long)match.key, (unsigned)match.bits);
    }

    add_signal_to_list(s_signals[idx]);
    if (decoded && s_decode_result_label) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "Sig #%u decoded: %s key=0x%llX (%u bits)",
                      (unsigned)s_signals[idx].capture_id, match.name,
                      (unsigned long long)match.key, (unsigned)match.bits);
        set_hidden_label(s_decode_result_label, buf);
    }

    s_accum_edge_count = 0;
    s_accum_truncated = false;
}

static void set_capture_active(bool active) {
    if (s_active == active) return;
    if (active) {
        if (s_hotcold_active) {
            Serial.println("quarky-tab5: [cc1101-scan] can't start capture -- "
                            "hot/cold mode is running, stop it first");
            return;
        }
        if (!isr_capture_start()) return;
        s_accum_edge_count = 0;
        s_accum_truncated = false;
        s_last_edge_time_us = 0;
        s_active = true;
    } else {
        isr_capture_stop();
        s_active = false;
    }
    update_status_label();
}

// ── Hot/cold: continuous RSSI sampling against a learned floor. Real
// technique ported from Poseidon's subghz_jam_detect.cpp (warmup baseline
// mean over the first samples, then a live delta-from-baseline readout) --
// repurposed here as a signal-finder rather than a jam alarm (same real
// underlying primitive: CC1101 getRSSI() sampled continuously). ──
namespace {
constexpr int kWarmupSamples = 30;
int s_hc_samples = 0;
int32_t s_hc_sum = 0;
float s_hc_baseline = -90.0f;
uint32_t s_hc_last_sample_ms = 0;
} // namespace

static void set_hotcold_active(bool active) {
    if (s_hotcold_active == active) return;
    if (active) {
        if (s_active) {
            Serial.println("quarky-tab5: [cc1101-scan] can't start hot/cold -- "
                            "a capture is running, stop it first");
            return;
        }
        if (!Cc1101Hw::is_present() && !Cc1101Hw::init()) {
            Serial.println("quarky-tab5: [cc1101-scan] hot/cold REFUSED -- CC1101 not present");
            return;
        }
        Cc1101Hw::set_frequency_mhz(kCommonFreqsMhz[s_freq_idx]);
        // getRSSI() only reports a live reading while the chip is actually
        // in RX -- set_frequency_mhz() alone leaves it IDLE (confirmed by
        // reading RadioLib's real CC1101::setFrequency(), see
        // cc1101_spectrum.cpp's own comment on this same fact). Real bug
        // found and fixed in this session's own review (never verified on
        // hardware): without this call, hot/cold mode would have sampled a
        // stale/idle RSSI register the whole time.
        if (!Cc1101Hw::enable_async_rx()) {
            Serial.println("quarky-tab5: [cc1101-scan] hot/cold REFUSED -- enable_async_rx() failed");
            return;
        }
        s_hc_samples = 0;
        s_hc_sum = 0;
        s_hc_last_sample_ms = 0;
        s_hotcold_active = true;
    } else {
        Cc1101Hw::idle();
        s_hotcold_active = false;
        if (s_hotcold_bar) lv_bar_set_value(s_hotcold_bar, 0, LV_ANIM_OFF);
    }
    update_status_label();
}

static void hotcold_poll() {
    if (!s_hotcold_active) return;
    uint32_t now = millis();
    if (now - s_hc_last_sample_ms < 100) return; // ~10Hz sample rate, matches
                                                   // Poseidon's own ~20-30ms
                                                   // cadence closely enough
                                                   // for a hand-held "hotter/
                                                   // colder" indicator
    s_hc_last_sample_ms = now;
    float rssi = Cc1101Hw::rssi_dbm();

    if (s_hc_samples < kWarmupSamples) {
        s_hc_sum += (int32_t)rssi;
        s_hc_samples++;
        if (s_hc_samples == kWarmupSamples) {
            s_hc_baseline = (float)s_hc_sum / (float)kWarmupSamples;
        }
        if (s_hotcold_rssi_label) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Learning baseline... %.1f dBm", (double)rssi);
            lv_label_set_text(s_hotcold_rssi_label, buf);
        }
        return;
    }

    float delta = rssi - s_hc_baseline; // higher (less negative) = hotter,
                                         // closer to a transmitting source
    if (s_hotcold_rssi_label) {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "RSSI: %.1f dBm  (baseline %.1f, delta %+.1f)",
                      (double)rssi, (double)s_hc_baseline, (double)delta);
        lv_label_set_text(s_hotcold_rssi_label, buf);
    }
    if (s_hotcold_bar) {
        // Map delta in [0, 30] dBm above baseline to a 0-100 bar -- 30dBm of
        // headroom above a learned noise floor is a generous real-world span
        // for "someone is pressing a remote nearby" without needing exact
        // calibration; the bar is a relative indicator, not an absolute
        // measurement.
        int pct = (int)((delta / 30.0f) * 100.0f);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        lv_bar_set_value(s_hotcold_bar, pct, LV_ANIM_ON);
    }
}

// ── Screen ────────────────────────────────────────────────────────────────

static void on_sub_file_selected(const char *path, void *) {
    std::strncpy(s_loaded_path, path, sizeof(s_loaded_path) - 1);
    s_loaded_path[sizeof(s_loaded_path) - 1] = '\0';
    bool ok = Cc1101Record::load(storage, s_loaded_path, s_loaded_signal);
    s_has_loaded_signal = ok;
    char buf[220];
    if (ok) {
        std::snprintf(buf, sizeof(buf), "Loaded: %s (%u edges @ %.3fMHz)%s", s_loaded_path,
                      (unsigned)s_loaded_signal->edge_count,
                      (double)(s_loaded_signal->freq_hz / 1000000.0),
                      s_loaded_signal->truncated ? " [truncated]" : "");
    } else {
        std::snprintf(buf, sizeof(buf), "Failed to load %s", s_loaded_path);
    }
    if (s_loaded_status_label) lv_label_set_text(s_loaded_status_label, buf);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("CC1101 Scan", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    update_status_label();
    s_freq_label = lv_label_create(content);
    update_freq_label();

    lv_obj_t *btn_grid = lv_obj_create(content);
    lv_obj_set_width(btn_grid, LV_PCT(100));
    lv_obj_set_height(btn_grid, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_all(btn_grid, 2, 0);
    lv_obj_set_style_pad_gap(btn_grid, 4, 0);

    lv_obj_t *freq_btn = lv_button_create(btn_grid);
    lv_obj_set_width(freq_btn, LV_PCT(32));
    lv_obj_t *freq_lbl = lv_label_create(freq_btn);
    lv_label_set_text(freq_lbl, "Next Freq");
    lv_obj_add_event_cb(freq_btn, [](lv_event_t *) {
        if (s_active || s_hotcold_active) return; // don't retune mid-session
        s_freq_idx = (s_freq_idx + 1) % kCommonFreqCount;
        update_freq_label();
    }, LV_EVENT_CLICKED, nullptr);

    s_toggle_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_toggle_btn, LV_PCT(32));
    s_toggle_lbl = lv_label_create(s_toggle_btn);
    lv_label_set_text(s_toggle_lbl, "Start Capture");
    lv_obj_add_event_cb(s_toggle_btn, [](lv_event_t *) {
        set_capture_active(!s_active);
        lv_label_set_text(s_toggle_lbl, s_active ? "Stop Capture" : "Start Capture");
    }, LV_EVENT_CLICKED, nullptr);

    s_hotcold_btn = lv_button_create(btn_grid);
    lv_obj_set_width(s_hotcold_btn, LV_PCT(32));
    s_hotcold_lbl = lv_label_create(s_hotcold_btn);
    lv_label_set_text(s_hotcold_lbl, "Hot/Cold: Start");
    lv_obj_add_event_cb(s_hotcold_btn, [](lv_event_t *) {
        set_hotcold_active(!s_hotcold_active);
        lv_label_set_text(s_hotcold_lbl, s_hotcold_active ? "Hot/Cold: Stop" : "Hot/Cold: Start");
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *replay_btn = lv_button_create(btn_grid);
    lv_obj_set_width(replay_btn, LV_PCT(32));
    lv_obj_t *replay_lbl = lv_label_create(replay_btn);
    lv_label_set_text(replay_lbl, "Replay Selected");
    lv_obj_add_event_cb(replay_btn, [](lv_event_t *) {
        const CapturedSignal *found = find_signal_by_id(s_selected_capture_id);
        if (!found) {
            Serial.println("quarky-tab5: [cc1101-scan] Replay tapped, nothing selected");
            return;
        }
        Cc1101Replay::transmit(*found);
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *save_btn = lv_button_create(btn_grid);
    lv_obj_set_width(save_btn, LV_PCT(32));
    lv_obj_t *save_lbl = lv_label_create(save_btn);
    lv_label_set_text(save_lbl, "Save as .sub");
    lv_obj_add_event_cb(save_btn, [](lv_event_t *) {
        const CapturedSignal *found = find_signal_by_id(s_selected_capture_id);
        if (!found) {
            set_hidden_label(s_save_status_label, "Select a signal first.");
            return;
        }
        char path[96];
        std::snprintf(path, sizeof(path), "%s/capture_%u.sub", kCaptureDir,
                      (unsigned)s_selected_capture_id);
        bool ok = Cc1101Record::save(storage, path, *found);
        char buf[128];
        std::snprintf(buf, sizeof(buf), ok ? "Saved to %s" : "Save failed (%s)", path);
        set_hidden_label(s_save_status_label, buf);
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *load_btn = lv_button_create(btn_grid);
    lv_obj_set_width(load_btn, LV_PCT(32));
    lv_obj_t *load_lbl = lv_label_create(load_btn);
    lv_label_set_text(load_lbl, "Load from SD");
    lv_obj_add_event_cb(load_btn, [](lv_event_t *) {
        // Deep-navigates from /quarky/sub (parent of the bundled Flipper
        // SubGhz database, /quarky/sub/flipperdb -- confirmed genuinely
        // deep, same shape as Flipper-IRDB) rather than a flat picker --
        // matches IrClone's own real browser exactly (project owner's
        // explicit 2026-08-25 request). Kept separate from kCaptureDir
        // (/quarky/captures/subghz, where THIS project's own new captures
        // save to) the same way IR keeps its own /quarky/ir browse root
        // and /quarky/captures/ir save target as two distinct real trees.
        DeepFileBrowser::push(storage, kSubLibraryRootDir, kSubFileExt, on_sub_file_selected);
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *replay_loaded_btn = lv_button_create(btn_grid);
    lv_obj_set_width(replay_loaded_btn, LV_PCT(32));
    lv_obj_t *replay_loaded_lbl = lv_label_create(replay_loaded_btn);
    lv_label_set_text(replay_loaded_lbl, "Replay Loaded");
    lv_obj_add_event_cb(replay_loaded_btn, [](lv_event_t *) {
        if (!s_has_loaded_signal) return;
        Cc1101Replay::transmit(*s_loaded_signal);
    }, LV_EVENT_CLICKED, nullptr);

    s_decode_result_label = lv_label_create(content);
    lv_label_set_long_mode(s_decode_result_label, LV_LABEL_LONG_WRAP);
    set_hidden_label(s_decode_result_label, "");

    s_save_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_save_status_label, LV_LABEL_LONG_WRAP);
    set_hidden_label(s_save_status_label, "");

    s_replay_status_label = lv_label_create(content);
    lv_label_set_text(s_replay_status_label, "Replay: Idle");

    s_loaded_status_label = lv_label_create(content);
    lv_label_set_text(s_loaded_status_label, "Loaded: none");

    s_hotcold_rssi_label = lv_label_create(content);
    lv_label_set_text(s_hotcold_rssi_label, "Hot/Cold: not running");
    s_hotcold_bar = lv_bar_create(content);
    lv_obj_set_size(s_hotcold_bar, LV_PCT(100), 16);
    lv_bar_set_range(s_hotcold_bar, 0, 100);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));
    if (s_signal_count == 0) {
        s_placeholder = lv_list_add_text(s_list, "No signals captured yet");
    } else {
        for (size_t i = 0; i < s_signal_count; i++) add_signal_to_list(s_signals[i]);
    }

    lv_obj_add_event_cb(content, [](lv_event_t *) {
        bool was_active = s_active;
        bool was_hc = s_hotcold_active;
        s_status_label = nullptr;
        s_freq_label = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_lbl = nullptr;
        s_hotcold_btn = nullptr;
        s_hotcold_lbl = nullptr;
        s_hotcold_bar = nullptr;
        s_hotcold_rssi_label = nullptr;
        s_list = nullptr;
        s_placeholder = nullptr;
        s_decode_result_label = nullptr;
        s_save_status_label = nullptr;
        s_replay_status_label = nullptr;
        s_loaded_status_label = nullptr;
        if (was_active) set_capture_active(false);
        if (was_hc) set_hotcold_active(false);
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() {
    ScreenStack::push(build_screen());
}

void register_module() {
    s_signals = static_cast<CapturedSignal *>(
        heap_caps_malloc(sizeof(CapturedSignal) * kMaxCapturedSignals, MALLOC_CAP_SPIRAM));
    if (!s_signals) {
        Serial.println("quarky-tab5: [cc1101-scan] heap_caps_malloc FAILED for s_signals -- "
                        "CC1101 Scan will not be registered");
        return;
    }
    s_accum_edges = static_cast<EdgeSample *>(
        heap_caps_malloc(sizeof(EdgeSample) * kMaxEdgesPerSignal, MALLOC_CAP_SPIRAM));
    if (!s_accum_edges) {
        Serial.println("quarky-tab5: [cc1101-scan] heap_caps_malloc FAILED for s_accum_edges -- "
                        "CC1101 Scan will not be registered");
        heap_caps_free(s_signals);
        s_signals = nullptr;
        return;
    }
    s_loaded_signal = static_cast<CapturedSignal *>(
        heap_caps_malloc(sizeof(CapturedSignal), MALLOC_CAP_SPIRAM));
    if (!s_loaded_signal) {
        Serial.println("quarky-tab5: [cc1101-scan] heap_caps_malloc FAILED for s_loaded_signal -- "
                        "CC1101 Scan will not be registered");
        heap_caps_free(s_signals);
        s_signals = nullptr;
        heap_caps_free(s_accum_edges);
        s_accum_edges = nullptr;
        return;
    }
    g_registry.register_module({"cc1101_scan", "CC1101 Scan/Decode", Category::SUBGHZ,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void poll() {
    hotcold_poll();

    if (s_active) {
        if (s_overrun) {
            Serial.println("quarky-tab5: [cc1101-scan] capture overrun -- stopping");
            set_capture_active(false);
            if (s_status_label) lv_label_set_text(s_status_label, "Status: Overrun -- check antenna");
        } else {
            EdgeSample chunk[kDrainChunk];
            size_t finalizes_this_tick = 0;
            size_t processed = 0;
            while (processed < kMaxEdgesPerSignal) {
                size_t n = isr_capture_read(chunk, kDrainChunk);
                if (n == 0) break;
                for (size_t i = 0; i < n; i++) {
                    uint32_t t = chunk[i].timestamp_us;
                    if (s_accum_edge_count > 0 && (t - s_last_edge_time_us) > kBurstGapThresholdUs) {
                        if (finalizes_this_tick < kMaxFinalizesPerPoll) {
                            finalize_burst();
                            finalizes_this_tick++;
                        }
                    }
                    if (s_accum_edge_count < kMaxEdgesPerSignal) {
                        s_accum_edges[s_accum_edge_count++] = chunk[i];
                    } else {
                        s_accum_truncated = true;
                    }
                    s_last_edge_time_us = t;
                }
                processed += n;
            }
            if (s_accum_edge_count > 0 && finalizes_this_tick < kMaxFinalizesPerPoll) {
                uint32_t now_us = micros();
                if ((now_us - s_last_edge_time_us) > kBurstGapThresholdUs) finalize_burst();
            }
        }
    }

    // Replay status (independent of capture/hot-cold state).
    if (s_replay_status_label) {
        const char *trunc_suffix = Cc1101Replay::last_transmit_was_truncated()
                                        ? " [TRUNCATED]" : "";
        switch (Cc1101Replay::state()) {
            case Cc1101Replay::State::kIdle:
                lv_label_set_text(s_replay_status_label, "Replay: Idle");
                break;
            case Cc1101Replay::State::kTransmitting: {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "Replay: Transmitting...%s", trunc_suffix);
                lv_label_set_text(s_replay_status_label, buf);
                break;
            }
            case Cc1101Replay::State::kDone: {
                char buf[64];
                std::snprintf(buf, sizeof(buf), "Replay: Done%s", trunc_suffix);
                lv_label_set_text(s_replay_status_label, buf);
                break;
            }
            case Cc1101Replay::State::kFailed: {
                char buf[112];
                std::snprintf(buf, sizeof(buf), "Replay: Failed (%s)", Cc1101Replay::failure_reason());
                lv_label_set_text(s_replay_status_label, buf);
                break;
            }
        }
    }
}

size_t signal_count() { return s_signal_count; }

const CapturedSignal *get_signal(size_t index) {
    if (index >= s_signal_count) return nullptr;
    return &s_signals[index];
}

bool is_capturing() { return s_active; }

} // namespace Cc1101Scan
