#include "cc1101_jammer.h"
#include "cc1101_hw.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdio>

extern FeatureRegistry g_registry;

// Real timing constants/algorithms ported from Bruce's rf_jammer.cpp
// (~/src/firmware/src/modules/rf/rf_jammer.cpp:8-9,212-336,503-542), read
// directly for this port -- see cc1101_jammer.h's header comment for the
// full scope note (full+intermittent only, matching this phase's spec).

namespace Cc1101Jammer {
namespace {

constexpr uint32_t kMaxSequence = 50;  // rf_jammer.cpp MAX_SEQUENCE
constexpr uint32_t kDurationCycles = 3; // rf_jammer.cpp DURATION_CYCLES

// Real crash class found on real hardware in cc1101_replay.cpp (2026-08-25,
// see that file's own header comment): a tight delayMicroseconds() loop
// with no yield starves this task's pinned core's IDLE task past the
// task_wdt timeout. Jammer is WORSE than that case, not just similarly
// exposed -- both run_full_jammer()/run_itmt_jammer() below are `while
// (!s_stop_requested)` loops with no upper bound at all (jamming is
// designed to run until the user stops it), so without a periodic yield
// this crashes within the watchdog window on every single run, 100%
// reproducible, not just on a long signal. Same fix: a vTaskDelay(1) paced
// by elapsed time, checked once per outer-loop pass.
constexpr uint32_t kYieldIntervalUs = 500000;

struct TaskArgs {
    JamMode mode;
    float freq_mhz;
};

volatile bool s_running = false;
volatile bool s_stop_requested = false;
volatile bool s_task_done = false;
volatile uint32_t s_pulse_count = 0;
volatile uint32_t s_elapsed_ms = 0;
JamMode s_mode = JamMode::kFull;

// rf_jammer.cpp's real send_optimized_pulse() (line 503), verbatim.
void send_optimized_pulse(int pin, int width) {
    for (uint32_t i = 0; i < (uint32_t)width; i += 6) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pin, LOW);
        delayMicroseconds(1);
    }
    digitalWrite(pin, HIGH);
    delayMicroseconds(width / 3);
    digitalWrite(pin, LOW);
    uint32_t lowPeriod = width / 4;
    if (lowPeriod > 0) delayMicroseconds(lowPeriod);
}

// rf_jammer.cpp's real send_random_pattern() (line 522), verbatim except
// the EscPress/millis-window check (replaced by s_stop_requested + the same
// 250ms window).
void send_random_pattern(int pin, int numPulses) {
    uint32_t startTime = millis();
    for (int i = 0; i < numPulses && !s_stop_requested; i++) {
        uint32_t pulseWidth = 1 + (micros() % 60);
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(pin, LOW);
        uint32_t spaceWidth = 1 + (micros() % 8);
        delayMicroseconds(spaceWidth);
        if (millis() - startTime > 250) break;
    }
}

// rf_jammer.cpp's real run_full_jammer() (line 212), ported: 3-phase
// rotation every 100ms (micro-glitches / variable-width bursts / sustained
// carrier+hard cuts), same phase durations and pulse counts.
void run_full_jammer(int pin) {
    uint32_t startTime = millis();
    uint8_t phase = 0;
    uint32_t last_yield_us = micros();
    while (!s_stop_requested) {
        if (micros() - last_yield_us > kYieldIntervalUs) {
            vTaskDelay(1);
            last_yield_us = micros();
        }
        uint32_t elapsed = millis() - startTime;
        phase = (elapsed / 100) % 3;
        switch (phase) {
            case 0:
                for (int i = 0; i < 100 && !s_stop_requested; i++) {
                    digitalWrite(pin, HIGH);
                    delayMicroseconds(3);
                    digitalWrite(pin, LOW);
                    delayMicroseconds(1);
                    s_pulse_count++;
                }
                break;
            case 1:
                for (int i = 0; i < 50 && !s_stop_requested; i++) {
                    uint32_t w = 2 + (micros() % 18);
                    digitalWrite(pin, HIGH);
                    delayMicroseconds(w);
                    digitalWrite(pin, LOW);
                    delayMicroseconds(1);
                    s_pulse_count++;
                }
                break;
            case 2:
                digitalWrite(pin, HIGH);
                delayMicroseconds(80);
                digitalWrite(pin, LOW);
                delayMicroseconds(2);
                digitalWrite(pin, HIGH);
                delayMicroseconds(80);
                s_pulse_count += 2;
                break;
        }
        s_elapsed_ms = millis() - startTime;
    }
    digitalWrite(pin, LOW);
}

// rf_jammer.cpp's real run_itmt_jammer() (line 278): forward sweep 10-500us
// (kMaxSequence steps of 10us each), reverse sweep, then a random noise
// burst -- same real structure, EscPress checks replaced with
// s_stop_requested.
void run_itmt_jammer(int pin) {
    uint32_t startTime = millis();
    uint32_t sequenceValues[kMaxSequence];
    for (uint32_t i = 0; i < kMaxSequence; i++) sequenceValues[i] = 10 * (i + 1);

    uint32_t last_yield_us = micros();
    while (!s_stop_requested) {
        if (micros() - last_yield_us > kYieldIntervalUs) {
            vTaskDelay(1);
            last_yield_us = micros();
        }
        for (uint32_t sequence = 0; sequence < kMaxSequence && !s_stop_requested; sequence++) {
            for (uint32_t d = 0; d < kDurationCycles && !s_stop_requested; d++) {
                send_optimized_pulse(pin, (int)sequenceValues[sequence]);
                s_pulse_count++;
            }
        }
        for (int sequence = (int)kMaxSequence - 1; sequence >= 0 && !s_stop_requested; sequence--) {
            send_optimized_pulse(pin, (int)sequenceValues[sequence]);
            s_pulse_count++;
        }
        if (!s_stop_requested) {
            send_random_pattern(pin, 200);
            s_pulse_count += 200;
        }
        s_elapsed_ms = millis() - startTime;
    }
    digitalWrite(pin, LOW);
}

void jammer_task(void *arg) {
    TaskArgs *args = static_cast<TaskArgs *>(arg);
    int pin = Cc1101Hw::gdo0_pin();

    if (args->mode == JamMode::kFull) run_full_jammer(pin);
    else run_itmt_jammer(pin);

    Cc1101Hw::idle();
    delete args;
    s_task_done = true;
    vTaskDelete(nullptr);
}

} // namespace

void start(JamMode mode, float freq_mhz) {
    if (s_running) {
        Serial.println("quarky-tab5: [cc1101-jammer] start() REFUSED -- already running");
        return;
    }
    if (!Cc1101Hw::is_present() && !Cc1101Hw::init()) {
        Serial.println("quarky-tab5: [cc1101-jammer] start() REFUSED -- CC1101 not present");
        return;
    }
    Cc1101Hw::set_frequency_mhz(freq_mhz);
    if (!Cc1101Hw::enable_async_tx()) {
        Serial.println("quarky-tab5: [cc1101-jammer] start() REFUSED -- enable_async_tx() failed");
        return;
    }
    pinMode(Cc1101Hw::gdo0_pin(), OUTPUT);
    digitalWrite(Cc1101Hw::gdo0_pin(), LOW);

    Serial.printf("quarky-tab5: [cc1101-jammer] STARTING JAM on %.3fMHz, mode=%s -- "
                  "ACTIVELY TRANSMITTING. Authorized test environment only.\n",
                  (double)freq_mhz, mode == JamMode::kFull ? "FULL" : "INTERMITTENT");

    TaskArgs *args = new TaskArgs{mode, freq_mhz};
    s_mode = mode;
    s_stop_requested = false;
    s_task_done = false;
    s_pulse_count = 0;
    s_elapsed_ms = 0;
    s_running = true;

    BaseType_t created = xTaskCreatePinnedToCore(
        jammer_task, "cc1101_jam", 4096, args, 5,
        nullptr, (ARDUINO_RUNNING_CORE == 0) ? 1 : 0);
    if (created != pdPASS) {
        delete args;
        s_running = false;
        Serial.println("quarky-tab5: [cc1101-jammer] xTaskCreatePinnedToCore() FAILED");
    }
}

void stop() {
    s_stop_requested = true;
}

namespace { void update_status_ui(const JammerStatus &st); }

bool poll(JammerStatus *out) {
    if (s_task_done) {
        s_running = false;
        s_task_done = false;
        Serial.println("quarky-tab5: [cc1101-jammer] jam session stopped");
    }
    JammerStatus st;
    st.running = s_running;
    st.mode = s_mode;
    st.pulse_count = s_pulse_count;
    st.elapsed_ms = s_elapsed_ms;
    update_status_ui(st);
    if (out) *out = st;
    return s_running;
}

// ===========================================================================
// UI: mode pick + start/stop, with an unmistakable "ACTIVELY TRANSMITTING"
// banner while jamming -- same clarity bar the spec sets for KeeLoq (Task 8),
// applied here too per this phase's plan Task 7 Step 2 (a jammer is even
// more clearly an active-transmission feature than KeeLoq replay).
// ===========================================================================
namespace {

lv_obj_t *s_mode_dropdown = nullptr;
lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_banner = nullptr;
lv_obj_t *s_start_btn = nullptr;
lv_obj_t *s_start_lbl = nullptr;
float s_ui_freq_mhz = 433.92f;
bool s_ui_active = false;

void update_status_ui(const JammerStatus &st) {
    if (!s_status_label) return;
    char buf[80];
    if (st.running) {
        std::snprintf(buf, sizeof(buf), "%s: %u pulses, %lu ms",
                      st.mode == JamMode::kFull ? "FULL" : "INTERMITTENT",
                      (unsigned)st.pulse_count, (unsigned long)st.elapsed_ms);
        if (s_banner) {
            lv_label_set_text(s_banner, "*** ACTIVELY TRANSMITTING -- JAMMING ***");
            lv_obj_remove_flag(s_banner, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        std::snprintf(buf, sizeof(buf), "Idle");
        if (s_banner) lv_obj_add_flag(s_banner, LV_OBJ_FLAG_HIDDEN);
    }
    lv_label_set_text(s_status_label, buf);
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("CC1101 Jammer", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *note = lv_label_create(content);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_label_set_text(note, "RF jamming -- authorized test environment only. "
                             "Real regulatory exposure -- confirm authorization before starting.");

    s_mode_dropdown = lv_dropdown_create(content);
    lv_dropdown_set_options(s_mode_dropdown, "Full\nIntermittent");

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Idle");

    s_banner = lv_label_create(content);
    lv_obj_set_style_text_color(s_banner, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_flag(s_banner, LV_OBJ_FLAG_HIDDEN);

    s_start_btn = lv_button_create(content);
    lv_obj_set_style_bg_color(s_start_btn, lv_palette_main(LV_PALETTE_RED), 0);
    s_start_lbl = lv_label_create(s_start_btn);
    lv_label_set_text(s_start_lbl, "Start Jamming");
    lv_obj_add_event_cb(s_start_btn, [](lv_event_t *) {
        if (s_ui_active) {
            stop();
            lv_label_set_text(s_start_lbl, "Start Jamming");
            s_ui_active = false;
            return;
        }
        uint32_t sel = lv_dropdown_get_selected(s_mode_dropdown);
        start(sel == 0 ? JamMode::kFull : JamMode::kIntermittent, s_ui_freq_mhz);
        lv_label_set_text(s_start_lbl, "Stop Jamming");
        s_ui_active = true;
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(content, [](lv_event_t *) {
        s_mode_dropdown = nullptr;
        s_status_label = nullptr;
        s_banner = nullptr;
        s_start_btn = nullptr;
        s_start_lbl = nullptr;
        if (s_ui_active) { stop(); s_ui_active = false; }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

} // namespace

void open_screen() { ScreenStack::push(build_screen()); }

void register_module() {
    g_registry.register_module({"cc1101_jammer", "CC1101 Jammer", Category::SUBGHZ,
                                 Affinity::TAB5_NATIVE, open_screen, nullptr});
}

} // namespace Cc1101Jammer
