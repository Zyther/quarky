#include "ir_learn.h"
#include "../../../boards/tab5/pins_config.h"
#include "../../hal/gpio53_arbiter.h"
#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <esp32-hal-rmt.h>
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage;

namespace IrLearn {
namespace {

// 1 tick = 1 microsecond, same choice/reasoning as IrCommon's TX channel
// (ir_common.cpp's kRmtTickHz) -- matches every duration unit this project's
// IR data already uses, no scaling needed converting ticks <-> microseconds.
constexpr uint32_t kRmtTickHz = 1000000;

// Idle-gap threshold (ticks == microseconds at the 1MHz tick rate above)
// that ends a capture -- passed to rmtSetRxMaxThreshold(). STARTING VALUE,
// NOT yet real-hardware-confirmed (project owner is doing that separately;
// see this file's header and hal/ir_unit.h). Reasoned the same way
// rf433_scan.cpp's own kBurstGapThresholdUs is: it must clear the longest
// genuine INTRA-frame gap with margin, while staying below the gap between
// one press's frame and that same remote's own auto-repeat frame, so a
// capture ends promptly after ONE frame rather than spanning into repeats.
// For NEC-family protocols the header space is ~4.5ms (a widely-documented
// consumer-IR fact, the same class of "well-established, not measured from
// this specific chip" knowledge IrCommon.h already leans on for its ~0.33
// duty-cycle default -- see kAssumedDutyCycle below), and the gap before an
// auto-repeat frame is commonly several tens of ms. 8ms clears the header
// figure with ~1.8x margin while staying well under the repeat-gap figure.
// Task 15's own real capture runs (hal/ir_unit.h) did not report a
// dedicated gap-length measurement, so this specific value needs its own
// real-hardware confirmation -- a genuine remote button held down, checking
// whether one press yields one clean frame or gets cut mid-frame / merges
// repeats.
//
// SECOND thing to confirm in that same real-hardware pass (raised by an
// independent review round, 2026-08-21): this module arms ONE
// rmtReadAsync() call per Start-Capture tap and never re-arms while
// State::kWaiting, relying on the assumption that a still-idle input
// (nothing pressed yet) does NOT itself trigger a premature "done" via this
// threshold -- i.e. that the idle-gap timer only starts counting once the
// FIRST real edge arrives, not from the moment rmtInit()/rmtReadAsync() was
// called. Espressif's own public ESP-IDF RMT docs state this plainly
// ("the actual reception starts at the first level change of the received
// signal"), which is why this module is built the way it is -- but the
// exact register-level behavior lives in `rmt_ll_rx_enable()`, which
// wasn't available to verify directly in this project's toolchain (only
// prebuilt .a libs, no HAL source). If real-hardware testing shows a
// capture completing near-instantly with 0 pulses regardless of whether a
// button was pressed, THIS is the assumption to revisit first -- the fix
// would be re-arming rmtReadAsync() in poll() whenever a completion
// contains no real mark data, rather than surfacing that as State::kFailed.
constexpr uint16_t kIdleThresholdTicks = 8000; // 8ms

// Neither of these is measurable from an already-demodulated RX signal (see
// hal/ir_unit.h's IRM-3638T citation -- carrier is stripped before it ever
// reaches GPIO54). Both are the same well-established consumer-IR defaults
// this project already cites elsewhere without a chip-specific measurement:
// 38kHz is the near-universal carrier for the NEC/RC5/Sony family (the same
// family IrTvbGone's world_ir_codes.h database targets), and 1/3 is the
// exact duty-cycle default ir_tvbgone.cpp's decode_and_transmit() and
// ir_file_format.h's own real .ir-format citation (0.330000) both already
// use for the identical reason (this class of receiver is broadly
// duty-cycle-tolerant, so the original signal's exact figure is not
// recoverable and not usually needed to reproduce a working transmit).
constexpr uint32_t kAssumedCarrierHz = 38000;
constexpr float kAssumedDutyCycle = 1.0f / 3.0f;

// Sized like IrCommon::kMaxDurationsPerTransmit's own symbol buffer
// (ir_common.cpp's s_symbols): kMaxPulses durations -> half as many
// two-duration RMT symbols, rounded up. Function-scope static, not a stack
// array or a plain global -- same reasoning as ir_common.cpp's s_symbols
// (2KB at this size; small, but keeping the convention rather than special-
// casing this one buffer).
constexpr size_t kMaxRxSymbols = (kMaxPulses + 1) / 2;
static rmt_data_t s_rx_symbols[kMaxRxSymbols];
static size_t s_rx_symbol_count = 0; // in: capacity passed to rmtReadAsync();
                                      // out: actual symbols read, once
                                      // rmtReceiveCompleted() is true

State s_state = State::kIdle;
LearnedCode s_result{};
char s_error[64] = "";

// SD save convention: matches this project's existing per-feature
// `/quarky/captures/<domain>/` directories (nfc_tag_library.cpp's
// /quarky/captures/nfc, rf433_scan.cpp's /quarky/captures/rf433, etc. --
// see this task's own controller notes for the full cross-check). Filename
// is timestamp-keyed (matching wifi_pmkid.cpp's capture_<ms>.pcap /
// ble_sniffer.cpp's sniff_<ms>.csv), not id-keyed like rf433_scan.cpp's
// capture_<id>.sub -- this module holds only ONE most-recent capture at a
// time (no session ring/list the way Rf433Scan keeps kMaxCapturedSignals),
// so there is no stable small integer id to key off; millis() is unique
// enough for a single-slot "save what I just captured" action.
constexpr char kSaveDir[] = "/quarky/captures/ir";

lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_toggle_btn = nullptr;
lv_obj_t *s_toggle_label = nullptr;
lv_obj_t *s_save_btn = nullptr;
lv_obj_t *s_save_status_label = nullptr;

// Releases the RMT RX channel and the GPIO53 arbiter claim. Safe to call
// defensively (rmtDeinit()/Gpio53Arbiter::release() are both no-ops if
// nothing is held) -- shared by the normal capture-complete path in poll()
// and the user/teardown-initiated cancel path.
void teardown_rx() {
    rmtDeinit(TAB5_IR_RX_GPIO);
    Gpio53Arbiter::release(Gpio53Arbiter::Owner::kIr);
}

// Converts the RMT symbols captured this run into s_result's
// pulse_widths_us[]/count. Mirror image of ir_common.cpp's transmit_raw()
// packing (mark/space -> duration0/duration1 pairs), run in reverse
// (duration0/duration1 -> mark/space durations), with one real-hardware-
// specific twist transmit_raw() doesn't need: RMT's level bits are the RAW
// GPIO level actually observed on TAB5_IR_RX_GPIO, and the IRM-3638T is
// active-low (idle HIGH, pulled LOW during a demodulated carrier burst --
// see hal/ir_unit.h's RX POLARITY note, confirmed for the IRM-36xx/TSOP38
// family as a whole, independently re-confirmed for this specific unit by
// Task 15's real capture). So level == 0 (LOW) is when a real burst was
// present -- a "mark" in this project's mark-first .ir/IrCommon convention
// -- and level == 1 (HIGH) is a "space". This INVERTS the raw GPIO reading;
// do not copy this inversion into a context wired to a non-active-low
// receiver.
//
// Returns false if no real mark was ever observed (nothing captured, or the
// idle threshold fired before the user pressed anything).
bool convert_symbols_to_code(const rmt_data_t *symbols, size_t n_symbols, LearnedCode *out) {
    out->count = 0;
    out->truncated = false;
    bool seen_mark = false;

    for (size_t i = 0; i < n_symbols; i++) {
        const uint16_t durs[2] = {(uint16_t)symbols[i].duration0, (uint16_t)symbols[i].duration1};
        const uint8_t levels[2] = {(uint8_t)symbols[i].level0, (uint8_t)symbols[i].level1};

        for (int j = 0; j < 2; j++) {
            // duration == 0 is RMT's own real end-of-transmission marker
            // (same convention ir_common.cpp's transmit_raw() writes on the
            // TX side for an odd final duration) -- nothing real follows it.
            if (durs[j] == 0) {
                out->carrier_hz = kAssumedCarrierHz;
                out->duty_cycle = kAssumedDutyCycle;
                return seen_mark && out->count > 0;
            }

            bool is_mark = (levels[j] == 0); // active-low inversion, see comment above

            if (!seen_mark) {
                if (!is_mark) {
                    // Still in the idle gap between arming the receiver and
                    // the first real burst -- could be milliseconds or many
                    // seconds (whenever the user gets around to pressing
                    // the remote) and is not part of the real signal. Drop
                    // until the first mark arrives so pulse_widths_us[0] is
                    // always a real mark, matching this project's mark-
                    // first convention.
                    continue;
                }
                seen_mark = true;
            }

            if (out->count >= kMaxPulses) {
                out->truncated = true;
                out->carrier_hz = kAssumedCarrierHz;
                out->duty_cycle = kAssumedDutyCycle;
                return true;
            }
            out->pulse_widths_us[out->count++] = durs[j];
        }
    }

    out->carrier_hz = kAssumedCarrierHz;
    out->duty_cycle = kAssumedDutyCycle;
    return seen_mark && out->count > 0;
}

void update_ui() {
    if (s_toggle_label) {
        lv_label_set_text(s_toggle_label, s_state == State::kWaiting ? "Cancel" : "Start Capture");
    }
    if (s_save_btn) {
        if (s_state == State::kDone) lv_obj_remove_state(s_save_btn, LV_STATE_DISABLED);
        else lv_obj_add_state(s_save_btn, LV_STATE_DISABLED);
    }
    if (!s_status_label) return;

    switch (s_state) {
        case State::kIdle:
            lv_label_set_text(s_status_label, "Status: Idle -- tap Start Capture, then press a "
                                               "remote button at the IR unit's receiver.");
            break;
        case State::kWaiting:
            lv_label_set_text(s_status_label, "Status: Waiting for remote...");
            break;
        case State::kDone: {
            char buf[128];
            uint32_t duration_us = 0;
            for (size_t i = 0; i < s_result.count; i++) duration_us += s_result.pulse_widths_us[i];
            std::snprintf(buf, sizeof(buf),
                          "Status: Captured %u pulses (~%lu.%01lu ms)%s. Carrier/duty are "
                          "assumed (%lu Hz, %.2f) -- not measurable from this receiver.",
                          (unsigned)s_result.count, (unsigned long)(duration_us / 1000),
                          (unsigned long)((duration_us % 1000) / 100),
                          s_result.truncated ? " [TRUNCATED]" : "",
                          (unsigned long)s_result.carrier_hz, (double)s_result.duty_cycle);
            lv_label_set_text(s_status_label, buf);
            break;
        }
        case State::kFailed: {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Status: Failed -- %s", s_error);
            lv_label_set_text(s_status_label, buf);
            break;
        }
    }
}

void cancel_capture() {
    if (s_state != State::kWaiting) return;
    Serial.println("quarky-tab5: [ir-learn] Capture cancelled by user");
    teardown_rx();
    s_state = State::kIdle;
}

bool start_capture() {
    if (s_state == State::kWaiting) {
        // Idempotent refuse, not a re-arm -- matches this project's
        // established "refuse rather than lie" convention (e.g.
        // Rf433Common::capture_start()'s own idempotent re-arm refusal).
        // Re-arming here would leak the in-flight rmtReadAsync() request.
        Serial.println("quarky-tab5: [ir-learn] start_capture() called while already "
                        "waiting -- ignored");
        return false;
    }

    if (!Gpio53Arbiter::claim(Gpio53Arbiter::Owner::kIr)) {
        Serial.println("quarky-tab5: [ir-learn] start_capture() REFUSED -- PORT.A is "
                        "held by another owner (NFC/RFID2 or RF433)");
        std::snprintf(s_error, sizeof(s_error), "PORT.A busy (NFC/RFID2 or RF433)");
        s_state = State::kFailed;
        return false;
    }

    if (!rmtInit(TAB5_IR_RX_GPIO, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_1, kRmtTickHz)) {
        Serial.println("quarky-tab5: [ir-learn] rmtInit(RX) FAILED");
        Gpio53Arbiter::release(Gpio53Arbiter::Owner::kIr);
        std::snprintf(s_error, sizeof(s_error), "rmtInit(RX) failed");
        s_state = State::kFailed;
        return false;
    }

    if (!rmtSetRxMaxThreshold(TAB5_IR_RX_GPIO, kIdleThresholdTicks)) {
        Serial.println("quarky-tab5: [ir-learn] rmtSetRxMaxThreshold() FAILED");
        teardown_rx();
        std::snprintf(s_error, sizeof(s_error), "rmtSetRxMaxThreshold() failed");
        s_state = State::kFailed;
        return false;
    }
    // Deliberately NOT calling rmtSetRxMinThreshold(): rmtInit() calloc's a
    // fresh bus object (esp32-hal-rmt.c), which zero-initializes
    // signal_range_min_ns -- the noise filter is already disabled (0 ==
    // disabled per rmtSetRxMinThreshold's own doc comment) by that default,
    // verified by reading esp32-hal-rmt.c directly rather than assumed.
    // Real consumer-IR bit periods (as short as ~560us for NEC) are so much
    // longer than any real electrical glitch that no filter is needed here.

    // CRITICAL, hardware-specific (see this file's header and hal/ir_unit.h):
    // the IRM-3638T already demodulates the 38kHz carrier in its own
    // hardware -- the signal arriving at TAB5_IR_RX_GPIO is already a clean
    // mark/space digital signal. Enabling RMT's OWN RX carrier
    // demodulation on top of that would corrupt every capture. carrier_en
    // = false here disables it explicitly. Verified against
    // esp32-hal-rmt.c's real rmtSetCarrier(): carrier_en=false always
    // forces carrier_cfg.frequency_hz to 0 regardless of carrier_level
    // (rmtSetCarrier(pin, carrier_en, carrier_level, ...) ->
    // `carrier_cfg.frequency_hz = carrier_en ? frequency_Hz : 0`), so
    // carrier_level's value is irrelevant when carrier_en is false -- passed
    // as false here only for readability, not because it does anything.
    // rmtSetCarrier() itself doesn't direction-check (unlike
    // rmtSetRxMaxThreshold/rmtSetRxMinThreshold, which do), so calling it on
    // an RX-mode bus is accepted. Also verified: a freshly-created RMT RX
    // channel is never given a carrier config by rmtInit() itself (no
    // rmt_apply_carrier() call anywhere in its RX branch) -- the pre-driver
    // default is already "no demodulation" -- this call is made anyway so
    // that fact is explicit in this file rather than an unstated assumption
    // about a framework default that could change later.
    if (!rmtSetCarrier(TAB5_IR_RX_GPIO, false, false, 0, 0)) {
        Serial.println("quarky-tab5: [ir-learn] rmtSetCarrier() FAILED");
        teardown_rx();
        std::snprintf(s_error, sizeof(s_error), "rmtSetCarrier() failed");
        s_state = State::kFailed;
        return false;
    }

    s_rx_symbol_count = kMaxRxSymbols; // capacity in; rmtReadAsync() updates
                                       // this to the actual count once the
                                       // read completes (checked in poll()
                                       // via rmtReceiveCompleted())
    if (!rmtReadAsync(TAB5_IR_RX_GPIO, s_rx_symbols, &s_rx_symbol_count)) {
        Serial.println("quarky-tab5: [ir-learn] rmtReadAsync() FAILED");
        teardown_rx();
        std::snprintf(s_error, sizeof(s_error), "rmtReadAsync() failed");
        s_state = State::kFailed;
        return false;
    }

    Serial.printf("quarky-tab5: [ir-learn] Armed -- waiting for remote on GPIO%d\n",
                  TAB5_IR_RX_GPIO);
    s_state = State::kWaiting;
    return true;
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("IR Learn", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *warn = lv_label_create(content);
    lv_label_set_text(warn, "Raw capture only (no NEC/RC5/Sony decode). Point a real "
                            "remote at the IR unit's receiver and press a button.");
    lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);

    s_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);

    s_toggle_btn = lv_button_create(content);
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_obj_add_event_cb(s_toggle_btn, [](lv_event_t *) {
        if (s_state == State::kWaiting) {
            cancel_capture();
        } else {
            if (s_save_status_label) lv_label_set_text(s_save_status_label, "");
            start_capture();
        }
        update_ui();
    }, LV_EVENT_CLICKED, nullptr);

    s_save_btn = lv_button_create(content);
    lv_obj_t *save_lbl = lv_label_create(s_save_btn);
    lv_label_set_text(save_lbl, "Save to SD (.ir)");
    lv_obj_add_event_cb(s_save_btn, [](lv_event_t *) {
        if (s_state != State::kDone) return;
        IrFileFormat::IrSignal sig{};
        char name[IrFileFormat::kNameMaxLen];
        std::snprintf(name, sizeof(name), "Learned_%lu", (unsigned long)millis());
        if (!to_ir_signal(s_result, name, &sig)) {
            if (s_save_status_label) lv_label_set_text(s_save_status_label, "Nothing to save.");
            return;
        }
        char path[80];
        std::snprintf(path, sizeof(path), "%s/learn_%lu.ir", kSaveDir, (unsigned long)millis());
        bool ok = IrFileFormat::write(storage, path, &sig, 1);
        char buf[112];
        if (ok) {
            std::snprintf(buf, sizeof(buf), "Saved to %s", path);
            Serial.printf("quarky-tab5: [ir-learn] saved: %s (%u pulses)\n", path,
                          (unsigned)s_result.count);
        } else {
            std::snprintf(buf, sizeof(buf), "Save failed (%s)", path);
            Serial.printf("quarky-tab5: [ir-learn] failed to save: %s\n", path);
        }
        if (s_save_status_label) lv_label_set_text(s_save_status_label, buf);
    }, LV_EVENT_CLICKED, nullptr);

    s_save_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_save_status_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_save_status_label, "");

    update_ui();

    // Teardown on delete (Back button) -- same convention/reasoning as
    // rf433_scan.cpp/ir_tvbgone.cpp's own build_screen(): a capture left
    // armed must not keep holding the GPIO53 arbiter claim (and the RMT RX
    // channel) after the screen closes, or NFC/RFID2 and RF433 would stay
    // locked out of PORT.A for no reason the user can see.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        s_status_label = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        s_save_btn = nullptr;
        s_save_status_label = nullptr;
        if (s_state == State::kWaiting) {
            Serial.println("quarky-tab5: [ir-learn] Screen closed mid-capture -- cancelling");
            cancel_capture();
        }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() { ScreenStack::push(build_screen()); }

} // namespace

void register_module() {
    g_registry.register_module({"ir_learn", "IR Learn", Category::IR,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void poll() {
    if (s_state != State::kWaiting) return;
    if (!rmtReceiveCompleted(TAB5_IR_RX_GPIO)) return;

    // Capture finished (idle threshold hit, or the buffer filled) -- tear
    // down the RMT channel/arbiter claim now, before touching UI, so PORT.A
    // is freed for NFC/RFID2/RF433 as soon as possible rather than staying
    // held through the (fast, but non-zero) conversion below.
    size_t n_symbols = s_rx_symbol_count;
    teardown_rx();

    bool ok = convert_symbols_to_code(s_rx_symbols, n_symbols, &s_result);
    if (ok) {
        Serial.printf("quarky-tab5: [ir-learn] Captured %u pulses%s\n",
                      (unsigned)s_result.count, s_result.truncated ? " (TRUNCATED)" : "");
        s_state = State::kDone;
    } else {
        Serial.println("quarky-tab5: [ir-learn] Capture completed with no usable data -- "
                        "idle threshold fired before any real mark was seen");
        std::snprintf(s_error, sizeof(s_error), "No signal detected (timed out)");
        s_state = State::kFailed;
    }
    update_ui();
}

State state() { return s_state; }

const LearnedCode &result() { return s_result; }

const char *error_reason() { return s_error; }

bool to_ir_signal(const LearnedCode &code, const char *name, IrFileFormat::IrSignal *out) {
    if (code.count == 0) return false;
    std::memset(out, 0, sizeof(*out));
    out->type = IrFileFormat::SignalType::kRaw;
    std::strncpy(out->name, name, IrFileFormat::kNameMaxLen - 1);
    out->name[IrFileFormat::kNameMaxLen - 1] = '\0';
    out->frequency_hz = code.carrier_hz;
    out->duty_cycle = code.duty_cycle;

    size_t n = code.count;
    bool truncated = code.truncated;
    if (n > IrFileFormat::kMaxRawSamples) {
        // Defensive only -- kMaxPulses == IrFileFormat::kMaxRawSamples
        // exactly (see ir_learn.h), so this branch cannot currently trigger,
        // but a future change to either constant shouldn't silently
        // overflow out->data[].
        n = IrFileFormat::kMaxRawSamples;
        truncated = true;
    }
    std::memcpy(out->data, code.pulse_widths_us, n * sizeof(uint16_t));
    out->data_count = n;
    out->truncated = truncated;
    return true;
}

} // namespace IrLearn
