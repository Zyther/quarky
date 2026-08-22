#include "ir_tvbgone.h"
#include "ir_common.h"
#include "world_ir_codes.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <cstdint>
#include <cstdio>

extern FeatureRegistry g_registry;

namespace IrTvbGone {
namespace {

// ── Real decode algorithm ───────────────────────────────────────────────
// Cross-validated against Bruce's TV-B-Gone.cpp (read_bits()/code_ptr/
// bitsleft_r/bits_r) and UniGeek's IRUtil.cpp (identical unpacking) -- see
// world_ir_codes.h's provenance header. Reimplemented here rather than
// copy-pasted verbatim: the donors' versions are file-scope globals
// (code_ptr, bitsleft_r, bits_r) shared across one in-progress decode,
// which this port keeps as plain locals in a small BitReader instead, with
// no behavioral difference.
struct BitReader {
    const IrCode *code;
    uint8_t code_ptr = 0;
    uint8_t bitsleft = 0;
    uint8_t bits = 0;
};

uint8_t read_bits(BitReader &r, uint8_t count) {
    uint8_t tmp = 0;
    while (count--) {
        if (r.bitsleft == 0) {
            r.bits = r.code->codes[r.code_ptr++];
            r.bitsleft = 8;
        }
        tmp = (uint8_t)((tmp << 1) | ((r.bits >> --r.bitsleft) & 1));
    }
    return tmp;
}

// Largest real code in world_ir_codes.h has numpairs (a uint8_t) far below
// 255, but 255 is the real type-derived ceiling -- 2 durations/pair = 510,
// safely under IrCommon::kMaxDurationsPerTransmit (1024).
constexpr size_t kMaxDurationsPerCode = 255 * 2;

// Decodes `code`'s compressed pairs into `out_durations` (real microsecond
// mark/space values, mark first -- both donors' identical
// `rawData[k*2]=times[ti]*10, rawData[k*2+1]=times[ti+1]*10` unpacking,
// with the *10 here because world_ir_codes.h's times[] stores TENS of
// microseconds, same as both donors' own comment), and transmits via
// IrCommon::transmit_raw().
//
// carrier_hz: code->timer_val is already a real kHz value (see
// world_ir_codes.h's own `freq_to_timerval(x) = x/1000` macro), so *1000
// recovers Hz.
//
// duty_cycle: NOT part of this dataset (TV-B-Gone's real format only ever
// stored a frequency, never an explicit duty cycle) -- 1/3 is the
// well-established real-world default for this class of consumer IR
// protocol, the same value IrFileFormat's own real .ir-format citation
// names as the typical stored figure (0.330000) for raw signals of this
// kind. Not a guess pulled from nowhere, but also not read from
// world_ir_codes.h itself -- worth remembering if a real TV genuinely
// needs its own exact duty cycle to respond (unlikely: IR receivers are
// broadly duty-cycle-tolerant, which is exactly why the original TV-B-Gone
// format never bothered to store one).
bool decode_and_transmit(const IrCode *code) {
    BitReader r{code};
    static uint16_t s_durations[kMaxDurationsPerCode];
    uint8_t numpairs = code->numpairs;
    for (uint8_t k = 0; k < numpairs; k++) {
        uint16_t ti = (uint16_t)(read_bits(r, code->bitcompression) * 2);
        s_durations[k * 2] = (uint16_t)(code->times[ti] * 10);
        s_durations[k * 2 + 1] = (uint16_t)(code->times[ti + 1] * 10);
    }
    uint32_t carrier_hz = (uint32_t)code->timer_val * 1000;
    return IrCommon::transmit_raw(s_durations, (size_t)numpairs * 2, carrier_hz, 1.0f / 3.0f);
}

// ── Run state ────────────────────────────────────────────────────────────

// Real inter-code gap: both Bruce's TV-B-Gone.cpp and UniGeek's IRUtil.cpp
// use delay_ten_us(20500) == 205ms between codes. Cited value, not
// invented -- gives a receiving TV time to react/settle before the next
// code, same reasoning the original hardware keychain used.
constexpr uint32_t kInterCodeGapMs = 205;

enum class Region { kNA, kEU };

bool s_active = false;
Region s_region = Region::kNA;
uint32_t s_code_idx = 0;
uint32_t s_total_codes = 0;
uint32_t s_last_send_ms = 0;

enum class EndReason { kCompleted, kUserStopped, kInitFailed, kTransmitFailed };

const IrCode *const *region_table(Region region, uint32_t *out_count) {
    if (region == Region::kNA) {
        *out_count = (uint32_t)(sizeof(NApowerCodes) / sizeof(NApowerCodes[0]));
        return NApowerCodes;
    }
    *out_count = (uint32_t)(sizeof(EUpowerCodes) / sizeof(EUpowerCodes[0]));
    return EUpowerCodes;
}

// UI widgets. Null when the screen isn't open (same convention as
// rf433_bruteforce.cpp's s_status_label et al.).
lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_progress_bar = nullptr;
lv_obj_t *s_region_buttons[2] = {nullptr, nullptr};
lv_obj_t *s_toggle_btn = nullptr;
lv_obj_t *s_toggle_label = nullptr;

void update_ui() {
    for (lv_obj_t *btn : s_region_buttons) {
        if (!btn) continue;
        if (s_active) {
            lv_obj_add_state(btn, LV_STATE_DISABLED);
        } else {
            lv_obj_remove_state(btn, LV_STATE_DISABLED);
        }
    }
    if (s_toggle_label) {
        lv_label_set_text(s_toggle_label, s_active ? "Stop" : "Start");
    }
    if (!s_status_label) return;

    const char *region_name = s_region == Region::kNA ? "NA" : "EU";
    char buf[96];
    if (s_active) {
        std::snprintf(buf, sizeof(buf), "Sending %s code %lu / %lu", region_name,
                      (unsigned long)(s_code_idx + 1), (unsigned long)s_total_codes);
    } else if (s_code_idx == 0) {
        std::snprintf(buf, sizeof(buf), "Idle -- %s region selected", region_name);
    } else {
        std::snprintf(buf, sizeof(buf), "Idle -- last run: %lu / %lu (%s)",
                      (unsigned long)s_code_idx, (unsigned long)s_total_codes, region_name);
    }
    lv_label_set_text(s_status_label, buf);

    if (s_progress_bar && s_total_codes > 0) {
        int32_t pct = (int32_t)((uint64_t)s_code_idx * 100 / s_total_codes);
        lv_bar_set_value(s_progress_bar, pct, LV_ANIM_OFF);
    }
}

void finish_run(EndReason reason) {
    s_active = false;
    IrCommon::deinit();
    // Same ordering fix rf433_bruteforce.cpp's review round required:
    // update_ui() first (flips buttons back to enabled/Start), THEN the
    // terminal message, so the terminal message isn't immediately
    // clobbered by update_ui()'s own generic status-label write.
    update_ui();
    if (!s_status_label) return;

    const char *region_name = s_region == Region::kNA ? "NA" : "EU";
    char buf[112];
    switch (reason) {
        case EndReason::kCompleted:
            std::snprintf(buf, sizeof(buf), "Done -- sent all %lu %s codes",
                          (unsigned long)s_total_codes, region_name);
            break;
        case EndReason::kUserStopped:
            std::snprintf(buf, sizeof(buf), "Stopped by user at %lu / %lu (%s)",
                          (unsigned long)s_code_idx, (unsigned long)s_total_codes, region_name);
            break;
        case EndReason::kInitFailed:
            std::snprintf(buf, sizeof(buf), "Failed to start -- PORT.A is held by "
                                            "another owner (NFC/RFID2 or RF433)");
            break;
        case EndReason::kTransmitFailed:
            std::snprintf(buf, sizeof(buf), "Stopped -- transmit failed at code %lu (%s)",
                          (unsigned long)s_code_idx, region_name);
            break;
    }
    lv_label_set_text(s_status_label, buf);
}

void toggle_click_cb(lv_event_t *) {
    if (s_active) {
        Serial.println("quarky-tab5: [ir-tvbgone] Stop tapped by user");
        finish_run(EndReason::kUserStopped);
        return;
    }

    if (!IrCommon::init()) {
        finish_run(EndReason::kInitFailed);
        return;
    }

    s_code_idx = 0;
    region_table(s_region, &s_total_codes);
    s_active = true;
    s_last_send_ms = 0; // fires the first code on the very next poll() tick
    Serial.printf("quarky-tab5: [ir-tvbgone] Start tapped -- region=%s, %lu codes\n",
                  s_region == Region::kNA ? "NA" : "EU", (unsigned long)s_total_codes);
    update_ui();
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("TV-B-Gone", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *warn = lv_label_create(content);
    lv_label_set_text(warn, "Sweeps real TV power-off codes over the IR unit's "
                             "TX LED. Point it at the target device.");
    lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);

    // Region selection: 2 checkable buttons acting as a radio group (manual
    // mutual exclusion), matching nfc_mifare_crack.cpp's established
    // replacement for lv_dropdown -- that widget reproducibly mis-resolved
    // row taps on this real hardware (see that file's own s_mode_buttons
    // comment), so new screens this session use checkable buttons instead.
    static const char *const kRegionLabels[2] = {"NA", "EU"};
    for (int i = 0; i < 2; i++) {
        lv_obj_t *btn = lv_button_create(content);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, kRegionLabels[i]);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            lv_obj_t *tapped = static_cast<lv_obj_t *>(lv_event_get_target(e));
            for (int j = 0; j < 2; j++) {
                if (s_region_buttons[j] == tapped) {
                    s_region = static_cast<Region>(j);
                } else if (s_region_buttons[j] != nullptr) {
                    lv_obj_remove_state(s_region_buttons[j], LV_STATE_CHECKED);
                }
            }
            lv_obj_add_state(tapped, LV_STATE_CHECKED);
            update_ui();
        }, LV_EVENT_CLICKED, nullptr);
        s_region_buttons[i] = btn;
    }
    lv_obj_add_state(s_region_buttons[static_cast<int>(s_region)], LV_STATE_CHECKED);

    s_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);

    s_progress_bar = lv_bar_create(content);
    lv_obj_set_size(s_progress_bar, LV_PCT(100), 16);
    lv_bar_set_range(s_progress_bar, 0, 100);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);

    s_toggle_btn = lv_button_create(content);
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_obj_add_event_cb(s_toggle_btn, toggle_click_cb, LV_EVENT_CLICKED, nullptr);

    update_ui();

    // Teardown on delete (Back button) -- same convention and same real
    // reasoning as rf433_bruteforce.cpp's build_screen(): a sweep in
    // progress is stopped here, not left running silently in the
    // background, matching that task's own review-round fix for exactly
    // this class of feature (cancellable, multi-second-to-multi-minute
    // hardware transmission).
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        bool was_active = s_active;
        s_status_label = nullptr;
        s_progress_bar = nullptr;
        s_region_buttons[0] = nullptr;
        s_region_buttons[1] = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        if (was_active) {
            Serial.println("quarky-tab5: [ir-tvbgone] Screen closed mid-run -- stopping sweep");
            s_active = false;
            IrCommon::deinit();
        }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() { ScreenStack::push(build_screen()); }

} // namespace

void register_module() {
    g_registry.register_module({"ir_tvbgone", "TV-B-Gone", Category::IR,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void poll() {
    if (!s_active) return;
    if (millis() - s_last_send_ms < kInterCodeGapMs) return; // not due yet

    if (s_code_idx >= s_total_codes) {
        Serial.println("quarky-tab5: [ir-tvbgone] database exhausted -- sweep complete");
        finish_run(EndReason::kCompleted);
        return;
    }

    uint32_t total = 0;
    const IrCode *const *table = region_table(s_region, &total);
    bool ok = decode_and_transmit(table[s_code_idx]);
    s_last_send_ms = millis();
    if (!ok) {
        Serial.printf("quarky-tab5: [ir-tvbgone] transmit failed at code %lu -- "
                      "stopping sweep\n", (unsigned long)s_code_idx);
        finish_run(EndReason::kTransmitFailed);
        return;
    }

    s_code_idx++;
    update_ui();
}

} // namespace IrTvbGone
