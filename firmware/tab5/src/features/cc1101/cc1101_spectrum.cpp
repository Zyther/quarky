#include "cc1101_spectrum.h"
#include "cc1101_hw.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../ui/theme.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>

extern FeatureRegistry g_registry;

namespace Cc1101Spectrum {
namespace {

// Module's documented FULL range, per this phase's spec Section 5 --
// deliberately NOT the project owner's narrower 868-925MHz intended tuning
// (the spec explicitly instructs not to hard-code that narrower band as a
// limit).
constexpr float kFreqStartMhz = 855.0f;
constexpr float kFreqEndMhz = 925.0f;
constexpr int kNumBins = 71; // 1MHz steps across the 70MHz span -- a real,
                              // legible bar count for a 1280px-wide chart
                              // (matches wifi_spectrum.cpp's own
                              // one-bin-per-real-unit approach, 14 WiFi
                              // channels there vs. 71 1MHz bins here).
constexpr uint32_t kMinStepIntervalMs = 30; // real per-step cadence Poseidon's
                                             // own autoscan uses a delay(15)
                                             // settle between retune+RSSI-read;
                                             // 30ms here is poll()-tick-paced
                                             // rather than blocking, so it is a
                                             // MINIMUM interval, not a delay()

lv_obj_t *s_chart = nullptr;
lv_chart_series_t *s_series = nullptr;
bool s_active = false;
int s_bin = 0;
uint32_t s_last_step_ms = 0;

static void spectrum_on_theme() {
    if (s_chart && s_series) {
        lv_chart_set_series_color(s_chart, s_series, Theme::color(Token::Chart));
    }
}

} // namespace

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("CC1101 Spectrum", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    // %d, not %f: this project's lv_conf.h has LV_USE_FLOAT=0, so LVGL's own
    // lightweight lv_snprintf() does not implement float conversion at all --
    // a %f here silently prints the literal letter "f" instead of a number
    // (real finding, project owner caught it on real hardware 2026-08-25).
    // Every frequency value on this screen is a whole MHz, so integer
    // formatting loses nothing.
    lv_obj_t *hdr = lv_label_create(content);
    lv_label_set_text_fmt(hdr, "%d - %d MHz (module's full documented range)",
                           (int)kFreqStartMhz, (int)kFreqEndMhz);

    s_chart = lv_chart_create(content);
    lv_obj_set_size(s_chart, LV_PCT(98), LV_PCT(85));
    lv_chart_set_type(s_chart, LV_CHART_TYPE_BAR);
    lv_chart_set_axis_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, -100, 0); // dBm
    lv_chart_set_point_count(s_chart, kNumBins);
    lv_chart_set_div_line_count(s_chart, 5, 0);
    s_series = lv_chart_add_series(s_chart, Theme::color(Token::Chart), LV_CHART_AXIS_PRIMARY_Y);
    Theme::add_listener(spectrum_on_theme);
    spectrum_on_theme();
    for (int i = 0; i < kNumBins; i++) lv_chart_set_next_value(s_chart, s_series, -100);

    // Start/mid/end frequency labels under the chart, project owner's own
    // request (2026-08-25) -- the header text above already states the
    // full range as one sentence, but a reader scanning the chart itself
    // has no way to tell which bar is which frequency without these.
    // Plain 3-way flex row (space-between), not pixel-precise x-axis tick
    // labels aligned to individual bars -- this chart has no built-in LVGL
    // tick-label feature for LV_CHART_TYPE_BAR, and 71 real per-bin labels
    // would be illegible at this width regardless; three anchor points
    // (start/mid/end) is what was actually asked for.
    lv_obj_t *freq_row = lv_obj_create(content);
    lv_obj_set_width(freq_row, LV_PCT(98));
    lv_obj_set_height(freq_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(freq_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(freq_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(freq_row, 0, 0);
    lv_obj_set_style_border_width(freq_row, 0, 0);
    lv_obj_set_style_bg_opa(freq_row, LV_OPA_TRANSP, 0);

    lv_obj_t *lbl_start = lv_label_create(freq_row);
    lv_label_set_text_fmt(lbl_start, "%d", (int)kFreqStartMhz);
    lv_obj_t *lbl_mid = lv_label_create(freq_row);
    lv_label_set_text_fmt(lbl_mid, "%d", (int)((kFreqStartMhz + kFreqEndMhz) / 2.0f));
    lv_obj_t *lbl_end = lv_label_create(freq_row);
    lv_label_set_text_fmt(lbl_end, "%d", (int)kFreqEndMhz);

    lv_obj_add_event_cb(s_chart, [](lv_event_t *) {
        Theme::remove_listener(spectrum_on_theme);
        s_active = false;
        s_chart = nullptr;
        s_series = nullptr;
        Cc1101Hw::idle();
    }, LV_EVENT_DELETE, nullptr);

    s_bin = 0;
    s_last_step_ms = 0;
    s_active = true;
    return screen;
}

void start() {
    if (!Cc1101Hw::is_present()) Cc1101Hw::init();
    ScreenStack::push(build_screen());
}

void register_module() {
    g_registry.register_module({"cc1101_spectrum", "CC1101 Spectrum", Category::SUBGHZ,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void poll() {
    if (!s_active || !s_chart) return;
    uint32_t now = millis();
    if (now - s_last_step_ms < kMinStepIntervalMs) return;
    s_last_step_ms = now;

    float freq = kFreqStartMhz + (kFreqEndMhz - kFreqStartMhz) * s_bin / (float)(kNumBins - 1);
    Cc1101Hw::set_frequency_mhz(freq);
    // setFrequency() (per RadioLib's own implementation, confirmed by
    // reading CC1101.cpp directly) IDLEs the chip before writing the new
    // FREQ registers and does NOT resume RX on its own -- real per-step
    // retune+resume-RX sequence, same shape as Poseidon's own autoscan
    // handler (setSidle()/setMHZ()/SetRx() per step, subghz_scan.cpp 'A').
    Cc1101Hw::enable_async_rx();
    float rssi = Cc1101Hw::rssi_dbm();

    lv_chart_set_series_value_by_id(s_chart, s_series, s_bin, (int32_t)rssi);
    s_bin = (s_bin + 1) % kNumBins;
}

} // namespace Cc1101Spectrum
