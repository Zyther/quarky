#include "wifi_spectrum.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>

extern FeatureRegistry g_registry;

namespace WifiSpectrumFeature {

namespace {
// Passive per-channel dwell for the async scan kicked off each hop. Short
// enough to keep all 14 channels refreshing at a "live" cadence (14 x
// (kDwellMs + scan/SDIO overhead) is a couple of seconds per full sweep,
// comparable to wifi_scan.cpp's own scan times), long enough to actually
// catch a beacon on a mostly-idle channel (APs beacon roughly every 100ms).
constexpr uint32_t kDwellMs = 150;
// Safety backstop, not a real-world expectation: if a single channel's scan
// never completes (radio wedged, SDIO hiccup), give up on it after this many
// ms rather than stall the whole sweep on one channel forever -- the same
// give-up-and-move-on shape wifi_common.cpp's wifi_scan_poll() uses for its
// own (longer, whole-sweep) scan timeout.
constexpr uint32_t kPerChannelTimeoutMs = 1000;
} // namespace

static lv_obj_t *s_chart = nullptr;
static lv_chart_series_t *s_series = nullptr;
static bool s_active = false;
static bool s_scanning = false;
static uint32_t s_scan_started_ms = 0;
static uint8_t s_channel = 1;

static lv_obj_t *build_screen() {
    // Menu-bar Back button + flex content area -- see ui/screen_scaffold.cpp
    // for why every sub-screen must build through this rather than
    // hand-positioning its own Back button.
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WiFi Spectrum", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_chart = lv_chart_create(content);
    lv_obj_set_size(s_chart, LV_PCT(95), LV_PCT(80));
    lv_chart_set_type(s_chart, LV_CHART_TYPE_BAR);
    // LVGL 9 renamed the v8-era lv_chart_set_range() to
    // lv_chart_set_axis_range() (confirmed against the actual
    // .pio/libdeps/tab5/lvgl/src/widgets/chart/lv_chart.h this project pulls
    // in via lvgl/lvgl@^9.2.0 in platformio.ini) -- lv_chart_set_range() does
    // not exist in this tree and would fail to compile.
    lv_chart_set_axis_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, -100, 0); // dBm
    lv_chart_set_point_count(s_chart, 14); // channels 1-14
    s_series = lv_chart_add_series(s_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    for (int i = 0; i < 14; i++) lv_chart_set_next_value(s_chart, s_series, -100);

    // Real-hardware finding (2026-08-12, controller verification pass): the
    // chart alone gives no indication of which bar is which channel. LVGL's
    // built-in axis ticks render numeric divisions of the axis range, not
    // arbitrary per-bar text, so a plain row of labels underneath -- one per
    // channel, evenly spaced the same way the chart's own bars are -- is
    // simpler and more legible here than fighting the tick-label API for a
    // 14-category axis.
    lv_obj_t *labels_row = lv_obj_create(content);
    lv_obj_remove_style_all(labels_row);
    lv_obj_set_size(labels_row, LV_PCT(95), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(labels_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(labels_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    for (int ch = 1; ch <= 14; ch++) {
        lv_obj_t *label = lv_label_create(labels_row);
        lv_label_set_text_fmt(label, "%d", ch);
    }

    // The chart (and s_series/s_active/s_scanning) must not outlive the
    // screen -- the scaffold's Back button pops the screen and deletes it
    // (and everything parented under it, including this chart) via
    // ScreenStack::pop(), so clear these from the chart's own
    // LV_EVENT_DELETE rather than a click handler, the same pattern Task 3's
    // wifi_scan.cpp established for its list (closes the same "stale
    // pointer after Back" class of bug).
    lv_obj_add_event_cb(s_chart, [](lv_event_t *e) {
        s_active = false;
        s_scanning = false;
        s_chart = nullptr;
        s_series = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    s_channel = 1;
    s_scanning = false;
    s_active = true;
    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_spectrum", "WiFi Spectrum", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    // Same AP+STA coexistence handling wifi_common.cpp's wifi_scan_begin()
    // uses: c2link_wifi's SoftAP (the WiFi C2 transport) must stay up, so
    // flip to AP_STA rather than a bare STA mode that would silently drop it.
    if (WiFi.getMode() == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);
    } else if (WiFi.getMode() == WIFI_OFF) {
        WiFi.mode(WIFI_STA);
    }
    ScreenStack::push(build_screen());
}

// -----------------------------------------------------------------------------
// Resolution of this task brief's flagged open question: esp_wifi_sta_get_
// ap_info() was NOT used, and the code never tries it first. Two independent,
// checked-not-guessed reasons:
//
//   1. It is the wrong shape of data even when it succeeds. It reports the
//      RSSI of whichever single AP the STA is CURRENTLY ASSOCIATED to, at
//      that AP's own fixed channel -- one number, one channel, only while
//      connected. A spectrum analyzer needs an independent ambient reading
//      per channel as it hops across all 14, which that call cannot produce
//      regardless of association state: the brief's own original poll()
//      sketch would have sampled the same one associated AP's RSSI into
//      whichever channel bin the hop counter happened to be on, which is a
//      logic bug independent of whether the call itself errors.
//   2. It would also just error in this device's actual boot configuration.
//      main.cpp's test_credentials_configured() guard skips the STA connect
//      attempt on every boot because kTestSsid/kTestPassword are still the
//      documented placeholder values, so the STA is never associated to
//      begin with. esp_wifi_sta_get_ap_info() is documented (ESP-IDF) to
//      return ESP_ERR_WIFI_NOT_CONNECT in exactly that state -- the outcome
//      the brief itself flagged as likely.
//
// Went straight to the per-channel-scan fallback the brief names as the
// alternative: an async, passive WiFi.scanNetworks() restricted to one
// channel at a time via WiFiScan's own channel parameter, so each hop only
// dwells on (and only reports) the channel it is currently on. This reuses
// wifi_common.cpp's wifi_scan_begin()/wifi_scan_poll() async-scan-via-poll
// shape (kick off, poll scanComplete() from here rather than block, delete
// results when done) one channel at a time instead of all 2.4GHz channels in
// one sweep. Same source as any phone WiFi-analyzer app: strongest
// beacon/probe-response heard per channel, not a true RF power meter (that
// needs different silicon), which matches this task's own framing ("the kind
// of tool built into any WiFi analyzer app").
void poll() {
    if (!s_active || !s_chart) return;
    uint32_t now = millis();

    if (!s_scanning) {
        int16_t rc = WiFi.scanNetworks(true, false, true, kDwellMs, s_channel);
        if (rc == WIFI_SCAN_FAILED) {
            return; // radio busy this tick; retry the same channel next poll()
        }
        s_scanning = true;
        s_scan_started_ms = now;
        return;
    }

    int16_t n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
        if (now - s_scan_started_ms <= kPerChannelTimeoutMs) return; // still in flight
        n = 0; // timed out; treat as "nothing heard" and move on rather than stall
    } else if (n < 0) {
        n = 0; // WIFI_SCAN_FAILED
    }

    int8_t best_rssi = -100;
    for (int16_t i = 0; i < n; i++) {
        int32_t rssi = WiFi.RSSI(i);
        if (rssi > best_rssi) best_rssi = (int8_t)rssi;
    }
    WiFi.scanDelete();

    lv_chart_set_series_value_by_id(s_chart, s_series, s_channel - 1, best_rssi);

    s_scanning = false;
    s_channel = (s_channel % 14) + 1;
}

} // namespace WifiSpectrumFeature
