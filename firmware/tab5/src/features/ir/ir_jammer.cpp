#include "ir_jammer.h"
#include "ir_common.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <esp_random.h>

extern FeatureRegistry g_registry;

namespace IrJammer {
namespace {

// Real citations: see ir_jammer.h's own header comment.
constexpr uint32_t kCarrierHz = 38000;
constexpr float kDutyCycle = 1.0f / 3.0f;

// One burst = kBurstPairs mark/space pairs, each duration randomized in
// [kMinSegmentUs, kMaxSegmentUs). Real bound reasoning: real consumer-IR
// protocols' own individual mark/space segments (see
// features/ir/ir_nec_encode.h's cited NEC timings: 560us bit mark,
// 560/1690us bit space; world_ir_codes.h's real TV-B-Gone data spans a
// similar range) mostly fall within a few hundred microseconds to
// low-single-digit milliseconds -- staying in that same real range (not
// e.g. far shorter or far longer) is what keeps this jam pattern inside
// the carrier-activity envelope a real receiver's AGC actually watches,
// rather than something a receiver could trivially filter as out-of-band
// noise. 20 pairs/burst keeps one IrCommon::transmit_raw() call short
// (well under the ~50ms single bursts this project's other IR features
// already transmit without incident -- see ir_common.h's own
// kMaxDurationsPerTransmit comment).
constexpr int kBurstPairs = 20;
constexpr size_t kBurstDurations = kBurstPairs * 2;
constexpr uint16_t kMinSegmentUs = 150;
constexpr uint16_t kMaxSegmentUs = 900;

bool s_active = false;
uint16_t s_durations[kBurstDurations];

lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_toggle_btn = nullptr;
lv_obj_t *s_toggle_label = nullptr;

void update_ui() {
    if (s_toggle_label) {
        lv_label_set_text(s_toggle_label, s_active ? "Stop" : "Start Jamming");
    }
    if (s_status_label) {
        lv_label_set_text(s_status_label, s_active
                                               ? "Jamming -- transmitting continuous IR noise."
                                               : "Idle. Point the IR unit's TX LED at the target "
                                                 "receiver, then tap Start.");
    }
}

// esp_random() % (kMaxSegmentUs - kMinSegmentUs) + kMinSegmentUs, matching
// this project's own established esp_random()-for-jitter/randomization
// idiom (ble_karma.cpp/ble_sourapple.cpp -- real hardware RNG, not
// Arduino's weaker random()).
uint16_t random_segment_us() {
    return kMinSegmentUs + static_cast<uint16_t>(esp_random() % (kMaxSegmentUs - kMinSegmentUs));
}

void start() {
    if (!IrCommon::init()) {
        if (s_status_label) {
            lv_label_set_text(s_status_label, "Failed to start -- PORT.A is held by "
                                              "another owner (NFC/RFID2 or RF433)");
        }
        return;
    }
    s_active = true;
    Serial.println("quarky-tab5: [ir-jammer] Start tapped");
    update_ui();
}

void stop() {
    s_active = false;
    IrCommon::deinit();
    Serial.println("quarky-tab5: [ir-jammer] Stop tapped");
    update_ui();
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("IR Jammer", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *warn = lv_label_create(content);
    lv_label_set_text(warn, "Transmits continuous IR noise to disrupt a nearby real IR "
                             "receiver. No named protocol -- see this feature's own header "
                             "comment for why random short bursts are enough.");
    lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);

    s_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);

    s_toggle_btn = lv_button_create(content);
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_obj_add_event_cb(s_toggle_btn, [](lv_event_t *) {
        if (s_active) stop(); else start();
    }, LV_EVENT_CLICKED, nullptr);

    update_ui();

    // Teardown on delete (Back button) -- same convention and real
    // reasoning as ir_tvbgone.cpp's build_screen(): a jam session left
    // running after the screen closes would keep transmitting IR noise
    // (and keep holding the GPIO53/PORT.A arbiter claim) with no visible
    // indication or way to stop it.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        bool was_active = s_active;
        s_status_label = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        if (was_active) {
            Serial.println("quarky-tab5: [ir-jammer] Screen closed mid-jam -- stopping");
            s_active = false;
            IrCommon::deinit();
        }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start_screen() { ScreenStack::push(build_screen()); }

} // namespace

void register_module() {
    g_registry.register_module({"ir_jammer", "IR Jammer", Category::IR,
                                 Affinity::TAB5_NATIVE, start_screen, nullptr});
}

void poll() {
    if (!s_active) return;

    // One bounded burst per tick -- see ir_jammer.h's own header comment
    // for why this must never become a loop over many bursts in one call.
    for (size_t i = 0; i < kBurstDurations; i++) {
        s_durations[i] = random_segment_us();
    }
    if (!IrCommon::transmit_raw(s_durations, kBurstDurations, kCarrierHz, kDutyCycle)) {
        Serial.println("quarky-tab5: [ir-jammer] transmit_raw() failed -- stopping");
        stop();
    }
}

} // namespace IrJammer
