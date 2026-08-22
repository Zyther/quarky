#include "ir_common.h"
#include "../../../boards/tab5/pins_config.h"
#include "../../hal/gpio53_arbiter.h"
#include <Arduino.h>
#include <esp32-hal-rmt.h>

// See ir_common.h's header comment for why this uses the RMT peripheral
// rather than bit-banging or an IRremoteESP8266-style library.

namespace IrCommon {
namespace {

bool s_began = false;

// 1 tick = 1 microsecond -- matches every duration unit this project's IR
// data already uses (WORLD_IR_CODES.h's times[], IrFileFormat::IrSignal's
// data[]), so no scaling is needed converting durations_us into RMT ticks.
constexpr uint32_t kRmtTickHz = 1000000;

// rmt_data_t's duration fields are 15 bits (max 32767 ticks = 32.767ms at
// this tick rate) -- generously above any real single mark/space segment
// in consumer IR protocols (the longest are AGC header pulses/gaps, a few
// ms at most). Clamp defensively rather than let a pathological value
// silently wrap in the bitfield.
constexpr uint16_t kMaxSegmentTicks = 32767;

// Sized for the largest call this project ever makes (kMaxDurationsPerTransmit
// durations -> half as many two-duration RMT symbols, rounded up). A
// function-local static, not a stack array: this project's own established
// practice this session (see rf433_scan.h's kMaxEdgesPerSignal comment) is
// to keep any buffer that scales with a real-hardware-derived cap off the
// stack, since a caller reached through an LVGL click-handler callback has
// already been shown to have a small stack margin.
static rmt_data_t s_symbols[(kMaxDurationsPerTransmit + 1) / 2];

} // namespace

bool init() {
    if (!Gpio53Arbiter::claim(Gpio53Arbiter::Owner::kIr)) {
        Serial.println("quarky-tab5: [ir-common] init() REFUSED -- PORT.A is "
                       "held by another owner (NFC/RFID2 or RF433)");
        return false;
    }
    if (!rmtInit(TAB5_IR_TX_GPIO, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, kRmtTickHz)) {
        Serial.println("quarky-tab5: [ir-common] rmtInit() FAILED");
        Gpio53Arbiter::release(Gpio53Arbiter::Owner::kIr);
        return false;
    }
    // Idle LOW (LED off) once a transmission ends -- matches ir_unit.h's
    // own idle-LOW convention for TAB5_IR_TX_GPIO.
    rmtSetEOT(TAB5_IR_TX_GPIO, 0);
    s_began = true;
    Serial.printf("quarky-tab5: [ir-common] init() OK -- TX=GPIO%d via RMT\n",
                  TAB5_IR_TX_GPIO);
    return true;
}

void deinit() {
    if (!s_began) return;
    s_began = false;
    rmtDeinit(TAB5_IR_TX_GPIO);
    Gpio53Arbiter::release(Gpio53Arbiter::Owner::kIr);
}

bool transmit_raw(const uint16_t *durations_us, size_t count, uint32_t carrier_hz,
                  float duty_cycle) {
    if (!s_began) return false;
    if (count < 1 || count > kMaxDurationsPerTransmit) return false;

    // REAL BUG FOUND AND FIXED via a real-hardware failure (2026-08-21): a
    // full TV-B-Gone sweep produced no response from a real LG TV at any
    // range/aim. esp32-hal-rmt.h's own doc comment for rmtSetCarrier()'s
    // `carrier_level` param ("true means positive polarity") is misleading
    // -- the actual implementation (esp32-hal-rmt.c's rmtSetCarrier())
    // passes this value straight through to ESP-IDF's real
    // rmt_carrier_config_t::flags.polarity_active_low, whose OWN doc
    // comment (rmt_types.h) states: "Specify the polarity of carrier, by
    // default it's modulated to base signal's high level" -- i.e. the
    // real default (false/0) is what modulates the carrier onto level=1
    // ("mark") segments, matching this module's own mark-first convention.
    // Passing `true` here (as this code originally did) sets
    // polarity_active_low=true, INVERTING that: the carrier ends up gated
    // onto the "space" segments instead of "mark", producing a completely
    // wrong signal (steady/unmodulated LED during real marks, 38kHz
    // blinking during real spaces) regardless of code-database accuracy,
    // range, or aim -- which is exactly the silent total failure that was
    // observed. Fixed by passing `false` (the real documented default:
    // carrier on the high/mark level).
    if (!rmtSetCarrier(TAB5_IR_TX_GPIO, true, false, carrier_hz, duty_cycle)) {
        Serial.println("quarky-tab5: [ir-common] rmtSetCarrier() FAILED");
        return false;
    }

    size_t num_symbols = (count + 1) / 2;
    for (size_t i = 0; i < num_symbols; ++i) {
        size_t d0_idx = i * 2;
        size_t d1_idx = d0_idx + 1;
        uint16_t d0 = durations_us[d0_idx];
        s_symbols[i].duration0 = d0 > kMaxSegmentTicks ? kMaxSegmentTicks : d0;
        s_symbols[i].level0 = 1; // mark
        if (d1_idx < count) {
            uint16_t d1 = durations_us[d1_idx];
            s_symbols[i].duration1 = d1 > kMaxSegmentTicks ? kMaxSegmentTicks : d1;
            s_symbols[i].level1 = 0; // space
        } else {
            // Odd count: no trailing space value. duration=0 is RMT's own
            // real end-of-transmission marker (the historical rmt_item32_t
            // convention this struct preserves) -- correctly terminates
            // right after the final mark, rather than transmitting a
            // fabricated space duration that was never in the real data.
            s_symbols[i].duration1 = 0;
            s_symbols[i].level1 = 0;
        }
    }

    return rmtWrite(TAB5_IR_TX_GPIO, s_symbols, num_symbols, 1000);
}

} // namespace IrCommon
