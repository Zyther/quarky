#pragma once

#include <cstddef>
#include <cstdint>

// ===========================================================================
// IR common transmit adaptation layer (Phase 3 Task 16).
//
// WHY THIS EXISTS: the plan's original Task 16 context assumed donor IR
// code (Bruce's TV-B-Gone.cpp / Poseidon's ir_tvbgone.cpp) would need
// adapting from "a bit-banged GPIO LED" to "whatever Task 15's real chip
// API turned out to be" -- Task 15 found the IR unit has no chip at all
// (see hal/ir_unit.h), just a bare transistor+LED on TAB5_IR_TX_GPIO, so in
// one sense no adaptation is needed: the donor code's own assumption was
// already correct. What DOES need adapting is the CARRIER GENERATION
// method: both Bruce's TV-B-Gone.cpp (via IRremoteESP8266's `IRsend` class)
// and the original AVR TV-B-Gone (via precise NOP-count timing loops tuned
// per CPU clock speed) generate the 38kHz-ish carrier themselves, and this
// project has neither of those libraries nor that AVR-era bit-bang
// approach available/appropriate. Poseidon's own Cardputer-ADV IR port
// (ir_tvbgone.cpp) DOES bit-bang the carrier by hand via digitalWrite() +
// delayMicroseconds() in a tight loop -- workable on a dedicated FreeRTOS
// task at low priority, but this project already hit a real timing
// hazard from ANY multi-millisecond blocking operation sharing loop()'s
// task (see hal/ir_unit.h's bring-up-spike watchdog-reset finding), and a
// software carrier loop is exactly the kind of jittery timing a real
// receiving TV's IR demodulator is least tolerant of if it gets preempted
// mid-burst by FreeRTOS.
//
// This project instead uses the ESP32-P4's real RMT (Remote Control
// Transceiver) peripheral, via Arduino-ESP32's own `esp32-hal-rmt.h`
// wrapper (confirmed present and built for this target:
// framework-arduinoespressif32-libs/esp32p4/include/esp_driver_rmt, guarded
// by `SOC_RMT_SUPPORTED` in the core header -- RMT is a standard,
// universally-present ESP32-family peripheral, not P4-specific). RMT
// generates the modulated carrier and precise mark/space timing entirely
// in hardware once started -- no CPU involvement, no jitter, no watchdog
// risk -- which is the same real reason IRremoteESP8266/IRremote's own
// ESP32 backend uses RMT internally rather than bit-banging. Every other
// IR feature task (Task 17 receive, Task 18 universal remote, Task 19
// jammer) should transmit through this module rather than touching
// TAB5_IR_TX_GPIO or the RMT API directly, so the hardware-arbitration and
// carrier-generation details stay in exactly one place.
// ===========================================================================

namespace IrCommon {

// Real spec-derived cap: matches IrFileFormat::kMaxRawSamples (the real
// Flipper .ir format's own stated 1024-element max for a raw signal's
// `data` field, features/ir/ir_file_format.h) -- also a safe superset of
// WORLD_IR_CODES.h's largest real code (numpairs is a uint8_t, so at most
// 255 pairs = 510 durations). One constant serves both callers rather than
// each guessing its own bound.
constexpr size_t kMaxDurationsPerTransmit = 1024;

// Claims Gpio53Arbiter::Owner::kIr and initializes the RMT TX channel on
// TAB5_IR_TX_GPIO. Returns false (nothing touched) if PORT.A is held by
// another owner (NFC/RFID2 or RF433), or if rmtInit() itself fails.
//
// Deliberately independent of hal::IrUnit (Task 15's plain-GPIO bring-up
// HAL): rmtInit() reassigns the pin's ESP32 peripheral-manager ownership
// to the RMT peripheral (the same GPIO-ownership-transfer mechanism
// documented at length in hal/rf433_gpio.cpp's header -- perimanSetPinBus()
// calling the PREVIOUS owner's deinit callback), so composing this with
// IrUnit::begin()'s plain pinMode(OUTPUT) would just have one silently
// undo the other's pin configuration. Call this instead of IrUnit::begin()
// for any real transmit; IrUnit's begin()/set_tx() remain only for the
// Task 15 bring-up spike's own plain-GPIO presence test.
bool init();

// Releases the RMT TX channel and the Owner::kIr claim. Safe to call
// defensively even if init() was never called or failed.
void deinit();

// Transmits one IR signal: alternating mark/space durations in
// microseconds (durations_us[0] is always a mark -- carrier ON -- per
// both this project's own .ir raw-signal convention, ir_file_format.h,
// and the cross-validated real TV-B-Gone donor transmit-loop convention;
// see world_ir_codes.h's provenance header), at the given carrier
// frequency (Hz) and duty cycle (0.0-1.0, typically ~0.33 for consumer IR).
// Blocking -- returns once the whole signal has finished transmitting
// (real transmissions are well under 100ms; this is not the kind of
// multi-second operation that needs poll()-driven chunking on its own,
// though a CALLER iterating many signals in sequence -- e.g. IrTvbGone's
// whole-database sweep -- still must not do so in one blocking loop; see
// ir_tvbgone.h's own header for why).
//
// Requires init() to have succeeded first. Returns false if not
// initialized, if count is 0 or exceeds kMaxDurationsPerTransmit, or if
// the underlying RMT carrier/write call fails.
bool transmit_raw(const uint16_t *durations_us, size_t count, uint32_t carrier_hz,
                  float duty_cycle);

} // namespace IrCommon
