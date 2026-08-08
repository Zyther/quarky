#pragma once

// -----------------------------------------------------------------------------
// Single, bounded, one-shot bring-up of the esp-hosted SDIO link to the Tab5's
// ESP32-C6 radio co-processor.
// -----------------------------------------------------------------------------
// The ESP32-P4 has no radio of its own: both WiFi (RadioEspHosted / C2LinkWifi)
// and BLE (C2LinkBle) are proxied to the onboard C6 over a 4-bit SDIO bus by
// Espressif's esp-hosted. Everything radio-shaped on this board therefore sits
// downstream of ONE transport, and if that transport is broken every radio API
// fails -- which is exactly what produced the boot crash-loop this module was
// written for: the wrong C6 GPIOs meant `card init failed`, each subsequent
// WiFi/BLE call re-ran the whole failing bring-up (arduino-esp32 resets its
// `lowLevelInitDone` latch on failure, so every retry re-toggles the C6 reset
// line and re-runs SDIO card init), and ~20-30s of that retry storm sank the
// rail far enough to trip the brownout detector and reset the chip. Forever.
//
// The pin bug itself is fixed at compile time by this repo's own board variant
// (boards/variants/quarky_tab5_p4/pins_arduino.h). This module is the
// defence-in-depth half: bring the link up EXACTLY ONCE, latch the result, and
// let every radio entry point cheaply ask "is the C6 there?" instead of
// rediscovering the answer the expensive way. A C6 that is absent, reflashed
// with mismatched firmware, or electrically unhappy then costs one failed
// attempt and a clear log line -- not an infinite reset loop -- and the
// display/touch/LVGL stack keeps running.
//
// Call begin() once from setup(), AFTER the UI is up (so a slow or failing
// radio bring-up can never stop the screen from appearing), and gate all
// radio work on the result.
// -----------------------------------------------------------------------------
namespace hosted_link {

// Attempts C6 bring-up on the first call and caches the outcome; every later
// call returns that cached outcome without touching the hardware. Safe to call
// from anywhere, any number of times. Logs its own success/failure to Serial.
bool begin();

// The latched result of begin(). False if begin() has not run yet or failed.
bool available();

}  // namespace hosted_link
