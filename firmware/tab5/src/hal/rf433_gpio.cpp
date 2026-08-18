#include "rf433_gpio.h"
#include "../../boards/tab5/pins_config.h"
#include <Arduino.h>

// ===========================================================================
// RF433R/T GPIO pins -- see pins_config.h for the full real-hardware
// research trail (2026-08-08 doc research that came up empty, then the
// real hardware tests that resolved both pins).
//
// Short version: BOTH pins are now CONFIRMED on real hardware, by separate
// tests with separate methods.
//   T=GPIO53, confirmed 2026-08-09: a slow deliberate blink on the pin was
//     observed at 433.920MHz by an independent second device listening on
//     that band, correlated with the GPIO53 window and not the GPIO54 one.
//   R=GPIO53, confirmed 2026-08-18: an interrupt-driven edge capture
//     (features/rf433/rf433_common.cpp) recorded a regular, sustained
//     burst-repeat structure while a second Bruce-firmware device
//     transmitted a 433MHz sub continuously -- see this plan's Task 1.
// The 2026-08-09 loop()-polling attempt that saw nothing on R was a false
// negative from sampling far too slowly for OOK pulse timing, not evidence
// against the pin.
//
// ===========================================================================
// !! DO NOT CALL init() UNCONDITIONALLY AT BOOT !! (added 2026-08-18)
// ===========================================================================
// GPIO53 is not RF433's to keep. TAB5_RF433T_PIN, TAB5_RF433R_PIN and
// TAB5_EXTERNAL_I2C_SDA_GPIO are all the same physical pin, because the Tab5
// has exactly ONE HY2.0 PORT.A socket and exactly one unit is plugged into it
// at a time -- NFC, RFID2, or RF433R/T, never several.
//
// Calling this from setup() therefore stole the pin from the I2C bus on every
// boot regardless of what was actually connected. That is not a theoretical
// concern: on 2026-08-18 (Phase 3 Task 2 Step 3, first attempt) the boot
// census correctly detected the ST25R3916 NFC unit at 0x50, and then
// St25r3916::init() failed 2/2 with `esp32-hal-i2c-ng.c: bus is not
// initialized` on that same just-working bus, because setup() called this
// function in between.
//
// The damage is bigger than "the pin now has a different function".
// pinMode() routes through perimanSetPinBus(pin, ESP32_BUS_TYPE_GPIO, ...)
// (cores/esp32/esp32-hal-gpio.c:161), and perimanSetPinBus() calls the
// PREVIOUS owner's deinit callback before reassigning
// (cores/esp32/esp32-hal-periman.c:174-183). For an I2C SDA pin that callback
// is i2cDetachBus() -> i2cDeinit(), which deletes the whole I2C master bus.
// One pinMode() on GPIO53 destroys Wire1 entirely.
//
// So: call this (or do a bare pinMode) only while an RF433 feature is actually
// running, and expect whoever wants the I2C bus next to re-establish it. The
// same applies to Rf433Common::capture_start(), which does its own on-demand
// pinMode(TAB5_RF433R_PIN, INPUT) -- that is correct, and it is also why
// pressing the 'r' RF433 trigger invalidates Wire1 for the rest of the boot
// unless the I2C side re-begins (hal/nfc_pn532.cpp's
// ensureExternalI2CBegun() detects exactly this and recovers).
// ===========================================================================

bool Rf433Gpio::init() {
    pinMode(TAB5_RF433R_PIN, INPUT);
    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);
    Serial.printf("quarky-tab5: RF433 GPIO configured -- T=GPIO%d (confirmed "
                  "2026-08-09, 433.92MHz listener), R=GPIO%d (confirmed "
                  "2026-08-18, interrupt-driven receive test)\n",
                  TAB5_RF433T_PIN, TAB5_RF433R_PIN);
    return true;
}
