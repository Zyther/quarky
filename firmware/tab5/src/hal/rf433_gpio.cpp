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
//
// >>> THIS IS THE CANONICAL WRITE-UP OF THE GPIO53 SHARING PROBLEM. <<<
// main.cpp's setup(), hal/nfc_pn532.cpp and features/rf433/rf433_common.cpp
// all point here rather than repeating it, so the framework line numbers
// below exist in exactly one place and only have to be re-checked once after
// an Arduino-ESP32 bump.
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
//
// --- AND THE SAME HAZARD RUNS IN REVERSE ---------------------------------
// That I2C recovery takes GPIO53 BACK: i2cInit() calls perimanClearPinBus()
// on both pins (esp32-hal-i2c-ng.c:107) and re-routes GPIO53 to the I2C
// peripheral. If an RF433 capture is running at that moment it is NOT told --
// the peripheral manager's GPIO deinit callback (gpioDetachBus(),
// cores/esp32/esp32-hal-gpio.c:105-107) is a no-op returning true, so
// Rf433Common keeps s_capturing == true and keeps its ISR installed while the
// pin belongs to someone else. The capture then records ~0 edges, or -- if the
// I2C driver's pin setup leaves the GPIO interrupt unmasked (inside the
// prebuilt i2c_new_master_bus(); not verified) -- timestamps I2C traffic and
// returns fake but plausible pulse data. No error flag is set in either case.
// See features/rf433/rf433_common.cpp's header for the same warning from the
// capture side.
//
// Neither direction is defended against in code, deliberately: the real fix is
// arbitration of GPIO53 (a claim/release owner token both subsystems respect),
// not back-references between hal/ and features/. It is unreachable today --
// both RF433 and NFC are serial-trigger spikes with no launcher tile, so
// nothing runs concurrently -- and becomes reachable the moment either grows a
// UI that can stay open while the other runs.
// ===========================================================================

bool Rf433Gpio::init() {
    // NOTE for Task 6 (RF433 transmit/replay), which will be this function's
    // first real caller: TAB5_RF433R_PIN and TAB5_RF433T_PIN are THE SAME PIN
    // (both GPIO53 -- one HY2.0 socket, one data line). The INPUT line below is
    // therefore immediately and entirely overridden by the OUTPUT line, and the
    // pin always ends up an output. It is kept only so this function still
    // reads as "configure both roles" against the IRF433 interface, and because
    // deleting it would make the asymmetry with capture_start() (which sets
    // INPUT for receive) look deliberate rather than forced. If you need
    // receive, do not call this -- call Rf433Common::capture_start(), which
    // sets the pin back to INPUT itself.
    pinMode(TAB5_RF433R_PIN, INPUT);
    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);
    Serial.printf("quarky-tab5: RF433 GPIO configured -- T=GPIO%d (confirmed "
                  "2026-08-09, 433.92MHz listener), R=GPIO%d (confirmed "
                  "2026-08-18, interrupt-driven receive test)\n",
                  TAB5_RF433T_PIN, TAB5_RF433R_PIN);
    return true;
}
