#include "rf433_gpio.h"
#include "../../boards/tab5/pins_config.h"
#include <Arduino.h>

// ===========================================================================
// Task 18 real-hardware research: RF433R/T GPIO pins -- GENUINELY UNKNOWN
// ===========================================================================
//
// The brief's placeholder (-1 / TODO) could not be resolved from research,
// and this is recorded honestly rather than guessed at, per this project's
// established norm (see pins_config.h's other TODO-and-cite-why entries,
// e.g. the SD power-enable pin).
//
// What was checked (2026-08-08):
//
//   * M5Stack's Tab5 product page (docs.m5stack.com/en/core/Tab5) pin table
//     lists exactly ONE physical HY2.0-4P connector, labelled PORT.A, wired
//     to GPIO 53/54 -- see TAB5_EXTERNAL_I2C_SDA_GPIO/SCL_GPIO in this file
//     and nfc_pn532.cpp. Those two pins are already spoken for by the NFC/
//     RFID2 I2C bus (nfc_pn532.cpp): wiring a raw digital RF433 signal onto
//     either line in parallel with live I2C traffic would corrupt the I2C
//     bus, not merely be a wasted pin. So even if a Tab5-specific RF433 pin
//     existed, it could not be 53 or 54 while NFC/RFID2 are attached to
//     PORT.A.
//   * M5Stack's RF433T/R unit docs (docs.m5stack.com/en/unit/rf433_t and
//     .../rf433_r) explicitly say GPIO assignment is host-specific ("Since
//     each host device has different pin configurations, refer to the
//     RF433T/R Pin Compatibility Table... before use") and each unit's own
//     PinMap section names a generic "PORT.A"/"PORT.B" -- NOT Tab5's actual
//     PORT.A, but a placeholder for whatever multi-port host the doc
//     template was written against (Tab5 has no PORT.B at all). Neither page
//     lists Tab5 in a compatibility table.
//   * github.com/m5stack/M5Unit-RF433 (the unit's driver/example repo) and
//     the M5Stack community thread "How to use RF Unit RF433T and RF433R"
//     contain no Tab5-specific pin numbers either.
//
// So: no vendor source anywhere states which GPIO(s) Tab5 exposes for a
// second/third HY2.0-style connection, or which pin the physically-connected
// RF433R/T unit in this deployment actually uses. Determining this would
// require either continuity-testing the physical unit's wiring (not
// something this task's author has hardware access to do) or the project
// owner supplying the answer directly. Per the brief's own fallback
// guidance, this is left as an explicit, clearly-marked TODO rather than a
// fabricated GPIO number -- and Rf433Gpio::init() below refuses to touch any
// pin until it is filled in, rather than silently calling pinMode()/
// digitalWrite() on -1 and printing a false "configured" success line.
//
// TODO(owner): once the RF433R/T unit's actual wiring is known (e.g. a
// second HY2.0 socket on an add-on base, the M5-Bus rear connector, or the
// GPIO header stamp pads -- see the many free M5-Bus GPIOs listed in
// pins_config.h's Task-18 section), override these via build flag
// (-DTAB5_RF433R_PIN=<n> -DTAB5_RF433T_PIN=<n>) or edit the #ifndef defaults
// directly.
// ===========================================================================

bool Rf433Gpio::init() {
#if TAB5_RF433R_PIN < 0 || TAB5_RF433T_PIN < 0
    Serial.println("quarky-tab5: RF433R/T GPIO pins UNKNOWN (TODO in "
                    "pins_config.h -- TAB5_RF433R_PIN/TAB5_RF433T_PIN are "
                    "unset placeholders) -- RF433 unit NOT initialized. See "
                    "rf433_gpio.cpp for what was checked.");
    return false;
#else
    pinMode(TAB5_RF433R_PIN, INPUT);
    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);
    Serial.println("quarky-tab5: RF433R/T GPIO configured");
    return true;
#endif
}
