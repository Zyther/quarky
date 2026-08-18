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
