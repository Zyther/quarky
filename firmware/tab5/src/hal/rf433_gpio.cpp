#include "rf433_gpio.h"
#include "../../boards/tab5/pins_config.h"
#include <Arduino.h>

// ===========================================================================
// RF433R/T GPIO pins -- see pins_config.h for the full real-hardware
// research trail (2026-08-08 doc research that came up empty, then the
// 2026-08-09 real hardware tests that resolved TAB5_RF433T_PIN and gave a
// reasoned-but-unconfirmed value for TAB5_RF433R_PIN).
//
// Short version: TAB5_RF433T_PIN=53 is CONFIRMED (independent 433.92MHz
// listener test). TAB5_RF433R_PIN=53 is a HYPOTHESIS (same connector
// position, not independently verified with a proper receive test -- a
// same-day loop()-polling attempt was inconclusive due to sampling rate,
// not evidence the pin is wrong). Configuring a GPIO as INPUT is always
// side-effect-free regardless of confidence level, so R's pin is set up
// unconditionally below; only T's OUTPUT-and-drive behavior carries the
// "confirmed" guarantee.
// ===========================================================================

bool Rf433Gpio::init() {
    pinMode(TAB5_RF433R_PIN, INPUT);
    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);
    Serial.printf("quarky-tab5: RF433 GPIO configured -- T=GPIO%d (confirmed "
                  "2026-08-09), R=GPIO%d (hypothesis, unconfirmed)\n",
                  TAB5_RF433T_PIN, TAB5_RF433R_PIN);
    return true;
}
