#include <Arduino.h>
#include "hal/device.h"

void setup() {
    Serial.begin(115200);
    delay(500);
    Device::instance().init();
    Serial.printf("quarky-cardputer-adv: display=%s keyboard=%s\n",
                  Device::instance().display_ready() ? "OK" : "FAIL",
                  Device::instance().keyboard_ready() ? "OK" : "FAIL");
}

void loop() {
    delay(1000);
}
