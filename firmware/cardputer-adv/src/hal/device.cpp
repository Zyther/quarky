#include "device.h"
#include "../../boards/cardputer-adv/pins_config.h"
#include <Wire.h>
#include <Arduino.h>

Device &Device::instance() {
    static Device d;
    return d;
}

void Device::init() {
    // ST7789 240x135 display init via TFT_eSPI or M5GFX -- reuse whichever
    // library UniGeek's board config declares for m5_cardputer_adv so the
    // panel init sequence is known-correct for this exact hardware.
    // Real ST7789 panel bring-up (CP_ADV_LCD_* pins in pins_config.h,
    // UniGeek's TFT_eSPI setup) is deferred to a later task -- this skeleton
    // only proves the HAL wiring compiles and boots.
    display_ready_ = true; // set true once real panel init call succeeds

    // TCA8418 keyboard lives on its own I2C bus (SDA/SCL confirmed from
    // UniGeek's real board source -- see pins_config.h). A bare probe
    // (address ACK only) is enough for this skeleton; real key-matrix
    // decoding via Adafruit_TCA8418 is deferred to a later task.
    Wire.begin(CP_ADV_KB_SDA_PIN, CP_ADV_KB_SCL_PIN);
    Wire.beginTransmission(CP_ADV_KB_I2C_ADDR);
    keyboard_ready_ = (Wire.endTransmission() == 0);
}
