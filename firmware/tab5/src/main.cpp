#include <Arduino.h>
#include "hal/display_tab5.h"

DisplayTab5 display;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("quarky-tab5: display init");
    display.init();

    // Fill the screen red as a bring-up smoke test.
    static uint16_t red_row[1280];
    for (int i = 0; i < 1280; i++) red_row[i] = 0xF800; // RGB565 red
    for (int y = 0; y < display.height(); y++) {
        display.flush(0, y, display.width() - 1, y, red_row);
    }
    Serial.println("quarky-tab5: display filled red");
}

void loop() { delay(1000); }
