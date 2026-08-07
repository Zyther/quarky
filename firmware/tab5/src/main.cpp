#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("quarky-tab5: boot ok");
}

void loop() {
    delay(1000);
}
