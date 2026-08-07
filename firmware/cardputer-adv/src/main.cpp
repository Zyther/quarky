#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("quarky-cardputer-adv: boot ok");
}

void loop() {
    delay(1000);
}
