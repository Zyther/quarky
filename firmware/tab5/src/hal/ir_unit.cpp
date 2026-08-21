#include "ir_unit.h"
#include "../../boards/tab5/pins_config.h"
#include "gpio53_arbiter.h"
#include <Arduino.h>

// See ir_unit.h's header comment for the full real-hardware citation trail
// (M5Stack "Unit IR" SKU U002, IRM-3638T receiver, GPIO53/54 mapping).

namespace IrUnit {
namespace {
bool s_began = false;
} // namespace

bool begin() {
    if (!Gpio53Arbiter::claim(Gpio53Arbiter::Owner::kIr)) {
        Serial.println("quarky-tab5: [ir-unit] begin() REFUSED -- PORT.A is "
                       "held by another owner (NFC/RFID2 or RF433)");
        return false;
    }
    pinMode(TAB5_IR_TX_GPIO, OUTPUT);
    digitalWrite(TAB5_IR_TX_GPIO, LOW);
    pinMode(TAB5_IR_RX_GPIO, INPUT);
    s_began = true;
    Serial.printf("quarky-tab5: [ir-unit] begin() OK -- TX=GPIO%d, RX=GPIO%d\n",
                  TAB5_IR_TX_GPIO, TAB5_IR_RX_GPIO);
    return true;
}

void end() {
    if (!s_began) return;
    s_began = false;
    Gpio53Arbiter::release(Gpio53Arbiter::Owner::kIr);
}

void set_tx(bool level) {
    digitalWrite(TAB5_IR_TX_GPIO, level ? HIGH : LOW);
}

bool read_rx() {
    return digitalRead(TAB5_IR_RX_GPIO) == HIGH;
}

} // namespace IrUnit
