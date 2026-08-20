#include "gpio53_arbiter.h"
#include <Arduino.h>

// Implementation is a single file-scope Owner variable, no locking needed
// (main-task only -- see the header's comment on why an ISR must never call
// into this).

namespace Gpio53Arbiter {
namespace {

Owner s_owner = Owner::kNone;

const char *ownerName(Owner o) {
    switch (o) {
        case Owner::kNone:       return "none";
        case Owner::kExternalI2c: return "external I2C (NFC/RFID2)";
        case Owner::kRf433:      return "RF433";
    }
    return "?";
}

} // namespace

bool claim(Owner owner) {
    if (s_owner == owner) return true; // idempotent re-claim
    if (s_owner != Owner::kNone) {
        Serial.printf("quarky-tab5: [gpio53-arbiter] claim(%s) REFUSED -- GPIO53 "
                      "is currently held by %s\n",
                      ownerName(owner), ownerName(s_owner));
        return false;
    }
    s_owner = owner;
    return true;
}

void release(Owner owner) {
    if (s_owner != owner) return; // not held by this owner -- safe no-op
    s_owner = Owner::kNone;
}

Owner current_owner() { return s_owner; }

} // namespace Gpio53Arbiter
