#pragma once

// Runtime ownership arbiter for GPIO53 -- shared between the external
// I2C bus (NFC/RFID2, hal/nfc_pn532.cpp) and RF433 capture/transmit
// (features/rf433/rf433_common.cpp), because both are wired to the
// same physical pin on the Tab5's single HY2.0 PORT.A connector. See
// boards/tab5/pins_config.h for the hardware story and
// features/rf433/rf433_common.cpp's header comment for the concurrent-
// access hazard this exists to close.
//
// Main-task only. Neither claim() nor release() is ISR-safe or
// intended to be called from one -- confirmed as of this file's
// writing that nothing calls into this from Rf433Common's ISR
// (isr_edge() only touches its own ring buffer), and that must stay
// true for any future caller too.
namespace Gpio53Arbiter {

enum class Owner { kNone, kExternalI2c, kRf433 };

// Claims the pin for `owner`. Returns true if `owner` now holds it --
// either it just claimed a free pin, or it already held it (idempotent,
// matching this project's established "refuse rather than lie"
// convention, e.g. Rf433Common::capture_start()'s own idempotent
// re-arm). Returns false, and leaves the current owner untouched, if
// the pin is held by the OTHER owner -- callers must treat false as a
// real refusal, not attempt the underlying pin operation anyway.
bool claim(Owner owner);

// Releases the pin if `owner` currently holds it. No-op (not an error)
// if `owner` doesn't hold it -- safe to call defensively from a
// teardown path that isn't sure whether claim() ever succeeded.
void release(Owner owner);

Owner current_owner();

} // namespace Gpio53Arbiter
