#pragma once
#include "infc.h"
#include <cstdint>

// NAMING NOTE (Task 18 real-hardware research, see nfc_pn532.cpp header
// comment and pins_config.h): despite the class name, the physical unit
// wired to the RFID2 instance of this class is NOT a PN532 -- M5Stack's own
// docs identify Unit RFID2 as a WS1850S part at I2C 0x28. The NFC-labelled
// unit may or may not be a real PN532 either (M5Stack's current "Unit NFC" /
// "NFC Universal Unit" product uses an ST25R3916 at 0x50; only an older,
// possibly-EOL Grove NFC module was PN532 at 0x24). The class name is kept
// because the task brief's "Interfaces" contract specifies it, and because
// detect() below (a bare I2C address probe) is chip-protocol-agnostic -- it
// works identically regardless of which of these three chips actually
// answers. Phase 3, when it implements real read/write, MUST NOT assume PN532
// framing for either instance without first re-confirming the chip via the
// firmware-version/UID read that protocol defines.
class NfcPN532 : public INFC {
public:
    explicit NfcPN532(uint8_t i2c_addr) : i2c_addr_(i2c_addr) {}
    bool detect(const char *label) override;

private:
    uint8_t i2c_addr_;
};

// Brings up the external HY2.0 PORT.A I2C bus (Wire1) if it isn't up already:
// asserts the EXT_5V_EN gate on the internal-bus IO-expander, waits out the
// connected unit's power-on reset, then Wire1.begin()/setClock(). Idempotent
// and safe to call from any module.
//
// Exposed (Phase 3 Task 2) so that features/nfc/st25r3916_driver.cpp -- which
// talks to the very same physical unit this HAL probes, just at register level
// -- can reuse this exact sequence instead of re-deriving it. That matters:
// PORT.A has NO power at reset until EXT_5V_EN is asserted, a bug that cost a
// full "(nothing responded)" hardware run to find (see nfc_pn532.cpp's header
// comment). Duplicating the bring-up would mean duplicating the chance of
// getting it wrong.
//
// Returns whether the external I2C bus is actually usable after this call.
// GPIO53 (this bus's SDA line) is shared with RF433 capture/transmit (see
// hal/gpio53_arbiter.h) -- this function claims it via Gpio53Arbiter before
// touching Wire1, and returns false WITHOUT touching the bus if an active
// RF433 session currently holds the pin. It also returns false if
// Wire1.begin() itself fails. Callers that go on to touch Wire1/Wire1-backed
// drivers after this returns false will see those calls fail too (a dead or
// unclaimed bus fails its own reads/writes cleanly), so checking the return
// value here is a chance to fail earlier and with a clearer message, not the
// only line of defense.
bool nfc_ensure_external_i2c_begun();

// Releases this module's Gpio53Arbiter claim on GPIO53 (Owner::kExternalI2c).
// No-op if this module doesn't currently hold it -- safe to call defensively
// from a teardown path that isn't sure whether nfc_ensure_external_i2c_begun()
// ever succeeded. Call when an NFC/RFID2 "session" (e.g. a screen) ends, so
// RF433 can claim the pin afterward; see features/nfc/nfc_read.cpp's
// teardown().
void nfc_release_external_i2c();

// Diagnostic: labelled scan of the external HY2.0 PORT.A I2C bus (GPIO 53/54
// on this board -- see TAB5_EXTERNAL_I2C_SDA_GPIO/SCL_GPIO in pins_config.h).
// Not part of INFC; a bring-up aid in the same shape as touch_tab5.cpp's
// internal-bus census. Safe to call at any time -- it lazily begins the
// external I2C bus if this is the first call to touch it.
void nfc_scan_external_i2c_bus();
