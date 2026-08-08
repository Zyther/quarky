#include "nfc_pn532.h"
#include "../../boards/tab5/pins_config.h"
#include "io_expander.h"
#include <Wire.h>
#include <Arduino.h>

// ===========================================================================
// Task 18 real-hardware research: which bus, which addresses, which chips
// ===========================================================================
//
// WHICH BUS. The Tab5's touch/display/IMU/RTC/codec devices (Tasks 5/6/10 --
// see pins_config.h) all live on the INTERNAL I2C bus, GPIO 31/32
// (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO), driven by the `Wire` instance.
//
// The HY2.0-4P peripheral units this task brings up are on a DIFFERENT,
// EXTERNAL bus. Confirmed from M5Stack's own product page
// (docs.m5stack.com/en/core/Tab5, fetched 2026-08-08), whose pin table lists
// exactly one physical HY2.0-4P connector, labelled PORT.A:
//     HY2.0-4P (PORT.A): G53, G54
// This matches this project's own variant file (boards/variants/
// quarky_tab5_p4/pins_arduino.h), which already documents GPIO 53/54 as
// "Tab5's actual external I2C bus pins" (set there for an unrelated reason --
// keeping Wire.begin()'s no-args fallback off the C6 SDIO bus -- but the
// label was already correct). Recorded here as
// TAB5_EXTERNAL_I2C_SDA_GPIO/SCL_GPIO in pins_config.h, and driven via the
// second Arduino I2C peripheral, `Wire1`, so this bus is never conflated with
// the internal one `Wire` already owns.
//
// The Tab5 has only ONE physical HY2.0 connector (PORT.A). Three units are
// asked for in this task's brief: NFC, RFID2, and RF433R/T. The NFC and
// RFID2 units are both I2C devices at distinct addresses, so both can share
// PORT.A's single physical socket simultaneously (via a splitter/hub cable or
// a pass-through port on one of the units, which is a normal I2C bus
// topology) -- this is why they are handled in this file, together, on the
// same `Wire1` bus. RF433R/T is NOT an I2C device (see rf433_gpio.cpp for why
// its GPIO pin cannot be determined from documentation).
//
// WHICH ADDRESSES / WHICH CHIPS. The brief's placeholders (0x24 for NFC,
// 0x28 for RFID2) were explicitly flagged as unconfirmed. Real-hardware
// research (2026-08-08, via M5Stack's own documentation) found:
//
//   * Unit RFID2: CONFIRMED as a WS1850S chip at I2C 0x28.
//     docs.m5stack.com/en/product_i2c_addr lists "WS1850S (0x28)" for Unit
//     RFID2; docs.m5stack.com/en/unit/rfid2 confirms the chip and address
//     directly ("I2C @0x28"). This is NOT a PN532 -- the brief's assumption
//     that both units are PN532-based is wrong for this one, the same class
//     of error as the ST7121-vs-GT911 and ST7121-vs-ST7123 mistakes found in
//     the display/touch hotfixes (a community/brief assumption that didn't
//     match the specific real part). The address placeholder (0x28) happened
//     to be right regardless; the chip identity was not.
//   * NFC unit: UNRESOLVED BY DOCUMENTATION ALONE. M5Stack currently sells a
//     "Unit NFC" / "NFC Universal Unit" using an ST25R3916-AQWT at I2C 0x50
//     (docs.m5stack.com/en/unit/Unit_NFC: "I2C @0x50 (100K / 400K)"). An
//     older Grove-cabled NFC module using a real PN532 at the datasheet
//     default 0x24 also exists in M5Stack/community history (referenced in
//     M5Stack community threads as "NFC PN532 grove v1.1"), but no longer
//     appears in M5Stack's current product/I2C-address documentation. Which
//     one the physically-connected unit actually is could not be determined
//     from research; it was left to the real-hardware census below to
//     resolve. RESOLVED 2026-08-08 by that census, once the port-power bug
//     immediately below was fixed: the connected unit answers at 0x50, i.e.
//     it is the CURRENT ST25R3916-based Unit NFC, not a PN532. The 0x24
//     guess is retired (see TAB5_NFC_I2C_ADDR in pins_config.h).
//
// WHICH POWER RAIL (added by the HY2.0 port-power hotfix, 2026-08-08).
// Task 18's first hardware run scanned all of 0x08-0x77 on this bus with a
// physical M5Stack NFC unit plugged into PORT.A and found "(nothing
// responded)". That is not a wrong-address symptom -- a wrong address still
// leaves the *right* one visible in a full sweep -- it is a dead-bus symptom.
// The cause turned out to be identical in kind to the C6 SDIO hotfix's second
// root cause: PORT.A's 5V pin is gated by EXT_5V_EN, P2 of the PI4IOE5V6408
// IO-expander at 0x43 on the INTERNAL bus, and nothing in this project was
// asserting it. The plugged-in unit simply had no supply. See
// TAB5_EXT_5V_EN_IOEXP_* in pins_config.h for the three corroborating
// sources. ensureExternalI2CBegun() below now asserts that gate, and waits
// for the unit's power-on reset, before the bus is touched.
//
// detect() below is therefore a bare I2C address ACK probe, not a real PN532
// GetFirmwareVersion exchange -- correcting the brief's code comment, which
// described the PN532 command but whose actual code (Wire.beginTransmission
// / endTransmission with no command bytes written) never sent it. A protocol
// command that assumes PN532 framing would be actively wrong to run against
// a WS1850S or ST25R3916, so a bare presence probe -- chip-agnostic, and
// sufficient to prove "the HAL can talk to the HY2.0 units", exactly this
// task's stated scope -- is what is actually implemented, on both the old
// bus and this one.
// ===========================================================================

namespace {

bool s_external_bus_begun = false;

// Turn on the external 5V bus that feeds PORT.A's red wire (and the rear
// M5-Bus / side 2.54-10P header). Off at reset; nothing on PORT.A can answer
// until this is asserted. The gate itself lives on the INTERNAL I2C bus, so
// Wire -- not Wire1 -- has to be up first. Wire.begin() is idempotent (the
// framework logs "Bus already started in Master Mode" and returns), and
// display_tab5.cpp / touch_tab5.cpp / hosted_link.cpp all call it the same
// way for the same reason: no module may assume it is the first user.
void ensureExternalPortPowered() {
    Wire.begin(TAB5_INTERNAL_I2C_SDA_GPIO, TAB5_INTERNAL_I2C_SCL_GPIO);
    bool ok = tab5_ioexp::set_output(TAB5_EXT_5V_EN_IOEXP_I2C_ADDR,
                                     TAB5_EXT_5V_EN_IOEXP_BIT, true);
    Serial.printf("quarky-tab5: EXT_5V_EN (ioexp 0x%02X P%d) assert: %s\n",
                  TAB5_EXT_5V_EN_IOEXP_I2C_ADDR, TAB5_EXT_5V_EN_IOEXP_BIT,
                  ok ? "OK" : "FAILED (I2C write to IO-expander failed)");
    // Even on failure, fall through and probe anyway: the scan output is the
    // diagnostic, and reporting "power enable failed AND bus empty" is more
    // useful than refusing to look.
    delay(TAB5_EXT_5V_SETTLE_MS);
}

void ensureExternalI2CBegun() {
    if (s_external_bus_begun) {
        return;
    }
    ensureExternalPortPowered();
    Wire1.begin(TAB5_EXTERNAL_I2C_SDA_GPIO, TAB5_EXTERNAL_I2C_SCL_GPIO);
    Wire1.setClock(TAB5_EXTERNAL_I2C_FREQ_HZ);
    s_external_bus_begun = true;
}

const char *labelForExternalI2CAddr(uint8_t a) {
    switch (a) {
        case 0x24: return "PN532 (datasheet default I2C addr)";
        case 0x28: return "WS1850S (M5Stack Unit RFID2)";
        case 0x50: return "ST25R3916 (M5Stack Unit NFC / NFC Universal Unit)";
        default:   return "unidentified";
    }
}

} // namespace

bool NfcPN532::detect(const char *label) {
    ensureExternalI2CBegun();

    // Bare presence probe: address the device and see whether anything ACKs.
    // See the file header comment for why this -- not a PN532 protocol
    // exchange -- is the right check here.
    Wire1.beginTransmission(i2c_addr_);
    bool present = (Wire1.endTransmission() == 0);
    Serial.printf("quarky-tab5: NFC unit '%s' at 0x%02X: %s\n", label, i2c_addr_,
                  present ? "detected" : "not found");
    return present;
}

void nfc_scan_external_i2c_bus() {
    ensureExternalI2CBegun();
    Serial.println("quarky-tab5: external HY2.0 PORT.A I2C census (GPIO "
                    "53/54):");
    bool any = false;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        Wire1.beginTransmission(a);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("quarky-tab5:   0x%02X  %s\n", a, labelForExternalI2CAddr(a));
            any = true;
        }
    }
    if (!any) {
        Serial.println("quarky-tab5:   (nothing responded)");
    }
}
