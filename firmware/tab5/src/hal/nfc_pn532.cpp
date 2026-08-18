#include "nfc_pn532.h"
#include "../../boards/tab5/pins_config.h"
#include "io_expander.h"
#include <Wire.h>
#include <Arduino.h>
#include <esp32-hal-periman.h> // perimanGetPinBusType() -- see
                                // externalI2CPinStillOwnedByI2C() below for why
                                // this file needs to ask who owns GPIO53

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

// Latched separately from the bus itself. The power gate is a bit on an
// IO-expander on the INTERNAL bus; nothing that happens to GPIO53 can disturb
// it, so once asserted it stays asserted for the boot. Keeping it out of the
// bus latch means a Wire1 re-init never pays the 200 ms EXT_5V settle again.
bool s_external_port_powered = false;

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

// True while GPIO53 is still registered to the Arduino peripheral manager as
// this bus's I2C SDA line. Goes false the moment anything else claims the pin.
//
// GPIO53 is shared three ways (TAB5_EXTERNAL_I2C_SDA_GPIO, TAB5_RF433T_PIN,
// TAB5_RF433R_PIN) and an RF433 pinMode() on it destroys this entire I2C bus.
// >>> The full incident write-up, mechanism and framework citations live in
// >>> hal/rf433_gpio.cpp's header comment -- the ONE canonical copy. <<<
// Short version, enough to read this function: any pinMode() on GPIO53 runs
// the peripheral manager's deinit callback for the pin's previous owner, and
// for an I2C SDA pin that callback deletes the whole master bus, producing
// `esp32-hal-i2c-ng.c: bus is not initialized` on a bus that worked seconds
// earlier.
//
// A plain "did we call begin() once" latch cannot see that -- it stays true
// after the peripheral underneath it has been destroyed. Asking the peripheral
// manager who currently owns the pin is the direct question, and it is cheap
// (an array lookup).
//
// WHY CHECKING SDA ALONE IS ENOUGH, and why that is not obvious: SCL (GPIO54)
// is not shared with anything and so is never the pin that gets stolen -- but
// that is not the real reason. The real reason is that the two pins' periman
// registrations are created and destroyed strictly together:
// i2cInit() registers both (esp32-hal-i2c-ng.c:148-149) and i2cDeinit() clears
// both in the same branch that sets initialized=false
// (esp32-hal-i2c-ng.c:200-201). There is no state where SDA is released and
// SCL is not. That coupling is a framework implementation detail one refactor
// away from silently breaking this check, so it is written down rather than
// relied on silently.
bool externalI2CPinStillOwnedByI2C() {
    return perimanGetPinBusType(TAB5_EXTERNAL_I2C_SDA_GPIO) ==
           ESP32_BUS_TYPE_I2C_MASTER_SDA;
}

// Returns false if the bus could not be brought up. Callers must not treat a
// failure as "probably fine": the entire point of the recovery path below is
// to stop silent `bus is not initialized` failures, and swallowing begin()'s
// result here would reintroduce exactly that, one layer up.
bool beginExternalI2C() {
    // Safe to call again after a teardown: TwoWire::begin() only short-circuits
    // ("Bus already started in Master Mode", Wire.cpp:299-303) when
    // i2cIsInit() is true -- and the i2cDeinit() that stole the pin is exactly
    // what cleared that flag. When it is false, begin() re-runs initPins() and
    // i2cInit() in full, which re-registers GPIO53/54 with the peripheral
    // manager and rebuilds the bus. So no explicit "release the pin back"
    // step is needed on the RF433 side; the I2C side re-claims what it needs,
    // when it needs it.
    if (!Wire1.begin(TAB5_EXTERNAL_I2C_SDA_GPIO, TAB5_EXTERNAL_I2C_SCL_GPIO)) {
        Serial.printf("quarky-tab5: Wire1.begin(SDA=GPIO%d, SCL=GPIO%d) FAILED -- "
                      "the external PORT.A I2C bus is NOT usable. Every NFC/RFID2 "
                      "access after this will fail; expect "
                      "`esp32-hal-i2c-ng.c: bus is not initialized`.\n",
                      TAB5_EXTERNAL_I2C_SDA_GPIO, TAB5_EXTERNAL_I2C_SCL_GPIO);
        return false;
    }
    Wire1.setClock(TAB5_EXTERNAL_I2C_FREQ_HZ);
    return true;
}

void ensureExternalI2CBegun() {
    if (!s_external_port_powered) {
        ensureExternalPortPowered();
        s_external_port_powered = true;
    } else if (externalI2CPinStillOwnedByI2C()) {
        // Fast path: powered, and the bus still owns its pin. Nothing to do.
        return;
    } else {
        // ===================================================================
        // RECOVERY PATH -- and it is not free. READ THIS BEFORE ASSUMING IT IS
        // SAFE TO TRIGGER FROM ANYWHERE.
        // ===================================================================
        // Re-initializing Wire1 takes GPIO53 BACK, which is the exact mirror
        // image of the bug this whole mechanism exists to fix. i2cInit() calls
        // perimanClearPinBus() on both pins (esp32-hal-i2c-ng.c:107) and then
        // hands GPIO53's routing to the I2C peripheral.
        //
        // If an RF433 edge capture is running when that happens, it is NOT
        // told. Rf433Common keeps s_capturing == true and keeps its
        // attachInterrupt() handler installed, because the peripheral
        // manager's GPIO deinit callback -- gpioDetachBus(),
        // cores/esp32/esp32-hal-gpio.c:105-107 -- is a no-op that returns true
        // without detaching anything. So the capture module still believes it
        // owns a pin that now belongs to I2C.
        //
        // Which of two bad outcomes follows depends on whether the I2C
        // driver's own pin setup masks the GPIO interrupt (that path is inside
        // the prebuilt i2c_new_master_bus() and was NOT read, so this is
        // stated as the open question it is):
        //   - if it masks it: the capture quietly records ~0 edges;
        //   - if it does not: the still-installed ISR timestamps I2C bus
        //     transitions, i.e. this driver's own register reads, and the
        //     capture returns plausible-looking pulse data that is entirely
        //     fake. That is the worse case, and it is not obviously the less
        //     likely one.
        // Either way the RF433 side reports no error.
        //
        // NOT FIXED IN CODE, deliberately: hal/ calling into features/ would
        // be a layering violation, and this is a real arbitration problem (see
        // the note in features/rf433/rf433_common.cpp) that wants a proper
        // owner token, not a back-reference from the HAL. Today it is
        // unreachable in practice -- neither RF433 nor NFC has a launcher
        // tile, both are serial-trigger spikes, and a human cannot press 'r'
        // and 'n' simultaneously. It becomes reachable the moment either grows
        // a UI that can stay open while the other runs.
        Serial.printf("quarky-tab5: external I2C bus was torn down -- GPIO%d is "
                      "no longer owned by I2C (an RF433 pinMode() does this; see "
                      "hal/rf433_gpio.cpp). Re-initializing Wire1 and TAKING THE "
                      "PIN BACK -- any RF433 capture still running is now "
                      "invalid.\n",
                      TAB5_EXTERNAL_I2C_SDA_GPIO);
    }
    beginExternalI2C();
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

// Public thin wrapper over the file-local bring-up above. Deliberately a
// wrapper rather than moving ensureExternalI2CBegun() out of the anonymous
// namespace: everything in this file keeps calling the local symbol, so there
// is exactly one implementation and no chance of the two drifting.
void nfc_ensure_external_i2c_begun() {
    ensureExternalI2CBegun();
}

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
