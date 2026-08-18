#include "st25r3916_driver.h"
#include "../../hal/nfc_pn532.h"
#include "../../../boards/tab5/pins_config.h"
#include <Wire.h>
#include <Arduino.h>

// ===========================================================================
// SOURCES. Every register address, mode byte, command code and bit position
// below traces to one of these. Nothing here is recalled, inferred, or
// pattern-matched from a PN532/WS1850S/other-chip driver -- this project has
// already been burned three times by exactly that (ST7121 mistaken for
// ST7123, touch assumed to be a separate GT911, Unit RFID2 assumed PN532 when
// it is a WS1850S), and the whole reason this file exists as its own task is
// that no donor firmware in this program has any ST25R3916 code at all.
//
// [DS] PRIMARY SOURCE -- ST's own datasheet.
//      "ST25R3916/ST25R3917 -- High performance NFC universal device and
//      EMVCo reader", STMicroelectronics, doc ID DS12484 Rev 3.
//      https://www.st.com/resource/en/datasheet/st25r3916.pdf
//      (retrieved 2026-08-18; 156 pages; the copy read for this task was the
//      byte-identical mirror at
//      https://download.mikroe.com/documents/datasheets/ST25R3916%20Datasheet.pdf)
//      Sections used, by name so they stay findable if page numbers move:
//        - Sec 4.2.13  "Reader operation"          (p.42) -- Ready mode entry,
//                       osc_ok, then rx_en/tx_en before addressing a tag
//        - Table 11    "SPI operation modes"       (p.49) -- the two leading
//                       mode bits M1/M0, shared verbatim by the I2C framing
//        - Sec 4.3.4   "I2C interface"             (p.53-57) -- "The I2C
//                       address is 50h"; register write/read use "the same
//                       Register Write/Read mode byte as for SPI"
//        - Figure 20   "Writing a single register"   (p.54) -- the I2C
//                       register-write byte order used by write_register()
//        - Figure 25   "Sending a direct command"    (p.56) -- the one-byte
//                       I2C direct-command frame used by execute_command()
//        - Figure 26   "Read and Write mode for register space-B access"
//                       (p.56) -- the only figure whose legend spells the
//                       framing out in words: "S: Start, Sr: repeated Start,
//                       A: ACK, N: NAK, P: Stop", showing a register READ as
//                       S,addr+W,<mode byte>,Sr,addr+R,data...
//        - Table 13    "List of direct commands"   (p.58) -- C0/C1 Set
//                       default, C2/C3 Stop all activities, C8/C9 NFC field
//                       ON, FB Register space-B access
//        - Sec 4.4.1   "Set default"               (p.58) -- what C1 does:
//                       stop all activities, reset all registers to default,
//                       clear collision bits, and explicitly "No IRQ due to
//                       termination of direct command is produced"
//        - Table 21    "Operation control register" (p.72) -- address 02h,
//                       bit 7 en, bit 6 rx_en, bit 3 tx_en
//        - Table 98    "Auxiliary display register" (p.124) -- address 31h,
//                       read-only, bit 4 osc_ok = "Xtal oscillation is stable"
//        - Table 117   "IC identity register"      (p.134) -- address 3Fh,
//                       space A, read-only; ic_type<4:0> in bits 7..3 with
//                       "00101: ST25R3916/7"; ic_rev<2:0> in bits 2..0 with
//                       "010: rev 3.1"
//
// [REF] SECONDARY SOURCE -- ST's own reference driver, used to cross-check
//      every constant taken from [DS] and to copy the shape of the init /
//      chip-ID / oscillator-on sequences rather than invent one.
//      "STM32duino ST25R3916" v2.0.2, author=STMicroelectronics (the RFAL
//      ST25R3916 HAL, packaged for Arduino by ST's own stm32duino org).
//      https://github.com/stm32duino/ST25R3916
//      commit b7e708f1fe458cca4e0ec9d3b78402c99ffc4e71 (2026-01-27), read
//      2026-08-18. Files/lines used:
//        - src/st25r3916_com.cpp:48-56 -- ST25R3916_I2C_ADDR (0xA0>>1),
//          WRITE_MODE (0<<6), READ_MODE (1<<6), CMD_MODE (3<<6)
//        - src/st25r3916_com.h:101,184,225 -- REG_OP_CONTROL 0x02,
//          REG_AUX_DISPLAY 0x31, REG_IC_IDENTITY 0x3F
//        - src/st25r3916_com.h:264-268,918,1144-1158 -- op_control en/rx_en/
//          tx_en bit positions, aux_display osc_ok, ic_type/ic_rev masks and
//          the ic_type_st25r3916 (5U<<3) / ic_type_st25r3916B (6U<<3) values
//        - src/st25r3916.h:87-88,122 -- CMD_SET_DEFAULT 0xC1, CMD_STOP 0xC2,
//          CMD_SPACE_B_ACCESS 0xFB
//        - src/st25r3916.h:138 -- TOUT_OSC_STABLE 10 (ms), annotated
//          "DS: 700us"
//        - src/st25r3916.h:148,150 -- st25r3916TxRxOn()/TxRxOff() = set/clear
//          (rx_en | tx_en) in OP_CONTROL. This, not the C8 "NFC initial field
//          ON" direct command, is how RFAL turns the field on for a plain
//          reader; see the field_on() note below for why that matters here.
//        - src/st25r3916.cpp:101-119 -- Initialize(): Set Default, then
//          CheckChipID, and bail with ERR_HW_MISMATCH if the ID is wrong
//        - src/st25r3916.cpp:241-264 -- OscOn(): set en, wait for the
//          oscillator to stabilise, then require aux_display.osc_ok
//        - src/st25r3916.cpp:611-637 -- CheckChipID(): compares only the
//          MASKED ic_type field, and returns ic_rev separately as data
//
// [M5] The unit itself. docs.m5stack.com/en/unit/Unit_NFC (retrieved
//      2026-08-18): chip "ST25R3916-AQWT", "I2C @0x50 (100K / 400K)", and a
//      HY2.0-4P pinmap of exactly four wires -- black GND, red 5V, yellow
//      SDA, white SCL. This corroborates 0x50 independently of [DS], and it
//      establishes the constraint that shapes this whole driver: THERE IS NO
//      IRQ LINE. The ST25R3916 has a dedicated active-high IRQ output pin and
//      RFAL's flows are built around waiting on it ([REF] OscOn() waits for
//      ST25R3916_IRQ_MASK_OSC; ExecuteCommandAndGetResult() waits for
//      IRQ_MASK_DCT). None of that is available through a 4-pin connector, so
//      every wait in this file is a POLLED read of a status register instead.
//      Later NFC tasks must assume the same: no RFAL flow that blocks on an
//      interrupt can be used here unmodified.
//
// [EH] Corroboration only, cited for honesty about what was read, NOT used as
//      the source of any constant: wilson-elechouse/ST25R3916 (an ESP32 port
//      of [REF] that adds an Arduino-Wire I2C back end). Its
//      st25r3916_com.cpp read path is
//      beginTransmission / write(mode byte) / endTransmission(false) /
//      requestFrom -- i.e. the repeated-START framing [DS] Figure 26
//      documents, confirmed to be what a real Arduino I2C master does against
//      a real one of these chips. Every value it uses is identical to [REF]'s
//      because it is a fork of it.
//
// ---------------------------------------------------------------------------
// KNOWN-BAD COMMENT IN THE UPSTREAM REFERENCE, recorded so nobody "corrects"
// this file to match it: [REF]'s src/st25r3916_com.h:225 annotates
// REG_IC_IDENTITY as "Chip Id: 0 for old silicon, v2 silicon: 0x09". That is
// a leftover from the older ST25R3911 driver this library descends from. It
// contradicts both [DS] Table 117 AND the ic_type_st25r3916 (5U<<3 = 0x28)
// define sitting 900 lines below it in the same header, and [REF]'s own
// CheckChipID() ignores it. The value to expect is 0x28 in the type field,
// NOT 0x09.
// ===========================================================================

namespace {

// --- I2C transport ---------------------------------------------------------
// [M5]/[DS] Sec 4.3.4 agree on 0x50; TAB5_NFC_I2C_ADDR in pins_config.h is the
// same value, already confirmed on this exact hardware by Phase 1's PORT.A
// bus census (it is the address that answered). Use the project constant so
// there is one definition, and static_assert that it still matches what the
// datasheet says this driver is written for -- if someone ever repoints
// TAB5_NFC_I2C_ADDR at the retired 0x24 PN532 candidate, this file should
// fail to build rather than silently talk PN532-shaped nonsense to nothing.
constexpr uint8_t kI2cAddr = TAB5_NFC_I2C_ADDR;
static_assert(kI2cAddr == 0x50,
              "ST25R3916's I2C address is 50h per DS12484 Rev 3 Sec 4.3.4; "
              "TAB5_NFC_I2C_ADDR no longer matches the chip this driver is for");

// [DS] Table 11 / Sec 4.3.4: the first two bits of the byte following the I2C
// slave address select the operation. The I2C interface reuses the SPI mode
// byte verbatim ("the same Register Write mode byte as for SPI").
//   00 A5..A0 -> register write        01 A5..A0 -> register read
//   11 C5..C0 -> direct command
// Cross-checked against [REF] st25r3916_com.cpp:51-53.
constexpr uint8_t kModeWrite = 0U << 6; // 0x00
constexpr uint8_t kModeRead  = 1U << 6; // 0x40
constexpr uint8_t kModeCmd   = 3U << 6; // 0xC0

// Highest register address expressible in the 6 address bits of a space-A
// mode byte. Anything above this would silently alias.
constexpr uint8_t kMaxSpaceARegister = 0x3FU;

// --- Registers ([DS] Tables 21, 98, 117; [REF] st25r3916_com.h) ------------
constexpr uint8_t kRegOpControl  = 0x02U; // RW  Operation control
constexpr uint8_t kRegAuxDisplay = 0x31U; // R   Auxiliary display
constexpr uint8_t kRegIcIdentity = 0x3FU; // R   IC identity

constexpr uint8_t kOpControlEn   = 1U << 7; // enable oscillator + regulators
constexpr uint8_t kOpControlRxEn = 1U << 6; // enable Rx operation
constexpr uint8_t kOpControlTxEn = 1U << 3; // enable Tx operation (RF field)

constexpr uint8_t kAuxDisplayOscOk = 1U << 4; // 1 = Xtal oscillation is stable

// --- Direct commands ([DS] Table 13; [REF] st25r3916.h:87-88) --------------
// These bytes are complete as listed in the datasheet -- 0xC1 is already
// 11_000001, i.e. the mode bits are baked into the tabulated code.
constexpr uint8_t kCmdSetDefault = 0xC1U; // power-up state ([DS] Sec 4.4.1)
// (0xC2 "Stop all activities" is deliberately NOT defined here yet: nothing in
// this task issues it, and an unused constant is one more thing to keep true.
// Set Default already performs a Stop internally per [DS] Sec 4.4.1.)

// [REF] st25r3916.h:138 -- "Max timeout for Oscillator to get stable
// DS: 700us", with the driver itself allowing 10 ms. We have no IRQ line
// ([M5]), so this is a polling budget rather than an interrupt wait.
constexpr uint32_t kOscStableTimeoutMs = 10U;

// Settle delay after Set Default. [DS] Sec 4.4.1 documents no completion IRQ
// for this command, and Table 13 marks it "Interrupt after termination: No",
// so there is nothing to wait ON -- but it does reset every register, so give
// the chip a moment before reading one back. 1 ms is far more than a register
// reset needs and costs nothing at init time.
constexpr uint32_t kSetDefaultSettleMs = 1U;

bool s_initialized = false;

// Read one space-A register.
//
// Framing ([DS] Sec 4.3.4 + Figure 26's spelled-out legend):
//   S, slave addr+W, <01 A5..A0>, Sr, slave addr+R, data, NAK, P
//
// ARDUINO-ESP32 SPECIFIC, verified in this framework's own source rather than
// assumed (packages/framework-arduinoespressif32/libraries/Wire/src/Wire.cpp):
//   * endTransmission(false) does NOT transmit anything and does NOT report
//     bus errors -- it only sets an internal `nonStop` flag and unconditionally
//     returns 0 (Wire.cpp:445-476). Checking its return value would be
//     checking a constant.
//   * It also deliberately KEEPS the bus lock held, handing it to the
//     following requestFrom(). Returning early between the two would leak that
//     lock and wedge Wire1 for every other caller. Hence: once the deferred
//     path is entered, requestFrom() must always run.
//   * requestFrom() with `nonStop` set issues the real combined
//     write-then-repeated-START-read via i2cWriteReadNonStop() and returns the
//     number of bytes actually received (Wire.cpp:517-534). THAT return value
//     is the only place a NACK from the chip becomes visible, so it is what
//     this function tests.
bool readRegisterRaw(uint8_t reg, uint8_t *val_out) {
    if (val_out == nullptr || reg > kMaxSpaceARegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    if (Wire1.write(static_cast<uint8_t>(reg | kModeRead)) != 1) {
        // Nothing has gone out on the wire yet; close the transaction the
        // normal way so the lock is released.
        Wire1.endTransmission(true);
        return false;
    }
    Wire1.endTransmission(false); // deferred; see note above -- return value
                                  // is a constant 0 on this core, not a status
    if (Wire1.requestFrom(kI2cAddr, static_cast<size_t>(1)) != 1) {
        return false;
    }
    if (!Wire1.available()) {
        return false;
    }
    *val_out = static_cast<uint8_t>(Wire1.read());
    return true;
}

} // namespace

namespace St25r3916 {

bool read_register(uint8_t reg, uint8_t *val_out) {
    nfc_ensure_external_i2c_begun();
    return readRegisterRaw(reg, val_out);
}

bool write_register(uint8_t reg, uint8_t val) {
    nfc_ensure_external_i2c_begun();
    if (reg > kMaxSpaceARegister) {
        return false;
    }
    // [DS] Sec 4.3.4 / Figure 20 "Writing a single register": S, slave addr+W,
    // <00 A5..A0>, data, P.
    Wire1.beginTransmission(kI2cAddr);
    // Checked for the same reason readRegisterRaw() checks: consistency, and
    // because a short write would otherwise send a mode byte with no data and
    // report success. In practice 2 bytes never fail to enqueue into the
    // 128-byte TX buffer, so this is belt-and-braces rather than a live risk.
    const bool queued = (Wire1.write(static_cast<uint8_t>(reg | kModeWrite)) == 1) &&
                        (Wire1.write(val) == 1);
    const bool sent = (Wire1.endTransmission(true) == 0); // always run: releases
                                                          // the Wire1 lock
    return queued && sent;
}

bool execute_command(uint8_t cmd) {
    nfc_ensure_external_i2c_begun();
    // [DS] Sec 4.3.4 "Direct command mode" / Figure 25: S, slave addr+W,
    // <11 C5..C0>, P. The OR with kModeCmd is a belt-and-braces no-op for the
    // tabulated codes (which already carry the mode bits) and is exactly what
    // [REF] st25r3916ExecuteCommand() does.
    Wire1.beginTransmission(kI2cAddr);
    Wire1.write(static_cast<uint8_t>(cmd | kModeCmd));
    return Wire1.endTransmission(true) == 0;
}

bool read_chip_id(uint8_t *id_out) {
    if (id_out == nullptr) {
        return false;
    }
    // Deliberately does NOT compare against the expected value. Per this
    // task's brief the caller compares, so a mismatch is visible in the log
    // rather than collapsed into a bare false that cannot be told apart from
    // an I2C failure.
    return read_register(kRegIcIdentity, id_out);
}

bool init() {
    if (s_initialized) {
        return true;
    }

    // Reuses hal/nfc_pn532.cpp's bring-up: asserts EXT_5V_EN on the internal-
    // bus IO-expander (PORT.A is UNPOWERED at reset -- see that file's header)
    // and begins Wire1 at TAB5_EXTERNAL_I2C_FREQ_HZ. Idempotent.
    nfc_ensure_external_i2c_begun();

    // [REF] st25r3916Initialize() step 1 ([REF] st25r3916.cpp:117): put the
    // chip in its power-up state before touching anything else. [DS] Sec 4.4.1:
    // this performs Stop all activities, resets all registers to default, and
    // clears all collision bits.
    if (!execute_command(kCmdSetDefault)) {
        Serial.println("quarky-tab5: [st25r3916] Set Default (0xC1) NACKed -- "
                       "nothing is answering at I2C 0x50 on Wire1");
        return false;
    }
    delay(kSetDefaultSettleMs);

    // [REF] st25r3916Initialize() step 2 ([REF] st25r3916.cpp:123-125):
    // CheckChipID, and refuse to proceed on a mismatch (it returns
    // ERR_HW_MISMATCH). Same policy here.
    uint8_t id = 0;
    if (!read_chip_id(&id)) {
        Serial.println("quarky-tab5: [st25r3916] IC identity read (reg 0x3F) failed");
        return false;
    }

    const uint8_t type = id & kIcIdentityIcTypeMask;
    const uint8_t rev  = id & kIcIdentityIcRevMask;
    Serial.printf("quarky-tab5: [st25r3916] IC identity (reg 0x3F) = 0x%02X "
                  "(ic_type=0x%02X, ic_rev=%u)\n",
                  id, type, (unsigned)rev);

    if (type == kIcTypeSt25r3916) {
        Serial.printf("quarky-tab5: [st25r3916] ic_type 0x%02X == ST25R3916/7 "
                      "(DS12484 Rev 3 Table 117: 00101b) -- MATCH\n", type);
    } else if (type == kIcTypeSt25r3916B) {
        // Not a failure: the B variant is the same programming model for
        // everything this driver does. Called out because [REF]'s own
        // CheckChipID additionally requires ic_rev >= 1 on the B part, and
        // because later tasks that use RC calibration must branch on it.
        Serial.printf("quarky-tab5: [st25r3916] ic_type 0x%02X == ST25R3916B "
                      "(not the -AQWT the M5Stack docs list) -- accepted, but "
                      "note the B variant needs the RC-calibration step\n", type);
    } else {
        Serial.printf("quarky-tab5: [st25r3916] ic_type 0x%02X is NEITHER "
                      "ST25R3916 (0x%02X) nor ST25R3916B (0x%02X) -- refusing "
                      "to drive this part\n",
                      type, kIcTypeSt25r3916, kIcTypeSt25r3916B);
        return false;
    }

    s_initialized = true;
    return true;
}

bool field_on() {
    if (!init()) {
        return false;
    }

    // [DS] Sec 4.2.13 "Reader operation": "The Ready mode has to be entered by
    // setting the bit en of the Operation control register. In this mode the
    // oscillator is started and the regulators are enabled. When the
    // oscillator operation is stable an interrupt is sent and bit osc_ok
    // indicates it." We have no IRQ line ([M5]), so we poll osc_ok.
    //
    // [REF] OscOn() (st25r3916.cpp:241-264) does the same three things in the
    // same order -- check en, set en, then REQUIRE aux_display.osc_ok before
    // reporting success -- differing only in that it sleeps on the OSC
    // interrupt where this polls.
    uint8_t op = 0;
    if (!read_register(kRegOpControl, &op)) {
        return false;
    }
    if ((op & kOpControlEn) == 0) {
        if (!write_register(kRegOpControl, static_cast<uint8_t>(op | kOpControlEn))) {
            return false;
        }
    }

    bool osc_ok = false;
    // Track whether the polling loop ever managed to READ the register at all.
    // Without this, "the oscillator never stabilised" and "I2C was dead for the
    // whole 10 ms" produce the identical message, and they call for opposite
    // investigations (a crystal/analog problem vs. a bus problem -- and on this
    // board a torn-down Wire1 is a live possibility, see hal/rf433_gpio.cpp's
    // GPIO53 note).
    bool aux_read_ok = false;
    uint8_t aux = 0;
    const uint32_t deadline = millis() + kOscStableTimeoutMs;
    do {
        if (read_register(kRegAuxDisplay, &aux)) {
            aux_read_ok = true;
            if ((aux & kAuxDisplayOscOk) != 0) {
                osc_ok = true;
                break;
            }
        }
    } while ((int32_t)(millis() - deadline) < 0);

    if (!osc_ok) {
        // [REF] OscOn() returns ERR_SYSTEM in exactly this case. Do not press
        // on and enable the transmitter against an unstable carrier.
        if (aux_read_ok) {
            Serial.printf("quarky-tab5: [st25r3916] oscillator did not report "
                          "osc_ok within %u ms -- aux_display (0x31) last read "
                          "0x%02X, bit 4 clear. The chip is talking; the "
                          "crystal is not stabilising. Field NOT enabled\n",
                          (unsigned)kOscStableTimeoutMs, aux);
        } else {
            Serial.printf("quarky-tab5: [st25r3916] could not read aux_display "
                          "(0x31) even once in %u ms -- this is an I2C failure, "
                          "NOT an oscillator problem. Field NOT enabled\n",
                          (unsigned)kOscStableTimeoutMs);
        }
        return false;
    }

    // [DS] Sec 4.2.13: "Before sending any command to a transponder the
    // transmitter and receiver have to be enabled by setting the bits rx_en
    // and tx_en." tx_en (Table 21 bit 3) is the bit that actually turns the
    // RF field on. This is [REF]'s st25r3916TxRxOn() (st25r3916.h:148).
    //
    // WHY NOT the C8 "NFC initial field ON" direct command ([DS] Table 13):
    // that one performs Initial RF Collision Avoidance first and signals
    // completion via an interrupt -- which this 4-wire unit cannot deliver.
    // [REF] likewise uses the OP_CONTROL bits, not C8, for plain reader
    // field-on; C8 belongs to the NFCIP-1 peer-to-peer flows.
    if (!read_register(kRegOpControl, &op)) {
        return false;
    }
    return write_register(
        kRegOpControl,
        static_cast<uint8_t>(op | kOpControlRxEn | kOpControlTxEn));
}

void field_off() {
    if (!s_initialized) {
        return;
    }
    // [REF] st25r3916TxRxOff() (st25r3916.h:150) / Deinitialize()
    // (st25r3916.cpp:~330): clear rx_en and tx_en, and deliberately LEAVE the
    // oscillator (en) running -- "Disable Tx and Rx, Keep OSC On". Restarting
    // the crystal costs ~700 us ([REF] st25r3916.h:138) every time the field
    // is toggled, which a scan loop does constantly.
    uint8_t op = 0;
    if (!read_register(kRegOpControl, &op)) {
        return;
    }
    write_register(kRegOpControl,
                   static_cast<uint8_t>(op & ~(kOpControlRxEn | kOpControlTxEn)));
}

} // namespace St25r3916
