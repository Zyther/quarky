#include "ws1850s_driver.h"
#include "../../hal/nfc_pn532.h"
#include "../../../boards/tab5/pins_config.h"
#include <Wire.h>
#include <Arduino.h>

// ===========================================================================
// THE FINDING THIS FILE EXISTS TO RECORD: the RFID2 unit is NOT PN532-
// protocol silicon. It is MFRC522/PN512-protocol silicon.
//
// The Phase 3 plan's Task 3 brief, and the Phase 3 spec it derives from, both
// state that WS1850S is "PN532-register-compatible" and instruct this file to
// implement the PN532 host-controller frame format (preamble 00, start code
// 00 FF, TFI D4/D5, LEN/LCS/DCS, GetFirmwareVersion = 0x02). Every real
// source consulted for this task contradicts that, and the contradiction is
// unanimous:
//
//   [B]  BRUCE, this project's own cited donor, has a module named after this
//        exact unit: src/modules/rfid/RFID2.cpp ("Read and Write RFID tags
//        using RFID2 module from M5Stack"). Line 17 defines
//        RFID2_I2C_ADDRESS 0x28; line 22 constructs
//        `MFRC522DriverI2C{RFID2_I2C_ADDRESS, sda, scl}`; line 32 calls
//        `mfrc522.PCD_Init()`; line 34 checks `mfrc522.PCD_GetVersion()`.
//        Bruce ALSO has real PN532 code (src/modules/rfid/PN532.cpp) and it
//        is a different module talking to a different device -- PN532.cpp:284
//        probes PN532_I2C_ADDRESS, which the Adafruit library it uses defines
//        as (0x48 >> 1) = 0x24, not 0x28.
//
//   [U]  UNIGEEK, likewise a cited donor, likewise has both, likewise
//        separate: screens/module/MFRC522Screen.cpp:13 sets
//        `I2C_ADDRESS = 0x28`, :211 calls `PCD_Init()`, and its own title()
//        returns the string "M5 RFID 2" (:28) -- i.e. UniGeek's driver FOR
//        THIS UNIT is an MFRC522 driver (MFRC522_I2C, from
//        github.com/kkloesener/MFRC522_I2C per its platformio.ini:64).
//        Its PN532 screens (PN532I2cScreen.cpp) target 0x24 and display
//        "Transport: I2C (0x24)" (:415).
//
//   [M5] M5STACK'S OWN CURRENT LIBRARY for this unit, github.com/m5stack/
//        M5Unit-RFID (branch main, read 2026-08-19 via the GitHub API), is
//        decisive about the family: src/unit/unit_WS1850S.hpp declares
//        `class UnitWS1850S : public UnitMFRC522` with the doc comment
//        "Functionally compatible with MFRC522. Supports NFC-A (always) and
//        NFC-B (if PN512-compatible silicon)", and src/unit/pn512_register.hpp
//        states "WS1850S is PN512-compatible silicon" / "NXP PN512 extends the
//        MFRC522 register map with NFC-B / NFC-F support".
//
//        >>> PN512, NOT PN532. <<< That one-digit difference is almost
//        certainly the origin of the brief's premise: PN512 IS an
//        MFRC522-family register-mapped part, PN532 is a firmware-driven
//        host-controller part with a completely different interface. They are
//        not interchangeable and no amount of framing adaptation bridges them.
//
//   [DS] NXP's own MFRC522 datasheet (rev 3.9, 27 April 2016,
//        https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf, downloaded and
//        read 2026-08-19) supplies the primary-source framing that both donor
//        implementations are implementations OF -- see the citations at each
//        function below. Sec 2 "General description": "The MFRC522 is a highly
//        integrated reader/writer IC for contactless communication at
//        13.56 MHz."
//
//   [M5D] M5Stack's product documentation for the unit itself,
//        docs.m5stack.com/en/unit/rfid2 (retrieved 2026-08-19): "Read/Write
//        IC: WS1850S", "Operating Freq: 13.56MHz", "Communication Interface:
//        I2C @0x28", "Supported Prot: ISO/IEC 14443 Type A/Type B". And for
//        the previous-generation unit, docs.m5stack.com/en/unit/rfid: chip
//        MFRC522, "Protocol: I2C @0x28", same 13.56 MHz -- i.e. M5Stack
//        swapped the read/write IC (RC522 -> WS1850S) at the SAME I2C address
//        with no interface change, which is exactly why both donors' MFRC522
//        code drives both units unmodified.
//
// Consequences, so nobody re-litigates this from the spec text alone:
//   1. There is no "PN532 frame layer" in this file, because sending PN532
//      frames to this chip writes bytes into MFRC522 register 0x00.
//   2. GetFirmwareVersion (PN532 cmd 0x02) has no counterpart here. The
//      equivalent bring-up readback is VersionReg (0x37), which is precisely
//      what BOTH donors use for this unit ([B] RFID2.cpp:34 PCD_GetVersion(),
//      [U] via MFRC522_I2C.cpp:1242 PCD_ReadRegister(VersionReg)).
//   3. The PN532 hypothesis was not merely argued down, it was TESTED. A
//      pn532_frame_probe() shipped in this file's first commit, sending the
//      real GetFirmwareVersion frame the brief asked for
//      (00 00 FF 02 FE D4 02 2A 00) to 0x28, with the framing ported from
//      UniGeek's own hand-rolled PN532 I2C code
//      (screens/module/PN532I2cScreen.cpp:927-960 _nfcSendCmdReadAck, :18-24
//      _nfcReadI2C). On real hardware, 2026-08-19, the unit did not respond to
//      it, while this file's MFRC522 framing read VersionReg (0x37) = 0x15 --
//      stable across a control re-read. The probe was then removed, having
//      served its purpose; git history has it if the question is reopened.
//
// SOURCE MANIFEST for every constant and sequence below:
//   [DS]  NXP MFRC522 datasheet rev 3.9, 27 April 2016,
//         https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf (primary source:
//         register addresses, I2C framing, VersionReg semantics). Downloaded
//         and read 2026-08-19.
//   [MV2] RFID_MFRC522v2 (the library Bruce's RFID2.cpp uses), read 2026-08-19
//         at ~/src/firmware/.pio/libdeps/m5stack-cardputer/RFID_MFRC522v2/src/
//   [MI2] MFRC522_I2C (the library UniGeek's MFRC522Screen.cpp uses), read
//         2026-08-19 at
//         ~/src/unigeek-main/firmware/.pio/libdeps/freenove/MFRC522_I2C/src/
//
// NO IRQ LINE (same constraint Task 2 documented for the NFC unit): the
// HY2.0-4P connector carries GND / 5V / SDA / SCL only. MFRC522 has an IRQ
// pin; it is unreachable here. Every wait in this driver and in every later
// RFID2 feature must be a polled register read (ComIrqReg), never an
// interrupt. MFRC522 flows are polled by default in both donor libraries, so
// this bites less here than it does on the ST25R3916 side -- but it still
// rules out the IRQ-driven variants (e.g. the readMifareClassicIrq example).
// ===========================================================================

namespace {

// [M5D]/[B]/[U] all agree on 0x28, and Phase 1's PORT.A bus census already
// records it in pins_config.h. Use the project constant, and fail the BUILD
// if it is ever repointed, rather than talking MFRC522 framing to whatever
// happens to be at the new address.
constexpr uint8_t kI2cAddr = TAB5_RFID2_I2C_ADDR;
static_assert(kI2cAddr == 0x28,
              "M5Stack Unit RFID2 (WS1850S) is I2C 0x28 per docs.m5stack.com/"
              "en/unit/rfid2 and both donor drivers; TAB5_RFID2_I2C_ADDR no "
              "longer matches the chip this driver is written for");

// --- Registers ([DS] Sec 9.2 Table 20; [MV2] MFRC522Constants.h:10-70) -----
constexpr uint8_t kRegCommand      = 0x01U; // CommandReg    [MV2]:10
constexpr uint8_t kRegMode         = 0x11U; // ModeReg       [MV2]:28
constexpr uint8_t kRegTxMode       = 0x12U; // TxModeReg     [MV2]:29
constexpr uint8_t kRegRxMode       = 0x13U; // RxModeReg     [MV2]:30
constexpr uint8_t kRegTxControl    = 0x14U; // TxControlReg  [MV2]:31
constexpr uint8_t kRegTxASK        = 0x15U; // TxASKReg      [MV2]:32
constexpr uint8_t kRegModWidth     = 0x24U; // ModWidthReg   [MV2]:~44
constexpr uint8_t kRegTMode        = 0x2AU; // TModeReg      [MV2]:55
constexpr uint8_t kRegTPrescaler   = 0x2BU; // TPrescalerReg [MV2]:56
constexpr uint8_t kRegTReloadH     = 0x2CU; // TReloadRegH   [MV2]:57
constexpr uint8_t kRegTReloadL     = 0x2DU; // TReloadRegL   [MV2]:58
constexpr uint8_t kRegVersion      = 0x37U; // VersionReg    [MV2]:70

// Highest address expressible in the 6 register-address bits ([DS] Fig 17
// labels the field "REGISTER ADDRESS [A5:A0]").
constexpr uint8_t kMaxRegister = 0x3FU;

// --- Commands ([DS] Sec 10; [MV2] MFRC522Constants.h:82-92) ----------------
constexpr uint8_t kCmdSoftReset = 0x0FU; // PCD_SoftReset [MV2]:92

// CommandReg bit 4 = PowerDown. Both donors poll it after a soft reset to
// learn when the chip has finished restarting ([MI2] MFRC522_I2C.cpp:239,
// [MV2] MFRC522v2.cpp:158-159).
constexpr uint8_t kCommandPowerDownBit = 1U << 4;

// TxControlReg bits 0/1 = Tx1RFEn / Tx2RFEn, the antenna driver outputs.
// Both donors gate the field with exactly this mask ([MI2]:249-261,
// [MV2]:166-176).
constexpr uint8_t kTxControlAntennaOn = 0x03U;

// --- PCD_Init register programme -------------------------------------------
// Ported value-for-value from [MV2] MFRC522v2.cpp:104-133, which is
// byte-identical to [MI2] MFRC522_I2C.cpp:216-226 except that [MV2]
// additionally resets the baud-rate and ModWidth registers first. The
// donor comments explaining each value are preserved at the use site.
constexpr uint8_t kInitTxMode      = 0x00U;
constexpr uint8_t kInitRxMode      = 0x00U;
constexpr uint8_t kInitModWidth    = 0x26U;
constexpr uint8_t kInitTMode       = 0x80U;
constexpr uint8_t kInitTPrescaler  = 0xA9U;
constexpr uint8_t kInitTReloadH    = 0x03U;
constexpr uint8_t kInitTReloadL    = 0xE8U;
constexpr uint8_t kInitTxASK       = 0x40U;
constexpr uint8_t kInitMode        = 0x3DU;

// [MI2]:210 -- "Give the MFRC522 module some time after power-up so that VDD
// and the internal oscillator can settle" before the soft reset. PORT.A's 5V
// rail is switched on by nfc_ensure_external_i2c_begun(), which already waits
// TAB5_EXT_5V_SETTLE_MS, so this is belt-and-braces on a warm bus and correct
// on a cold one.
constexpr uint32_t kPowerUpSettleMs = 50U;

// [MI2]:236 / [MV2]:151-153 -- "The datasheet does not mention how long the
// SoftReset command takes to complete... Section 8.8.2 says the oscillator
// start-up time is the start up time of the crystal + 37.74us. Let us be
// generous: 50ms." [MV2] additionally bounds the poll at 3 tries; [MI2] spins
// forever. Bounded here, for the same reason Task 1 bounded its capture: an
// unbounded wait on absent hardware is a hang, not a diagnostic.
constexpr uint32_t kResetPollIntervalMs = 50U;
constexpr uint8_t  kResetPollMaxTries   = 3U;

// [MV2]:135 -- "Optional delay of 4ms. Some board do need more time after
// init to be ready".
constexpr uint32_t kPostInitSettleMs = 4U;

bool s_initialized = false;

// --- I2C register access ---------------------------------------------------
//
// [DS] Sec 8.1.4.6 "Register write access": "The first byte of a frame
// indicates the device address according to the I2C-bus rules. The second
// byte indicates the register address followed by up to n-data bytes. In one
// frame all data bytes are written to the same register address."
//
// Note what that last sentence means and how it differs from a typical I2C
// device: there is NO auto-increment. Writing N bytes after one register
// address writes all N to that same register (which is how the FIFO is
// filled). Any later RFID2 feature that assumes a block write walks up
// consecutive registers will be wrong.
//
// Identical in both donors: [MI2] MFRC522_I2C.cpp:38-45,
// [MV2] MFRC522DriverI2C.cpp:22-30.
bool writeRegisterRaw(uint8_t reg, uint8_t val) {
    if (reg > kMaxRegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    const bool queued = (Wire1.write(reg) == 1) && (Wire1.write(val) == 1);
    // endTransmission() always runs even if the queueing failed: it is what
    // releases the Wire1 lock. Same discipline as st25r3916_driver.cpp, and
    // for the same reason -- a leaked lock wedges the bus for every other
    // caller, including the NFC unit sharing it.
    const bool sent = (Wire1.endTransmission(true) == 0);
    return queued && sent;
}

// [DS] Sec 8.1.4.7 "Register read access": "Firstly, a write access to the
// specific register address must be performed... The second byte indicates
// the register address. No data bytes are added... After the write access,
// read access can start. The host sends the device address of the MFRC522.
// In response, the MFRC522 sends the content of the read access register."
//
// IMPORTANT DIFFERENCE FROM THE NFC UNIT'S DRIVER: [DS] Figure 17 draws the
// write cycle terminating in a STOP (P) and the read cycle beginning with its
// own START (S) -- a repeated START is NOT required here, unlike the
// ST25R3916 ([DS12484] Figure 26) where it is. So this uses
// endTransmission(true), not the deferred endTransmission(false) +
// requestFrom() dance st25r3916_driver.cpp needs. Both donors do the same
// ([MI2] MFRC522_I2C.cpp:70-80, [MV2] MFRC522DriverI2C.cpp:52-66); do not
// "harmonise" the two drivers by copying one framing onto the other.
bool readRegisterRaw(uint8_t reg, uint8_t *val_out) {
    if (val_out == nullptr || reg > kMaxRegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    if (Wire1.write(reg) != 1) {
        Wire1.endTransmission(true);
        return false;
    }
    if (Wire1.endTransmission(true) != 0) {
        // The address-write NACKed: nothing is at 0x28. Both donors ignore
        // this status and read anyway, which is how a missing unit turns into
        // a plausible-looking 0x00. Checked here instead.
        return false;
    }
    if (Wire1.requestFrom(kI2cAddr, static_cast<size_t>(1)) != 1) {
        return false;
    }
    if (!Wire1.available()) {
        return false;
    }
    *val_out = static_cast<uint8_t>(Wire1.read());
    return true;
}

// [MI2] MFRC522_I2C.cpp:232-243 / [MV2] MFRC522v2.cpp:145-160.
bool softReset() {
    if (!writeRegisterRaw(kRegCommand, kCmdSoftReset)) {
        return false;
    }
    uint8_t cmd = 0;
    for (uint8_t tries = 0; tries < kResetPollMaxTries; tries++) {
        delay(kResetPollIntervalMs);
        if (!readRegisterRaw(kRegCommand, &cmd)) {
            return false;
        }
        if ((cmd & kCommandPowerDownBit) == 0) {
            return true;
        }
    }
    Serial.printf("quarky-tab5: [ws1850s] chip still reports PowerDown "
                  "(CommandReg 0x01 = 0x%02X, bit 4 set) %u ms after SoftReset "
                  "-- continuing anyway, as the donor drivers do, but expect "
                  "trouble\n",
                  cmd, (unsigned)(kResetPollIntervalMs * kResetPollMaxTries));
    return true; // [MV2] likewise proceeds after its 3 tries.
}

} // namespace

namespace Ws1850sDriver {

const char *version_name(uint8_t version) {
    // [MV2] MFRC522v2.cpp:209-218 PCD_GetVersion() / [MI2] MFRC522_I2C.cpp:
    // 1244-1255 PCD_DumpVersionToSerial(). Every name below is the donors'
    // own, not this project's gloss -- the "Fudan Semiconductor FM17522"
    // attribution in particular is [MI2] MFRC522_I2C.cpp:337's own comment
    // ("case 0x88: // Fudan Semiconductor FM17522 clone"), and the enum names
    // Version_FM17522 / Version_FM17522_1 / Version_FM17522E are [MV2]
    // MFRC522Constants.h:114-116.
    switch (version) {
        case kVersionCounterfeit: return "counterfeit chip (donor libs' label for 0x12)";
        case kVersionFm17522:     return "Fudan Semiconductor FM17522 clone";
        case kVersionFm17522E:    return "FM17522E ([MV2] Version_FM17522E)";
        case kVersionFm17522_1:   return "FM17522 variant 1 ([MV2] Version_FM17522_1)";
        case kVersionMfrc522V0_0: return "MFRC522 v0.0";
        case kVersionMfrc522V1_0: return "MFRC522 v1.0";
        case kVersionMfrc522V2_0: return "MFRC522 v2.0";
        case kVersionCommsFailureLow:
        case kVersionCommsFailureHigh:
            return "COMMUNICATION FAILURE (not a chip version)";
        default: return "not a value the donor libraries name";
    }
}

bool read_register(uint8_t reg, uint8_t *val_out) {
    nfc_ensure_external_i2c_begun();
    return readRegisterRaw(reg, val_out);
}

bool write_register(uint8_t reg, uint8_t val) {
    nfc_ensure_external_i2c_begun();
    return writeRegisterRaw(reg, val);
}

bool get_version(uint8_t *version_out) {
    // [DS] Sec 9.3.4.8 "VersionReg register -- Shows the MFRC522 software
    // version. MFRC522 version 1.0 software version is: 91h. MFRC522 version
    // 2.0 software version is: 92h."
    //
    // Deliberately does NOT compare, for the same reason Task 2's
    // read_chip_id() does not: the caller decides, so an unexpected value is
    // visible in the log rather than collapsed into a false that cannot be
    // told apart from an I2C NACK. That matters more here than it did there,
    // because no source found for this task states what WS1850S reports.
    return read_register(kRegVersion, version_out);
}

bool get_firmware_version(uint8_t out[4]) {
    if (out == nullptr) {
        return false;
    }
    // See the header's SIGNATURE NOTE. out[1..3] are padding, not data: this
    // chip has no PN532-style 4-byte IC/Ver/Rev/Support response, and zeroing
    // them is preferable to leaving stack garbage that a caller might print
    // as if it meant something.
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    return get_version(&out[0]);
}

bool init() {
    if (s_initialized) {
        return true;
    }

    // Reuses hal/nfc_pn532.cpp's bring-up: asserts EXT_5V_EN on the internal-
    // bus IO-expander (PORT.A is UNPOWERED at reset) and begins/recovers
    // Wire1. Idempotent, and it is also the GPIO53 teardown recovery -- see
    // hal/rf433_gpio.cpp for why that is needed at all.
    nfc_ensure_external_i2c_begun();

    // [MI2] MFRC522_I2C.cpp:206-213: settle, then soft reset (this unit has
    // no NRSTPD line broken out to the connector, so the hard-reset branch of
    // both donors' PCD_Init is not applicable -- soft reset is the only path).
    delay(kPowerUpSettleMs);
    if (!softReset()) {
        Serial.println("quarky-tab5: [ws1850s] SoftReset write to CommandReg "
                       "(0x01) failed -- nothing is answering at I2C 0x28 on "
                       "Wire1. Is the RFID2 unit (not the NFC unit, not RF433R) "
                       "the thing plugged into PORT.A?");
        return false;
    }

    // [MV2] MFRC522v2.cpp:104-133, comments quoted from that source.
    //
    // Both donors discard every one of these nine writes' status. This file
    // criticises exactly that habit up in readRegisterRaw() ("both donors
    // ignore it and read anyway, which is how a missing unit turns into a
    // plausible-looking 0x00"), so it would be inconsistent to repeat it here
    // -- the results are accumulated instead. Deliberately accumulated rather
    // than early-returned: if the bus dies partway through, the programme
    // should still be attempted in full so the VersionReg read below reports
    // on the same chip state a donor driver would have produced, and ONE
    // diagnostic covers the whole block instead of nine.
    bool programmed = true;
    programmed &= write_register(kRegTxMode, kInitTxMode);      // reset baud rates
    programmed &= write_register(kRegRxMode, kInitRxMode);
    programmed &= write_register(kRegModWidth, kInitModWidth);  // reset ModWidthReg
    // "When communicating with a PICC we need a timeout if something goes
    // wrong. f_timer = 13.56 MHz / (2*TPreScaler+1)."
    programmed &= write_register(kRegTMode, kInitTMode);        // TAuto=1
    programmed &= write_register(kRegTPrescaler, kInitTPrescaler); // 169 => 40kHz
    programmed &= write_register(kRegTReloadH, kInitTReloadH);  // 0x3E8 => 25 ms
    programmed &= write_register(kRegTReloadL, kInitTReloadL);
    // "Force a 100 % ASK modulation independent of the ModGsPReg setting"
    programmed &= write_register(kRegTxASK, kInitTxASK);
    // "Set the preset value for the CRC coprocessor for the CalcCRC command
    // to 0x6363 (ISO 14443-3 part 6.2.4)"
    programmed &= write_register(kRegMode, kInitMode);
    if (!programmed) {
        // Not fatal on its own -- the VersionReg check below is the real
        // gate, and a chip that answers it is worth reporting on. But a
        // partially-programmed PCD gives wrong timeouts and wrong modulation
        // rather than an obvious failure, so it must not pass silently.
        Serial.println("quarky-tab5: [ws1850s] at least one PCD_Init register "
                       "write NACKed -- the chip is only PARTIALLY configured. "
                       "Timer/ASK/CRC settings may not be what this driver "
                       "believes. Treat any tag-level result after this as "
                       "suspect.");
    }

    // Both donors end PCD_Init with the antenna ON ([MV2]:133, [MI2]:226).
    // Kept, so init() leaves the chip in the state every donor flow that
    // follows expects -- field_off() is available for a caller that wants the
    // field down.
    if (!field_on()) {
        Serial.println("quarky-tab5: [ws1850s] antenna enable (TxControlReg "
                       "0x14) failed during init");
        return false;
    }

    delay(kPostInitSettleMs);

    uint8_t version = 0;
    if (!get_version(&version)) {
        Serial.println("quarky-tab5: [ws1850s] VersionReg (0x37) read failed");
        return false;
    }
    Serial.printf("quarky-tab5: [ws1850s] VersionReg (0x37) = 0x%02X -- %s\n",
                  version, version_name(version));

    // The ONLY hard failure condition, and it is the donors' own test, not an
    // invented expected value: [MI2] MFRC522_I2C.cpp:1256-1257 -- "When 0x00
    // or 0xFF is returned, communication probably failed". Anything else is
    // accepted and logged, because no source consulted for this task
    // documents what a WS1850S reports here, and rejecting an undocumented
    // but plausible value would be fabricating a spec.
    if (version == kVersionCommsFailureLow || version == kVersionCommsFailureHigh) {
        Serial.printf("quarky-tab5: [ws1850s] VersionReg 0x%02X means the BUS "
                      "answered, not the chip (a floating/stuck SDA reads as "
                      "0x00 or 0xFF). Treating init() as FAILED.\n", version);
        return false;
    }

    s_initialized = true;
    return true;
}

bool field_on() {
    nfc_ensure_external_i2c_begun();
    // [DS] Sec 9.3.2.5 TxControlReg (address 14h), bits Tx1RFEn/Tx2RFEn:
    // "controls the logical behavior of the antenna driver pins TX1 and TX2".
    // After a reset these are disabled ([MV2]:164 "After a reset these pins
    // are disabled").
    //
    // Read-modify-write with the "already on?" short-circuit both donors use
    // verbatim ([MI2] MFRC522_I2C.cpp:249-254, [MV2] MFRC522v2.cpp:166-169).
    uint8_t value = 0;
    if (!readRegisterRaw(kRegTxControl, &value)) {
        return false;
    }
    if ((value & kTxControlAntennaOn) == kTxControlAntennaOn) {
        return true; // already on; do not rewrite
    }
    return writeRegisterRaw(kRegTxControl,
                            static_cast<uint8_t>(value | kTxControlAntennaOn));
}

void field_off() {
    nfc_ensure_external_i2c_begun();
    // [MI2] MFRC522_I2C.cpp:259-261 / [MV2] MFRC522v2.cpp:174-176:
    // PCD_ClearRegisterBitMask(TxControlReg, 0x03).
    uint8_t value = 0;
    if (!readRegisterRaw(kRegTxControl, &value)) {
        return;
    }
    writeRegisterRaw(kRegTxControl,
                     static_cast<uint8_t>(value & ~kTxControlAntennaOn));
}

} // namespace Ws1850sDriver
