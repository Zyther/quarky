#pragma once
#include <cstdint>

// ===========================================================================
// Real low-level driver for the Tab5's RFID2 unit (WS1850S, I2C 0x28 on
// Wire1).
//
// !! READ THIS BEFORE CHANGING ANYTHING HERE !!
//
// This task's brief specified a PN532 host-controller frame layer (preamble
// 00, start code 00 FF, TFI D4/D5, GetFirmwareVersion command 0x02). THAT
// PREMISE IS WRONG FOR THIS CHIP, and it was refuted by reading the real
// donor source the brief itself pointed at, not by opinion:
//
//   * BOTH donor projects drive the M5Stack RFID2 unit at I2C 0x28 with the
//     MFRC522 register interface, NOT with PN532 frames. Bruce's
//     src/modules/rfid/RFID2.cpp -- the file literally named after this unit
//     -- constructs an MFRC522DriverI2C at 0x28. UniGeek's
//     screens/module/MFRC522Screen.cpp (its title() returns "M5 RFID 2")
//     constructs an MFRC522_I2C at 0x28. Both projects ALSO have real PN532
//     code, and it is separate, and it talks to a different device at 0x24.
//   * M5Stack's own current library for this unit derives UnitWS1850S from
//     UnitMFRC522 and documents WS1850S as "PN512-compatible silicon".
//     PN512, not PN532 -- an easily-misread one-digit difference, and the
//     most likely origin of the assumption this file had to overturn.
//
// So the protocol implemented below is the MFRC522/PN512 register interface,
// ported from those real donor sources and cross-checked against NXP's own
// MFRC522 datasheet. See ws1850s_driver.cpp's header for the full source
// manifest with file/line citations.
//
// CONFIRMED ON REAL HARDWARE 2026-08-19 (RFID2 unit on PORT.A): this driver's
// MFRC522 framing reads VersionReg (0x37) = 0x15, stable across a control
// re-read, while a real PN532 GetFirmwareVersion frame sent to the same
// address got no response. The correction above is therefore verified at
// register level, not merely argued from donor source.
//
// Bus access reuses hal/nfc_pn532.cpp's nfc_ensure_external_i2c_begun(),
// which owns the PORT.A EXT_5V_EN power gate and the GPIO53 teardown
// recovery. Do not re-derive bus bring-up here.
// ===========================================================================
namespace Ws1850sDriver {

// --- Version register decoding ---------------------------------------------
// MFRC522 datasheet (NXP, rev 3.9) Sec 9.3.4.8: VersionReg (address 37h)
// "Shows the MFRC522 software version"; 91h = version 1.0, 92h = version 2.0.
// The donor libraries additionally recognise several clone/derivative values.
//
// NO EXPECTED VALUE IS ASSERTED FOR WS1850S. Unlike Task 2's ST25R3916 (whose
// datasheet documents the exact IC-identity value to expect), no public
// WiseSun document found for this task states what WS1850S reports here.
// Inventing one would be exactly the fabrication this project forbids, so the
// pass/fail test is the donor libraries' own: a read that returns 0x00 or
// 0xFF means the bus, not the chip, answered (see kVersionCommsFailure*).
//
// OBSERVED on this project's hardware, 2026-08-19: 0x15 -- which is NOT one of
// the values any donor library names (see the constants below). That is a
// single observation, not a datasheet fact: it is worth knowing and worth
// logging, but do NOT turn it into a required value. A different production
// run of the same unit reporting something else would be a pass, not a fault.
constexpr uint8_t kVersionCommsFailureLow  = 0x00U; // no device / SDA stuck low
constexpr uint8_t kVersionCommsFailureHigh = 0xFFU; // no device / bus pulled up

// Values the donor libraries name. Listed for LOGGING only -- a WS1850S
// reporting something outside this list is not a failure, it is data.
constexpr uint8_t kVersionCounterfeit = 0x12U;
constexpr uint8_t kVersionFm17522     = 0x88U;
constexpr uint8_t kVersionFm17522E    = 0x89U;
constexpr uint8_t kVersionFm17522_1   = 0xB2U;
constexpr uint8_t kVersionMfrc522V0_0 = 0x90U;
constexpr uint8_t kVersionMfrc522V1_0 = 0x91U;
constexpr uint8_t kVersionMfrc522V2_0 = 0x92U;

// Human-readable name for a VersionReg value, or "unknown to the donor
// libraries" for anything unlisted. Never returns nullptr.
const char *version_name(uint8_t version);

// Brings the chip into a known state: soft reset, then the donor libraries'
// PCD_Init register programme (baud rates, timer, 100% ASK, CRC preset), then
// antenna on. Confirms communication by reading VersionReg and refuses only
// on the 0x00/0xFF comms-failure values. Idempotent.
bool init();

// The bring-up readback this task exists to perform.
//
// SIGNATURE NOTE: the 4-byte out[] is the shape the Phase 3 plan specified for
// a PN532 GetFirmwareVersion response (IC, Ver, Rev, Support). This silicon
// has no such 4-byte response -- its whole version identity is the single
// VersionReg byte. The signature is kept so the plan's stated interface still
// compiles, and filled as:
//     out[0] = VersionReg (0x37) raw value
//     out[1..3] = 0        (padding; these fields do not exist on this chip)
// New code should prefer get_version() below, which does not pretend
// otherwise. Returns false on any I2C failure.
bool get_firmware_version(uint8_t out[4]);

// The honest single-byte form: reads VersionReg (0x37). Does NOT compare
// against anything -- the caller decides, so a surprising value shows up in
// the log instead of collapsing into a bare false that cannot be told apart
// from an I2C NACK.
bool get_version(uint8_t *version_out);

// (A pn532_frame_probe() lived here between 2026-08-19's two commits: it sent
// a real PN532 GetFirmwareVersion frame to 0x28 so the plan's premise could be
// falsified on hardware rather than only in argument. It served its purpose --
// the unit did not respond to it, recorded above -- and was removed once that
// result existed, rather than left as permanently-unreachable success-path
// parsing code. Recover it from git history if the question is ever reopened.)

// Antenna driver (the RF field). MFRC522 TxControlReg (0x14) bits Tx1RFEn /
// Tx2RFEn. After any reset the field is OFF.
bool field_on();
void field_off();

// --- Additive low-level access ---------------------------------------------
// Same rationale as Task 2's ST25R3916 driver: this file is specified as the
// layer every later RFID2 feature builds on, and all of them need register
// access. These are the exact primitives the functions above are built from,
// so exposing them adds no new protocol surface.
//
// `reg` is a 6-bit MFRC522 register address (0x00-0x3F).
bool read_register(uint8_t reg, uint8_t *val_out);
bool write_register(uint8_t reg, uint8_t val);

} // namespace Ws1850sDriver
