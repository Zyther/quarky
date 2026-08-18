#pragma once
#include <cstdint>

// Real ST25R3916 register-level driver for the Tab5's NFC unit (I2C 0x50 on
// Wire1 -- see hal/nfc_pn532.cpp's header comment for the chip-identity
// research this continues). NOT PN532 framing -- despite the HAL class
// being named NfcPN532 for interface-contract reasons, this file's protocol
// is entirely ST25R3916's own, built from ST's real datasheet/reference
// driver (cited in st25r3916_driver.cpp), not ported from any donor project.
namespace St25r3916 {

// --- IC identity register decoding -----------------------------------------
// ST25R3916/7 datasheet DS12484 Rev 3, Table 117 "IC identity register"
// (register space A, address 3Fh, type R):
//   bits 7..3  ic_type<4:0>  -- 00101b identifies ST25R3916/7
//   bits 2..0  ic_rev<2:0>   -- 010b is "rev 3.1" (silicon-revision dependent)
// The type field is the identity check; the revision field is informational
// and MUST NOT be part of a pass/fail comparison, since a different silicon
// revision of the very same part reports a different value there.
constexpr uint8_t kIcIdentityIcTypeMask  = 0x1FU << 3; // 0xF8
constexpr uint8_t kIcTypeSt25r3916       = 5U << 3;    // 0x28 -- 00101b
constexpr uint8_t kIcTypeSt25r3916B      = 6U << 3;    // 0x30 -- 00110b
constexpr uint8_t kIcIdentityIcRevMask   = 0x07U;

// Brings the chip out of reset / into a known register state and confirms
// I2C communication works via read_chip_id() internally. Must be called
// before field_on()/read_chip_id() are meaningful. Idempotent.
bool init();

// Reads the chip's IC Identity register. Returns false on any I2C failure.
// *id_out receives the raw register value regardless of whether it matches
// the datasheet's documented ST25R3916 identity value -- callers (this
// task's own verification step) compare it themselves so a mismatch is
// visible rather than silently swallowed.
bool read_chip_id(uint8_t *id_out);

// Enables/disables the RF field (required before any tag can be detected --
// analogous to a PN532's RFConfiguration + field-on sequence, but this
// chip's own real command for it, per the cited datasheet section).
bool field_on();
void field_off();

// --- Additive low-level access (not in the Task 2 brief's minimum contract)
// Exposed because this file is specified as "the foundation every later
// NFC-unit feature task builds its actual tag-protocol logic on top of", and
// every one of those tasks needs register and direct-command access. They are
// the exact primitives init()/read_chip_id()/field_on() are themselves built
// from, so exposing them adds no new protocol surface -- only reuse.
//
// `reg` addresses register space A only (0x00-0x3F). Register space B needs
// the FBh prefix byte (datasheet Figure 26) and is deliberately not supported
// yet -- no space-B register is needed by anything in this phase, and an
// untested prefix path would be exactly the kind of unverified guess this
// task exists to avoid.
bool read_register(uint8_t reg, uint8_t *val_out);
bool write_register(uint8_t reg, uint8_t val);

// Sends one direct command. `cmd` is the complete command byte from the
// datasheet's Table 13 (those codes already include the '11' direct-command
// mode bits -- e.g. Set Default is 0xC1, not 0x01).
bool execute_command(uint8_t cmd);

} // namespace St25r3916
