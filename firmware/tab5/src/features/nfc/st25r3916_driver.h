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

// Register space B (added by Phase 3 Task 4's fix round; see the [DS] Figure 26
// / [REF] st25r3916_com.cpp citations in the .cpp). `reg` is the 6-bit address
// WITHIN space B (0x00-0x3F) -- do NOT pre-OR the 0x40 space-B marker that
// [REF]'s own ST25R3916_SPACE_B constant uses, this API keeps the two spaces in
// separate functions instead of multiplexing one address argument.
bool read_register_b(uint8_t reg, uint8_t *val_out);
bool write_register_b(uint8_t reg, uint8_t val);

// --- ISO14443-A / NFC-A polled reader --------------------------------------
// Added by Phase 3 Task 4's fix round to replace an RFAL-based tag-read path
// that could never have worked on this hardware: ST's RFAL I2C driver requires
// a wired IRQ pin (it returns ERR_PARAM from its constructor without one and
// gates every interrupt read on digitalRead(int_pin)), and the Tab5's HY2.0
// PORT.A connector is GND/5V/SDA/SCL only. Everything below therefore polls
// the IRQ *status registers* (0x1A-0x1D, read-and-clear) that the IRQ pin
// would merely have signalled -- see the .cpp for the exact citations.
//
// Scope, stated honestly: this is a SINGLE-TAG reader. The full ISO14443-3
// bit-frame anticollision loop (walking the UID bit by bit when two tags
// answer at once) is NOT implemented; a collision is detected and reported as
// kCollision instead of being resolved. That covers "present one tag to the
// reader", which is what this phase's tag-read feature does.
struct Iso14443aTag {
    uint8_t atqa[2];  // SENS_RES, as received (LSB first, exactly as on the wire)
    uint8_t uid[10];  // NFCID1, cascade tags stripped
    uint8_t uid_len;  // 4, 7 or 10
    uint8_t sak;      // SEL_RES of the final cascade level
};

enum class NfcaResult : uint8_t {
    kNoTag,          // nothing answered REQA within FDT -- the normal idle result
    kFound,          // *out is filled
    kCollision,      // more than one tag in the field (not resolved -- see above)
    kProtocolError,  // a tag answered but the exchange did not follow ISO14443-3
    kHardwareError,  // I2C/chip failure -- the unit is not usable right now
};

// One-shot bring-up for the NFC-A poller: init(), then field_on() (which
// starts and stabilises the oscillator), then the ST analog/mode register
// programme for NFC-A 106 kb/s, then the 5 ms guard time. field_on() runs
// BEFORE the register programme, not after -- deliberately, since the Mode
// definition register cannot be written until the oscillator reports
// osc_ok (see the .cpp for the citation). Costs ~35 I2C register writes plus
// a ~10 ms oscillator wait and the 5 ms guard time (~30 ms total at this
// bus's 100 kHz) -- call it ONCE when a feature screen starts scanning, never
// per poll() tick.
bool nfca_poller_begin();

// Runs one complete detection pass: WUPA -> anticollision -> SELECT, through
// however many cascade levels the tag's UID needs, then SLP_REQ to park the
// tag in HALT so the next pass's WUPA can wake it again. (WUPA rather than
// REQA is load-bearing, not a preference -- see the comment at its call site
// in the .cpp.) Bounded: each exchange's wait-for-IRQ loop gives up after
// ~25 ms of wall clock regardless of what the chip does; the I2C transaction
// time layered on top of that wait is not itself counted against the bound,
// so the true worst case can run a few ms past ~25 ms. That is still well
// inside a poll() tick's ~50 ms budget with margin. Typical real cost is
// 1-3 ms (no tag) or 6-12 ms (tag found), dominated by I2C, not by RF.
NfcaResult nfca_detect(Iso14443aTag *out);

// Stops the poller: field_off() plus a Stop-all-activities so no timer or
// receive state is left running. Safe to call when begin() was never called.
void nfca_poller_end();

} // namespace St25r3916
