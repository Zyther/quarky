#pragma once
#include <cstddef>
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
//
// `keep_active` (added by Phase 3 Task 13, EMV/APDU reader): when true, the
// trailing SLP_REQ (HALT) that normally ends every successful pass is
// skipped, leaving the tag ACTIVE so a caller can immediately chain an
// ISO14443-4 RATS + APDU exchange onto it (iso14443_4_activate() below
// requires exactly this -- RATS sent to a HALTed tag gets no answer, since
// HALT is a real ISO14443-3 state and only WUPA, not RATS, wakes a tag from
// it). Defaults to false, i.e. the ORIGINAL auto-HALT behavior, so every
// existing caller (nfc_read.cpp is the only one that calls this function
// today; nfc_tag_library.cpp/nfc_mifare_crack.cpp/nfc_amiibo.cpp all operate
// on the separate WS1850S/RFID2 unit and never touch this function or this
// file at all) is unaffected without being touched. A caller that passes
// true is responsible for eventually calling nfca_poller_end() (which turns
// the field off outright -- that always ends the tag's session, HALTed or
// not) rather than relying on a HALT it never sent.
NfcaResult nfca_detect(Iso14443aTag *out, bool keep_active = false);

// Stops the poller: field_off() plus a Stop-all-activities so no timer or
// receive state is left running. Safe to call when begin() was never called.
void nfca_poller_end();

// --- ISO14443-4 (T=CL) activation and single-APDU exchange -----------------
// Added by Phase 3 Task 13 (EMV/APDU reader). Built entirely on the same
// polled-IRQ transceive() primitive nfca_detect() already uses internally --
// see st25r3916_driver.cpp's "ISO14443-4 / EMV APDU exchange" SOURCES section
// for the real RATS/I-block citations (ST's own RFAL ISO-DEP layer,
// ~/src/wilson-elechouse/ST25R3916/NFC-RFAL/src/rfal_isoDep.cpp/.h).
//
// Real call sequence (see features/nfc/nfc_emv_read.cpp for the actual user):
//   nfca_poller_begin() -> nfca_detect(&tag, /*keep_active=*/true) -> a single
//   iso14443_4_activate() -> any number of apdu_transceive() calls -> either
//   nfca_poller_end() (tears the field down, ending the session) or a fresh
//   nfca_detect() for another tag. Single-tag only, matching nfca_detect()'s
//   own documented scope -- there is no multi-target CID/DID addressing here.

// Sends RATS (Request for Answer To Select, 0xE0) and validates the ATS
// response well enough to confirm the tag entered ISO14443-4 (T=CL) protocol
// mode -- NOT a full TA/TB/TC parse (this task's brief explicitly does not
// require one), beyond extracting two real fields it cannot work without:
// FWI from TB (if present), which sizes the per-APDU timeout apdu_transceive()
// uses, and FSCI from T0's low nibble (if present), which is the CARD's own
// declared maximum receivable frame size and therefore decides whether an
// outgoing C-APDU has to be chained -- see iso14443_4_get_card_fsc() and
// apdu_transceive() below. Requires nfca_poller_begin() to
// have already run and a tag to already be ACTIVE (i.e. the most recent
// nfca_detect() call used keep_active=true and returned kFound). Returns
// false on any protocol/timeout/I2C failure, in which case no APDU exchange
// should be attempted.
bool iso14443_4_activate();

// Copies the ATS captured by the most recent successful
// iso14443_4_activate() into `out` (verbatim, CRC-stripped, TL byte first)
// and returns how many bytes were written -- 0 if no activation has succeeded
// since the last one was attempted, or if out/cap are unusable. Added by the
// Flipper ".nfc" export path (nfc_flipper_format.h), which needs the card's
// real T0/TA(1)/TB(1)/TC(1)/T1...Tk interface bytes; activate() itself
// consumes only TB's FWI nibble, so without this the ATS is discarded.
size_t iso14443_4_get_ats(uint8_t *out, size_t cap);

// The card's FSC (Frame Size for proximity Card) in BYTES, as decoded from the
// FSCI nibble of the ATS's T0 by the most recent iso14443_4_activate(): the
// largest frame, INCLUDING the PCB byte and the two CRC bytes, that this card
// said it can receive. Always in [16, 256] -- the ISO14443-4 table's own range,
// clamped at the top exactly as ST's RFAL clamps it (a card declaring the
// ISO14443-3-Amd2 512..4096 codes, or an RFU one, is treated as 256), which is
// also the largest this driver could ever transmit given its 255-byte frame
// buffer. Returns the ISO14443-A 5.2.3 default of 32 when no activation has
// happened yet or the card's ATS carried no T0 byte at all -- i.e. it is never
// 0 and never needs a "did this succeed" check. Exposed mainly for diagnostics
// and for the Flipper ".nfc" export path; apdu_transceive() consults the same
// value internally and callers do not have to.
uint16_t iso14443_4_get_card_fsc();

// Sends one C-APDU wrapped in an ISO14443-4 I-block and returns the unwrapped
// R-APDU in `rx`. Transparently answers S(WTX) waiting-time-extension
// requests (bounded -- see the .cpp) since real EMV cards use these during
// slower operations (GET PROCESSING OPTIONS in particular).
//
// PICC->PCD I-block chaining IS handled (added 2026-08-23 after a real
// Mastercard-style card answered a READ RECORD with PCB 0x12 -- an I-block
// with the chaining bit set -- because its record does not fit this driver's
// declared FSD of 128 bytes): each chained fragment is acknowledged with an
// R(ACK) and its INF field appended, until a final non-chained I-block
// arrives, so a caller still sees exactly ONE logically-complete R-APDU
// regardless of how many frames it took. Bounded in both rounds and total
// size, and sharing (not extending) the same per-call time envelope the
// S(WTX) path already had -- see the .cpp's kMaxChainingRounds /
// kMaxReassembledLen / kMaxApduCallMs.
//
// PCD->PICC I-block chaining IS handled too (added 2026-08-24). The comment
// that used to sit here -- "every C-APDU this project sends is well under 32
// bytes and fits one frame by construction" -- was made false the day before
// by the PDOL-based GET PROCESSING OPTIONS path (nfc_emv_read.cpp's
// build_pdol_gpo()): a real Visa card's own PDOL was 27 bytes, producing a
// 62-byte I-block, and real PDOLs can be larger still. That frame was sent
// without ever being compared against the card's own declared FSC (which the
// driver did not even extract), and the card answered nothing at all -- a
// genuine hardware NRT timeout, reproduced against both a Visa and an older
// Mastercard card, and the standard behavior for a frame that overruns a
// card's declared frame size. So a C-APDU whose frame would exceed
// iso14443_4_get_card_fsc() is now
// split across as many chained I-blocks as it needs, each one acknowledged by
// the card with an R(ACK) before the next is sent, and the caller still passes
// exactly ONE C-APDU regardless. Both directions compose: a single call may
// send a chained command AND receive a chained response. Bounded in fragment
// count and sharing -- not extending -- the same per-call time envelope the
// S(WTX) and receive-chaining paths already use (see the .cpp's
// kMaxTxChainFragments / kMaxApduCallMs). `tx_len` still tops out at 254
// bytes, which is a buffer limit of this driver, not a frame-size one.
//
// `rx_cap` may be up to kMaxReassembledLen (512) -- larger than one frame,
// precisely so a reassembled response fits. Requires iso14443_4_activate() to
// have already succeeded. Returns false (and *rx_len = 0) on any
// protocol/timeout/I2C failure, on a response that would overrun `rx_cap`, or
// on an R-block/S-block response this function does not handle.
bool apdu_transceive(const uint8_t *tx, size_t tx_len,
                     uint8_t *rx, size_t rx_cap, size_t *rx_len);

// --- NFC-A Listen Mode (tag emulation) --------------------------------------
// Added by Phase 3 Task 24. See st25r3916_driver.cpp's "NFC-A Listen Mode"
// SOURCES section for the real register/command citations (ST's own RFAL
// Listen Mode implementation, ~/src/wilson-elechouse/ST25R3916/
// ST25R3916_ELECHOUSE/src/rfal_rfst25r3916.cpp's rfalListenStart()/
// rfalRunListenModeWorker(), translated into this driver's own polled-status-
// register style rather than ported as IRQ-driven code -- same reason and
// same treatment as the reader path above).
//
// Real hardware confirmed (not assumed) to do all of the following before
// this was implemented: the chip has a dedicated "Passive Target
// Anticollision" (PTA) hardware state machine that autonomously answers
// REQA/WUPA and walks the full anticollision/SELECT sequence on its own once
// armed with a UID/ATQA/SAK triple loaded into its Passive Target Memory
// (PT_A) -- firmware does not construct any of those responses itself. Both
// the arming writes (register + PT-memory writes) and the resulting state
// (Passive Target Status register, 0x21) are ordinary polled I2C access, so
// this needs no IRQ pin, exactly like nfca_detect() above.
//
// SCOPE: read-only UID/SAK/ATQA emulation only -- once a reader completes
// SELECT, this reports kSelected and stops there. Answering whatever the
// reader sends next (RATS, READ, etc. -- full memory-content emulation) is
// the plan's own explicitly out-of-scope stretch goal and is NOT
// implemented; see the .cpp for exactly which RFAL Listen Mode states this
// covers (IDLE/READY_A/ACTIVE_A only, no SLEEP_A/second-cascade support).
struct ListenConfig {
    uint8_t uid[10];  // Only the first uid_len bytes are used.
    uint8_t uid_len;  // MUST be 4 or 7 -- Listen Mode's PT memory format (like
                      // RFAL's own rfalLmConfPA) has no 10-byte/triple-cascade
                      // representation. listen_start() rejects anything else.
    uint8_t atqa[2];  // SENS_RES, wire order (LSB first).
    uint8_t sak;      // SEL_RES.
};

enum class ListenState : uint8_t {
    kNotArmed,       // listen_start() not yet called, or listen_stop() was.
    kIdle,           // Armed; the chip's PTA engine is waiting for a reader.
    kSelected,       // A reader completed anticollision+SELECT with our UID
                     // (latched until the reader's field goes away).
    kHardwareError,  // I2C failure while polling -- listen_stop() and retry.
};

// Arms Listen Mode with the given UID/ATQA/SAK. Ends any active reader-path
// session first (nfca_poller_end(), if one was open) and any prior Listen
// Mode session -- the two share REG_MODE/REG_PASSIVE_TARGET on this silicon
// and cannot run concurrently. Returns false on a bad uid_len or any I2C
// failure during the arming sequence, in which case Listen Mode is NOT armed.
bool listen_start(const ListenConfig &cfg);

// One non-blocking polling tick: reads the Passive Target Status register
// plus the IRQ status registers ONCE and returns the updated state. Same
// bounded-cost shape as nfca_detect() -- a few I2C register reads, no
// waiting -- so it is safe to call every poll() tick, including when never
// armed (returns kNotArmed immediately without touching the bus).
ListenState listen_poll();

// Last state computed by listen_poll(), without touching the bus.
ListenState listen_get_state();

// Tears Listen Mode down: stops chip activity, restores REG_MODE (targ bit
// back to initiator/0) and REG_PASSIVE_TARGET (back to fully disabled) so a
// later nfca_poller_begin() reader-path call is unaffected, then clears
// rx_en (same "leave the oscillator running" policy as field_off()). Safe to
// call when never armed.
void listen_stop();

} // namespace St25r3916
