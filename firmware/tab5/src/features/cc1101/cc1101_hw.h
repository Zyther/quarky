// CC1101 SPI HAL wrapper for the Tab5's own M-Bus-connected M5Stack CC1101
// Module (Phase 10) -- distinct from Phase 5's Cardputer-ADV hydra-hat
// CC1101, a physically separate radio on a different device (see this
// phase's spec Section 1). Wraps jgromes/RadioLib's real CC1101 class --
// the library M5Stack's own CC1101 module documentation links directly
// (docs.m5stack.com/en/module/Module_CC1101#softwares, also a real embedded
// hyperlink in docs/vendor/Module_CC1101.pdf), confirmed real/actively
// maintained 2026-08-24. NOT LSatan/SmartRC-CC1101-Driver-Lib, this task's
// first attempt -- see platformio.ini's lib_deps citation for the two real
// problems that library hit (maintenance risk, and a real-hardware repeat-
// init failure) before this switch.
#pragma once

#include <cstddef>
#include <cstdint>

namespace Cc1101Hw {

// Configures the M-Bus SPI bus from pins_config.h's TAB5_CC1101_* constants
// (MOSI=G18, MISO=G19, SCK=G5, CSN=G45, GDO0=G4, GDO2=G48 -- all real,
// vendor-PDF-confirmed and DIP-switch-set on the physical module, 2026-08-24;
// see pins_config.h's own citation) and calls RadioLib's CC1101::begin(),
// which performs its own real chip-presence check internally (retries the
// VERSION status register up to 10 times against known CC1101 silicon
// revisions before giving up) -- unlike the prior driver, is_present()
// below reflects begin()'s own real result rather than a separate,
// independently-fragile probe. Safe to call more than once (RadioLib's own
// Module::init()/term() lifecycle handles re-entry; this HAL does not layer
// any additional idempotency guard on top of what the library already
// provides).
bool init();

// Real chip-presence result cached from the most recent init() call's
// RadioLib begin() return code (true iff RADIOLIB_ERR_NONE).
bool is_present();

// Wraps CC1101::setFrequency(). Module's documented range is 855-925MHz
// (spec Section 5) -- callers should not hard-code the project owner's own
// narrower 868-925MHz intended tuning as a hard limit.
void set_frequency_mhz(float mhz);

// RadioLib has no getFrequency() getter for this module -- this returns the
// last value passed to set_frequency_mhz() (or init()'s own default), not a
// register readback.
float frequency_mhz();

// Wraps CC1101::getRSSI() (real dBm value, RadioLib's own conversion).
float rssi();

// Alias of rssi() -- Tasks 3+ (spectrum/hot-cold/scan) spell it this way in
// their own interface sketches (plan's Task 3/5 Interfaces lines).
inline float rssi_dbm() { return rssi(); }

// ---------------------------------------------------------------------------
// Async/direct-mode raw OOK support (Tasks 3/4/6/7/8) -- added on top of
// Task 1's packet-mode-only surface. Real RadioLib API, confirmed from
// CC1101.h/.cpp (jgromes/RadioLib, same library Task 1 already adopted):
// setOOK(true) + receiveDirectAsync()/transmitDirectAsync() puts the chip
// into the same "GDO0 carries a raw demodulated/modulated serial bitstream"
// mode Poseidon's cc1101_hw.cpp (ELECHOUSE_cc1101.SetRx()/SetTx() +
// pinMode(CC1101_GDO0, INPUT/OUTPUT), see subghz_scan.cpp) and Bruce's
// SmartRC-based rf_bruteforce.cpp/rf_jammer.cpp (setPktFormat(3), "async
// serial mode") both use under a different library's naming -- RadioLib's
// receiveDirect(false)/transmitDirect(false) (the `false` = async, no
// internal bit-clock sync) is functionally the same real CC1101 hardware
// feature (PKTCTRL0.PKT_FORMAT = 3, per TI's own CC1101 datasheet), not a
// different mechanism. Once either is active, the caller drives GDO0
// directly via Arduino's own pinMode()/digitalRead()/digitalWrite()/
// attachInterrupt() on gdo0_pin() -- this HAL does not wrap that part,
// matching how Cc1101Hw already left SPI/pin ownership to the caller in
// Task 1.
// ---------------------------------------------------------------------------

// Configures OOK/ASK modulation and puts the chip into async direct-mode
// RECEIVE (GDO0 becomes the raw demodulated data line -- HIGH/LOW tracks the
// over-the-air OOK signal in real time, exactly like an RF433R module's data
// pin). Returns false if either RadioLib call fails.
bool enable_async_rx();

// Configures OOK/ASK modulation and puts the chip into async direct-mode
// TRANSMIT (GDO0 becomes the data line the caller bit-bangs to modulate the
// carrier -- HIGH transmits, LOW is silence, exactly like an RF433T module's
// data pin). Returns false if either RadioLib call fails.
bool enable_async_tx();

// Returns the chip to IDLE (CC1101::standby()) -- call after an async RX/TX
// session before starting another, matching Poseidon's own
// cc1101_set_idle()-before-next-mode discipline (subghz_scan.cpp/
// subghz_jam_detect.cpp).
void idle();

// The M-Bus GDO0 GPIO (TAB5_CC1101_GDO0_GPIO / G4) -- exposed so RX/TX
// modules can attachInterrupt()/digitalWrite() it directly, the same shape
// rf433_common.h/rf433_replay.cpp already use for TAB5_RF433R_PIN/
// TAB5_RF433T_PIN.
int gdo0_pin();

} // namespace Cc1101Hw
