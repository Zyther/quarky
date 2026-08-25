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

} // namespace Cc1101Hw
