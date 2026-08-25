#pragma once

// ===========================================================================
// CC1101 spectrum analyzer (Phase 10 Task 5).
//
// Donor reference (this phase's spec Section 2 / Phase 5 spec Section 1):
// Bruce rf_spectrum.cpp/rf_waterfall.cpp, Poseidon (most visually developed
// version). Real technique both use: retune the chip across a frequency
// range and sample getRSSI() at each step -- there is no separate hardware
// spectrum-sweep mode on the CC1101, so this is the same real primitive
// Cc1101Hw::rssi_dbm() already exposes, driven across a range instead of one
// fixed frequency (the same relationship Poseidon's subghz_scan.cpp 'A'
// autoscan handler -- setMHZ()+delay(15)+getRSSI() per step -- has to its
// own single-frequency scan screen).
//
// Unlike Phase 5's Cardputer-ADV version (240x135 screen, spec Section 2.4
// raises rendering it on the Tab5 instead), this phase runs natively on the
// Tab5's own 1280x720 display already, so it renders directly via LVGL's
// lv_chart at full fidelity from the start -- no cross-device mirroring
// question to resolve.
//
// poll()-driven, one frequency step per tick (same shape as
// wifi_spectrum.cpp's per-channel-hop poll()) -- never a blocking sweep loop
// inside a click handler.
// ===========================================================================

namespace Cc1101Spectrum {

void register_module();
void start();

// Called from main.cpp's loop(). No-ops unless the spectrum screen is open.
void poll();

} // namespace Cc1101Spectrum
