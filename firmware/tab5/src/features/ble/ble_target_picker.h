#pragma once
#include "ble_common.h" // BleDeviceInfo

// -----------------------------------------------------------------------------
// Shared BLE target picker: "scan -> tap a device -> hand it to the feature".
//
// WHY THIS EXISTS (2026-08-17, project-owner UX finding, the fix deferred by
// whole-branch review finding I3):
//
// Four connect-based BLE features (ble_flood.cpp, ble_fastpair_exploit.cpp,
// ble_hfp_exploit.cpp, ble_whisperpair.cpp) used to obtain their target by
// calling BleScanFeature::first_device_addr() -- i.e. whatever device happened
// to land in slot 0 of a PRIOR, SEPARATE "BLE Scan" screen run. That is
// counter-intuitive on a touch device: tapping "Fast Pair Exploit" either did
// nothing useful (no prior scan) or silently attacked a device the user never
// chose. Finding I3's minimal fix only made the no-prior-scan case say so on
// screen; it did not give the user a way to pick.
//
// The correct shape -- scan as step one of the feature, then let the user tap
// the target -- already existed twice in this codebase (ble_gatt_explorer.cpp
// and ble_clone.cpp, independently implemented). Rather than paste a fifth and
// sixth near-duplicate of that ~150 lines of carefully-reasoned scan / dedup /
// real-addr_type / LV_EVENT_DELETE-teardown logic, it lives here once.
//
// USAGE, from a feature's own start():
//
//     static void on_target_selected(const BleDeviceInfo &t) {
//         ScreenStack::push(build_screen(t.addr, t.addr_type));
//     }
//     void start() {
//         BleTargetPicker::start("Fast Pair Exploit -- pick a target",
//                                on_target_selected);
//     }
//
// The callback is a plain function pointer, not std::function: every consumer
// is a file-scope static, this is firmware, and it matches how the rest of
// this tree passes callbacks (FeatureRegistry's own start/stop members are
// plain function pointers too).
//
// A whole BleDeviceInfo is handed over rather than just an ble_addr_t so the
// receiving feature can also show the target's name/RSSI if it wants to. Every
// current consumer only reads .addr/.addr_type, which is exactly the
// (const uint8_t addr_val[6], uint8_t addr_type) shape their existing
// build_screen() functions already take -- so wiring the picker in changed
// nothing about what those features DO with a target, only how they get one.
//
// SCREEN CHOREOGRAPHY: start() pushes the picker's own screen. When a row is
// tapped, the picker pops ITSELF first and then invokes the callback, so the
// feature's screen replaces the picker on the screen stack instead of stacking
// on top of it. Backing out of the feature therefore lands on the launcher,
// not on a stale, no-longer-scanning device list. Both calls happen inside the
// same LVGL event dispatch, so no intermediate frame is ever rendered -- there
// is no visible flash of the launcher in between. (Popping a screen from
// inside a click handler on one of its own descendants is the pattern
// screen_scaffold.cpp's Back button already uses and real hardware has already
// proven; LVGL's lv_event_mark_deleted() marks the in-flight event so dispatch
// stops cleanly when our handler returns.)
//
// SINGLE INSTANCE: like every other feature screen in this project, only one
// picker can be on screen at a time, so its state is file-scope static and
// start() resets it. There is no need (and no way, given the screen stack) for
// two concurrent pickers.
// -----------------------------------------------------------------------------
namespace BleTargetPicker {

using TargetSelectedFn = void (*)(const BleDeviceInfo &target);

// Pushes the picker screen and starts its own BLE discovery scan. The callback
// runs on the main/LVGL task, after the picker screen has already been popped.
void start(const char *screen_title, TargetSelectedFn on_selected);

// Wire into main.cpp's loop(). No-ops unless the picker screen is open and
// still scanning.
void poll();

} // namespace BleTargetPicker
