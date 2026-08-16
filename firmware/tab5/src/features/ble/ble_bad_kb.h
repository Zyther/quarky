#pragma once

// -----------------------------------------------------------------------------
// Task 15 (2nd Phase 2 plan): BLE Bad-KB -- a real, launcher-tile Ducky-script
// text-entry feature, contingent on Task 2's BLE HID spike
// (features/ble/ble_hid_spike.h/.cpp), which is real-hardware-confirmed
// working end to end (a real macOS host paired with "QuarkyKB" and accepted
// an injected keystroke).
//
// This file defines NO GATT service, report map, or characteristic table of
// its own. There is, and must remain, exactly ONE HID GATT service in this
// project's ATT database -- ble_hid_spike.cpp's s_hid_svcs, registered once
// at boot via the c2link_ble GATT-hook mechanism (main.cpp's setup(), see
// c2link_ble_add_gatt_hook(BleHidSpike::register_service)). ble_hid_spike.h's
// own header comment explains why any other registration path is unsafe:
// NimBLE drains the queued service-definition list exactly once, inside the
// ble_gatts_start() that ble_hs_start() runs automatically at boot; adding a
// service afterwards (Task 15's original brief sketch, calling
// ble_gatts_count_cfg()/ble_gatts_add_svcs() from this feature's own
// build_screen(), at arbitrary runtime whenever a user taps the launcher
// tile) either never finalizes the new service in the live ATT database, or
// -- if ble_gatts_start() is called a second time to force it -- frees the
// heap block backing every already-registered ATT attribute while leaving
// them linked in ble_att_svr_list: a real, disassembly-confirmed
// use-after-free (task-2-review.md finding C1, the exact defect Task 2's own
// implementation had to fix). This feature therefore only DRIVES
// BleHidSpike's existing, proven public API (start()/stop()/send_key()/
// is_connected()) -- it never touches NimBLE's GATT registration surface
// itself.
//
// Ducky-script parsing here is a small, disclosed-as-reduced subset: STRING
// <text> types literal text (a..z/A..Z -- shift/uppercase not modeled -- and
// space), ENTER and bare newlines send the Enter keycode, and any other line
// content is skipped (not typed) -- consistent with Ducky script's own
// line-oriented command format, where unimplemented commands (DELAY, REM,
// modifier keys, ...) are real, considered future work, not silently
// mis-typed as literal text.
//
// Typing itself is poll()-driven, not a single blocking call: see
// ble_bad_kb.cpp's file comment for why (loop()'s Global Constraint of no
// blocking call greater than ~50ms, and the real, user-reported watchdog
// crash class this project already hit once from a different long blocking
// call inside a click handler -- wifi_connect.cpp's fix, see its own
// comments).
// -----------------------------------------------------------------------------
namespace BleBadKbFeature {
void register_module();
void start();

// No-ops unless the BLE Bad-KB screen is open. Two independent jobs, both
// cheap every tick:
//   - updates the status label's "Paired"/"Advertising..." text from
//     BleHidSpike::is_connected(), since this feature does not (and must
//     not -- NimBLE allows only one) install its own competing GAP event
//     callback.
//   - if a script is mid-flight (the "Send" button was tapped), advances the
//     Ducky-script parser by exactly one action (one HID keystroke send, or
//     one no-HID bookkeeping step) per call, so a long script's typing is
//     spread across many loop() iterations instead of blocking one.
void poll();
} // namespace BleBadKbFeature
