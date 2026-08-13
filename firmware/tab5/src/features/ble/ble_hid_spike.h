#pragma once

// -----------------------------------------------------------------------------
// Second Phase 2 plan, Task 2 -- the BLE HID / Bad-KB SPIKE.
//
// hal/c2link_ble.cpp already proves the PERIPHERAL role works over this
// project's raw ESP-IDF NimBLE host on real hardware: the Tab5 advertises,
// accepts an inbound connection, and serves a CUSTOM (Nordic UART) GATT
// service across the esp-hosted SDIO link to the ESP32-C6. That is NOT the
// same thing this spike asks.
//
// BLE HID is a *profile*, not just another service UUID. A host OS only
// treats a peripheral as a keyboard if the peripheral presents the Human
// Interface Device Service (0x1812) in the exact shape HID hosts expect --
// Report Map (0x2A4B) carrying a valid USB HID report descriptor, an Input
// Report characteristic (0x2A4D) with both a CCCD and a Report Reference
// descriptor (0x2908), HID Information (0x2A4A), and the 0x1812 UUID plus the
// HID-keyboard appearance (0x03C1) in the advertisement so the OS classifies
// it before it ever connects. None of that structure has ever been exercised
// by this project, and the Phase 2 spec's Risks section flags BLE HID over a
// remote-radio (esp-hosted) transport as an open question.
//
// Two questions, both answerable only on real hardware:
//   1. Does a real host OS enumerate "QuarkyKB" AS A KEYBOARD (not as a
//      generic/unknown BLE device) and pair with it?
//   2. Once paired, does a notified input report actually register as a
//      keypress in a text field on that host?
//
// A negative result is a real result: Task 15 (the Bad-KB feature) must not be
// dispatched until the project owner decides how to re-scope it.
//
// This is a spike, not a feature: no FeatureModule is registered and there is
// no launcher tile. Serial-debug triggers ('h' = start, 'j' = send keystroke,
// see main.cpp) are the only entry points, by design.
//
// SIDE EFFECT, disclosed (same class as ble_spam.cpp's): legacy BLE
// advertising is single-instance system-wide, so start() STOPS
// c2link_ble.cpp's always-on C2 advertisement and replaces it with the HID
// keyboard advertisement. The C2 advertisement does not come back by itself;
// reboot to restore it. See ble_hid_spike.cpp's file-level comment for the
// second, larger side effect (a GATT-server restart).
// -----------------------------------------------------------------------------
namespace BleHidSpike {

// Registers the HID GATT service (once) and begins advertising as "QuarkyKB"
// with the HID appearance and the 0x1812 service UUID. Safe to call twice --
// the second call is a no-op while already advertising or connected.
//
// Must not be called while a C2 BLE connection is open: registering a service
// into an already-running GATT server requires ble_gatts_start(), which
// NimBLE refuses (BLE_HS_EBUSY) while any peer is connected. The failure is
// logged plainly rather than worked around.
void start();

// Sends one 'a' key-down report followed 50ms later by a key-up (all-zero)
// report, as GATT notifications on the Input Report characteristic.
//
// Does nothing (and says so) unless a host is connected AND has enabled
// notifications on the report's CCCD. That guard matters: NimBLE's
// ble_gatts_notify_custom() returns 0 for an unsubscribed characteristic, so
// without it a completely undelivered keystroke would log as a success and
// produce a FALSE POSITIVE for the exact question this spike exists to
// answer.
void send_test_keystroke();

// Stops the HID advertisement and terminates any open connection. Does NOT
// restore c2link_ble's C2 advertisement (see the header comment above).
void stop();

} // namespace BleHidSpike
