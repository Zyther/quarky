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
// reboot to restore it. Nothing else about c2link_ble is disturbed -- the
// shared GATT server is never rebuilt at runtime (see register_service()).
// -----------------------------------------------------------------------------
namespace BleHidSpike {

// Queues the HID GATT service for registration. Does ONLY
// ble_gatts_count_cfg() + ble_gatts_add_svcs() -- never ble_gatts_start().
//
// Must be installed as a c2link_ble GATT hook (c2link_ble_add_gatt_hook, see
// main.cpp) so it runs inside c2link_ble.init() before the NimBLE host task
// starts. That timing is not a style preference: NimBLE drains the queued
// service-def list exactly once, in the ble_gatts_start() that ble_hs_start()
// performs automatically at host startup, and calling ble_gatts_start() again
// afterwards frees the heap block every already-registered ATT attribute
// lives in while leaving them linked -- a use-after-free of the live GATT
// server. Task 2's first implementation did exactly that; see
// task-2-review.md finding C1 and c2link_ble.h's hook comment for the
// disassembly-backed mechanism.
//
// Registration is gated behind QUARKY_SERIAL_DEBUG at the call site in
// main.cpp: this is spike-only structure, and a default build should not
// carry a HID keyboard service in its ATT database where any connected
// central can discover it.
void register_service();

// Begins advertising as "QuarkyKB" with the HID appearance and the 0x1812
// service UUID, and points the GAP Device Name characteristic at "QuarkyKB"
// too (several host stacks read it post-connect rather than trusting the
// advertised name). Safe to call twice -- the second call is a no-op while
// already advertising or connected.
//
// No-ops with a logged explanation if the NimBLE host has not synced, or if
// register_service() never ran (i.e. this is not a QUARKY_SERIAL_DEBUG
// build), since advertising a HID service that is not in the ATT database
// would produce a false negative.
void start();

// Sends one 'a' key-down report followed 50ms later by a key-up (all-zero)
// report, as GATT notifications on the Input Report characteristic.
//
// Does nothing (and says so) unless a host is connected AND has enabled
// notifications on the report's CCCD. That guard matters: NimBLE's
// ble_gatts_notify_custom() performs no subscription check at all -- it
// transmits the PDU unconditionally and returns 0, and an unsubscribed host
// simply discards it. Without the guard, the spike's primary failure mode
// (host never bound a keyboard driver) would log rc=0 twice and read as a
// success.
void send_test_keystroke();

// Stops the HID advertisement, terminates any open connection, and restores
// the previous GAP device name. Does NOT restore c2link_ble's C2
// advertisement (see the header comment above), and does not unregister the
// HID service -- there is no safe runtime way to do that (same finding C1),
// and the service is registered for the life of the boot regardless.
void stop();

} // namespace BleHidSpike
