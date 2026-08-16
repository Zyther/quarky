#pragma once
#include <cstdint> // uint8_t (send_key()'s keycode parameter, added Task 15)

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
// The result was positive, real-hardware-confirmed end to end (a real macOS
// host paired with "QuarkyKB" and accepted an injected keystroke), so Task 15
// (2nd Phase 2 plan) promoted this from a spike into the "BLE Bad-KB"
// launcher-tile feature -- see features/ble/ble_bad_kb.h/.cpp. This file
// itself still defines and owns the ONLY HID GATT service/advertising/
// keystroke-notify plumbing in the project; ble_bad_kb.cpp drives it through
// the public API below rather than duplicating any of it.
//
// This file itself still registers no FeatureModule and has no launcher tile
// of its own -- BleBadKbFeature (ble_bad_kb.cpp) is the tile; this remains
// the underlying HID transport it's built on. The 'h'/'j' serial-debug
// triggers (main.cpp) remain, as a headless-verification convenience
// separate from whether the tile exists.
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
// Called unconditionally from main.cpp's setup() (Task 15, 2nd Phase 2 plan)
// -- no longer gated behind QUARKY_SERIAL_DEBUG. That gate was correct for a
// spike with no launcher tile of its own ("a default build should not carry
// a BLE HID keyboard service... for a spike it cannot even trigger" -- this
// comment's own prior wording), but Task 15 made this a real, always-present
// launcher-tile feature (BleBadKbFeature, ble_bad_kb.cpp) any user can open,
// so the HID service must actually exist in the ATT database in every build,
// not just debug builds -- otherwise the tile would open to a screen whose
// "Send" button can never work.
void register_service();

// Begins advertising as "QuarkyKB" with the HID appearance and the 0x1812
// service UUID, and points the GAP Device Name characteristic at "QuarkyKB"
// too (several host stacks read it post-connect rather than trusting the
// advertised name). Safe to call twice -- the second call is a no-op while
// already advertising or connected. Also safe to call from a real
// launcher-tile open/close flow (BleBadKbFeature's build_screen(), not just
// a one-shot serial trigger) -- see ble_bad_kb.cpp's own comment at its
// start() call site for the one residual, pre-existing timing note (an
// in-flight async disconnect can make an immediate reopen no-op once).
//
// No-ops with a logged explanation if the NimBLE host has not synced, or if
// register_service() never ran, since advertising a HID service that is not
// in the ATT database would produce a false negative.
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
//
// Kept exactly as-is (own inline 50ms-gap implementation, not rewritten to
// call send_key() below) so its real-hardware-proven timing -- the one that
// actually registered a keypress on a real macOS host -- is never
// perturbed. Still the 'j' serial-debug trigger's entry point (main.cpp).
void send_test_keystroke();

// Task 15 (2nd Phase 2 plan, Bad-KB feature): sends one key-down report
// followed by a key-up (all-zero) report, as GATT notifications on the Input
// Report characteristic, for an ARBITRARY USB HID keycode -- generalizing
// send_test_keystroke()'s hardcoded 'a' (0x04). Same connected/
// notifications-enabled guards as send_test_keystroke(); also silently
// no-ops on keycode == 0 (ble_bad_kb.cpp's keycode_for() "unrecognized
// character" sentinel for this reduced Ducky-script subset).
//
// Uses its own tighter ~20ms-per-half-report gap (not send_test_keystroke()'s
// 50ms) deliberately: unlike send_test_keystroke() (a one-shot serial-debug
// trigger), this is meant to be called from ble_bad_kb.cpp's poll() -- once
// per loop() tick, for every character of a real script -- so its total
// blocking time per call must stay safely under this project's ~50ms
// no-blocking-call-longer-than-this Global Constraint. ~40ms total (20ms +
// 20ms) leaves headroom for that same tick's other poll() calls and
// lvgl_port_tick(); a 50ms single call would consume the entire budget by
// itself. See ble_bad_kb.cpp's file comment for why per-tick, not per-script,
// blocking is what actually matters here.
void send_key(uint8_t keycode);

// True from a successful BLE_GAP_EVENT_CONNECT until the matching
// BLE_GAP_EVENT_DISCONNECT, independent of whether that host has enabled
// input-report notifications yet (see send_key()'s separate guard for that).
// Lets a caller -- ble_bad_kb.cpp's own status label -- reflect live
// connection state without installing a second, competing gap_event_cb:
// NimBLE only supports one `event_cb` per ble_gap_adv_start() call, and
// this file's start() (below) already installs its own internal one, so a
// second feature-owned callback would either silently never fire or replace
// this file's, breaking send_key()'s own connected/notify-enabled tracking.
// Polling this from ble_bad_kb.cpp's own poll() is the safe alternative.
bool is_connected();

// Stops the HID advertisement, terminates any open connection, and restores
// the previous GAP device name. Does NOT restore c2link_ble's C2
// advertisement (see the header comment above), and does not unregister the
// HID service -- there is no safe runtime way to do that (same finding C1),
// and the service is registered for the life of the boot regardless.
void stop();

} // namespace BleHidSpike
