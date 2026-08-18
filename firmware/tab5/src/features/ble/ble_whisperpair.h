#pragma once

// -----------------------------------------------------------------------------
// WhisperPair -- CVE-2025-36911 detector (second Phase 2 plan, Task 18).
//
// Connects to a target the user picks in BleTargetPicker's own scan (2026-08-17
// UX fix; this used to be "the first BLE-scanned device", i.e. slot 0 of a
// prior, separate BLE Scan run) via Task 1's BleCentral helper,
// finds the Google Fast Pair service (0xFE2C) and its Key-based Pairing (KBP)
// characteristic, SUBSCRIBES to that characteristic's notify (real CCCD
// discovery, not a guessed handle -- see the .cpp), then writes a real
// ECDH-derived, AES-128-encrypted 16-byte KBP request at it while the
// accessory is NOT in pairing mode.
//
// The question the CVE asks, and the only question this answers: does the
// accessory respond at all to a KBP write outside pairing mode? A notify
// arriving inside the wait window means yes (vulnerable). Silence means the
// target is patched, or was never applicable. BOTH outcomes are real findings.
//
// DISCLOSED LIMITATION (matching UniGeek's own scope note): this does not
// complete a real Fast Pair key exchange. We never obtain the accessory's own
// public key, so the accessory cannot decrypt our probe and we could not
// decrypt its reply. This is a vulnerability DETECTOR, not an attack chain.
//
// poll() is NOT optional decoration. Every NimBLE callback here runs on the
// NimBLE host task, which must never touch LVGL (LV_USE_OS is LV_OS_NONE in
// this port). Those callbacks buffer their status text; poll() -- called from
// main.cpp's loop() on the main task -- renders it, gates the "Send Probe"
// button on the subscription actually having succeeded, and is what times out
// the post-probe notify wait window.
// -----------------------------------------------------------------------------
namespace BleWhisperPairFeature {
void register_module();
void start();
void poll();
}
