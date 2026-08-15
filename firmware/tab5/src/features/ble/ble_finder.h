#pragma once
namespace BleFinderFeature {
void register_module();
void start();
void poll();
// Serial-trigger-only ('f' in main.cpp's QUARKY_SERIAL_DEBUG chain): locks
// geiger mode onto the most recently seen tracker (s_last_seen_addr), the
// same target the in-progress-scan list most recently added a row for.
// Needed because the brief's own real-target-lock UI (tap a row to lock) is
// explicitly deferred -- without SOME way to set s_locked = true,
// update_geiger_ui() is permanently unreachable dead code. See ble_finder.cpp
// for the full resolution note.
void lock_last_seen();
}
