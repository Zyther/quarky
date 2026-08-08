#pragma once
#include <cstdint>

// Task 19: drives the persistent link-status label in the shell's status bar
// (built in Task 7) from the freshness of the two C2 transports (Task 11
// WiFi, Task 13 BLE). Deliberately just updates that one label rather than a
// separate always-open panel -- a full slide-out panel showing both
// transports independently is a reasonable Phase 2+ UI polish item, not
// required for the foundation phase.
namespace DevicesPanel {

// wifi_connected/ble_connected: whether that transport has received a frame
// recently enough to be considered "live" (see main.cpp's loop() for the
// freshness window). last_rtt_ms: age in ms of the freshest of the two, or
// (once Task 20's ping lands) an actual round-trip time -- either way, just
// a display value shown alongside "connected".
void update(bool wifi_connected, bool ble_connected, int32_t last_rtt_ms);

} // namespace DevicesPanel
