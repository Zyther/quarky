#pragma once
#include <cstdint>

// -----------------------------------------------------------------------------
// Second Phase 2 plan, Task 1 -- the BLE central/client-connect SPIKE.
//
// The first plan's Task 7 (features/ble/ble_scan.cpp) proved BLE central/
// OBSERVER role (ble_gap_disc(), passive/active scanning) works over this
// project's raw ESP-IDF NimBLE host concurrently with hal/c2link_ble.cpp's
// always-on C2 advertisement. BLE central/CLIENT-CONNECT role
// (ble_gap_connect() + ble_gattc_* GATT operations) is a DIFFERENT capability
// that has never been exercised on this hardware -- the P4 has no local BT
// radio, so every HCI command is proxied to the ESP32-C6 over esp-hosted
// SDIO, and whether that path supports an outbound connection at all (let
// alone while simultaneously advertising) is a real, open question.
//
// Five later tasks in this plan (GATT explorer, BLE flood, Fast Pair crypto
// exploit, HFP audio exploit, WhisperPair) all fundamentally require this
// role. A negative result here is a real result and must be reported plainly,
// not worked around.
// -----------------------------------------------------------------------------
namespace BleCentralSpike {

// Connects to the given address, runs ble_gattc_disc_all_svcs() to enumerate
// services, logs every UUID found via Serial, then disconnects.
//
// addr_type is the PEER's advertised address type (BLE_ADDR_PUBLIC /
// BLE_ADDR_RANDOM / ...) -- it must be whatever the scan actually saw, not a
// hardcoded BLE_ADDR_PUBLIC. Most modern BLE peripherals advertise with a
// random static or resolvable-private address, and connecting to one of those
// with the wrong peer type fails outright. Getting this wrong would produce a
// FALSE NEGATIVE for the exact question this spike exists to answer, so the
// type is plumbed through from BleScanFeature rather than assumed.
//
// The real answer this spike is looking for is "did BLE_GAP_EVENT_CONNECT
// fire with status==0, and did the service discovery callback report at least
// one real service UUID" -- not just "did ble_gap_connect() return 0" (that
// only means "attempt started", per ble_central.h's own doc comment; same
// class of gotcha as WIFI_SCAN_RUNNING vs WIFI_SCAN_FAILED from the first
// plan's Task 3).
void run(const uint8_t addr_val[6], uint8_t addr_type);

// Safety net only: if a connection opens but service discovery never
// completes (no BLE_HS_EDONE), forcibly disconnect after kSpikeTimeoutMs so
// the spike can never leave a connection dangling. That matters here
// specifically because the spike's SECOND question is whether c2link_ble's C2
// advertisement survives concurrent central-connect use -- an indefinitely
// held connection would muddy that observation. No-ops when idle. Call from
// loop().
void poll();

} // namespace BleCentralSpike
