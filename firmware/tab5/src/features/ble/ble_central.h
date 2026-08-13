#pragma once
#include <host/ble_gap.h>

// -----------------------------------------------------------------------------
// Shared BLE central/client-connect helper (second Phase 2 plan, Task 1).
//
// Every later task that needs to open an outbound BLE connection (GATT
// explorer, BLE flood, Fast Pair crypto exploit, HFP audio exploit,
// WhisperPair) calls these two functions rather than ble_gap_connect() /
// ble_gap_terminate() directly, so the connect/disconnect lifecycle logging
// lives in exactly one place -- the same reason c2link_ble.cpp owns the one
// and only NimBLE host in this project (raw ESP-IDF NimBLE, never
// NimBLE-Arduino; see that file's header comment for why).
// -----------------------------------------------------------------------------
namespace BleCentral {

// Initiates a connection to target. Our OWN address type is always
// BLE_OWN_ADDR_PUBLIC, matching every other raw-address use in this project's
// BLE code (ble_scan.cpp's ble_gap_disc, ble_spam.cpp's ble_gap_adv_start).
// The PEER address type is whatever the caller put in target.type -- a
// separate field, and one that must reflect what the peer actually
// advertised with, not a hardcoded assumption (see ble_central_spike.cpp).
//
// event_cb receives the same struct ble_gap_event dispatch shape
// hal/c2link_ble.cpp's gap_event_cb already uses (BLE_GAP_EVENT_CONNECT with
// event->connect.status == 0 on success, event->connect.conn_handle the
// handle to use for subsequent ble_gattc_* calls; BLE_GAP_EVENT_DISCONNECT
// with event->disconnect.reason when it drops).
//
// Returns the real ble_gap_connect() return code -- 0 means "connection
// attempt started", NOT "connected"; the caller's event_cb finds out the real
// outcome asynchronously via BLE_GAP_EVENT_CONNECT.
int connect(const ble_addr_t &target, int32_t timeout_ms, ble_gap_event_fn *event_cb, void *cb_arg);

// Wraps ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM) -- the
// standard "we're done, hang up" reason code. Safe to call on an
// already-disconnected handle (NimBLE returns a real error code, which is
// logged, not fatal).
int disconnect(uint16_t conn_handle);

} // namespace BleCentral
