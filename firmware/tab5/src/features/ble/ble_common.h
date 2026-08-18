#pragma once
#include <cstdint>
#include <lvgl.h>

// Shared BLE device-list model, reused by the deferred second plan's BLE
// spam/finder/sniffer features (Task 7).
struct BleDeviceInfo {
    uint8_t addr[6];
    // The peer's advertised address type (NimBLE BLE_ADDR_PUBLIC /
    // BLE_ADDR_RANDOM / BLE_ADDR_PUBLIC_ID / BLE_ADDR_RANDOM_ID), straight
    // from event->disc.addr.type. Recorded because ble_gap_connect() needs the
    // real peer type -- assuming BLE_ADDR_PUBLIC fails against the random
    // static / resolvable-private addresses most modern peripherals actually
    // advertise with. Added for the second Phase 2 plan's Task 1
    // central-connect spike.
    uint8_t addr_type;
    char addr_str[18];
    int8_t rssi;
    char name[32]; // empty string if no AD_TYPE_NAME field present
    // Classification label from BleClassify::classify() (e.g. "iBeacon",
    // "AirPods (Continuity)") -- empty string if nothing recognized. Added
    // for the second Phase 2 plan's Task 5 (BLE scan classification).
    char label[24];
};

void ble_addr_to_str(const uint8_t addr[6], char out[18]);

// ---------------------------------------------------------------------------
// Shared "is the NimBLE host actually up?" screen guard.
//
// Whole-branch review finding C1 (2026-08-17). Every BLE feature screen in
// this project has to answer the same question before it touches NimBLE:
// did the host ever come up at all? On this board it genuinely may not
// have -- BLE is proxied to the ESP32-C6 over esp-hosted/SDIO, and main.cpp
// has a real, documented degraded boot mode ("radios DISABLED for this
// boot ... UI and SD remain functional") reached when that co-processor
// fails to come up. The launcher still shows every BLE tile in that mode,
// so any of these screens can be opened with no host behind them.
//
// Why this is a hard crash and not a graceful failure: MOST NimBLE entry
// points (ble_gap_adv_start/stop/set_data, ble_gap_disc, ...) begin with an
// internal ble_hs_is_enabled() test and return BLE_HS_EDISABLED harmlessly.
// ble_hs_id_set_rnd() does NOT -- verified by disassembling the exact
// libbt.a this firmware links against: it goes straight to ble_hs_lock() ->
// ble_hs_lock_nested(), which dereferences a .bss function-table pointer
// (npl_funcs+68) that stays NULL until nimble_port_init() has run. Calling
// it on a radios-disabled boot is a LoadProhibited fault -> panic/reboot,
// not an error code. ble_hs_id_copy_addr() takes the same lock-first path.
//
// So the guard is mandatory, not defensive politeness, and it lives here
// rather than being hand-copied into each feature: eight files had already
// grown their own byte-identical inline copy of it, and the two files that
// were missing it (ble_sourapple.cpp, ble_findmy.cpp) were a real,
// reachable panic. One definition means a new BLE screen has one obvious
// thing to call instead of one obvious thing to forget.
//
// Both variants set an explanatory message on the UI element they are
// handed and return false, so the caller can `return screen;` immediately
// with a screen that says why it is empty (never a blank screen, and never
// a silent no-op). Two entry points rather than one because this codebase's
// BLE screens genuinely use two different status conventions -- a
// lv_label status line, or a lv_list of results -- and detecting the widget
// class at runtime would be more magic than passing the right call.
//
// Call these AFTER the screen's LV_EVENT_DELETE teardown handler has been
// registered (so teardown still sees consistent state on the bail-out path)
// and BEFORE the first NimBLE call and before arming any poll()-driven
// `s_active` flag.
extern const char *const kBleHostNotReadyMsg;
bool ble_require_host_ready(lv_obj_t *status_label,
                            const char *not_ready_msg = kBleHostNotReadyMsg);
bool ble_require_host_ready_list(lv_obj_t *list,
                                 const char *not_ready_msg = kBleHostNotReadyMsg);

// REMOVED 2026-08-17: ble_push_message_screen() / kBleNoScannedTargetMsg.
//
// They existed for finding I3's minimal fix -- the four connect-based BLE
// features (ble_flood.cpp, ble_fastpair_exploit.cpp, ble_hfp_exploit.cpp,
// ble_whisperpair.cpp) needed something to show when
// BleScanFeature::first_device_addr() returned null, i.e. when no separate
// BLE Scan screen run had left a device in slot 0. All four now run their own
// scan through BleTargetPicker (see ble_target_picker.h), so "no scanned
// target" is not a reachable state any more and both were left with zero
// callers. Deleted rather than kept as never-called code, per this project's
// no-dead-code discipline. If some future screen wants a one-label message
// screen, build_sub_screen() + one lv_label is the four lines this was.
