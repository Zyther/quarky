#pragma once
#include "ic2link.h"

// BLE GATT backend for the Tab5 <-> Cardputer-ADV control channel.
//
// This is the second half of Tab5's radio-selected C2 transport pair (see
// c2link_wifi.h for the first half): used when a WiFi feature is active on
// the device and the WiFi radio isn't free for C2. Control-message use only
// in this phase (CMD_*/RESP_STATUS/RESP_TELEMETRY) -- bulk transfers wait
// for the WiFi transport, per the foundation spec's §4.5 amendment.
//
// Exposes a Nordic-UART-Service-shaped GATT server: one write characteristic
// for inbound frames (peer -> Tab5), one notify characteristic for outbound
// frames (Tab5 -> peer). NUS is a de facto standard for exactly this
// "bidirectional byte pipe over GATT" shape, reused rather than inventing a
// custom service so any BLE debugging tool that already knows NUS works
// against this link for free.
class C2LinkBle : public IC2Link {
public:
    bool init(const uint8_t psk[16], const char *device_name);
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
    bool is_connected() override;
    // Call every loop() iteration. Inbound frames are received on NimBLE's own
    // host task (in the GATT write callback), queued, and dispatched to the
    // receive handler from here -- so the handler runs on the main task, the
    // same threading contract as C2LinkWifi::poll(). See c2link_ble.cpp.
    void poll();
};

// Hook invoked from init(), after c2link_ble has queued its own GATT service
// and BEFORE the NimBLE host task starts. Register additional GATT services
// here and nowhere else.
//
// Why this exists (task-2-review.md finding C1, 2026-08-14): NimBLE's
// ble_gatts_add_svcs() only QUEUES a service definition; the queue is drained
// exactly once, by the ble_gatts_start() that ble_hs_start() runs
// automatically when the host task comes up. Queuing a service after that
// point and calling ble_gatts_start() a second time by hand is a
// use-after-free: ble_gatts_start() -> ble_att_svr_start() ->
// ble_att_svr_free_start_mem() frees the heap block every already-registered
// ATT attribute (GAP, GATT, this file's own NUS service) lives in, without
// ever clearing ble_att_svr_list -- which is left holding dangling pointers
// that ble_gatts_start()'s own CCCD-cache loop then walks. Verified against
// the shipped esp32p4 libbt.a by disassembly, not inferred.
//
// So: any module that wants its own service in this project's single shared
// GATT server registers a hook here before c2link_ble.init() runs, and does
// only ble_gatts_count_cfg() + ble_gatts_add_svcs() inside it.
//
// Deliberately a hook rather than a direct #include of the feature module in
// c2link_ble.cpp: nothing in hal/ depends on features/ today (only ui/ does),
// and a hardware-abstraction layer reaching up into feature code to hardcode
// a spike's registration would be the wrong direction. main.cpp -- which
// already owns wiring hal to features -- installs the hook instead.
//
// Returns false if the hook table is full. Hooks must be added before
// init(); ones added afterwards are never invoked (and would be too late to
// be safe anyway).
using C2LinkBleGattHook = void (*)();
bool c2link_ble_add_gatt_hook(C2LinkBleGattHook hook);

// millis() timestamp of the last frame successfully dequeued and dispatched
// by poll() (0 if none yet). Free function, mirroring
// c2link_wifi_last_recv_ms() -- see Task 19 (devices_panel.cpp polls this to
// derive link freshness for the shell's status bar).
uint32_t c2link_ble_last_recv_ms();

// True once the NimBLE host and (remote C6) controller have synced and
// on_sync() has run. Task 7: features that also need the BLE host for a
// central/observer-role call (e.g. ble_gap_disc() scanning) check this
// first -- calling into NimBLE before sync leaves the host in an undefined
// state.
//
// Final whole-branch review finding M7 (2026-08-13): this used to
// parenthetically claim sync also means "this device is actively
// advertising as a GATT server" -- true only until Task 8 (BLE Spam) runs,
// which stops that advertisement in favor of its own (see ble_spam.cpp's
// file-level comment for the single-advertisement-instance constraint).
// The actual, still-correct contract this function's two real callers rely
// on is narrower: the host is synced, so central/observer calls are safe.
// Dropped the parenthetical rather than qualify it, since nothing consumes
// the advertising-state claim.
bool c2link_ble_host_synced();

// Restart this link's own C2 advertisement after some other feature took the
// radio's single legacy-advertising slot.
//
// Why this exists (whole-branch review finding I6, 2026-08-17). Legacy BLE
// advertising is single-instance system-wide -- this project has not
// configured NimBLE Extended Advertising -- so ANY feature that calls
// ble_gap_adv_start() (BLE Spam, Clone, Karma, Sour Apple, Find My, and the
// HID/Bad-KB path) implicitly stops the C2 advertisement this file started
// in on_sync(). That is an unavoidable radio constraint. What was NOT
// unavoidable is that it used to be permanent: six features stopped their
// own advertisement on teardown but nothing ever restarted C2's, so opening
// any of those six screens once killed the Tab5's BLE C2 link for the rest
// of the boot. ble_spam.cpp had cited a "c2link_ble_rearm_advertising()"
// call as the intended fix since the first Phase 2 plan; this is that
// function, finally written.
//
// Implementation is deliberately thin: it re-runs the same
// ble_gap_adv_set_fields()/ble_gap_adv_rsp_set_fields()/ble_gap_adv_start()
// sequence on_sync() runs, from the same stored device name and service
// UUID, so there is exactly one definition of what the C2 advertisement
// looks like. It is safe to call repeatedly and from any feature's
// LV_EVENT_DELETE teardown.
//
// Returns true if the advertisement is (or already was) up afterwards.
// Returns false, harmlessly and without touching NimBLE, when:
//   * the host never synced (radios-disabled boot) -- same guard every
//     feature screen needs, see c2link_ble_host_synced();
//   * a C2 peer is already CONNECTED, in which case there is deliberately
//     no advertisement to restore (this file only advertises while
//     unconnected; its own gap_event_cb re-arms on disconnect).
//
// Note on identity: features like Clone/Karma/Sour Apple/Find My leave a
// host-wide random identity behind via ble_hs_id_set_rnd() (see those files'
// disclosure comments). This re-arm keeps using the own-address type
// on_sync() inferred at boot, so the C2 advertisement returns under the same
// address the Cardputer-ADV saw at boot rather than under a leftover spoofed
// identity -- which is the behaviour the C2 link wants.
bool c2link_ble_rearm_advertising();
