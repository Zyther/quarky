# Phase 5: Cardputer-ADV Satellite — IR, CC1101 Hydra-Hat, nRF24 — Design

**Status:** Draft for review
**Date:** 2026-08-06
**Depends on:** Phase 1 Foundation — Cardputer-ADV `Device` HAL, `IC2Link`/`CommandDispatcher`, `LocalMenu` baseline, `FeatureModule` contract with `Affinity::CARDPUTER_ADV`, Tab5's descriptor-side pattern from Phase 1 Task 20 (ping feature).
**Scope:** IR (Cardputer-ADV's own transmit LED), CC1101 sub-GHz (hydra hat, 433MHz-tuned but usable ~300–900MHz), nRF24 (hydra hat) — all remotely controllable from the Tab5 UI, while remaining usable standalone from Cardputer-ADV's own screen/keyboard.

## 1. Feature Inventory and Source Mapping

### IR
| Feature | Donor reference | Notes |
|---|---|---|
| TV-B-Gone | Bruce `src/modules/ir/`, Poseidon `ir_tvbgone.cpp` | Static code-database transmit, simplest feature in this phase |
| IR receive/decode | Bruce (`IRremoteESP8266` fork), Poseidon `ir_learn.cpp`/`ir_learn_decode.cpp` | |
| Universal remote / multi-profile clone (Samsung/LG/Sony) | Poseidon `ir_clone.cpp/.h`, UniGeek Flipper-IRDB-compatible database | UniGeek's Flipper-IRDB compatibility is the most valuable single piece to port here — gives access to a large existing community remote database instead of hand-building one |
| IR jammer | Bruce | Continuous-noise transmit on the IR LED |

Cardputer-ADV's hardware here is IR-transmit-only (per Poseidon's research: "IR: transmit-only LED (GPIO 44, active-low)") — **IR receive/decode/learn features require confirming whether the specific Cardputer-ADV unit in this kit has an IR receiver diode populated**, since Poseidon's own hardware note only documents transmit. This is the first thing to check before committing to the receive/decode/clone rows above; if receive-only Cardputer-ADV units exist, TV-B-Gone and jammer (transmit-only) still work regardless.

### CC1101 (hydra hat, tuned 433MHz, usable ~300–900MHz per owner's hardware notes)
| Feature | Donor reference | Notes |
|---|---|---|
| Scan/copy with protocol decode (Princeton/CAME/NICE/Linear) | Poseidon `subghz_*.cpp`, `cc1101_hw.cpp/.h` | |
| Record RAW (Flipper `.sub` format) | Poseidon, Bruce `rf_record.cpp` | `.sub` format compatibility matters — both UniGeek and Poseidon maintain Flipper-compatible libraries (Poseidon ships 3,190+ baked `.sub` files); reuse that signal-library data directly if license/authorization covers it (already cleared per project owner) |
| Replay | Bruce `rf_send.cpp`, Poseidon | |
| Spectrum analyzer (bar/waterfall/oscilloscope + radar/persistence/sonar visual modes) | Bruce `rf_spectrum.cpp`/`rf_waterfall.cpp`, Poseidon (most visually developed version, added v0.6.2) | Poseidon's visualization modes are the most complete reference; UI-heavy feature, real screen real estate needed — Cardputer-ADV's own 240×135 screen is tight for this, consider whether the *rendered result* should also be mirrorable to the Tab5 (see Section 2.4) |
| Brute force (Came/Nice/Linear/Chamberlain/Holtek/Ansonic) | Bruce `rf_bruteforce.cpp`, UniGeek (38–44 brand/protocol decoders) | UniGeek's decoder breadth is the widest of the three donors — use as primary reference for protocol coverage |
| Jammer (full/intermittent) | Bruce `rf_jammer.cpp`, Poseidon, UniGeek | |
| Hot/cold signal finder | Poseidon | |
| KeeLoq auto-decode + rolling-code "Replay +1" | UniGeek (unique to UniGeek among the three donors) | Rolling-code attack — legally sensitive even under owner's stated test-lab authorization; implement but ensure Cardputer-ADV's UI (and any Tab5-mirrored control) makes unambiguous the target is being actively attacked, not passively scanned |

### nRF24 (hydra hat, shares Grove/SPI pins with CC1101 — electrically exclusive, only one active at a time per Phase 1's confirmed hardware constraint)
| Feature | Donor reference | Notes |
|---|---|---|
| 126-channel 2.4GHz spectrum sweep with peak-hold | UniGeek `utils/nrf24/`, Poseidon | |
| Jammer (10 curated channel-list modes: WiFi/BLE/BT-Classic/USB-dongle/FPV/RC/Zigbee/Full, sequential or FHSS hop) | UniGeek (most complete mode list of the three donors), Bruce `rf_jammer.cpp` equivalent for nRF24 | |
| MouseJack (CVE-2016-10761-class, unencrypted keyboard/mouse dongle detect + HID keystroke injection) | Bruce `nrf_mousejack.cpp` (marked WIP in Bruce itself), UniGeek (fingerprints Microsoft/Logitech, up to 12 targets/session — most mature of the three) | UniGeek's version is the primary reference given Bruce's own README flags theirs as unfinished |
| Promiscuous ESB sniffer (Travis Goodspeed trick) | Poseidon | |
| BLE spam via nRF24 (ADV_IND, CRC24+whitening) | Poseidon | Distinct from Phase 2's Tab5-native BLE spam — this is the nRF24 doing raw 2.4GHz framing to mimic BLE advertisements, not a real BLE radio; useful as a *second* concurrent BLE-spam source alongside the Tab5's, or as a fallback if Phase 2's esp-hosted BLE spike (see Phase 2 spec Risks) fails |

## 2. Architecture

### 2.1 Module Structure and Radio-Exclusivity Enforcement

```
firmware/cardputer-adv/src/features/
├── ir/
│   ├── ir_tvbgone.{h,cpp}
│   ├── ir_learn.{h,cpp}
│   ├── ir_clone.{h,cpp}          # Flipper-IRDB-compatible, ported from UniGeek
│   └── ir_jammer.{h,cpp}
├── cc1101/
│   ├── cc1101_scan.{h,cpp}
│   ├── cc1101_record.{h,cpp}     # .sub format read/write
│   ├── cc1101_replay.{h,cpp}
│   ├── cc1101_spectrum.{h,cpp}
│   ├── cc1101_bruteforce.{h,cpp}
│   ├── cc1101_jammer.{h,cpp}
│   ├── cc1101_keeloq.{h,cpp}
│   └── cc1101_hw.{h,cpp}         # SmartRC-CC1101-Driver-Lib wrapper, owns the shared hat SPI bus
├── nrf24/
│   ├── nrf24_spectrum.{h,cpp}
│   ├── nrf24_jammer.{h,cpp}
│   ├── nrf24_mousejack.{h,cpp}
│   ├── nrf24_sniffer.{h,cpp}
│   ├── nrf24_ble_spam.{h,cpp}
│   └── nrf24_hw.{h,cpp}          # RF24 lib wrapper, owns the shared hat SPI bus
└── hat_radio_lock.{h,cpp}        # arbitrates CC1101 vs nRF24 exclusivity
```

`hat_radio_lock.h` is the piece that doesn't exist in any donor project as a standalone concept but is required by this hardware: since CC1101 and nRF24 share the same physical Grove pins on the hydra hat (Phase 1's confirmed constraint), only one can be initialized at a time.

```cpp
// firmware/cardputer-adv/src/features/hat_radio_lock.h
#pragma once

enum class HatRadio { NONE, CC1101, NRF24 };

namespace HatRadioLock {
bool acquire(HatRadio which);   // false if the other radio is already active
void release(HatRadio which);
HatRadio current();
}
```

Every CC1101 and nRF24 feature's `on_start` callback calls `HatRadioLock::acquire()` before touching hardware and `on_stop`/completion calls `release()`. If acquisition fails (the other hat radio is active), the feature reports a clear "switch hat radio first" status back over the C2 link rather than silently failing or corrupting SPI transactions — this must surface as real UI feedback on the Tab5, not just a Cardputer-ADV-local log line, since the Tab5 is the primary point of interaction and a remote user needs to know *why* a feature refused to start.

### 2.2 Capability Negotiation Reflects Hardware State

Extending Phase 1's capability negotiation (`CommandDispatcher` reporting registered feature IDs on connect): the Cardputer-ADV should also report *current hat radio state* (`HatRadioLock::current()`) in its `RESP_STATUS` payload, so the Tab5 UI can gray out CC1101 tiles while nRF24 is active and vice versa, rather than the user discovering the conflict only after tapping a tile and getting a rejection.

### 2.3 Remote vs Local Operation

Every feature module's core logic (the `on_start`/`on_stop`/telemetry functions) is identical whether triggered from the local Cardputer-ADV menu or from a Tab5-originated C2 command — per Phase 1's HAL design principle, feature logic is never duplicated between standalone and remote-controlled paths. The `LocalMenu` (Phase 1 Task 16) gains real entries for every feature in this phase's inventory, each calling the same `on_start` the `CommandDispatcher` calls remotely.

### 2.4 Spectrum/Waterfall Visualization on a Small Screen

Cardputer-ADV's 240×135 screen is cramped for CC1101's spectrum/waterfall/radar visual modes (Poseidon's most visually developed feature). Two options, not mutually exclusive:
1. Render locally on Cardputer-ADV at reduced fidelity (matches donor precedent, works standalone).
2. Stream the raw scan data (not a rendered image) back to the Tab5 over the C2 bulk channel, where it's rendered at full 1280×720 fidelity using LVGL's `lv_chart` — a genuinely better experience than any donor project can offer on their native hardware, and a natural showcase for the "why build a Tab5 command center" premise.

Recommend implementing (1) first for standalone parity, then (2) as a natural enhancement once the bulk channel is proven in Phase 1 — but this is a judgment call worth confirming with the project owner before the Phase 5 implementation plan locks it in.

## 3. Risks / Open Questions

- **IR receive hardware presence on this specific Cardputer-ADV unit is unconfirmed** (see Section 1) — resolve before planning receive/decode/clone tasks.
- **KeeLoq rolling-code replay** is the most legally sensitive feature in this program's full inventory (attacking rolling-code locks, e.g. vehicle/garage systems, is meaningfully different from passive scanning even under an owner's own-equipment authorization) — flagged here for explicit acknowledgment, not blocking, since the project owner has already established authorization for this kind of work.
- **Hat radio exclusivity UX**: `hat_radio_lock`'s rejection path needs to be genuinely clear to a remote Tab5 user, not just logged locally — this is called out as an explicit Definition of Done item below because it's easy to under-build (a silent no-op) and hard to notice missing until a real remote user hits it.

## 4. Testing Strategy

- Host-native tests: `.sub`/Flipper-format parsers, protocol decoders (Princeton/CAME/NICE/Linear/KeeLoq), MouseJack fingerprint matching logic, `hat_radio_lock`'s acquire/release/conflict-rejection state machine (fully testable without hardware).
- On-device verification against real 433MHz devices you own (for CC1101 scan/replay/bruteforce) and real 2.4GHz peripherals (an unencrypted wireless mouse/keyboard dongle for MouseJack testing, a spare BLE device for nRF24 BLE-spam).
- Explicit test of the hat-radio-conflict path: start a CC1101 feature, attempt to start an nRF24 feature from the Tab5 UI while CC1101 is active, confirm the Tab5 shows a clear rejection message rather than a silent failure or hang.
- Explicit test of standalone/remote parity: trigger the same feature once from Cardputer-ADV's local menu and once from the Tab5 UI, confirm identical behavior.

## 5. Definition of Done

1. IR receive-hardware presence confirmed; feature set scoped accordingly (transmit-only vs full receive+transmit).
2. Every CC1101 feature in Section 1 verified on real 433MHz hardware, including at least one non-433MHz test signal within the hat's usable 300–900MHz range to confirm the "tuned for 433 but usable wider" hardware claim holds for scan/spectrum features.
3. Every nRF24 feature in Section 1 verified against real 2.4GHz targets.
4. `hat_radio_lock` conflict rejection verified end-to-end from the Tab5 UI (clear user-facing message, not silent failure).
5. Capability negotiation correctly reflects live hat-radio state on the Tab5's Devices panel.
6. At least one feature from each of IR/CC1101/nRF24 verified working identically from Cardputer-ADV's local menu and from Tab5 remote control.
