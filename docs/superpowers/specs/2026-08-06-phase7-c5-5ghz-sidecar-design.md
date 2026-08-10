# Phase 7: ESP32-C5 5GHz Sidecar — Design

**Status:** Draft for review
**Date:** 2026-08-06
**Depends on:** Phase 1 Foundation — `shared/c2proto` (extended here for a second satellite class), `FeatureModule` contract's `Affinity::C5_NODE` (defined in Phase 1 but unused until now).
**Scope:** A new, third firmware target (`firmware/c5-node/`) providing 5GHz WiFi scan/deauth/PMKID — the one radio capability nothing else in this kit can reach, since neither the Tab5's C6 nor the Cardputer-ADV's S3 support 5GHz.
**Hardware note:** this phase requires acquiring an ESP32-C5 dev board — it is not part of the owner's currently-confirmed hardware inventory (Tab5, Cardputer-ADV, hydra hat, GNSS/SX1262 hat, HY2.0 units). Flagging this explicitly since every other phase's Definition of Done assumes hardware-in-hand; this one doesn't yet.

## 1. Source Mapping

This phase has the strongest single-source precedent of the whole program: **Poseidon's TRIDENT node (`~/src/poseidon-tab5/c5_node/`) is a real, "shipped and hardware-verified" ESP32-C5 satellite doing exactly this job** — 5GHz WiFi scan/deauth/PMKID plus Zigbee, linked to a brain device over ESP-NOW. The architecture this phase needs (C5 satellite, ESP-NOW-linked, commanded by a brain device) is the one piece of the entire three-donor codebase that's a near-exact structural match to what we need, just with the Tab5 standing in for Poseidon's Cardputer as the brain.

| Feature | Donor reference | Notes |
|---|---|---|
| 5GHz WiFi scan | Poseidon `c5_node/main/wifi_scanner.c` | ESP-IDF native (not Arduino) — TRIDENT is a standalone ESP-IDF project, not PlatformIO/Arduino |
| 5GHz deauth | Poseidon `c5_node/main/wifi_attacker.c` | |
| PMKID capture | Poseidon `c5_node/main/pmkid_capture.c` | |
| Handshake capture | Poseidon `c5_node/main/hs_capture.c` | Poseidon's own docs flag this as constrained by ESP-NOW's payload size — directly informed Phase 1's decision to split control/bulk channels; this phase should use Phase 1's bulk WiFi-socket channel for handshake exfiltration rather than inheriting TRIDENT's original limitation |
| Zigbee sniffing | Poseidon `c5_node/main/zb_sniffer.c` | The C6 (Tab5's co-processor) already supports 802.15.4/Zigbee per its spec sheet — **this specific feature may belong on the Tab5 natively instead of the C5 satellite**; worth a scoping decision early in this phase's implementation plan rather than assuming it belongs here just because TRIDENT has it |
| ESP-NOW C2 protocol (`posei_msg_t`) | Poseidon `c5_node/main/proto.h`, `proto.c` | **Not reused directly** — this program already has its own `shared/c2proto` from Phase 1, which was explicitly designed with TRIDENT's payload-size lesson already baked in. The C5 node in this program speaks `c2proto`, not `posei_msg_t`. |

## 2. Architecture

### 2.1 Framework Choice: ESP-IDF, Not Arduino

Unlike Tab5 and Cardputer-ADV (both Arduino-ESP32 per Phase 1's foundation decision), this phase's C5 node follows **Poseidon's TRIDENT precedent and uses native ESP-IDF**, for the same reasons TRIDENT does: 5GHz WiFi + raw frame injection on the C5 is lower-level, performance-sensitive work where TRIDENT's own team chose IDF over Arduino, and there's no Arduino library reuse motivation here the way there was for Tab5/Cardputer-ADV (this node has no display, no UI, no NFC/RF433 peripherals — it's a pure headless radio satellite, so Arduino's main value-add — broad library ecosystem reuse — doesn't apply). `shared/c2proto` and `shared/feature_contract` (Phase 1) are plain C++ with no Arduino dependency, so both compile cleanly into an ESP-IDF component without modification — confirmed by Phase 1's native PlatformIO test environments already building them with plain `gnu++17`, no Arduino framework involved.

### 2.2 Repo Structure

```
firmware/c5-node/
├── CMakeLists.txt              # ESP-IDF project, not PlatformIO — matches TRIDENT's own build system
├── main/
│   ├── main.c
│   ├── wifi_scanner.c/.h
│   ├── wifi_attacker.c/.h        # deauth
│   ├── pmkid_capture.c/.h
│   ├── hs_capture.c/.h
│   └── c2_bridge.c/.h            # adapts shared/c2proto (C++) to this component's C code via an extern "C" shim
└── components/
    └── c2proto_shim/             # wraps shared/c2proto and shared/feature_contract as an ESP-IDF component
```

### 2.3 Link Topology: Direct to Tab5, Not Through Cardputer-ADV

Per the Phase 1 foundation spec's device-role definition, the C5 node is a **pure satellite of the Tab5 directly** — it does not route through Cardputer-ADV. This mirrors TRIDENT's own topology (linked to the Cardputer brain, not through any intermediary) and keeps the C2 topology simple: the Tab5 maintains two independent ESP-NOW peer relationships (Cardputer-ADV from Phase 1, C5-node from this phase), each with its own PSK from the same pairing flow (Phase 1 Task 12's QR/typed-key pattern, run a second time for this device).

### 2.4 Zigbee Scoping Decision

Flagged in Section 1: the C6 co-processor (already on the Tab5) supports 802.15.4/Zigbee/Thread natively, per Phase 1's confirmed hardware research. Recommend Zigbee sniffing be scoped as a **Tab5-native feature (added to Phase 2's WiFi/BLE suite retroactively, or its own small Phase 2.5)** rather than built into this C5 node, since it doesn't need 5GHz and doesn't need a satellite at all — TRIDENT only has it because Poseidon's Cardputer (ESP32-S3) has no 802.15.4 capability of its own, which is not our situation. This should be confirmed with the project owner before the Phase 7 implementation plan is written, since it changes which phase's plan actually contains this feature.

## 3. Risks / Open Questions

- **Hardware not yet acquired** (Section header note) — this phase can be fully spec'd and implementation-planned ahead of time, but on-device Definition-of-Done verification blocks on the owner acquiring an ESP32-C5 dev board.
- **ESP-IDF/Arduino boundary**: `shared/c2proto` and `shared/feature_contract` need to prove they genuinely compile as a plain ESP-IDF component with no hidden Arduino-only assumptions creeping in from Phases 2–6's work on the Arduino targets — worth a compile-check spike early in this phase rather than discovering an Arduino dependency crept in only once this phase is underway.
- **PMKID/handshake capture exfiltration** over the Phase 1 bulk WiFi socket depends on the C5 node being able to join the same WiFi network as the Tab5 (or run its own AP) — TRIDENT's original design didn't need this since it stayed within ESP-NOW's limits by accepting the fragmentation constraint; confirm the bulk-channel approach (a deliberate improvement over TRIDENT) doesn't reintroduce complexity TRIDENT avoided by simply accepting smaller capture windows.
- **Zigbee scoping** (Section 2.4) should be settled as an explicit decision, not left ambiguous, before this phase's implementation plan is written.

## 4. Testing Strategy

- `c2proto`/`feature_contract` ESP-IDF-component compile check as an early spike (Risks section) — a build-only test, no hardware needed.
- 5GHz scan/deauth/PMKID verified against a real 5GHz-capable test AP/router you control.
- C2 link (both control and bulk channels) verified end-to-end against the Tab5, reusing the same manual verification pattern established in Phase 1 Task 20 (ping-style round trip) before layering real 5GHz features on top.

## 5. Definition of Done

1. ESP32-C5 hardware acquired.
2. Zigbee scoping decision made and documented (this phase vs. Tab5-native).
3. `shared/c2proto` and `shared/feature_contract` confirmed building cleanly as ESP-IDF components.
4. C5 node pairs with the Tab5 via the same PSK/QR flow as Cardputer-ADV, verified with a ping-style round trip (mirroring Phase 1 Task 20).
5. 5GHz scan, deauth, and PMKID capture all verified against a real 5GHz test AP.
6. Handshake/PMKID data exfiltration verified over the Phase 1 bulk WiFi socket channel.
