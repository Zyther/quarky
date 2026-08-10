# Phase 6: GNSS/SX1262 LoRa on Cardputer-ADV — Design

**Status:** Draft for review
**Date:** 2026-08-06
**Depends on:** Phase 1 Foundation — Cardputer-ADV `Device`/`IC2Link`/`CommandDispatcher`/`FeatureModule` contract; Phase 5's `hat_radio_lock` pattern (extended here to a second hat, since the GNSS/SX1262 hat is physically separate from the CC1101/nRF24 hydra hat and can run concurrently with it).
**Scope:** LoRa (SX1262) and GNSS features on the owner's confirmed GNSS+SX1262 hat. This is the weakest-precedent phase — none of the three donor projects have a working, complete implementation to port from directly, so more of this phase is original design than adaptation.

## 1. Why This Phase Is Different From 2–4

Recap from the foundation research:
- **Bruce** has real SX1262/SX1276 support via RadioLib (`LoRaRF.cpp`), proven on the LilyGO T-Lora Pager — this is the most usable reference for the *driver* layer, but Bruce has no GNSS-tagged LoRa feature and no GNSS module support tied to it (its GPS/wardriving code is WiFi-only).
- **Poseidon** has the most feature-complete LoRa/GNSS *feature* set on paper (LoRa scan, beacon TX, Meshtastic LongFast listener, Meshtastic chat/nodes/page/position, live GPS fix) but ships it entirely unverified against this program's hardware — Poseidon's own hardware table lists LoRa+GPS via a different hat (M5Stack CAP-LoRa1262) than a GNSS+SX1262 combo unit, so pin mapping won't transfer directly.
- **UniGeek** has SX1262 pins electrically reserved on Cardputer-ADV but the driver is explicitly unimplemented (open TODO) — confirms the physical wiring is plausible but contributes no working code.

Net effect: **RadioLib (Bruce's dependency) is the one piece of proven, reusable code for the radio driver itself.** Everything above the driver layer — GNSS parsing, Meshtastic protocol handling, the scan/beacon/chat feature UI — is closer to a fresh build guided by Poseidon's *feature list* as a spec, not by any donor's working code.

## 2. Feature Inventory and Source Mapping

### LoRa (SX1262)
| Feature | Donor reference | Notes |
|---|---|---|
| LoRa scan (listen on a frequency/spreading-factor combo, log received packets) | Poseidon `lora_hw.cpp/.h`, `radio_lora.cpp` (unverified against this hardware) | Driver layer: Bruce's `RadioLib` `LoRaRadioVariant` SX1262 path is the proven starting point |
| Beacon TX (send a test/identifier packet) | Poseidon | Straightforward once scan/RX is working, since it's the same radio in TX mode |
| LoRa analyzer (RSSI/SNR over time) | Poseidon | |
| Meshtastic LongFast listener (passive decode of public Meshtastic mesh traffic) | Poseidon `src/mesh/meshtastic*.{c,h}` | Meshtastic's protocol is public/documented (protobuf-based); this is the most implementation-heavy item in the phase — treat as its own sub-task with real protocol-spec reference (meshtastic.org's firmware/protobufs), not just Poseidon's code, since Poseidon's own version is unverified |
| Meshtastic chat/nodes/page/position | Poseidon `mesh_chat/nodes/page/position/status.cpp` | Builds on the LongFast listener; realistically sequenced *after* passive listening/decoding is solid |

### GNSS
| Feature | Donor reference | Notes |
|---|---|---|
| Live GPS fix (lat/lon/alt/time display) | Poseidon `gps.cpp/.h` (via `TinyGPSPlus`, also used by Bruce for its WiFi wardriving) | `TinyGPSPlus` is a proven, widely-used NMEA parser — safe to depend on directly regardless of donor-specific integration code |
| GPS wardriving (tags WiFi/BLE scan results with location) | Bruce `wardriving.cpp`, Poseidon `wifi_wardrive.h` | **This is the completion of Phase 2's deferred wardriving row** — Phase 2 built the WiFi-scan half on the Tab5; this phase adds the GPS-tagging half on Cardputer-ADV. Requires a C2 protocol addition: Cardputer-ADV streams GPS fixes to the Tab5 (or the Tab5 requests a fix on demand) so Tab5-native WiFi scan results can be tagged — this is new cross-device coordination, not present in any single donor project, since none of them split WiFi-scan and GPS across two physically separate devices. |
| WiGLE CSV export | Bruce `wigle.cpp` | Export format only, straightforward once wardriving data exists |

## 3. Architecture

### 3.1 Module Structure

```
firmware/cardputer-adv/src/features/
├── lora/
│   ├── lora_hw.{h,cpp}           # RadioLib SX1262 wrapper, owns the GNSS/SX1262 hat's SPI bus
│   ├── lora_scan.{h,cpp}
│   ├── lora_beacon.{h,cpp}
│   ├── lora_analyzer.{h,cpp}
│   └── meshtastic/
│       ├── mt_longfast_listener.{h,cpp}
│       ├── mt_protobuf_decode.{h,cpp}   # Meshtastic protobuf schema decode
│       ├── mt_chat.{h,cpp}
│       ├── mt_nodes.{h,cpp}
│       └── mt_position.{h,cpp}
└── gnss/
    ├── gnss_hw.{h,cpp}           # TinyGPSPlus wrapper
    ├── gnss_live_fix.{h,cpp}
    └── gnss_wardrive_bridge.{h,cpp}  # streams fixes to Tab5 over C2 for wardrive tagging
```

### 3.2 Cross-Device Wardriving Coordination (New Protocol Surface)

This is the one place in Phase 6 that extends `shared/c2proto` beyond what Phase 1 defined. Two workable shapes:

**Option A — push model:** Cardputer-ADV streams `RESP_TELEMETRY` frames containing GPS fixes at a fixed interval (e.g. 1Hz) whenever wardriving mode is active; Tab5 buffers the latest fix and stamps it onto WiFi scan results as they arrive locally.

**Option B — pull model:** Tab5 sends a lightweight `CMD_GET_STATUS`-style request for "current GPS fix" each time it needs to tag a scan result; Cardputer-ADV responds once per request.

Recommend **Option A** — wardriving is inherently a streaming/continuous activity (matches how both Bruce and Poseidon model it), and a 1Hz GPS fix is small enough to fit comfortably in the existing ESP-NOW control-channel frame size (`kMaxPayload = 200` bytes from Phase 1, well over what a lat/lon/alt/timestamp struct needs), so no bulk-channel involvement is needed. This reuses Phase 1's `RESP_TELEMETRY` message type rather than requiring a new `MsgType` — the payload's internal structure (a `GnssFix` struct: `float lat, lon; int16_t alt_m; uint32_t unix_time`) is new but the framing isn't.

### 3.3 Hat Radio Exclusivity

Unlike Phase 5's CC1101/nRF24, the GNSS+SX1262 hat is a **separate physical unit** from the hydra hat (per the owner's confirmed hardware inventory: two distinct hats). Confirm during implementation whether Cardputer-ADV's SPI bus can genuinely run both hats concurrently (UniGeek's research noted SD, LoRa-CS, CC1101, and nRF24 all share *one* SPI bus electrically, even though LoRa's chip-select is a separate pin from CC1101/nRF24's) — if concurrent operation isn't reliable, extend `hat_radio_lock` (Phase 5) to arbitrate three-way (CC1101 / nRF24 / SX1262) rather than assuming the GNSS/SX1262 hat is automatically free of the exclusivity constraint just because it's a different physical unit.

## 4. Risks / Open Questions

- **Highest implementation risk in the whole program**: this phase has no working reference implementation to port, only a driver library (RadioLib) and a feature list (Poseidon's, unverified). Budget more exploratory/spike time here than Phases 2–5, which are genuine ports.
- **Meshtastic protocol compatibility** depends on tracking Meshtastic's own protobuf schema versions — verify against a current Meshtastic device/app if available, since a stale schema silently fails to decode rather than erroring clearly.
- **SPI bus sharing between the two hats** is unconfirmed (Section 3.3) — resolve early, since it determines whether `hat_radio_lock` needs a three-way extension or the GNSS/SX1262 hat is genuinely independent.
- **GNSS cold-fix time** (potentially several minutes outdoors, longer/never indoors) means on-device testing for this phase needs actual outdoor time with sky visibility — plan bring-up sessions accordingly rather than expecting quick desk-testing turnaround.

## 5. Testing Strategy

- Host-native tests: Meshtastic protobuf decode against captured/known-good sample payloads (obtainable from a real Meshtastic device or public sample captures), NMEA sentence parsing edge cases (already covered by TinyGPSPlus's own test suite — don't re-test the library, test only the integration glue), WiGLE CSV format writer.
- On-device: LoRa scan/beacon tested against a second LoRa radio you control (even a basic LoRa dev board on the same frequency/SF works as a test partner) rather than only listening passively, so both TX and RX paths get exercised.
- GNSS live-fix tested outdoors with real sky visibility; do not treat an indoor "no fix" result as a failure.
- Meshtastic listener tested against a real Meshtastic device broadcasting on LongFast if one is available; if not, this row's Definition of Done should explicitly note "protocol-decode-verified against sample captures, live-mesh-unverified" rather than being silently skipped.

## 6. Definition of Done

1. RadioLib SX1262 driver confirmed working on the actual GNSS/SX1262 hat hardware (basic TX/RX proven before any feature logic is built on top).
2. LoRa scan, beacon, and analyzer verified against a real second LoRa radio.
3. Meshtastic LongFast listener verified against either a real Meshtastic device or documented sample captures (with the distinction noted per Section 5).
4. GNSS live fix verified outdoors with real satellite lock.
5. Wardriving cross-device coordination (Section 3.2) verified end-to-end: a Tab5-native WiFi scan result correctly tagged with a Cardputer-ADV-sourced GPS fix, exported to a valid WiGLE CSV.
6. Hat concurrency question (Section 3.3) resolved and `hat_radio_lock` updated accordingly if needed.
