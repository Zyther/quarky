# Phase 10: Tab5-Native CC1101 via M-Bus — Design

**Status:** Draft for review
**Date:** 2026-08-09
**Depends on:** Phase 1 Foundation — `IRadio` pattern (Task 9, as an architectural precedent for a Tab5-native radio HAL, though CC1101 is SPI not the C6's WiFi), `IStorage`/SD (Task 10), `FeatureModule`/`FeatureRegistry` contract, LVGL shell/launcher. Not dependent on Phase 5 (Cardputer-ADV Satellite's hydra-hat CC1101) architecturally — this is a physically separate CC1101 unit on a different device — but should reuse the same driver library and, where the logic is genuinely hardware-agnostic, the same protocol-decode code (see Section 4.3).
**Scope:** An M5Stack CC1101 Module (855-925MHz, tuned by the project owner toward the 868-925MHz sub-range) attached directly to the Tab5's own onboard M5-Bus stacking connector — not the Cardputer-ADV's hydra hat. Phase number deliberately non-sequential (per project owner direction, mirroring how the Chameleon Ultra phase was inserted by renumbering rather than appended) — this phase is intentionally the highest number in the current roadmap, added after the original 1-8 program was drafted, once this specific hardware became available.

## 1. What This Adds Beyond Phase 5's Hydra-Hat CC1101

Phase 5 already specs a full CC1101 feature set for the Cardputer-ADV's hydra hat. This phase is the same *class* of hardware (an M5Stack-packaged CC1101 module) attached to the Tab5 instead, directly, with no satellite/C2 relay needed for its own operation — it runs entirely Tab5-native, the same pattern Phase 2 (Tab5 WiFi/BLE) established: no Cardputer-ADV involvement for the radio work itself, whatever UI/launcher integration exists is local to the Tab5.

Two CC1101 units existing on two different devices is a deliberate hardware choice by the project owner (this module was ordered separately from the hydra hat), not a redundancy to eliminate — keep both. A user might run Tab5-native sub-GHz work standing alone (no Cardputer-ADV in the loop at all) or in parallel with the Cardputer-ADV's own hydra-hat CC1101 work for two-radio scenarios (e.g., simultaneous scan on one and replay on the other).

## 2. Real Reference Material

- **The physical module**: M5Stack "CC1101 Module (855-925MHz)" — a real, currently-shipping product (confirmed via M5Stack's own store listing and official docs at `docs.m5stack.com/en/module/Module_CC1101`), built around the E07-900M10S module (TI CC1101 transceiver), SPI interface, supporting 2-FSK/4-FSK/GFSK/MSK/ASK/OOK modulation, RSSI/LQI reporting, independent 64-byte RX/TX FIFOs.
- **M-Bus attachment, per the module's own official docs**: MOSI on M-Bus pin 7, MISO on pin 9, SCK on pin 10, CSN on pin 8 (DIP-switch selectable), GDO0/GDO2 interrupt lines DIP-switch selectable across a shared set of pins (module docs list pins 1/20/22 as CSN/GDO0/GDO2 candidates — **the module's own documentation does not specify which physical GPIO on any particular host those M-Bus pin *positions* correspond to**; that mapping is host-specific and must be sourced from the host's own schematic, not assumed from the module's side of the connector).
- **Driver library**: reuse `SmartRC-CC1101-Driver-Lib`, the same library Phase 5's `cc1101_hw.{h,cpp}` wraps, for consistency — one proven CC1101 driver, two host integrations.
- **Feature logic donor references**: same as Phase 5's CC1101 table (Poseidon `subghz_*.cpp`/`cc1101_hw.cpp/.h`, Bruce `rf_record.cpp`/`rf_send.cpp`/`rf_bruteforce.cpp`, UniGeek's broader protocol-decoder set) — do not re-list here, see Phase 5 Section 1 for the full feature-to-donor mapping; port the same way.

## 3. Real Hardware Fact Still Needed Before Implementation

**Tab5's own M5-Bus-pin-to-P4-GPIO mapping is not yet confirmed and must not be assumed.** A generic M5-Bus pinout table exists at `docs.m5stack.com/en/learn/interface/mbus`, but it documents classic-ESP32 GPIO numbering (e.g. `G23`/`G19`/`G18` for MOSI/MISO/SCK) inherited from the original Core/Core2-era M5-Bus products — the Tab5 is an ESP32-P4 (an entirely different chip generation with different GPIO numbering), so that table almost certainly does not apply verbatim. This is exactly the failure mode this project has hit repeatedly already (the eval-board's C6 SDIO pins being wrong for the real Tab5, ST7123-vs-ST7121 panel misidentification, PN532-vs-actual-chip assumptions on the HY2.0 units) — a generic/adjacent-product datasheet number that looks authoritative but is wrong for this specific board revision.

Before any wiring/driver work starts: source Tab5's actual M-Bus-to-GPIO mapping from Tab5's own schematic or pinout documentation (the same class of source this project used successfully for the C6 SDIO pins — espp's BSP, M5Stack's own Tab5-specific docs — not the generic M5-Bus page). If no Tab5-specific M-Bus pin documentation exists publicly, this becomes a real-hardware continuity-test task (probe each M-Bus pin position with the physical module attached and a multimeter/logic analyzer, or trace against a real Tab5 schematic if M5Stack publishes one) before any SPI transaction can be attempted.

Also confirm, once the module is in hand: which DIP-switch position it's actually set to for CSN/GDO0/GDO2 (the module supports multiple selectable pin positions precisely because different host devices wire the M-Bus differently) — read the switches directly rather than assuming a default.

## 4. Architecture

### 4.1 Module Structure

```
firmware/tab5/src/features/cc1101/
├── cc1101_scan.{h,cpp}
├── cc1101_record.{h,cpp}
├── cc1101_replay.{h,cpp}
├── cc1101_spectrum.{h,cpp}
├── cc1101_bruteforce.{h,cpp}
├── cc1101_jammer.{h,cpp}
├── cc1101_keeloq.{h,cpp}
└── cc1101_hw.{h,cpp}         # SmartRC-CC1101-Driver-Lib wrapper, Tab5 M-Bus SPI pins
```

Mirrors Phase 5's `firmware/cardputer-adv/src/features/cc1101/` module list closely (same feature set, same underlying driver library) — this parallel structure is intentional, not accidental duplication.

### 4.2 Shared Logic vs. Duplication (real design decision, not deferred)

Protocol decode/identify tables, `.sub` file format read/write, and KeeLoq decode logic are pure, hardware-agnostic logic — identical whether the CC1101 bytes came from the Tab5's M-Bus module or the Cardputer-ADV's hydra hat. Rather than maintaining two independent copies of this logic across `firmware/tab5/` and `firmware/cardputer-adv/` (a real duplication risk once both phases exist), factor this into a new **`shared/subghz_proto`** library — following this project's established pattern (`shared/c2proto`, `shared/feature_contract`) of pulling reusable, pure-logic code out of any one firmware tree and into `shared/`, with its own native test suite. Both `cc1101_protocol_decode.cpp` (this phase) and Phase 5's equivalent module should consume the same `shared/subghz_proto` functions rather than each hand-rolling their own.

Only the hardware-facing half (`cc1101_hw.{h,cpp}` — SPI bus ownership, GPIO pins, DIP-switch-dependent pin selection) stays firmware-tree-specific, since that's genuinely different per host device.

If this phase lands before Phase 5 is implemented (plausible, since the CC1101 module arrives "in a few weeks" per the owner's stated timeline and Phase 5's own hydra-hat hardware timeline is unspecified here), this phase should create `shared/subghz_proto` itself rather than waiting for Phase 5 to do it — Phase 5's implementation then consumes what this phase already built, and its own spec/plan should be revisited at that time to reference this shared library instead of a from-scratch port.

### 4.3 UI Pattern

Same list-and-select / progress-and-stop patterns Phase 2 (WiFi scan) and Phase 5 (hydra-hat CC1101) already establish — no new UI pattern needed, reuse what exists.

### 4.4 Data Format

Captures land in `/quarky/captures/subghz/` (distinct from Phase 3's `/quarky/captures/rf433/`, since this is full CC1101-class capability — spectrum/waterfall data, `.sub`-format raw captures, not just fixed-code RF433 signals) — continuing the established `/quarky/captures/<category>/` convention.

## 5. Risks / Open Questions

- **M-Bus GPIO mapping is the single blocking unknown** (Section 3) — nothing in this phase can start against real hardware until it's resolved.
- **Frequency range**: the module's advertised range is 855-925MHz; the owner's stated intended tuning (868-925MHz) is a subset of that, consistent with the real hardware — no discrepancy, just note the module's full documented range in code/UI rather than hard-coding the owner's narrower intended-use band as a hard limit, in case future work wants the module's full range.
- **Physical/electrical conflict with other M-Bus-attached hardware**: if any other M5-Bus module is ever stacked simultaneously (none currently planned), confirm SPI bus sharing behavior — out of scope until it's an actual configuration, flagged here only so it isn't forgotten.
- **`shared/subghz_proto`'s existence depends on which phase (this one or Phase 5) actually gets implemented first** — see Section 4.2. Whichever phase's implementation plan is written second should explicitly check for the shared library's existence rather than assume it doesn't exist yet.

## 6. Testing Strategy

- Protocol decode/identify, `.sub` format read/write, and KeeLoq logic: native host tests against `shared/subghz_proto`, following the same table-driven-against-known-samples approach Phase 3 specs for its own protocol decode work.
- SPI bus bring-up and real RF scan/replay: real hardware only, once the M-Bus GPIO mapping (Section 3) is confirmed — same "flash it, read real serial output, fix what's actually wrong" discipline this project has used for every other radio HAL bring-up so far.
- Scan/replay verified against the project owner's own equipment (garage door remote, doorbell, etc.), matching Phase 5's testing-strategy language for the same class of feature.

## 7. Definition of Done

- [ ] Tab5's real M-Bus-to-GPIO mapping confirmed from an authoritative, Tab5-specific source (not the generic Core-series M-Bus table) and recorded with citations, the same way the C6 SDIO pins were documented in Phase 1.
- [ ] CC1101 module's DIP-switch position (CSN/GDO0/GDO2) confirmed against the physical unit in hand.
- [ ] `shared/subghz_proto` exists (created by this phase or already present from Phase 5) and both device trees' CC1101 feature modules consume it rather than duplicating protocol logic.
- [ ] At least one real scan/capture and one real replay demonstrated against the owner's own RF equipment, on real hardware.
- [ ] This phase's `docs/phases/phase-10-tab5-cc1101-mbus.md` write-up completed per this program's per-phase documentation convention (`CLAUDE.md`), including the resolved M-Bus pin mapping as a durable reference for any future M-Bus module work.
