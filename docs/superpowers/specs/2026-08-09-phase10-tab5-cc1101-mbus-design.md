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
- **M-Bus attachment, per the module's own official docs**: MOSI on M-Bus pin 7, MISO on pin 9, SCK on ~~pin 10~~ **pin 11** (corrected 2026-08-24 — pin 10 was this section's original pre-research guess, before the module's own real PinMap table was actually read; see Section 3), CSN DIP-switch-selectable across pins 8/21/23/24, GDO0/GDO2 interrupt lines DIP-switch-selectable across a shared pool of pins 2/20/22 (**the module's own documentation does not specify which physical GPIO on any particular host those M-Bus pin *positions* correspond to**; that mapping is host-specific and must be sourced from the host's own schematic, not assumed from the module's side of the connector — Section 3 has the full resolution).
- **Driver library**: `jgromes/RadioLib` (corrected 2026-08-24 — this section originally specified `SmartRC-CC1101-Driver-Lib`; real-hardware testing during Task 1 found a repeat-call reliability bug in that library, and separately, M5Stack's own CC1101 module docs turned out to document RadioLib, not SmartRC, as this module's own Arduino library — see Section 3's Task-1 note and `platformio.ini`'s citation for the full account). **If Phase 5 (Cardputer-ADV hydra-hat CC1101) is implemented after this phase, it should also use RadioLib**, not the originally-planned SmartRC, for this same "one proven driver, two host integrations" consistency goal to still hold — check this file's own update rather than trusting Phase 5's own spec text verbatim if it still says SmartRC at that time.
- **Feature logic donor references**: same as Phase 5's CC1101 table (Poseidon `subghz_*.cpp`/`cc1101_hw.cpp/.h`, Bruce `rf_record.cpp`/`rf_send.cpp`/`rf_bruteforce.cpp`, UniGeek's broader protocol-decoder set) — do not re-list here, see Phase 5 Section 1 for the full feature-to-donor mapping; port the same way.

## 3. Real Hardware Fact: M-Bus GPIO Mapping (RESOLVED 2026-08-24)

**Resolved from real, Tab5-specific and module-specific vendor PDFs** (`docs/vendor/Tab5.pdf`, M5Stack's own Tab5 datasheet, "Tab5 Board PinMap Overview" page, Update Time 2026-08-05; `docs/vendor/Module_CC1101.pdf`, M5Stack's own CC1101 Module datasheet, PinMap + schematic pages, Update Time 2026-01-23) — not the generic `docs.m5stack.com/en/learn/interface/mbus` Core-series table, which documents classic-ESP32 GPIO numbering (`G23`/`G19`/`G18` for MOSI/MISO/SCK) that does not apply to the Tab5's ESP32-P4. This is the same failure mode this project has hit repeatedly before (eval-board C6 SDIO pins, ST7123-vs-ST7121 panel ID, PN532-vs-actual-chip assumptions) — resolved here by going to the two devices' own datasheets rather than an adjacent/generic one, and cross-validating both documents' pin tables against each other position-by-position (M5-Bus connector positions 1-30 are a standardized physical layout shared across both devices, confirmed identical in both PDFs' own tables).

**Fixed pins (no DIP switch involved) — MOSI/MISO/SCK are wired straight through on both the module and the Tab5's own M-Bus connector at positions 7/9/11:**

| Signal | M-Bus position | Tab5 GPIO |
|---|---|---|
| MOSI | 7 | **G18** |
| MISO | 9 | **G19** |
| SCK | 11 | **G5** |

**Switch-selectable pins — CC1101 module has SW1 (4-way, selects CSN's exposed connector position) and SW2 (6-way, shared pool selecting GDO0's and GDO2's exposed connector positions), read directly off the module's own schematic (`Module_CC1101.pdf` page 5) and cross-checked against its M-Bus pin table (page 6) and Tab5's M-Bus pin table (`Tab5.pdf` page 14):**

| Signal | Switch | Position to close (ON; all others in that signal's group OFF) | M-Bus position | Tab5 GPIO |
|---|---|---|---|---|
| CSN | SW1 | 1 | 8 | **G45** |
| GDO0 | SW2 | 6 | 20 | **G4** |
| GDO2 | SW2 | 2 | 22 | **G48** |

Exactly one switch per signal group must be closed at a time — every option within a group (e.g. SW1's 4 positions for CSN, or SW2's 3-position pools for GDO0/GDO2) ties back to the same internal net, so closing two simultaneously would short two different Tab5 GPIOs together through that net. None of G45/G4/G48 conflict with anything else already claimed in `pins_config.h`. Recommend a multimeter continuity check from each physical switch to its expected connector pin before first power-up, given the small SMD switch package — this is now a verification step, not an open research gap.

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
└── cc1101_hw.{h,cpp}         # RadioLib wrapper (corrected 2026-08-24, see Section 2), Tab5 M-Bus SPI pins
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

- ~~**M-Bus GPIO mapping is the single blocking unknown**~~ — **RESOLVED 2026-08-24**, see Section 3. Still needs the physical continuity-check/DIP-switch-set step on real hardware before first SPI transaction, but the mapping itself is no longer unknown.
- **Frequency range**: the module's advertised range is 855-925MHz; the owner's stated intended tuning (868-925MHz) is a subset of that, consistent with the real hardware — no discrepancy, just note the module's full documented range in code/UI rather than hard-coding the owner's narrower intended-use band as a hard limit, in case future work wants the module's full range.
- **Physical/electrical conflict with other M-Bus-attached hardware**: if any other M5-Bus module is ever stacked simultaneously (none currently planned), confirm SPI bus sharing behavior — out of scope until it's an actual configuration, flagged here only so it isn't forgotten.
- **`shared/subghz_proto`'s existence depends on which phase (this one or Phase 5) actually gets implemented first** — see Section 4.2. Whichever phase's implementation plan is written second should explicitly check for the shared library's existence rather than assume it doesn't exist yet.

## 6. Testing Strategy

- Protocol decode/identify, `.sub` format read/write, and KeeLoq logic: native host tests against `shared/subghz_proto`, following the same table-driven-against-known-samples approach Phase 3 specs for its own protocol decode work.
- SPI bus bring-up and real RF scan/replay: real hardware only, once the M-Bus GPIO mapping (Section 3) is confirmed — same "flash it, read real serial output, fix what's actually wrong" discipline this project has used for every other radio HAL bring-up so far.
- Scan/replay verified against the project owner's own equipment (garage door remote, doorbell, etc.), matching Phase 5's testing-strategy language for the same class of feature.

## 7. Definition of Done

- [x] Tab5's real M-Bus-to-GPIO mapping confirmed from an authoritative, Tab5-specific source (not the generic Core-series M-Bus table) and recorded with citations, the same way the C6 SDIO pins were documented in Phase 1. **DONE 2026-08-24**, see Section 3.
- [x] CC1101 module's DIP-switch position (CSN/GDO0/GDO2) physically set per Section 3's table. **DONE 2026-08-24** — project owner confirmed physical switches: SW1 position 1 ON (CSN), SW2 positions 2 and 6 ON (GDO2, GDO0), matching this section's derived mapping exactly.
- [ ] `shared/subghz_proto` exists (created by this phase or already present from Phase 5) and both device trees' CC1101 feature modules consume it rather than duplicating protocol logic.
- [ ] At least one real scan/capture and one real replay demonstrated against the owner's own RF equipment, on real hardware.
- [ ] This phase's `docs/phases/phase-10-tab5-cc1101-mbus.md` write-up completed per this program's per-phase documentation convention (`CLAUDE.md`), including the resolved M-Bus pin mapping as a durable reference for any future M-Bus module work.
