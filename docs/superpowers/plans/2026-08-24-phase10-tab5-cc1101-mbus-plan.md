# Phase 10: Tab5-Native CC1101 via M-Bus — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring up the M5Stack CC1101 Module on the Tab5's own M5-Bus connector and build the full CC1101 feature set (scan/decode, record/.sub, replay, spectrum + hot/cold signal finder, bruteforce, jammer, KeeLoq) natively on the Tab5 — independent of, and in parallel with, Phase 5's Cardputer-ADV hydra-hat CC1101 (both are real, kept-on-purpose, per the project owner's own 2026-08-24 direction; see this phase's spec Section 1 and `CLAUDE.md`'s 2026-08-24 rescope note).

**Architecture:** `firmware/tab5/src/features/cc1101/` mirrors Phase 5's `firmware/cardputer-adv/src/features/cc1101/` module list (`cc1101_hw`, `cc1101_scan`, `cc1101_record`, `cc1101_replay`, `cc1101_spectrum`, `cc1101_bruteforce`, `cc1101_jammer`, `cc1101_keeloq`) — same feature set, same underlying driver library, same `FeatureModule`/`FeatureRegistry` + LVGL `build_sub_screen()`/`ScreenStack` + `poll()`-driven pattern every prior Tab5 feature already uses. Only `cc1101_hw.cpp` is genuinely host-specific (SPI bus ownership, M-Bus GPIO pins); everything else is portable logic.

**Tech Stack:** `jgromes/RadioLib` (real `CC1101`/`Module` classes), added as a normal `lib_deps` registry entry (RadioLib@7.7.1 as of 2026-08-24) — not vendored, unlike `MFRC522_I2C`/`crapto1`, since it's a large, actively maintained community library (2,546 stars, MIT), not a small/fragile/single-maintainer one. This phase's first attempt used `LSatan/SmartRC-CC1101-Driver-Lib` instead (vendored in-tree, matching the `MFRC522_I2C`/`crapto1` precedent) before Task 1's own real-hardware testing found a real repeat-call bug in it and the project owner found RadioLib is M5Stack's own documented library for this exact module — see Task 1's own write-up and `platformio.ini`'s citation for the full account. Arduino-ESP32 SPI, LVGL 9.5, `IStorage`/`StorageSD`, `FeatureModule`/`FeatureRegistry` (`shared/feature_contract`).

## Global Constraints

- **M-Bus SPI pins are now confirmed** (spec Section 3, resolved 2026-08-24 from `docs/vendor/Tab5.pdf` + `docs/vendor/Module_CC1101.pdf`, both real M5Stack vendor PDFs, not the generic Core-series M-Bus table): `MOSI=G18, MISO=G19, SCK=G5` (fixed pins, no switch). `CSN=G45` (module's SW1, switch 1 ON, all others OFF), `GDO0=G4` (module's SW2, switch 6 ON), `GDO2=G48` (module's SW2, switch 2 ON) — all others in each signal's switch group must stay OFF, since every option in a group ties back to the same internal net. None of G45/G4/G48/G18/G19/G5 conflict with any GPIO already claimed elsewhere in `pins_config.h`.
- **Do not assume the physical module's switches are already in this position** — Task 1 below is a real hardware verification step (continuity check) before first SPI transaction, per the spec's own DoD item. This is a read/verify step, not a re-derivation of the mapping itself (that part is done).
- LVGL screens: every feature screen is built via `build_sub_screen(title, &content)` (`firmware/tab5/src/ui/screen_scaffold.h`) and tears down via an `LV_EVENT_DELETE` handler that nulls every static widget pointer and cancels in-flight state — matches every prior Tab5 feature module.
- Launcher tiles: every feature calls `g_registry.register_module({id, name, category, Affinity::TAB5_NATIVE, start, nullptr})`. A new `Category::SUBGHZ` (or reuse `Category::RF433` — decide in Task 3, see its Context note) needs a decision, since `shared/feature_contract/src/feature_module.h`'s existing categories were scoped for Phase 3's fixed-code RF433, not full CC1101-class sub-GHz.
- Long-running work (spectrum sweep, bruteforce, jammer, KeeLoq rolling-code capture) is `poll()`-driven from `main.cpp`'s `loop()`, never a blocking loop inside a click handler — same house rule as every prior phase (`wifi_connect.cpp`'s real-hardware crash lesson).
- Cross-task shared state written from any ISR (GDO0/GDO2 edge/packet interrupts, if used) needs `volatile` or a `portMUX_TYPE` critical section, matching `hal/c2link_ble.cpp`'s `s_rx_mux` / `features/ble/ble_scan.cpp`'s `s_devices_mux` precedent.
- SD captures: `extern StorageSD storage;` (declared `firmware/tab5/src/hal/storage_sd.h`, defined `main.cpp`). This phase's captures land in `/quarky/captures/subghz/` per the spec (Section 4.4) — distinct from Phase 3's `/quarky/captures/rf433/`, since this is full CC1101-class capability (spectrum/waterfall data, arbitrary-protocol `.sub` captures), not just fixed-code remotes.
- Real sources only for protocol/register-level work — no fabricated register maps, command bytes, or protocol timing. Every task below names a real donor reference (Poseidon/Bruce/UniGeek, all local checkouts under `~/src/`) or a real public spec (TI's CC1101 datasheet for register-level work, Flipper's own `.sub` RAW format already reverse-sourced and cited in Phase 3 Task 21) to port from — do not invent.
- **`shared/subghz_proto` decision (this phase creates it, per spec Section 4.2, since it lands before Phase 5):** deliberately scoped to CC1101's OWN protocol/format logic only in this phase — it does **not** retroactively migrate Phase 3's already-shipped, tested `rf433_protocol_decode.cpp`/`rf433_sub_format.cpp` into itself. Those RF433 modules stay as-is; migrating stable, committed Phase 3 code into a new shared library is a real regression risk on working code and is out of scope here. Task 2 explicitly discloses this as a deferred consolidation opportunity (some protocols, e.g. Princeton/CAME/NICE/Linear, exist in both RF433's narrower set and CC1101's broader one) rather than silently declaring it resolved — matches this project's Task 21 precedent of disclosing an open question rather than deciding it unilaterally.
- **Standing hardware-checkpoint instruction (project owner, carried forward from Phase 3):** any step requiring the physical CC1101 module to be attached, its DIP switches set, or otherwise physically manipulated must PAUSE and wait for the project owner's explicit confirmation before proceeding.
- **Superseded note, kept for history:** an earlier version of this driver dependency (`LSatan/SmartRC-CC1101-Driver-Lib`) had a real, disclosed, still-open upstream buffer-overflow bug in its `ReceiveData()` (`github.com/LSatan/SmartRC-CC1101-Driver-Lib/issues/185`) — moot now that Task 1 switched to `jgromes/RadioLib` (see Tech Stack), but **any task below implementing packet receive (Task 3, Task 8) must still independently verify RadioLib's own `receive()`/`readData()` bounds behavior against a real buffer smaller than the CC1101's max FIFO payload before trusting it** — a different library is not automatically assumed safe just because the prior one had a known bug; verify, don't assume.

---

## Task 1: M-Bus SPI bring-up + DIP-switch/continuity verification

**Files:**
- Modify: `firmware/tab5/platformio.ini` — `jgromes/RadioLib` added to `lib_deps` (real, actively maintained; see Tech Stack and this file's own citation for why not vendored). **DONE 2026-08-24** (superseded an earlier vendored-SmartRC attempt — see Step 3's own note).
- Create: `firmware/tab5/src/features/cc1101/cc1101_hw.h` / `.cpp` — thin wrapper over RadioLib's `CC1101`/`Module` classes, owns the M-Bus SPI bus. **DONE 2026-08-24.**
- Modify: `firmware/tab5/src/main.cpp` — temporary `QUARKY_SERIAL_DEBUG`-gated probe trigger, key `'x'` (matches every prior spike task's convention, e.g. Phase 3 Task 1/20/Task 15's `'i'`). **DONE 2026-08-24.**

**Interfaces:**
- Produced: `namespace Cc1101Hw { bool init(); bool is_present(); void set_frequency_mhz(float mhz); float frequency_mhz(); float rssi(); }` (real shape may grow once Task 3+ need more of RadioLib's `CC1101` surface — e.g. `transmit()`/`receive()`/`setOutputPower()` — this is a starting contract, not fixed).

**Context:** Per Global Constraints, the GPIO mapping and DIP-switch target positions are resolved and cited (spec Section 3). Real API used, confirmed from RadioLib's own header rather than guessed: `Module(cs, irq, rst, gpio, spi, spiSettings)` (the overload that does NOT call `SPI.begin()` itself, since we need custom M-Bus pins — `SPI.begin()` is called explicitly by `Cc1101Hw::init()` first), `CC1101(Module*)`, `begin(freq)` (real chip-presence check built in — retries the `VERSION` status register up to 10 times against known CC1101 silicon revisions, returns `RADIOLIB_ERR_CHIP_NOT_FOUND` on failure), `setFrequency()`, `getRSSI()`.

- [x] **PAUSE FOR HARDWARE — confirm DIP switch positions on the physical module.** **DONE 2026-08-24** — project owner confirmed SW1 position 1 ON (CSN), SW2 positions 2 and 6 ON (GDO2, GDO0), matching the derived mapping exactly.
- [x] **Step 1: Add the driver dependency.** **DONE 2026-08-24**, real history: first vendored `LSatan/SmartRC-CC1101-Driver-Lib` (matching `MFRC522_I2C`/`crapto1`'s vendoring precedent, over the project owner's real maintenance-risk concern about a live registry fetch) — then replaced with a live `jgromes/RadioLib` `lib_deps` entry once Step 3's real-hardware testing surfaced a bug in SmartRC and the project owner found RadioLib is M5Stack's own documented library for this module. See Step 3 and `platformio.ini`'s own citation for the full account; the vendored SmartRC copy no longer exists in the tree.
- [x] **Step 2: Implement `Cc1101Hw::init()`** — configures `MOSI=G18, MISO=G19, SCK=G5, CSN=G45, GDO0=G4, GDO2=G48` via the real pin-setter API, calls `Init()`. **DONE 2026-08-24.**
- [x] **PAUSE FOR HARDWARE — attach the physical module to the Tab5's M-Bus connector.** **DONE 2026-08-24** — project owner confirmed physically plugged in.
- [x] **Step 3: Real hardware verification.** **DONE 2026-08-24.** Real, root-caused detour along the way: the library originally used for this step (LSatan/SmartRC-CC1101-Driver-Lib, vendored) showed a real bug on real hardware -- `is_present()` returned TRUE on the very first call after boot, then FALSE on every subsequent call in the same session, reproduced 3/3 times after ruling out test-harness artifacts (a build-flag mistake on an intermediate reflash briefly masked the real behavior and was itself caught and corrected). Root cause not fully pinned to a single register-level explanation, but the project owner independently found that M5Stack's own CC1101 module docs (`docs.m5stack.com/en/module/Module_CC1101#softwares`, also a real embedded hyperlink in `docs/vendor/Module_CC1101.pdf`) document `jgromes/RadioLib`, not SmartRC, as this module's own Arduino library -- switched to RadioLib (real, 2,546 stars, MIT, actively maintained; see `platformio.ini`'s citation), which resolved the instability outright: 3/3 consecutive real-hardware presence checks in one session now all return TRUE, with a consistent `433.92 MHz` / `-74.0 dBm` RSSI reading (a plausible idle noise floor, unlike SmartRC's suspicious `-138`). `cc1101_hw.h`/`.cpp` rewritten against RadioLib's real `CC1101`/`Module` API; the vendored SmartRC copy was removed (`lib/SmartRC-CC1101-Driver-Lib/` no longer exists).
- [x] **Step 4: Commit** — pending project owner confirmation (not yet committed as of this checkbox).

**Model:** Direct (hardware bring-up spike with a resolved, cited pin mapping — matches Phase 3 Task 1's own model once its own pin question was settled).

---

## Task 2: `shared/subghz_proto` — protocol decode + `.sub` format library

**Files:**
- Create: `shared/subghz_proto/` (own `platformio.ini` `[env:native]`, mirroring `shared/feature_contract`'s real existing precedent — `platform = native`, `test_framework = unity`, `-Isrc`).
- Create: `shared/subghz_proto/src/subghz_protocol_decode.h` / `.cpp` — Princeton/CAME/NICE/Linear/Chamberlain/Holtek/Ansonic decode, ported from UniGeek's `SubGhzDecoders.cpp` (widest protocol coverage of the three donors, per Phase 5 spec Section 1).
- Create: `shared/subghz_proto/src/subghz_sub_format.h` / `.cpp` — Flipper `.sub` RAW format read/write for CC1101's broader capture set (arbitrary frequency/preset, not just RF433's fixed 433.92MHz OOK).
- Test: `shared/subghz_proto/test/test_subghz_protocol_decode.cpp`, `test/test_subghz_sub_format.cpp` — host-native, table-driven against known samples, matching Phase 3 Task 7/21's own testing approach.

**Interfaces:**
- Produces: `namespace SubghzProto { struct Decoded { const char *protocol_name; uint64_t code; uint8_t bits; }; bool decode(const EdgeSample *edges, size_t count, Decoded *out); bool write_sub(const char *path, IStorage &storage, ...); bool read_sub(const char *path, IStorage &storage, ...); }` (real shape TBD by what UniGeek's decoder set and the Flipper format actually need — starting assumption per this project's established caveat for research-gated interfaces).

**Context — licensing note, same disclosure Phase 3 Task 21 already surfaced for the project owner, not re-litigated here:** UniGeek's `SubGhzDecoders.cpp` is itself GPLv3-licensed (ported from Flipper Zero firmware). This task's decoder logic draws from the same source and inherits the same licensing question already flagged and left to the project owner's call in Phase 3 Task 21 — do not treat it as newly resolved just because it's being asked again here.

**Deferred, disclosed, not part of this task:** consolidating Phase 3's existing `rf433_protocol_decode.cpp`/`rf433_sub_format.cpp` to consume this new shared library instead of their own logic, even though some protocols overlap (Princeton/CAME/NICE/Linear appear in both). See Global Constraints — flagged as a real future opportunity, not attempted here to avoid regressing shipped, tested Phase 3 code.

- [ ] **Step 1: Research and cite UniGeek's real decoder set** (`~/src/unigeek-main`'s `SubGhzDecoders.cpp` or equivalent) — list every protocol actually ported, with real bit-timing constants cited from that source, not invented.
- [ ] **Step 2: Implement `subghz_protocol_decode`**, host-native tested against real captured-and-hand-verified or donor-provided sample edge sequences.
- [ ] **Step 3: Implement `subghz_sub_format`**, reusing the real Flipper `.sub` RAW format Phase 3 Task 21 already researched and cited (same field grammar: `Filetype:`/`Frequency:`/`Preset:`/`Protocol:`/`RAW_Data:`), extended for CC1101's variable frequency/preset (Task 21's version assumed RF433's fixed 433.92MHz).
- [ ] **Step 4: Run the host-native test suite, expect PASS.**
- [ ] **Step 5: Commit**

**Model:** Opus (protocol-decode porting from a real donor source is genuine research/correctness risk, matching this project's established model choice for comparable Phase 3 tasks — e.g. Task 7's own RF433 protocol decode).

---

## Task 3: CC1101 scan + protocol decode + hot/cold signal finder

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_scan.h` / `.cpp`
- Modify: `shared/feature_contract/src/feature_module.h` — resolve the `Category::SUBGHZ` vs `Category::RF433` question flagged in Global Constraints (recommend adding `Category::SUBGHZ`, since CC1101 is a materially broader capability than Phase 3's fixed-code RF433 and the launcher should distinguish them — confirm with real UI review, not just this plan's own assumption).

**Interfaces:**
- Consumes: `Cc1101Hw` (Task 1), `SubghzProto::decode()` (Task 2).
- Produces: `namespace Cc1101Scan { void start(); void stop(); bool poll(ScanResult *out); }` (real shape depends on Task 1/2's finished interfaces).

**Context:** Donor reference Poseidon `subghz_*.cpp`/`cc1101_hw.cpp/.h` (per Phase 5 spec Section 1) — same feature, same underlying chip, port the same way for the Tab5-native path. Hot/cold signal finder (Poseidon-only among the three donors, per Phase 5 spec) is a continuous RSSI-sampling mode built on the same scan-loop infrastructure — no separate file needed per the spec's own module structure (Section 4.1 lists no dedicated hot/cold file), fold in as a mode of this same module.

- [ ] **Step 1: Implement scan + protocol decode** (frequency sweep/listen, feed captured edges to `SubghzProto::decode()`).
- [ ] **Step 2: Implement hot/cold mode** (continuous RSSI sampling via `Cc1101Hw::rssi_dbm()`, real-time UI feedback).
- [ ] **Step 3: LVGL screen** — list-and-select / progress-and-stop pattern (Phase 2/5 precedent, spec Section 4.3).
- [ ] **PAUSE FOR HARDWARE — verify scan against a real 433MHz device the project owner owns** (garage remote, doorbell, etc., matching spec Section 6's testing strategy).
- [ ] **Step 4: Commit**

**Model:** Sonnet (feature logic on top of Task 1/2's already-researched primitives — lower novel-research risk than Task 2 itself).

---

## Task 4: CC1101 record (.sub save) + replay

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_record.h` / `.cpp`
- Create: `firmware/tab5/src/features/cc1101/cc1101_replay.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw`, `SubghzProto::write_sub()`/`read_sub()` (Task 2), `Cc1101Scan`'s capture path (Task 3).
- Produces: `namespace Cc1101Record { bool save(const char *name, const CapturedSignal &sig); }`, `namespace Cc1101Replay { bool transmit(const CapturedSignal &sig); }`.

**Context:** Donor reference Poseidon + Bruce `rf_record.cpp`/`rf_send.cpp` (Phase 5 spec Section 1). Captures land in `/quarky/captures/subghz/` (Global Constraints). Poseidon's baked `.sub` signal library (3,190+ files, already cleared for reuse per the project owner per Phase 5 spec) is a real, available asset — confirm with the project owner whether to bundle any of it now or defer, since it's a real content/storage-budget decision, not just a code one.

- [ ] **Step 1: Implement `Cc1101Record::save()`** via `SubghzProto::write_sub()`.
- [ ] **Step 2: Implement `Cc1101Replay::transmit()`.**
- [ ] **Step 3: Wire into the generic SD file browser** (Phase 3 Task 22's `firmware/tab5/src/ui/file_browser.*` — confirm it generalizes to `/quarky/captures/subghz/` the same way it already does for RF433/IR, per Task 22's own design).
- [ ] **PAUSE FOR HARDWARE — real round-trip: capture a real signal, save, reload from SD, replay, confirm the real target device responds** (matches Phase 3 Task 21's own real-round-trip verification bar).
- [ ] **Step 4: Commit**

**Model:** Sonnet.

---

## Task 5: CC1101 spectrum analyzer

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_spectrum.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw::rssi_dbm()` swept across a frequency range.
- Produces: `namespace Cc1101Spectrum { void start(float freq_start_mhz, float freq_end_mhz); void stop(); bool poll(SpectrumFrame *out); }`.

**Context:** Donor reference Bruce `rf_spectrum.cpp`/`rf_waterfall.cpp`, Poseidon (most visually developed version, per Phase 5 spec Section 1 — added Poseidon v0.6.2). Unlike Phase 5's Cardputer-ADV version (240×135 screen, cramped — spec Section 2.4 raises rendering it on the Tab5 instead as an enhancement), this phase runs natively on the Tab5's own 1280×720 display already, so the "render on the small screen vs. the big screen" tradeoff Phase 5 has to consider doesn't apply here — render directly via LVGL's `lv_chart` at full fidelity from the start.

- [ ] **Step 1: Implement frequency-sweep RSSI capture.**
- [ ] **Step 2: LVGL `lv_chart`-based bar/waterfall rendering.**
- [ ] **PAUSE FOR HARDWARE — verify against a real transmitting device within the module's documented 855-925MHz range** (spec Section 5 — note the module's full range, not just the owner's narrower 868-925MHz intended tuning, per the spec's explicit instruction not to hard-code the narrower band as a limit).
- [ ] **Step 3: Commit**

**Model:** Sonnet.

---

## Task 6: CC1101 bruteforce

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_bruteforce.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw`, protocol timing constants from `shared/subghz_proto` (Task 2).
- Produces: `namespace Cc1101Bruteforce { void start(Protocol p); void stop(); bool poll(BruteforceStatus *out); }`.

**Context:** Donor reference Bruce `rf_bruteforce.cpp`, UniGeek (Came/Nice/Linear/Chamberlain/Holtek/Ansonic — per Phase 5 spec Section 1, same protocol set as Task 2's decode library, since bruteforce needs to *generate* what decode needs to *recognize*).

- [ ] **Step 1: Implement bruteforce sequencing per protocol** (real code-space/timing per protocol, cited from the same donor source as Task 2's decode logic — do not invent separate timing constants for the same protocols).
- [ ] **Step 2: LVGL progress/stop screen.**
- [ ] **PAUSE FOR HARDWARE — verify against a real device the project owner owns and is authorized to test.**
- [ ] **Step 3: Commit**

**Model:** Sonnet.

---

## Task 7: CC1101 jammer

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_jammer.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw`.
- Produces: `namespace Cc1101Jammer { void start(JamMode mode); void stop(); }`.

**Context:** Donor reference Bruce `rf_jammer.cpp`, Poseidon, UniGeek (full/intermittent modes, per Phase 5 spec Section 1).

- [ ] **Step 1: Implement full + intermittent jam modes.**
- [ ] **Step 2: LVGL start/stop screen, unambiguous "actively transmitting" UI state** (same clarity bar the spec sets for KeeLoq in Task 8 — a jammer is even more clearly an active-transmission feature, must not read as passive).
- [ ] **PAUSE FOR HARDWARE — verify against the project owner's own equipment, in an authorized test environment only** (RF jamming has real regulatory exposure — confirm the project owner's own test-lab/RF-shielded-environment authorization applies here the same way it's already been established for KeeLoq, before any real-air test).
- [ ] **Step 3: Commit**

**Model:** Sonnet.

---

## Task 8: KeeLoq decode + rolling-code replay

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_keeloq.h` / `.cpp`
- Extend: `shared/subghz_proto` (Task 2) with KeeLoq decode logic if genuinely hardware-agnostic (per spec Section 4.2's own classification of KeeLoq decode as shared-library material).

**Interfaces:**
- Consumes: `Cc1101Hw`, `SubghzProto` KeeLoq decode.
- Produces: `namespace Cc1101Keeloq { bool decode(const CapturedSignal &sig, KeeloqInfo *out); bool replay_plus_one(const KeeloqInfo &info); }`.

**Context:** Donor reference UniGeek (unique to UniGeek among the three donors, per Phase 5 spec Section 1). **Legally sensitive, explicitly flagged by the spec itself (Section 3, Phase 5 spec):** rolling-code replay is meaningfully different from passive scanning even under the project owner's own-equipment authorization. Implement, but the UI must make unambiguous that the target is being actively attacked — same bar Phase 5's spec sets, applying identically here since it's the same feature against the same real risk, just on different hardware.

- [ ] **Step 1: Implement KeeLoq auto-decode**, cited from UniGeek's real implementation.
- [ ] **Step 2: Implement "Replay +1" rolling-code attack.**
- [ ] **Step 3: LVGL screen with an explicit, unambiguous "actively attacking" state indicator** — not just a generic "running" spinner.
- [ ] **PAUSE FOR HARDWARE — verify against the project owner's own KeeLoq-based equipment only, explicitly authorized.**
- [ ] **Step 4: Commit**

**Model:** Opus (legally/technically highest-risk task in this phase — matches this project's established practice of using the stronger model for genuinely sensitive protocol work).

---

## Task 9: Definition-of-Done bring-up log + Phase 10 documentation

**Files:**
- Create: `docs/phases/phase-10-tab5-cc1101-mbus.md`
- Modify: `.superpowers/sdd/2026-08-24-phase10-tab5-cc1101-mbus-plan/progress.md` (the SDD ledger this plan's execution will already be writing to)
- Modify: `CLAUDE.md` — update Phase 10's roadmap-table status to Complete.

**Context:** Per `CLAUDE.md`'s standing process ("every phase ends with documentation, not just a Definition-of-Done check") — walk every Definition-of-Done item from the spec (Section 7) against this plan's actual real-hardware task results, pass/fail + notes, then write the durable phase doc matching `docs/phases/phase-1-foundation.md`'s and `docs/phases/phase-3-nfc-rf433-ir.md`'s established depth: architecture decisions and why (including the `shared/subghz_proto` scoping decision from Task 2, and the resolved M-Bus GPIO/DIP-switch mapping as a durable reference for any future M-Bus module work, per the spec's own DoD item), real-hardware findings with root causes, known limitations/deferred work (the Phase 3 RF433 consolidation opportunity flagged in Task 2, KeeLoq's legal-sensitivity disclosure, etc.).

- [ ] **Step 1: Walk every DoD item from the spec (Section 7) against real task results, pass/fail + notes.**
- [ ] **Step 2: Write `docs/phases/phase-10-tab5-cc1101-mbus.md`.**
- [ ] **Step 3: Update `CLAUDE.md`'s phase table.**
- [ ] **Step 4: Commit**

**Model:** Direct (documentation task, matches every prior phase's own DoD-task precedent).
