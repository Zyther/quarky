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
- **`shared/subghz_proto` decision (this phase creates it, per spec Section 4.2, since it lands before Phase 5), revised 2026-08-25:** protocol decode IS consolidated — `rf433_protocol_decode.cpp`'s 6 already-proven decoders (of CC1101's 7 named protocols) are extracted into the shared library rather than duplicated, with `rf433_protocol_decode.cpp` becoming a thin consumer (its own public API/tests unchanged). Real reason for the reversal from this bullet's original text: the actual overlap turned out to be 6-of-7, not "some protocols," making duplication real waste and a real future-divergence risk rather than a cautious hedge worth deferring. `.sub` format is NOT consolidated — `subghz_sub_format` is a parallel, more general implementation (variable frequency/preset) alongside Task 21's fixed-433.92MHz `rf433_sub_format.cpp`, which is left untouched; the two formats' actual field grammar is close enough that a future consolidation is plausible, but that's a real second decision, not bundled into this one. See Task 2's own revised write-up for the full reasoning.
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

**Revised 2026-08-25, before implementation started** (real information the original write-up didn't have): checked exactly what CC1101's own named protocol scope is (Phase 5 spec Section 1: scan/decode wants Princeton/CAME/NICE/Linear, bruteforce wants Came/Nice/Linear/Chamberlain/Holtek/Ansonic — union of 7 protocols: Princeton, CAME, NiceFLO, Linear, Chamberlain, Holtek, Ansonic) against what Phase 3 Task 7's `rf433_protocol_decode.cpp` already has, ported and real-hardware-tested: Holtek, Holtek HT12X, CAME, NiceFLO, Chamberlain, Ansonic, Linear — **6 of the 7, missing only Princeton.** The original write-up below assumed only "some protocols overlap" and deferred consolidation as a future opportunity to avoid regression risk; with the overlap actually this close to total, re-porting fresh duplicates of 6 already-proven decoders is real, avoidable waste and a real future divergence risk (two independently-maintained copies of the same donor timing logic). Revised plan: **extract** (not re-port) the 6 already-tested decoder functions from `rf433_protocol_decode.cpp` into `shared/subghz_proto`, port Princeton fresh (the one genuinely new protocol), and make `rf433_protocol_decode.cpp` a thin consumer of the shared library — its own public API and `test/test_rf433_protocol_decode.cpp` must keep passing UNCHANGED, which is the real, concrete acceptance gate for the extraction not having broken anything.

**Files:**
- Create: `shared/subghz_proto/` (own `platformio.ini` `[env:native]`, mirroring `shared/feature_contract`'s real existing precedent — `platform = native`, `test_framework = unity`, `-Isrc`).
- Create: `shared/subghz_proto/src/subghz_protocol_decode.h` / `.cpp` — the 6 decoders extracted from `firmware/tab5/src/features/rf433/rf433_protocol_decode.cpp` (Holtek, Holtek HT12X, CAME, NiceFLO, Chamberlain, Ansonic, Linear — move the real, already-tested function bodies verbatim, do not re-derive their timing constants) plus Princeton, freshly ported from UniGeek's `SubGhzDecoders.cpp` (`~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.cpp`, real file, 1882 lines, 44 decoders total, GPLv3 — see licensing note below; only Princeton is being added here, not the other ~37).
- Modify: `firmware/tab5/src/features/rf433/rf433_protocol_decode.h` / `.cpp` — becomes a thin wrapper calling into `shared/subghz_proto`'s decoders; public API (function signatures Phase 3's callers already use) unchanged.
- Create: `shared/subghz_proto/src/subghz_sub_format.h` / `.cpp` — Flipper `.sub` RAW format read/write generalized for CC1101's variable frequency/preset, built on the same real spec `rf433_sub_format.h`/`.cpp` (Phase 3 Task 21) already researched and cited — read that file's own header comment first, it already documents the real field grammar, the two real header shapes (standard vs. custom-preset), the signed-duration/interleaving rules, and the multi-line `RAW_Data:` splitting convention in detail. Task 21's own module is NOT modified (RF433 stays on its fixed 433.92MHz version) — this is a parallel, more general implementation for CC1101's use, not a migration of Task 21's code.
- Test: `shared/subghz_proto/test/test_subghz_protocol_decode.cpp`, `test/test_subghz_sub_format.cpp` — host-native, table-driven against known samples, matching Phase 3 Task 7/21's own testing approach.

**Interfaces:**
- Produces: `namespace SubghzProto { struct Match { const char *name; uint64_t key; uint8_t bits; uint16_t te; }; bool decode(const unsigned int *dur, uint16_t count, Match *out); }` for protocol decode — deliberately mirroring `SubGhzDecoders::decode()`'s and `rf433_protocol_decode.cpp`'s own existing real signature shape (two-phase trial, `dur`/`count` pulse-duration array in) rather than inventing a new one, since callers on both the RF433 and CC1101 sides need to feed it the same kind of data. `namespace SubghzProto { bool encode_sub(uint32_t freq_hz, const char *preset, const EdgeSample *edges, size_t edge_count, char *buf, size_t buf_size, size_t *out_len); bool decode_sub(const char *text, size_t len, uint32_t *freq_hz_out, /* edges out */ ...); }` for the `.sub` format (real shape to be finalized against what `rf433_sub_format.cpp`'s own proven encode()/decode() signatures already look like, generalized to take frequency/preset as parameters instead of Task 21's fixed constants).
- **RAW must be a first-class, explicit path, not just an implicit fallback (project owner, 2026-08-25):** `encode_sub()`/`decode_sub()` take raw edges directly and are NOT gated on `decode()` having found a protocol match — an undecoded capture (`decode()` returns false) must still be fully save/replayable as a real `Protocol: RAW` `.sub` file, exactly matching Task 21's own `Rf433SubFormat` scope (which is Protocol:RAW only, no protocol-keyed writing at all) and every donor project's own real behavior (raw capture/replay always exists as the fallback for a signal no decoder recognizes). Task 3 (scan) is where this actually becomes a user-facing feature — call it out explicitly there too, not just here.

**Context — licensing note, same disclosure Phase 3 Task 21 already surfaced for the project owner, not re-litigated here:** UniGeek's `SubGhzDecoders.cpp` is itself GPLv3-licensed (ported from Flipper Zero firmware). This task's Princeton decoder (and the 6 extracted ones, already under this same licensing note since Task 7 first ported them) draws from the same source and inherits the same licensing question already flagged and left to the project owner's call in Phase 3 Task 21 — do not treat it as newly resolved just because it's being asked again here.

- [x] **Step 1: Extract the 6 already-proven decoders.** **DONE 2026-08-25** — verified verbatim: spot-checked against pre-edit `rf433_protocol_decode.cpp` (via `git diff`), timing constants and state machines byte-identical, ordering preserved exactly (holtek → princeton → holtek_ht12x → came → nice_flo → chamberlain → ansonic → linear, HT12X-before-CAME constraint intact).
- [x] **Step 2: Port Princeton fresh.** **DONE 2026-08-25** — real source confirmed at `SubGhzDecoders.cpp:78-126`; independently re-verified against the donor text fetched earlier in this same session — te_short=390/te_long=1170/te_delta=300, double-frame `last_data==data` guard, all match exactly.
- [x] **Step 3: Rewire `rf433_protocol_decode.cpp`.** **DONE 2026-08-25** — public API unchanged, `test_rf433_protocol_decode.cpp` untouched and passing (verified independently, not just taken on the implementer's word).
- [x] **Step 4: Implement `subghz_sub_format`.** **DONE 2026-08-25** — generalized frequency/preset params, RAW-first-class (not gated on a decode match), own `EdgeSample` type (matches `shared/feature_contract`/`shared/c2proto`'s established no-cross-dependency precedent).
- [x] **Step 5: Run the full host-native test suite.** **DONE 2026-08-25**, independently re-run by the controller (not just the implementer's own report): `shared/subghz_proto` 22/22, `firmware/tab5` 58/58 (including unchanged `test_rf433_protocol_decode`), and `pio run -e tab5` confirmed a real, clean build with `subghz_proto` present in the actual dependency graph.
- [ ] **Step 6: Commit** — pending project owner confirmation.

**Model:** Sonnet (the highest-risk part — protocol timing correctness — is now mostly an extraction of already-proven code rather than fresh porting; only Princeton and the generalized `.sub` format are genuinely new, both well-scoped against existing real references).

---

## Task 3: CC1101 scan + protocol decode + hot/cold signal finder

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_scan.h` / `.cpp`
- Modify: `shared/feature_contract/src/feature_module.h` — resolve the `Category::SUBGHZ` vs `Category::RF433` question flagged in Global Constraints (recommend adding `Category::SUBGHZ`, since CC1101 is a materially broader capability than Phase 3's fixed-code RF433 and the launcher should distinguish them — confirm with real UI review, not just this plan's own assumption). **Found already done** (2026-08-25, this task's own execution): `Category::SUBGHZ` already exists in `shared/feature_contract/src/feature_module.h` (`enum class Category { WIFI, BLE, SUBGHZ, NRF24, LORA, NFC, RF433, IR, UTILITY };`) — no edit was needed; all six new CC1101 feature modules (scan, spectrum, bruteforce, jammer, keeloq) register under it.

**Interfaces:**
- Consumes: `Cc1101Hw` (Task 1), `SubghzProto::decode()` (Task 2).
- Produces: `namespace Cc1101Scan { void start(); void stop(); bool poll(ScanResult *out); }` (real shape depends on Task 1/2's finished interfaces).

**Context:** Donor reference Poseidon `subghz_*.cpp`/`cc1101_hw.cpp/.h` (per Phase 5 spec Section 1) — same feature, same underlying chip, port the same way for the Tab5-native path. Hot/cold signal finder (Poseidon-only among the three donors, per Phase 5 spec) is a continuous RSSI-sampling mode built on the same scan-loop infrastructure — no separate file needed per the spec's own module structure (Section 4.1 lists no dedicated hot/cold file), fold in as a mode of this same module. **RAW capture is a required, first-class result of scanning, not just a side effect (project owner, 2026-08-25):** when `SubghzProto::decode()` finds no match, the captured edges must still be a fully valid, save/replayable result (`Protocol: RAW` via `SubghzProto::encode_sub()`, Task 2) — the UI must not treat "no protocol recognized" as a failure state with nothing to do next.

- [x] **Step 1: Implement scan + protocol decode.** **CODE-COMPLETE 2026-08-25** — `firmware/tab5/src/features/cc1101/cc1101_scan.{h,cpp}`: attachInterrupt(CHANGE)-driven edge capture on `Cc1101Hw::gdo0_pin()` (chip put into async/direct RX via a new `Cc1101Hw::enable_async_rx()`, added this session — real RadioLib API, `setOOK(true)`+`receiveDirectAsync()`, confirmed by reading `CC1101.cpp` directly), same portMUX-ring-buffer shape as `rf433_common.cpp` (ported, not redesigned). Finalized bursts are fed to `SubghzProto::decode()` (Task 2); RAW is first-class (`Cc1101Record::save()` always succeeds regardless of decode result). Donor: Poseidon `subghz_scan.cpp`'s real `gdo0_isr()`/`capture_now()` technique (cited in file header).
- [x] **Step 2: Implement hot/cold mode.** **CODE-COMPLETE 2026-08-25** — continuous `Cc1101Hw::rssi_dbm()` sampling against a learned baseline (30-sample warmup, then delta-from-baseline bar), ported from Poseidon's real `subghz_jam_detect.cpp` baseline/trigger technique, repurposed as a signal-finder per this task's own framing.
- [x] **Step 3: LVGL screen.** **CODE-COMPLETE 2026-08-25** — list-and-select / progress-and-stop pattern matching `rf433_scan.cpp`'s established shape (capture toggle, hot/cold toggle, frequency step, decode-on-capture result label, Save as .sub, Load from SD, Replay Selected/Loaded).
- [ ] **PAUSE FOR HARDWARE — verify scan against a real 433MHz device the project owner owns** (garage remote, doorbell, etc., matching spec Section 6's testing strategy). **code-complete-pending-hardware** — not run; needs the physical CC1101 module and the project owner present, per this plan's standing hardware-checkpoint instruction.
- [ ] **Step 4: Commit** — pending project owner confirmation (uncommitted, per this task's own instructions).

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

- [x] **Step 1: Implement `Cc1101Record::save()`** via `SubghzProto::encode_sub()` (the plan's original `write_sub()` name is `encode_sub()` in Task 2's actual finished interface — see Task 2's own revised write-up). **CODE-COMPLETE 2026-08-25** — `cc1101_record.{h,cpp}`; also added a symmetrical `load()` (via `SubghzProto::decode_sub()`) since Step 3 below needs a load path and the plan's Task 2 interface only specified pure encode/decode, not the SD wrapper split — a judgment call, not a spec answer.
- [x] **Step 2: Implement `Cc1101Replay::transmit()`.** **CODE-COMPLETE 2026-08-25** — `cc1101_replay.{h,cpp}`, same dedicated-core-pinned-FreeRTOS-task shape as `rf433_replay.cpp` (bit-bangs `Cc1101Hw::gdo0_pin()` after `Cc1101Hw::enable_async_tx()`), same real "why not poll()" reasoning ported from that file's header comment.
- [x] **Step 3: Wire into the generic SD file browser.** **CODE-COMPLETE 2026-08-25** — `cc1101_scan.cpp`'s screen adds "Save as .sub" / "Load from SD" / "Replay Selected" / "Replay Loaded" buttons using `ui/file_browser.h` against `/quarky/captures/subghz/`, mirroring `rf433_scan.cpp`'s own wiring.
- [ ] **PAUSE FOR HARDWARE — real round-trip: capture a real signal, save, reload from SD, replay, confirm the real target device responds** (matches Phase 3 Task 21's own real-round-trip verification bar). **code-complete-pending-hardware** — not run; needs the physical CC1101 module and the project owner present.
- [ ] **Step 4: Commit** — pending project owner confirmation.

**Model:** Sonnet.

---

## Task 5: CC1101 spectrum analyzer

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_spectrum.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw::rssi_dbm()` swept across a frequency range.
- Produces: `namespace Cc1101Spectrum { void start(float freq_start_mhz, float freq_end_mhz); void stop(); bool poll(SpectrumFrame *out); }`.

**Context:** Donor reference Bruce `rf_spectrum.cpp`/`rf_waterfall.cpp`, Poseidon (most visually developed version, per Phase 5 spec Section 1 — added Poseidon v0.6.2). Unlike Phase 5's Cardputer-ADV version (240×135 screen, cramped — spec Section 2.4 raises rendering it on the Tab5 instead as an enhancement), this phase runs natively on the Tab5's own 1280×720 display already, so the "render on the small screen vs. the big screen" tradeoff Phase 5 has to consider doesn't apply here — render directly via LVGL's `lv_chart` at full fidelity from the start.

- [x] **Step 1: Implement frequency-sweep RSSI capture.** **CODE-COMPLETE 2026-08-25** — `cc1101_spectrum.{h,cpp}`, poll()-driven one-step-per-tick sweep (same shape as `wifi_spectrum.cpp`'s per-channel hop), full 855-925MHz module range (71 bins, 1MHz steps) — deliberately NOT narrowed to the owner's 868-925MHz intended tuning, per spec Section 5. Each step retunes then calls the new `Cc1101Hw::enable_async_rx()` to resume RX after `setFrequency()` (confirmed by reading RadioLib's `CC1101::setFrequency()` directly: it IDLEs the chip and does not resume RX on its own), mirroring Poseidon's real `subghz_scan.cpp` autoscan retune+resume-RX sequence.
- [x] **Step 2: LVGL `lv_chart`-based bar rendering.** **CODE-COMPLETE 2026-08-25** — bar chart, -100..0dBm range, matches `wifi_spectrum.cpp`'s established pattern. Waterfall/persistence view (Poseidon's most-developed visualization) was NOT ported — scoped down to a live bar sweep only, a "thin review" judgment call given this session's budget, not a spec answer.
- [ ] **PAUSE FOR HARDWARE — verify against a real transmitting device within the module's documented 855-925MHz range.** **code-complete-pending-hardware** — not run; needs the physical CC1101 module and the project owner present.
- [ ] **Step 3: Commit** — pending project owner confirmation.

**Model:** Sonnet.

---

## Task 6: CC1101 bruteforce

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_bruteforce.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw`, protocol timing constants from `shared/subghz_proto` (Task 2).
- Produces: `namespace Cc1101Bruteforce { void start(Protocol p); void stop(); bool poll(BruteforceStatus *out); }`.

**Context:** Donor reference Bruce `rf_bruteforce.cpp`, UniGeek (Came/Nice/Linear/Chamberlain/Holtek/Ansonic — per Phase 5 spec Section 1, same protocol set as Task 2's decode library, since bruteforce needs to *generate* what decode needs to *recognize*).

- [x] **Step 1: Implement bruteforce sequencing per protocol.** **CODE-COMPLETE 2026-08-25** — `cc1101_bruteforce.{h,cpp}`; the 6 protocols' bit-timing tables (Came/Nice/Ansonic/Holtek 12-bit, Linear 10-bit, Chamberlain 9-bit) are copied VERBATIM from Bruce's real `~/src/firmware/src/modules/rf/rf_bruteforce.h`'s `brute_protocols[]` table (read directly this session, not re-derived) — same protocol set as `SubghzProto`'s decode table minus Princeton, matching this task's own Context note (bruteforce generates what decode recognizes). Runs on a dedicated core-pinned FreeRTOS task (same shape as `cc1101_replay.cpp`), bit-banging `Cc1101Hw::gdo0_pin()` after `enable_async_tx()`.
- [x] **Step 2: LVGL progress/stop screen.** **CODE-COMPLETE 2026-08-25** — protocol dropdown + Start/Stop + progress bar (code/total), matching Phase 2/5's list-and-select / progress-and-stop pattern.
- [ ] **PAUSE FOR HARDWARE — verify against a real device the project owner owns and is authorized to test.** **code-complete-pending-hardware** — not run; needs the physical CC1101 module, a real target device, and the project owner present.
- [ ] **Step 3: Commit** — pending project owner confirmation.

**Model:** Sonnet.

---

## Task 7: CC1101 jammer

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_jammer.h` / `.cpp`

**Interfaces:**
- Consumes: `Cc1101Hw`.
- Produces: `namespace Cc1101Jammer { void start(JamMode mode); void stop(); }`.

**Context:** Donor reference Bruce `rf_jammer.cpp`, Poseidon, UniGeek (full/intermittent modes, per Phase 5 spec Section 1).

- [x] **Step 1: Implement full + intermittent jam modes.** **CODE-COMPLETE 2026-08-25** — `cc1101_jammer.{h,cpp}`, ported from Bruce's real `~/src/firmware/src/modules/rf/rf_jammer.cpp`'s `run_full_jammer()` (3-phase rotation: micro-glitches / variable-width bursts / sustained-carrier-with-hard-cuts) and `run_itmt_jammer()` (10-500us forward+reverse sweep + random-noise burst, `send_optimized_pulse()`/`send_random_pattern()` ported verbatim) — same real timing constants, adapted from Bruce's blocking `while(sendRF)` shape into a background FreeRTOS task with a polled stop flag. Bruce's other two real modes (NOISE — CC1101 PN9 hardware mode, needs ELECHOUSE-specific register pokes this project's RadioLib HAL doesn't wrap; SWEEP) were deliberately NOT ported — out of this task's spec'd scope (full/intermittent only, per Phase 5 spec Section 1), not an oversight.
- [x] **Step 2: LVGL start/stop screen, unambiguous "actively transmitting" UI state.** **CODE-COMPLETE 2026-08-25** — red-styled Start button, persistent red "*** ACTIVELY TRANSMITTING -- JAMMING ***" banner while running (not a generic spinner), mode dropdown, pulse-count/elapsed-time status.
- [ ] **PAUSE FOR HARDWARE — verify against the project owner's own equipment, in an authorized test environment only.** **code-complete-pending-hardware** — not run; needs the physical CC1101 module, an RF-shielded/authorized test environment, and the project owner present.
- [ ] **Step 3: Commit** — pending project owner confirmation.

**Model:** Sonnet.

---

## Task 8: KeeLoq decode + rolling-code replay

**Files:**
- Create: `firmware/tab5/src/features/cc1101/cc1101_keeloq.h` / `.cpp`
- Extend: `shared/subghz_proto` (Task 2) with KeeLoq decode logic if genuinely hardware-agnostic (per spec Section 4.2's own classification of KeeLoq decode as shared-library material). **Judgment call (2026-08-25, this task's own execution): NOT done.** KeeLoq's raw-edge decode + cipher logic IS genuinely hardware-agnostic (same as the plan predicted) and would be a legitimate future extraction into `shared/subghz_proto` — deferred to `firmware/tab5/src/features/cc1101/cc1101_keeloq.cpp` instead, purely to keep this session's remaining budget on shipping working code across all 6 tasks rather than an additional refactor with no functional difference today (Phase 5's Cardputer-ADV KeeLoq module, if/when it lands, would currently have to duplicate this logic rather than share it — flagged here as real, disclosed future work, not silently dropped).

**Interfaces:**
- Consumes: `Cc1101Hw`, `SubghzProto` KeeLoq decode.
- Produces: `namespace Cc1101Keeloq { bool decode(const CapturedSignal &sig, KeeloqInfo *out); bool replay_plus_one(const KeeloqInfo &info); }`.

**Context:** Donor reference UniGeek (unique to UniGeek among the three donors, per Phase 5 spec Section 1). **Legally sensitive, explicitly flagged by the spec itself (Section 3, Phase 5 spec):** rolling-code replay is meaningfully different from passive scanning even under the project owner's own-equipment authorization. Implement, but the UI must make unambiguous that the target is being actively attacked — same bar Phase 5's spec sets, applying identically here since it's the same feature against the same real risk, just on different hardware.

- [x] **Step 1: Implement KeeLoq auto-decode**, cited from UniGeek's real implementation. **CODE-COMPLETE 2026-08-25** — `cc1101_keeloq.{h,cpp}`: raw-edge decode of RcSwitch "protocol 23" ported from UniGeek's real `~/src/unigeek-main/firmware/src/utils/rf/RCSwitchUtil.cpp` (`kProto[22]` real timing: te=400 fixed, firstData=25, zero/one te-multipliers {2,1}/{1,2}, 60% tolerance, real separator-gap segment-splitting from `decodeStream()`); KeeLoq cipher (encrypt/decrypt, 528 rounds, NLF=0x3A5C742E), unpack(), identify(), and the manufacturer hop-bit-layout table ported near-verbatim from `~/src/unigeek-main/firmware/src/utils/rf/KeeloqUtil.cpp`. Manufacturer keystore: SD-loaded `/quarky/keeloq/mfcodes`, same real text format as UniGeek's `KeeloqKeystore.cpp` (`mf_name;hex_key;learning_type`) — no keys bundled/fabricated, matches the donor's own "silently empty if absent" behavior.
- [x] **Step 2: Implement "Replay +1" rolling-code attack.** **CODE-COMPLETE 2026-08-25** — `replay_plus_one()`: `KeeloqUtil::step()`-equivalent counter-increment + re-encrypt, real waveform re-encode ported from `RCSwitchUtil::encodeToDurations()`'s real `keeloq` branch (11-pulse preamble + sync gap + 64-bit MSB-first data + trailer), transmitted from a dedicated FreeRTOS task (same shape as `cc1101_replay.cpp`).
- [x] **Step 3: LVGL screen with an explicit, unambiguous "actively attacking" state indicator.** **CODE-COMPLETE 2026-08-25** — persistent red "*** ACTIVELY ATTACKING -- TRANSMITTING ROLLING CODE ***" banner while `replay_plus_one()` is in flight (not a generic spinner), Replay +1 button itself red-styled and disabled until a signal is both decoded AND identified against a loaded manufacturer key.
- [ ] **PAUSE FOR HARDWARE — verify against the project owner's own KeeLoq-based equipment only, explicitly authorized.** **code-complete-pending-hardware** — not run; needs the physical CC1101 module, real owner-authorized KeeLoq equipment, and the project owner present. Also NOTE: never verified against a real captured KeeLoq frame (no fixture available in this session) — the raw-decode/cipher ports are believed correct against their real donor sources but are UNTESTED end-to-end; a real hardware capture is the only way to close that gap.
- [ ] **Step 4: Commit** — pending project owner confirmation.

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
