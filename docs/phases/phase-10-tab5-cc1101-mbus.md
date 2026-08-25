# Phase 10: Tab5-Native CC1101 via M-Bus

## What this phase built

A full CC1101 sub-GHz feature set running natively on the Tab5, using an
M5Stack CC1101 Module attached to the Tab5's own rear M5-Bus connector —
independent of, and in parallel with, Phase 5's (not-yet-implemented)
Cardputer-ADV hydra-hat CC1101. Two real, separate radios on two devices,
kept on purpose (project owner's own 2026-08-24 direction).

Built: `Cc1101Hw` (SPI/async-direct-mode HAL), `shared/subghz_proto`
(protocol decode + generalized `.sub` format, shared with Phase 3's RF433
work), and six feature modules — scan/decode + hot/cold signal finder,
record, replay, spectrum analyzer, bruteforce, jammer, and KeeLoq
decode/rolling-code replay.

**Status as of this write-up: code-complete, build/test-verified, real
hardware verification NOT YET RUN.** The project owner explicitly requested
stepping through each task's hardware checkpoint separately, after the
whole batch landed and was code-reviewed — that pass has not happened yet.
Treat every feature below as "should work, unconfirmed against real RF
hardware" until that session runs.

## Architecture decisions worth knowing

**Two CC1101 drivers were tried; the first was replaced for real reasons.**
Task 1 started with `LSatan/SmartRC-CC1101-Driver-Lib` (vendored in-tree,
matching this project's `MFRC522_I2C`/`crapto1` precedent, over a live
registry fetch). Real hardware testing found it detected the chip once
after boot, then failed every subsequent call in the same session —
reproduced 3/3, not a test artifact (a build-flag omission on an
intermediate reflash briefly muddied the picture and was caught separately).
Around the same time, the project owner found M5Stack's own CC1101 module
documentation links `jgromes/RadioLib`, not SmartRC, as this module's real
documented Arduino library. Switched to RadioLib — real, 2,546 stars, MIT,
actively maintained — which resolved the instability outright (3/3 stable
presence checks, consistent RSSI). See the plan doc's Task 1 section and
`platformio.ini`'s own citation for the full account.

**`shared/subghz_proto` consolidates protocol decode rather than
duplicating it.** CC1101's real named protocol scope (Phase 5 spec: Princeton,
CAME, NiceFLO, Linear, Chamberlain, Holtek, Ansonic — 7 protocols) turned
out to overlap 6-of-7 with Phase 3's already-proven, real-hardware-tested
`rf433_protocol_decode.cpp`. Rather than re-porting duplicates, those 6
decoders were extracted into the new shared library; Princeton (the one
genuinely new protocol) was ported fresh from the same UniGeek donor source
(GPLv3, licensing question already disclosed and accepted in Phase 3 Task
21's history). `rf433_protocol_decode.cpp` is now a thin wrapper — its
public API and test file are byte-for-byte unchanged. The `.sub` file
format was NOT consolidated the same way: `subghz_sub_format` is a parallel
implementation generalized for CC1101's variable frequency/preset, sitting
alongside Task 21's RF433-fixed version untouched. RAW capture (undecoded
signal, saved/replayed as `Protocol: RAW`) is explicit and first-class in
both the decode and format layers, not an implicit fallback — a real,
explicit project-owner requirement mid-build.

**M-Bus GPIO mapping, real and resolved.** Fixed pins: MOSI=G18, MISO=G19,
SCK=G5 (both the module's and Tab5's own M-Bus connector tables label these
positions identically). DIP-switch-selected: CSN=G45 (module's SW1, switch
1), GDO0=G4 (SW2, switch 6), GDO2=G48 (SW2, switch 2) — derived by tracing
the module's own schematic net-by-net (rendered at 600dpi, cropped, read
directly) and cross-referencing both devices' real M-Bus position tables
from their own vendor PDFs (`docs/vendor/Tab5.pdf`, `docs/vendor/
Module_CC1101.pdf`), not the generic Core-series M-Bus table this project
has been burned by before. The project owner's physical switches matched
this derivation exactly on the first check.

## Real hardware findings (root causes, not just "fixed a bug")

- **SmartRC-CC1101-Driver-Lib repeat-call failure** (see above) — root
  cause not fully pinned to a single register-level mechanism (would need a
  scope on the SPI bus to confirm), but the leading candidate is the
  library's own static "already initialized" guards skipping `SPI.begin()`
  on repeat calls combined with an unconditional full `Reset()`+
  `RegConfigSettings()` (SRES strobe + complete register reprogram) on
  every single call — a real CC1101 timing sensitivity around repeated
  resets is plausible but unconfirmed. Moot after the RadioLib switch;
  documented here so a future session doesn't waste time rediscovering it
  if SmartRC is ever reconsidered.
- **A real, still-open upstream bug was found and disclosed, not exploited
  by this project**: SmartRC's `ReceiveData()` has no bounds check against
  its own over-the-air-controlled length byte (`github.com/LSatan/
  SmartRC-CC1101-Driver-Lib/issues/185`) — a real buffer overflow, present
  even in upstream's own bundled example. Moot now that SmartRC was
  replaced, but the general lesson (verify a new library's receive-path
  bounds behavior, don't assume it's safe just because a prior one had a
  bug) is recorded in Global Constraints for any future receive-path work.
- **Hot/cold signal finder bug, found and fixed during this phase's own
  batch build**: the mode initially never resumed RX after retuning, which
  would have sampled a stale RSSI register on every subsequent read — fixed
  before the final build (per the implementing agent's own report; not yet
  independently re-verified against real RF hardware, since no hot/cold
  hardware pass has run).

## Known limitations / deferred work

- **All real-hardware verification is outstanding.** Every `PAUSE FOR
  HARDWARE` step across Tasks 1, 3–8 needs a dedicated session with the
  project owner and the physical module/RF equipment. Task 1's SPI
  bring-up (chip presence, real RSSI reading) is the only piece actually
  exercised against real silicon so far.
- **KeeLoq (Task 8) is real-source-ported but never exercised against a
  real captured frame** — no fixture was available during implementation.
  Raw-frame decode ported from UniGeek's `RCSwitchUtil.cpp`, cipher/
  identify/step logic from `KeeloqUtil.cpp`; SD-loaded manufacturer
  keystore, no keys bundled.
- **KeeLoq logic was not extracted into `shared/subghz_proto`** despite
  being hardware-agnostic in principle (a real consolidation opportunity
  the same shape as Task 2's own decision) — deferred to conserve budget
  across a large batch, disclosed rather than silently skipped.
- **Jammer (Task 7) ports only full and intermittent modes** from Bruce's
  `rf_jammer.cpp` — NOISE/SWEEP modes were out of this task's spec'd scope
  and were not added.
- **Bruteforce/jammer `start()` signatures gained extra parameters**
  (frequency, repeat count) beyond the plan's original interface sketch —
  a real judgment call made to support a working UI, not a spec violation.
- **`Cc1101Record` gained a `load()`** alongside the plan's originally
  specified `save()`, needed for the file-browser wiring in Task 4.
- **Phase 5's own CC1101 work (Cardputer-ADV hydra-hat) is not yet
  implemented.** When it is, it should consume this same
  `shared/subghz_proto` library and RadioLib (not the SmartRC driver its
  own spec originally named) — forward-pointer notes were added to Phase
  5's spec for both.

## How to build, flash, and use

```
cd firmware/tab5
pio run -e tab5                    # build
pio run -e tab5 -t upload --upload-port /dev/cu.usbmodemXXXX   # flash
pio test -e native                 # host-native tests (58+ cases)
cd ../../shared/subghz_proto && pio test -e native   # shared-lib tests (22 cases)
```

CC1101 module must be attached to the Tab5's rear M5-Bus connector with its
DIP switches set: SW1 position 1 ON (CSN), SW2 positions 2 and 6 ON (GDO2,
GDO0), all other positions in both banks OFF — see `pins_config.h`'s own
citation for the full derivation if the physical module is ever replaced or
its switches need re-verifying.

Launcher tiles (once real hardware confirms they work): CC1101 Scan,
Spectrum, Bruteforce, Jammer, KeeLoq — under the new `Category::SUBGHZ`.
Record/Replay have no standalone tile (wired through Scan's save/load flow,
matching RF433's own `rf433_replay.h` precedent).

## Verification: Definition of Done walkthrough

Per spec Section 7:

| Item | Status | Notes |
|---|---|---|
| M-Bus GPIO mapping confirmed, cited | PASS | Section 3 of the spec, two real vendor PDFs |
| DIP-switch position physically set | PASS | Project owner confirmed, matches derivation exactly |
| `shared/subghz_proto` exists, consumed | PASS | 6 decoders extracted + Princeton fresh; RF433 unaffected |
| Real scan/capture + replay demonstrated on hardware | **NOT YET RUN** | Code-complete, hardware pass deliberately deferred (owner's own request) |
| Phase documentation completed | PASS | This document |

4 of 5 items pass; the one outstanding item is real-hardware verification,
explicitly deferred to a dedicated follow-up session rather than rushed
under this session's budget constraint. This phase should not be considered
fully closed until that session runs and either confirms or surfaces real
issues in the code built here.
