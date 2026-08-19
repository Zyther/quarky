# Phase 3 Progress Handoff (2026-08-19)

**Why this file exists:** work on Phase 3 paused mid-plan when the project owner switched from Claude Code to Cursor for cost reasons. This is a self-contained summary for whoever (or whatever tool) picks this up next — it does not assume familiarity with Claude Code's "Subagent-Driven Development" process or its gitignored ledger at `.superpowers/sdd/2026-08-18-phase3-nfc-rf433-ir-plan/progress.md` (that ledger has the full blow-by-blow if you want it and still have access to this checkout, but everything load-bearing is repeated here).

## Where things stand

**Plan:** `docs/superpowers/plans/2026-08-18-phase3-nfc-rf433-ir-plan.md` — 20 tasks, sequenced risk-first (hardware unknowns get their own spike task before any feature is built on top of them).

**Done: Tasks 1-3 of 20, all real-hardware-verified, all on `main`.** Task 14 is also resolved (no separate implementation needed — see below). Everything is committed; there is no uncommitted work.

| Task | What | Status |
|---|---|---|
| 1 | RF433R receive-pin confirmation | **Done.** `TAB5_RF433R_PIN = GPIO53` confirmed via a real interrupt-driven capture test against a continuously-transmitting 433MHz source. |
| 2 | ST25R3916 (Tab5's "NFC unit") register-level bring-up | **Done.** Real chip-ID readback: `reg 0x3F = 0x2A` → `ic_type=0x28, ic_rev=2` — exact match to the driver's own datasheet-cited expected value. |
| 3 | RFID2 (WS1850S) protocol bring-up + 125kHz question | **Done.** Real finding: RFID2's WS1850S is **MFRC522/PN512-protocol, not PN532** as the project's docs originally (wrongly) assumed. Real chip readback: `VersionReg (0x37) = 0x15`. 125kHz/T5577/HID-Prox confirmed **not supported** by either NFC-family unit (neither chip has LF hardware) — this closes Task 14, no separate work needed there. |
| 4-20 | Everything else | **Not started.** |

**Commits, in order, all on `main`:** `536e83b` (RF433 stale-read fix, pre-existing plan context) → `9bae3ed` (Task 1) → `bfe869f`, `0503365`, `79491bc` (Task 2 + a real GPIO53 pin-conflict bug it found and fixed) → `8eac3d4`, `b30a686`, `a3a1b2a`, `dc8ac80` (Task 3 + three rounds of doc-correction sweeps).

## Two real bugs found and fixed along the way (know these before touching NFC/RFID2/RF433 code)

1. **GPIO53 is shared** between the external I2C bus's SDA line (used by both NFC-family units) and the RF433R/T unit's data pin — they're electrically the same pin on the Tab5's HY2.0 PORT.A connector. `main.cpp`'s boot sequence used to unconditionally claim GPIO53 for RF433 at every boot, silently breaking I2C. **Fixed**: RF433 now claims the pin on-demand only (`Rf433Common::capture_start()`), never at boot. **Residual, not yet closed**: this arbitration is currently a *documented convention* (comments in `main.cpp`, `hal/rf433_gpio.cpp`, `hal/nfc_pn532.cpp` all cross-reference each other), not a runtime interlock. It's safe today only because nothing yet has a persistent UI screen that could leave RF433 mid-capture while NFC/RFID2 tries to use I2C concurrently. **The first task that gives either one a real launcher-tile screen (Task 4 for NFC/RFID2, Task 5 for RF433) needs a real claim/release arbiter**, not just comments — this is flagged, not solved.

2. **The project's original assumption "RFID2's WS1850S is PN532-register-compatible" was wrong.** Real donor source (Bruce's `RFID2.cpp`, UniGeek's `MFRC522Screen.cpp`, M5Stack's own `M5Unit-RFID` library) unanimously shows it's MFRC522/PN512-family instead. This has been corrected everywhere it was found to have propagated (took 3 review rounds to find every instance — see "A note on verification discipline" below). **If you're about to write RFID2 feature code, port from Bruce's `RFID2.cpp` / UniGeek's `MFRC522Screen.cpp` (MFRC522-family), not their PN532 modules.** The NFC unit (ST25R3916) is a third, unrelated chip family — don't conflate the two "NFC-family" units' drivers.

## Next task: Task 4 (NFC common module + baseline tag read)

Read the plan file's Task 4 section directly — it was rewritten during Task 3's fix rounds to reflect the real MFRC522 protocol (the old text, which incorrectly said to port a PN532 command, is gone). Briefly: build the shared UID-formatting/result-card UI layer both NFC-family units use, with each unit's driver (`features/nfc/st25r3916_driver.{h,cpp}`, `features/nfc/ws1850s_driver.{h,cpp}` — both already exist and are real, working, hardware-confirmed drivers from Tasks 2-3) plugged in underneath.

## Standing project conventions (apply to all remaining tasks, not just Phase 3)

- **Real sources only.** No fabricated register maps, command bytes, or protocol details — ever. Every register/protocol claim in this codebase traces to a cited datasheet section, a cited donor-project file/line, or a real hardware test result. This project has been burned repeatedly by assumptions that turned out wrong (display panel misidentified, touch controller assumed separate when integrated, both NFC-family units' chip identities wrong in the original docs) — always verify against a real source before writing protocol code, and say so if you can't find one rather than guessing.
- **Hardware pause discipline.** Any step that needs a physical peripheral connected/swapped must get explicit, real-time confirmation from whoever's driving before proceeding — never assume connector state carries over from a previous session, since these units share one physical socket and get swapped by hand.
- **Review before trusting.** Every substantive change in this project's history has gone through independent review (a fresh pass checking the work against its stated requirements, verifying citations against real sources, and re-running or reasoning through the actual logic) before being considered done. Whatever tool/workflow continues this work, keep that discipline — this project's real bugs (the GPIO53 conflict, the wrong PN532 assumption, several ISR-safety issues in earlier BLE work) were caught by review, not by the first pass.
- **Build verification.** `cd firmware/tab5 && pio run` (and `PLATFORMIO_BUILD_FLAGS="-DQUARKY_SERIAL_DEBUG" pio run` for anything touching a serial-debug trigger) must compile clean before any change is considered done.

## A note on verification discipline (learned the expensive way during Task 3)

When a "wrong premise" correction needs to be found everywhere it propagated, searching for the exact wording you already found is not enough — it took three review rounds to catch every instance of the RFID2/PN532 mistake because each sweep searched for phrasing already seen rather than reading every hit a broad search returned. If you're ever correcting a similar wrong-premise situation: grep broadly, then **read every single hit**, not just the ones that look similar to what you already fixed. Undispatched task briefs are the highest-risk surface for this — a wrong sentence in a spec misleads a reader, but a wrong sentence in a not-yet-executed task's instructions becomes a wrong implementation.

## Real hardware state as of this handoff

The RFID2 unit was the last thing connected to PORT.A (used for Task 3's final test). The Tab5 is connected via USB. Serial port was `/dev/cu.usbmodem1101` this session — **re-check this**, ESP32 native-USB-CDC ports re-enumerate on every reset/reflash.

## Where everything lives

- Plan: `docs/superpowers/plans/2026-08-18-phase3-nfc-rf433-ir-plan.md`
- Design spec (corrected during this work): `docs/superpowers/specs/2026-08-06-phase3-tab5-nfc-rf433-design.md`
- New driver code: `firmware/tab5/src/features/rf433/rf433_common.{h,cpp}`, `firmware/tab5/src/features/nfc/st25r3916_driver.{h,cpp}`, `firmware/tab5/src/features/nfc/ws1850s_driver.{h,cpp}`
- Full task-by-task history with every review finding and fix: `.superpowers/sdd/2026-08-18-phase3-nfc-rf433-ir-plan/progress.md` (gitignored, local-only — read it directly in this checkout if you have access, it won't survive a fresh clone)
