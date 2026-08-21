# Quarky

Unified offensive/defensive security suite firmware. M5Stack Tab5 (ESP32-P4 + ESP32-C6) as a touch-driven command center, controlling an M5Stack Cardputer-ADV (ESP32-S3) satellite over a dual WiFi/BLE C2 transport. Built by consolidating features from three donor firmwares (Bruce, Poseidon, UniGeek — all local checkouts under `~/src/`), authorized for derivation.

Program is decomposed into phases; each phase has a design spec and (once its turn comes) an implementation plan:
- Specs: `docs/superpowers/specs/<date>-phaseN-*-design.md` (Phase 1 is `tab5-foundation-design.md`)
- Plans: `docs/superpowers/plans/<date>-*-plan.md`
- Phase documentation (what was actually built): `docs/phases/phase-N-*.md`

### Phase roadmap

Phase numbers are execution order, **except Phase 10** (see note). Don't infer this list from spec filenames alone — the gap after 8 is intentional.

| # | Name | Device | Status |
|---|---|---|---|
| 1 | Foundation | Both | Complete |
| 2 | Tab5-Native 2.4GHz WiFi/BLE Suite | Tab5 | Not started |
| 3 | Tab5-Native NFC/RFID2/RF433/IR Peripherals | Tab5 | Not started |
| 4 | Chameleon Ultra 3.0 Integration | Tab5 (BLE-central, fallback via Cardputer-ADV relay) | Not started |
| 5 | Cardputer-ADV Satellite — CC1101 Hydra-Hat, nRF24 | Cardputer-ADV | Not started |
| 6 | GNSS/SX1262 LoRa | Cardputer-ADV | Not started |
| 7 | ESP32-C5 5GHz Sidecar | C5 (new satellite) | Not started |
| 8 | Remaining Feature Sweep | Whichever fits | Not started |
| 10 | Tab5-Native CC1101 via M-Bus | Tab5 | Not started |

Phases 4-8 were renumbered from an original 4-7 spread when Phase 4 (Chameleon Ultra) was inserted on 2026-08-09, per the project owner's explicit direction to sequence it right before Phase 5's real Cardputer-ADV feature work. Phase 10 was deliberately given a non-sequential number (owner's choice, not a numbering mistake) when this hardware became available after the rest of the roadmap was drafted — it has no dependency on Phase 9 (which doesn't exist) and can run whenever its own M-Bus GPIO-mapping risk (see its spec) is resolved.

IR was moved from Phase 5 (Cardputer-ADV) to Phase 3 (Tab5) on 2026-08-18, per the project owner's explicit direction: they're adding a dedicated IR receiver/transmitter HY2.0 unit to the Tab5, replacing the original plan of using Cardputer-ADV's own onboard IR transmit LED (GPIO 44, active-low, transmit-only, receive-diode presence never confirmed — see Phase 5 spec's original Section 1 for that now-superseded hardware story). Phase 3's and Phase 5's specs were both updated accordingly.

**Corrected 2026-08-21** (real hardware arrived, real M5Stack datasheet read): the unit is M5Stack's "Unit IR" (SKU U002) — it is **not** an I2C device. Per its own schematic/pin map (`~/Downloads/ir.pdf`, fetched from M5Stack, Update Time 2026-08-06): HY2.0-4P wires are Black=GND, Red=5V, Yellow=IR_TX, White=IR_RX — a plain transistor-driven 940nm IR LED on the TX side and a IRM-3638T 38kHz-hardware-demodulating receiver module on the RX side, both driven as ordinary digital GPIO, no register/command protocol at all. The datasheet's own pin map labels this "PORT.B", but Tab5 has only one physical HY2.0-4P connector (PORT.A, GPIO53/54 — see `pins_config.h`'s own citation). Per the project owner: Tab5 does not have a physical PORT.B: PORT.A's same two signal pins (GPIO53/54) are used as plain GPIO for this unit, a THIRD mode alongside the I2C (NFC/RFID2) and raw-digital (RF433R/T) modes those pins already carry — needs the same GPIO53 arbiter treatment already established for RF433, extended with a third owner.

## Process: every phase ends with documentation, not just a Definition-of-Done check

When a phase's implementation plan reaches its final task(s), two things happen, not one:

1. **Definition-of-Done walkthrough** — a bring-up log verifying every DoD item from that phase's spec against real hardware results (pass/fail + notes per item). This already exists as the plan's own last task for Phase 1; later phases' plans should include an equivalent final task.
2. **Phase documentation** — a durable, developer-facing doc at `docs/phases/phase-N-<name>.md` describing what was built, the architecture decisions and why, real-hardware findings (root causes, not just "fixed a bug"), known limitations/deferred work, and how to build/flash/use what the phase delivered. See `docs/phases/phase-1-foundation.md` for the expected shape and depth. This is written FOR future readers (including future Claude sessions) who need to work on top of the phase without re-deriving its history — it's a distillation of the SDD ledger and task reports, not a duplicate of them.

Do this for every phase, not just Phase 1 — when starting Phase 2+'s final task(s), add the documentation step even though the phase's plan may predate this convention.

## Known UX debt to account for in future Tab5 UI work

The pairing screen (`firmware/tab5/src/ui/pairing_screen.cpp`) has two real usability problems, left unfixed in Phase 1 since fixing UI polish wasn't in scope for a foundation hardware bring-up:
- It displays a QR code for pairing, but the Cardputer-ADV has no camera — the QR half of the flow is dead weight as currently designed.
- The on-screen back button sits directly on top of the QR code and is small enough to be hard to tap reliably.

Any future phase that touches Tab5 UI — especially pairing flows or anything else QR-driven — should not copy this layout forward uncritically.

## Real-hardware verification environment notes (this session's Mac)

- The Bash tool's sandbox cannot see `/dev/cu.*` device nodes at all. Pass `dangerouslyDisableSandbox: true` on any Bash call touching a serial port or `pio run -t upload`.
- ESP32 native-USB-CDC ports re-enumerate on every reset — re-check `ls -la /dev/cu.usbmodem*` (sandbox disabled) before trusting a port name, especially right after a flash.
- `pio device monitor` doesn't work in this non-interactive shell (termios error) — use a reconnect-tolerant pyserial capture loop instead.
- This Mac has repeatedly hit near-full disk capacity from causes unrelated to this project; check `df -h /` during any build-heavy work and clean `.pio/build` (never `.pio/libdeps`) if needed.

## Full history

Every task, hotfix, and real-hardware finding for Phase 1 is recorded in the SDD ledger at `.superpowers/sdd/2026-08-06-tab5-foundation-plan/progress.md` (gitignored — local working record, not a committed artifact). `docs/phases/phase-1-foundation.md` is the durable distillation of it.
