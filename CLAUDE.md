# Quarky

Unified offensive/defensive security suite firmware. M5Stack Tab5 (ESP32-P4 + ESP32-C6) as a touch-driven command center, controlling an M5Stack Cardputer-ADV (ESP32-S3) satellite over a dual WiFi/BLE C2 transport. Built by consolidating features from three donor firmwares (Bruce, Poseidon, UniGeek — all local checkouts under `~/src/`), authorized for derivation.

Program is decomposed into phases; each phase has a design spec and (once its turn comes) an implementation plan:
- Specs: `docs/superpowers/specs/2026-08-06-phaseN-*-design.md` (Phase 1 is `tab5-foundation-design.md`)
- Plans: `docs/superpowers/plans/2026-08-06-*-plan.md`
- Phase documentation (what was actually built): `docs/phases/phase-N-*.md`

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
