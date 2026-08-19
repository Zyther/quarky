# Phase 4: Chameleon Ultra 3.0 Integration — Design

**Status:** Draft for review
**Date:** 2026-08-09
**Depends on:** Phase 1 Foundation — Tab5's raw ESP-IDF NimBLE work (Task 13), Cardputer-ADV's NimBLE-Arduino BLE client (Task 17), `FeatureModule`/`FeatureRegistry` contract, LVGL shell/launcher. Also supersedes Phase 3's explicit "revisit only if a Chameleon Ultra is added to the kit later" deferral of Chameleon-style emulation (`docs/superpowers/specs/2026-08-06-phase3-tab5-nfc-rf433-design.md`, Section 1) — a Chameleon Ultra 3.0 is now inbound, so that deferred scope becomes this phase.
**Scope:** Bridge to an external Chameleon Ultra 3.0 device for full arbitrary LF (125kHz) and HF (13.56MHz) tag read/clone/emulate/write, well beyond what the onboard NFC and RFID2 units (Phase 3) can do on their own (**corrected 2026-08-19: this said "onboard PN532-class" units — neither is a PN532. The NFC unit is an ST25R3916; the RFID2 unit is WS1850S, MFRC522/PN512-family. See the Phase 3 spec's chip-identity and chip-protocol corrections**). Positioned to land before Phase 5 (Cardputer-ADV Satellite) per the project owner's explicit sequencing — Chameleon Ultra hardware arrives first, and its BLE-central integration work should be proven before this program does its first "real" (non-foundation) Cardputer-ADV feature work.

## 1. What the Chameleon Ultra Actually Is

The Chameleon Ultra is a standalone RFID/NFC research tool with its own onboard radio hardware (LF + HF) and battery — this project's device does not do the RF work itself, it acts as a remote-control front-end that sends commands to the Chameleon Ultra over BLE (or USB) and renders whatever comes back. This is architecturally distinct from every other Phase 1-3 radio feature in this project, which all talk to a radio chip this project's own hardware owns directly (ST25R3916, WS1850S, CC1101, nRF24, the C6 — *"PN532" corrected 2026-08-19; this project owns no PN532*). Here, the "radio" is itself a whole separate smart peripheral with its own firmware and command protocol.

## 2. Real Reference Material (do not re-derive the protocol from scratch)

Bruce (this project's donor firmware, local checkout `~/src/firmware`) already has a real, working Chameleon Ultra bridge:

- `~/src/firmware/src/modules/rfid/chameleon.h` / `chameleon.cpp` — the application-level bridge: mode selection (LF/HF read/scan/clone/emulate/save/load/custom-UID, full-scan, battery info, factory reset), UI flow, and file I/O against Bruce's own dump format.
- The actual protocol/transport library is an external dependency, not vendored in Bruce's tree: `https://github.com/bmorcelli/ESP-ChameleonUltra` (referenced twice in `~/src/firmware/platformio.ini`, lines 181 and 247). This is the real source to port from for the BLE command protocol, framing, and connection handshake — do not invent frame formats or service/characteristic UUIDs; pull them from this library's actual source (or the Chameleon Ultra's own official firmware/protocol documentation, which that library itself is presumably built against) during implementation.
- Bruce's usage pattern: `ChameleonUltra chmUltra = ChameleonUltra(true);` then `chmUltra.connectToChamelon()`. The boolean constructor argument almost certainly selects BLE vs. USB-serial transport — confirm which during implementation rather than assuming; this is a one-line check against the library source, not a research project.

Port this feature set closely, matching this program's established principle (already applied to KeeLoq/CC1101/nRF24 donor logic in Phases 3, 5, 6) of reusing proven donor logic rather than re-deriving RF/protocol handling from first principles.

## 3. Feature Inventory

| Feature | Donor reference | Notes |
|---|---|---|
| LF (125kHz) read / scan / clone / emulate / save / load / custom UID | Bruce `chameleon.{h,cpp}` (`Chameleon::AppMode` `LF_*` modes) | Direct port of the mode list; LF band is something neither the Tab5's onboard NFC/RFID2 units nor the Cardputer-ADV's hydra hat can do at all (hydra hat is CC1101 sub-GHz + nRF24 2.4GHz — nothing at 125kHz), so this is genuinely new capability, not overlap. |
| HF (13.56MHz) read / scan / emulation / save / load / clone / write / custom UID | Bruce `chameleon.{h,cpp}` (`HF_*` modes) | Overlaps in *domain* with Phase 3's onboard NFC/RFID2 (both are 13.56MHz), but the Chameleon Ultra can do things the onboard reader units (ST25R3916 / WS1850S — *not* PN532s, corrected 2026-08-19) cannot: arbitrary UID/data emulation, not just passive read. Keep both — Phase 3's onboard units remain the fast/simple/no-external-hardware path; this phase is the power-user path when the Chameleon Ultra is attached. |
| Full scan (LF + HF sweep) | Bruce `FULL_SCAN_MODE` | |
| Battery info / factory reset | Bruce `getBatteryInfo()`/`factoryReset()` | Device-management, not a security feature — low effort, port as-is. |

## 4. Architecture

### 4.1 Connection: Tab5-direct primary, Cardputer-ADV-relay fallback

**Primary plan (per owner direction): the Tab5 connects directly to the Chameleon Ultra over BLE**, acting as a BLE **central** connecting out to the Chameleon Ultra's BLE **peripheral**. This is new territory for this project on the ESP32-P4: every prior Tab5 BLE work (Task 13, this project's C2 GATT server) has the Tab5 as a GATT *server*/peripheral, never as a client/central. Raw ESP-IDF NimBLE (already proven working for the server role on P4, replacing NimBLE-Arduino which doesn't build for P4 at all) does support a central/client role via `ble_gap_connect()`/`ble_gattc_*` — but this project has never yet exercised that role on this chip. Budget real spike time to prove `ble_gap_connect()`-driven central mode works on the P4 before committing to the full feature port; this is exactly the kind of "assumed to work, unverified" risk that has bitten this project's BLE work before (see Phase 1's NimBLE-Arduino P4 incompatibility, found only by trying to actually compile it).

**Fallback plan (owner-specified): if direct Tab5↔Chameleon BLE proves unworkable, relay through the Cardputer-ADV instead** — Cardputer-ADV connects to the Chameleon Ultra over BLE (reusing its already-proven NimBLE-Arduino central role from Task 17's C2 WiFi/BLE client work — Cardputer-ADV already successfully holds a central/client BLE connection today), and relays commands/responses to/from the Tab5 over the existing `c2proto`/`IC2Link` C2 channel. This adds relay latency and a new `c2proto` message shape (Chameleon command/response framing riding inside a `Frame` payload, similar in spirit to how Phase 5-6's remote features carry feature-specific payloads) but reuses an already-proven BLE role instead of a new one. If Cardputer-ADV ends up holding two simultaneous BLE connections (its existing C2 link to Tab5, plus a new one to the Chameleon Ultra), confirm NimBLE-Arduino's multi-connection-different-roles support actually works on this hardware before committing — don't assume it from the library's general capability claims alone.

Implementation should attempt the primary path first, structured so the fallback path can be swapped in without a redesign (i.e., keep the Chameleon command/response protocol logic itself decoupled from which device/transport carries it — this same discipline already exists in this project's `IC2Link` abstraction).

### 4.2 UI Pattern

Mode selection (LF/HF, read/scan/clone/emulate/save/load) as an `lv_list`-driven menu on the Tab5, matching Phase 3's NFC UI pattern for consistency (same device, same interaction class — tag read/emulate). Tag data (UID, type, dump contents) rendered the same result-card pattern Phase 3 established. Chameleon Ultra's own battery/connection status shown in a persistent status area, similar to the existing Cardputer-ADV connection indicator in the Tab5 shell status bar (Task 19).

### 4.3 Data Format

Chameleon Ultra dumps/saves land in `/quarky/captures/chameleon/` on Tab5's SD card, continuing the `/quarky/captures/<category>/` convention from Phases 2-3.

## 5. Risks / Open Questions

- **Tab5-as-BLE-central on the P4 is entirely unproven** — this is the single biggest risk in this phase and should be spiked first, before any feature-logic porting. If raw ESP-IDF NimBLE's central/client role has gaps or bugs on the P4 (parallel to the real gaps found in NimBLE-Arduino's P4 support during Phase 1), fall back to the Cardputer-ADV-relay path per Section 4.1 rather than forcing it.
- **Exact BLE service/characteristic UUIDs and frame format are not yet confirmed** — pull from `bmorcelli/ESP-ChameleonUltra`'s actual source during implementation, not from this document (which deliberately does not fabricate protocol-level details it hasn't verified).
- **USB fallback untested but plausible**: the Chameleon Ultra also supports a USB-serial CLI (the `ChameleonUltra(true)` constructor argument in Bruce's usage suggests the library supports both transports). If BLE (both direct and relayed) proves unworkable, a wired USB connection to the Tab5 is a third fallback worth considering, though the Tab5 would need USB-host role, which nothing in this project currently exercises — a bigger lift than either BLE path, so treat as a last resort only.
- **Feature overlap with Phase 3's onboard NFC/RFID2** is intentional, not a duplication bug — see Section 3's HF row. UI should make clear to the user which "backend" (onboard reader unit — ST25R3916 or WS1850S, *not* a PN532, corrected 2026-08-19 — vs. attached Chameleon Ultra) a given screen is driving, since they have different capabilities and the Chameleon Ultra may not always be attached/powered.

## 6. Testing Strategy

- Protocol/framing logic (command encode, response decode, LRC/CRC if the real protocol uses one) is pure logic once the real frame format is confirmed — cover with a native host test, no hardware needed, table-driven against known-good frames from the reference library or official docs.
- BLE central-role connection itself can only be verified on real hardware (this project has no BLE simulator) — real Chameleon Ultra 3.0 hardware is already inbound, so this is a real-hardware bring-up task like every other radio feature in this project, not a simulated one.
- LF/HF read/emulate features verified against real tags, following the same "known, deliberately weak/test tag first" discipline Phase 3 established for MIFARE work.

## 7. Definition of Done

- [ ] Tab5 (or, if the primary path is abandoned per Section 4.1's spike, Cardputer-ADV-relayed) successfully establishes a real BLE connection to a physical Chameleon Ultra 3.0 and completes at least one full command/response round trip (e.g., battery info query) — real hardware evidence, not just a compile.
- [ ] At least one LF and one HF read operation demonstrated against a real tag.
- [ ] At least one emulate operation demonstrated (Chameleon Ultra emulating a tag this project's own firmware told it to load).
- [ ] Chosen transport path (direct-to-Tab5 vs. Cardputer-ADV-relay) documented with the real reason for the choice, in this phase's eventual `docs/phases/phase-4-chameleon-ultra.md` write-up per this program's per-phase documentation convention (see `CLAUDE.md`).
