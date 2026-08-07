# Quarky: Tab5 Command-Center Security Suite — Foundation Design

**Status:** Approved for implementation planning
**Date:** 2026-08-06
**Scope of this document:** Sub-project 1 of a multi-phase program (see [Program Roadmap](#program-roadmap)). This spec covers the foundation only — HAL, UI shell, build system, and inter-device protocol. No offensive/defensive feature is implemented in this phase.

## 1. Background and Goal

Build a unified, multi-target offensive/defensive security suite that consolidates the capabilities of three existing ESP32 pentesting firmwares — **Bruce** (`~/src/firmware`), **Poseidon** (`~/src/poseidon-tab5`), and **UniGeek** (`~/src/unigeek-main`) — around an **M5Stack Tab5** (ESP32-P4 + ESP32-C6) acting as a touch-driven command center, with an **M5Stack Cardputer-ADV** (ESP32-S3) as a controllable satellite device for radio hardware the Tab5 doesn't carry, and a future **ESP32-C5** satellite for 5GHz work neither device can do natively.

The long-term goal (stated explicitly by the project owner) is complete feature parity: every capability across all three donor firmwares should eventually exist and function somewhere in this suite. Given the scope — 5 hardware targets, ~150+ distinct features, a from-scratch touch UI, and a from-scratch cross-device control protocol — the program is split into sequential sub-projects. This document specs only the first: the foundation everything else builds on.

**Licensing note:** the project owner has confirmed authorization to use and derive from Bruce (AGPLv3), Poseidon (claimed MIT, no LICENSE file present in the checked-out repo), and UniGeek (no LICENSE file present). Licensing hygiene for eventual distribution is out of scope for this phase per explicit direction, but should be revisited before any public release.

## 2. Research Summary: What the Three Donor Projects Actually Offer

### Bruce (`~/src/firmware`)
- License: AGPLv3.
- Broadest feature set of the three: WiFi (deauth, evil portal, karma, sniffer, PMKID/handshake, wardriving), BLE (spam, Bad-BLE, Fast Pair/HFP exploits), Sub-GHz via CC1101 (scan/record/replay/spectrum/waterfall/jam/bruteforce) plus simple GPIO RF433, nRF24 (jammer, mousejack, 2.4GHz spectrum), LoRa/SX1262/SX1276 via RadioLib, NFC/RFID (PN532, MFRC522, Chameleon Ultra emulation, EMV), IR (TV-B-Gone + universal remote), GPS/wardriving, FM transmit (Si4713), BadUSB (USB+BLE HID), a JS scripting engine (mquickjs), and more.
- ~35 supported boards, all glued together via per-board `.ini`/`pins_arduino.h`/`interface.cpp` quads and heavy compile-time `#ifdef` flags rather than a clean HAL.
- Custom (non-LVGL) UI with a pluggable graphics backend (TFT_eSPI/ArduinoGFX/LovyanGFX/M5GFX) and **already has working touch + on-screen virtual keyboard code** (`HAS_TOUCH` path in `src/core/mykeyboard.cpp`) — useful reference, though built for small low-res screens.
- Active ESP32-C5 (5GHz) support already exists (`boards/ESP32-C5*`). No ESP32-P4/Tab5 code. Cardputer-ADV board JSON exists but is a dead, unwired stub; the base Cardputer's TCA8418 I2C keyboard-controller code is a usable template for real ADV keyboard support.
- Radio libraries: `bmorcelli/SmartRC-CC1101-Driver-Lib` (CC1101), `nrf24/RF24` (nRF24), `jgromes/RadioLib` (SX1262/SX1276), `bmorcelli/rc-switch` (fixed-code RF433), `h2zero/NimBLE-Arduino`.

### Poseidon (`~/src/poseidon-tab5`)
- Despite its directory name, **this is not a Tab5 project.** It targets the Cardputer-Advance (ESP32-S3) directly, with a separate ESP32-C5 satellite ("TRIDENT") for 5GHz WiFi and Zigbee, linked over **ESP-NOW**. An uncommitted `[env:tab5]` PlatformIO stanza exists but is dead/non-functional (wrong screen geometry, no touch/display code, untested).
- 164 claimed features: rich WiFi/BLE/Sub-GHz/nRF24/LoRa/network-attack ("SaltyJack") suites, plus a hand-rolled FIDO2/U2F stack ("KERBEROS") and a heap-budget-aware architecture born from real ESP32-S3 SRAM pressure.
- No touch, no virtual keyboard — UI assumes a physical keyboard is always present (letter-mnemonic menu navigation).
- **Most directly reusable artifact: the ESP-NOW command/response protocol to its C5 satellite** (`c5_node/main/proto.h` — magic+version+type+seq+payload framing). This is the closest existing precedent for a brain-and-satellite radio architecture, just inverted relative to our design (there, Cardputer is the brain).
- Documented, real hardware limitation worth carrying forward: ESP-NOW's ~230-byte usable payload made full EAPOL handshake capture over that link impractical without fragmentation — directly informed this design's decision to split control (ESP-NOW) from bulk transfer (WiFi socket).
- License: claimed MIT, no LICENSE file present in this checkout.

### UniGeek (`~/src/unigeek-main`)
- License: none present in this checkout (one credited reference, `pico-fido`, is AGPL — code paths derived from it would inherit that).
- Multi-board (~18 targets) firmware, spiritually descended from Evil-M5Project and Bruce. **Ships real, working M5Stack Cardputer-ADV support today**, including working CC1101 and nRF24 drivers and the correct pin map.
- Confirms an important hardware constraint carried into this design: on Cardputer-ADV, CC1101 and nRF24 share the same Grove/hat SPI pins and are **electrically exclusive — only one hat radio active at a time.** SX1262/LoRa pins are reserved on this board but the driver is unimplemented (open TODO).
- **Best architecture reference of the three**: a clean `Device`/`IDisplay`/`INavigation`/`IStorage` interface-based HAL with a hard per-board isolation rule, enforced in project conventions. This pattern is adopted directly for both Tab5 and Cardputer-ADV in this design.
- Has an Android + browser companion app using a screen-mirror-over-BLE-UART protocol (phone/browser ↔ single board) — a useful conceptual reference for remote display/control, though single-device, not board-to-board.

### Confirmed Tab5 hardware (via Espressif/M5Stack documentation)
- **Main SoC:** ESP32-P4 (dual-core RISC-V, no native WiFi/BT/802.15.4 radio), 16MB flash, 32MB PSRAM, has a 2D PPA (pixel processing accelerator).
- **Radio co-processor:** ESP32-C6-MINI-1U, connected via **SDIO**, provides WiFi 6 (2.4GHz only — the C6 does **not** support 5GHz, which is why 5GHz work is pushed to a separate C5 satellite in phase 6) + BLE 5 + 802.15.4 (Zigbee/Thread).
- **Display:** 5" 1280×720 IPS, MIPI-DSI, GT911 multi-touch controller over I2C.
- **Camera:** MIPI-CSI, SC2356 2MP (available for future use; not used in this phase).
- **Storage:** microSD over SDIO (separate host from the C6 link — to be confirmed during hardware bring-up, see [Risks](#7-risks--open-questions)).
- **Radio bridge:** Espressif's official **esp-hosted-mcu** ("WiFiRemote") is the documented, supported solution for exactly this P4↔C6-over-SDIO pairing — it's the same architecture used on Espressif's own ESP32-P4-Function-EV-Board. Under Arduino-ESP32 v3.x, esp-hosted makes the C6's radio appear to application code as a normal local `WiFi.h`/NimBLE interface, which is the key enabler for reusing Bruce's WiFi/BLE module code with minimal changes in later phases.

### Owner's hardware inventory (confirmed in hand)
- 1x M5Stack Tab5
- 1x M5Stack Cardputer-ADV
- 1x combo CC1101 + nRF24 "hydra" hat (for Cardputer-ADV) — CC1101 tuned for 433MHz but usable across ~300–900MHz
- 1x GNSS + SX1262 (LoRa) hat (for Cardputer-ADV)
- Tab5-side M5 HY2.0 units: **NFC unit, RFID2 unit, RF433R (receiver) unit, RF433T (transmitter) unit**
- IR capability is intentionally kept Cardputer-ADV-only (no Tab5-side IR unit planned)

Since all target hardware is physically available, this foundation phase's Definition of Done ([Section 8](#8-definition-of-done)) includes real on-device verification, not just compile-clean builds.

## 3. Program Roadmap

This spec covers **Phase 1** only. The full program:

| Phase | Scope | Primary device(s) |
|---|---|---|
| 1 | **Foundation** (this spec): HAL, LVGL UI shell + virtual keyboard, multi-target build, Tab5↔Cardputer-ADV C2 protocol, feature-module contract | Tab5, Cardputer-ADV |
| 2 | Native 2.4GHz WiFi/BLE suite via esp-hosted (scan, deauth, evil portal, karma, PMKID, sniffer, BLE scan/spam/spoof) | Tab5 |
| 3 | Native peripherals: NFC/RFID2 (read/write/clone/emulate) and fixed-code RF433 (scan/replay/bruteforce) | Tab5 |
| 4 | Satellite firmware: IR, CC1101 hydra-hat sub-GHz (scan/record/replay/spectrum/jam/bruteforce, 300–900MHz), nRF24 (spectrum/jam/MouseJack/BLE-spam), all remotely controllable from Tab5 | Cardputer-ADV |
| 5 | GNSS/SX1262 LoRa: LoRa scan/beacon/Meshtastic, GPS wardriving — least mature area across all three donors, most from-scratch work | Cardputer-ADV |
| 6 | 5GHz sidecar modeled on Poseidon's TRIDENT: 5GHz WiFi scan/deauth/PMKID, ESP-NOW-linked directly to Tab5 | ESP32-C5 (new) |
| 7 | Remaining feature sweep: BadUSB/HID injection, FIDO2/KERBEROS, scripting engine, defensive/anomaly detection, stragglers | Allocated per fit once 2–6 land |

Each phase after Phase 1 gets its own spec → plan → implementation cycle.

## 4. Architecture

### 4.1 Repository Layout

Monorepo, board-per-directory (matching the pattern both Bruce and UniGeek already use):

```
quarky/
├── firmware/
│   ├── tab5/                 # ESP32-P4 target — Arduino-ESP32 v3.x + esp-hosted WiFiRemote
│   │   ├── platformio.ini
│   │   ├── src/
│   │   │   ├── hal/           # IDisplay, ITouch, IRadio, INFC, IRF433, IC2Link, IPower impls
│   │   │   ├── ui/            # LVGL shell: status bar, launcher, app-view stack
│   │   │   └── main.cpp
│   │   └── boards/tab5/       # pin map, board-specific config
│   ├── cardputer-adv/         # ESP32-S3 target — Arduino-ESP32 v3.x
│   │   ├── platformio.ini
│   │   ├── src/
│   │   │   ├── hal/            # extends UniGeek's Device/IDisplay/INavigation pattern + IC2Link
│   │   │   ├── remote/          # C2 command dispatcher → local feature invocation
│   │   │   └── main.cpp
│   │   └── boards/cardputer-adv/
│   └── c5-node/                # ESP32-C5 target — not built in this phase (Phase 6)
├── shared/
│   ├── c2proto/                 # wire protocol: framing, ESP-NOW + WiFi-socket transport, PSK crypto
│   │   ├── proto.h              # message types, frame structs
│   │   ├── transport_espnow.*
│   │   ├── transport_wifi.*
│   │   └── crypto.*             # PMK/LMK setup, HMAC-SHA256 helpers
│   └── feature_contract/         # FeatureModule interface, FeatureRegistry, capability negotiation
├── tools/
│   ├── provision_key.py          # generates/writes the PSK during first-time pairing
│   └── flash.sh
└── docs/superpowers/specs/       # this document and all future sub-project specs
```

### 4.2 Device Roles

- **Tab5 — command center.** Owns the LVGL UI and all user interaction (touch + on-screen keyboard). Runs WiFi/BLE natively via the C6 co-processor, transparently through esp-hosted's WiFiRemote (Arduino `WiFi.h`/NimBLE APIs work unmodified). Hosts the NFC, RFID2, RF433R, and RF433T HY2.0 units directly. Dispatches commands to satellites and renders their telemetry/results. This is the single point of interaction for the whole suite.
- **Satellite (present phase: Cardputer-ADV) — not headless.** Retains its own screen and physical keyboard; can run fully standalone with its local menu system unmodified (important for field use without the Tab5). A remote-control mode layers on top, accepting commands from the Tab5 over the C2 link and streaming results back, without disabling local operation. Hosts IR, the CC1101/nRF24 hydra hat, and the GNSS/SX1262 hat.
- **Satellite (future, Phase 6: C5-node) — pure satellite.** No UI. ESP-NOW controlled directly by the Tab5 (not routed through Cardputer-ADV). Hosts 5GHz WiFi only.

### 4.3 Hardware Abstraction Layer

Both firmware targets adopt UniGeek's interface-based `Device` HAL pattern — the one architecturally reusable piece identified across the three donor codebases — rather than Bruce's macro/`#ifdef`-heavy per-board coupling.

**Tab5 HAL interfaces:**
| Interface | Responsibility |
|---|---|
| `IDisplay` | MIPI-DSI panel driver + LVGL draw/flush callback |
| `ITouch` | GT911 multi-touch, feeds LVGL's input device layer |
| `IRadio` | Thin wrapper over esp-hosted WiFiRemote (WiFi + BLE) |
| `INFC` | PN532-based driver shared by the NFC unit and RFID2 unit |
| `IRF433` | Simple GPIO RX/TX driver for the RF433R/RF433T units (fixed-code protocols only — analogous to Bruce's `M5_RF_MODULE` path, distinct from CC1101's full-spectrum capability which lives on Cardputer-ADV) |
| `IC2Link` | ESP-NOW + WiFi-socket transport to satellites, built on `shared/c2proto` |
| `IPower` | Battery/charge state |

**Cardputer-ADV HAL** extends UniGeek's existing `Device` implementation for this exact board (it already has correct pin maps and working CC1101/nRF24 drivers) rather than rebuilding it, adding only:
- `IC2Link` (satellite side of the same transport)
- A remote-command dispatcher mapping incoming Tab5 commands onto the same local feature functions its own on-device menu already calls, so feature logic is never duplicated between standalone and remote-controlled operation.

### 4.4 UI Shell (Tab5)

- **Framework: LVGL**, chosen over extending Bruce's custom canvas UI because LVGL's `lv_keyboard` widget directly solves the "no physical keyboard, touch-only" requirement, is the de facto standard for this class of touch hardware, and the P4's PPA + 32MB PSRAM comfortably drive it at 1280×720.
- **Screen model:** a root shell (status bar showing battery, link state to each satellite, active-radio indicator; an app-launcher grid below it) that pushes/pops full-screen app views. Each app view owns one feature category (WiFi, BLE, Sub-GHz, NFC, etc.) — the same category-per-screen mental model Poseidon and UniGeek already use, adapted from keyboard-driven to touch-driven.
- **Text input** uses `lv_keyboard` bound to `lv_textarea` fields uniformly across the suite — SSID entry, frequency input, file names, PSK provisioning, etc. No custom keyboard/hit-testing code to maintain.
- **Touch targets** sized for finger input on a 5" tablet screen — explicitly not reusing Bruce's small-screen touch hit-box code.
- **Device status is first-class UI**, not bolted onto individual feature screens: a persistent "Devices" panel reachable from the status bar shows each connected satellite, its link quality, and what it's currently running.

### 4.5 Cross-Device Control Protocol (`shared/c2proto`)

**Amendment (2026-08-07, discovered during Task 11 implementation):** the original design below used ESP-NOW as the control channel. This turned out to be impossible on the Tab5: the ESP32-P4 has no native WiFi/BT radio of its own, and the esp-hosted/`esp_wifi_remote` RPC bridge that proxies WiFi to the onboard ESP32-C6 (see §4.1's `IRadio`) does **not** proxy the ESP-NOW API surface at all — confirmed by the complete absence of `libespnow.a` for the `esp32p4` target in the installed Arduino-ESP32 framework (every other Espressif target ships it), and zero ESP-NOW entries among the ~89 functions `esp_wifi_remote_api.h` actually proxies. `esp_now.h` exists only as a source-compatibility stub with nothing linkable behind it. This is a confirmed, verifiable platform gap, not a configuration issue — see `docs/superpowers/plans/2026-08-06-tab5-foundation-plan.md`'s ledger for the full verification trail. The design below is the corrected one; the redesign was directed by the project owner.

**Two transports, selected by which radio is free — not split by traffic shape.** The original ESP-NOW/WiFi-socket split was motivated by wanting the control channel to never interfere with whatever radio mode a feature has active. With ESP-NOW off the table, that same goal is met differently: Tab5 carries C2 traffic over **whichever of WiFi or BLE isn't the radio the active feature needs** — a WiFi attack in progress routes C2 over BLE, a BLE attack in progress routes C2 over WiFi. This is buildable today: unlike ESP-NOW, BLE (via NimBLE) **is** confirmed working on the Tab5 — `libbt.a` ships for `esp32p4` and contains a linkable `ble_hs_init`, verified directly against the installed framework.

**WiFi transport — TCP socket.**
- Used for both control (`CMD_START_FEATURE`, `CMD_STOP_FEATURE`, `CMD_GET_STATUS`, `RESP_STATUS`, `RESP_TELEMETRY`, `RESP_BULK_READY`) and bulk data (pcap/handshake files, `.sub` captures, firmware-update pushes) over the same connection — with ESP-NOW's payload ceiling no longer a factor, there's no longer a reason to split control and bulk framing on this transport the way the original Poseidon-inspired design did.
- Every frame (small or large) is `c2proto::Frame`-framed and HMAC-SHA256-authenticated using the provisioned key, same as before.
- Requires Tab5 and Cardputer-ADV to be WiFi-associated (one hosts an AP the other joins, or both join a shared network) — a real tradeoff against ESP-NOW's association-free operation, accepted as the cost of this platform gap.

**BLE transport — GATT service (Nordic-UART-style write/notify characteristics).**
- Carries the same `c2proto::Frame` framing and HMAC authentication, sized to fit within a negotiated extended ATT MTU (matching `c2proto::kMaxPayload = 200` bytes plus header, distinct from `shared/c2proto`'s original ESP-NOW-sized ceiling but still deliberately small).
- Control-message use only in this phase (`CMD_*`/`RESP_STATUS`/`RESP_TELEMETRY`) — if a bulk transfer is needed while BLE is the active C2 transport, it waits for the WiFi radio to free up rather than chunking large payloads over BLE; that's a Phase 2+ concern once real concurrent-feature scheduling exists, not a foundation-phase problem.
- Cardputer-ADV's ESP32-S3 has native BLE with no remoting involved, so this side of the link is unaffected by the P4-specific gap.

**Transport selection, this phase vs. later:** the foundation phase implements both transports as concrete `IC2Link` implementations and proves each works (demonstrated via Task 20's ping feature), with selection between them exposed as an explicit choice rather than automatic. Automatic radio-aware switching — "use BLE because a WiFi feature is currently running" — needs live state from `FeatureRegistry` about which radio a running feature holds, which doesn't exist until Phase 2+ features are real; building that scheduling logic against a foundation phase with no real features to schedule around would be speculative. This is documented explicitly as deferred, not silently dropped.

**Pairing/provisioning (static pre-shared key, one-time) — unaffected by the transport redesign:**
1. Tab5 generates a random 128-bit key and displays it as an on-screen QR code.
2. Cardputer-ADV has no camera, so the key is entered via its physical keyboard (as a short base32/hex string) or transferred via a file dropped on its SD card.
3. Both devices persist the key in NVS.
4. Re-pairing = wipe NVS on both sides, repeat from step 1.
5. No per-session pairing ceremony required after initial setup — chosen over interactive per-session pairing as the right tradeoff for a personal 2–3-device kit rather than a fleet.
6. The same key authenticates both transports (WiFi HMAC frames and BLE GATT frames) — one pairing ceremony, not one per transport.

**Command surface is deliberately minimal in v1** — the message types listed above and nothing feature-specific. Feature parameters and telemetry ride inside a generic payload blob defined per feature module (see below), so Phases 2–7 can add features without ever touching the core protocol.

**Forward note for Phase 6 (ESP32-C5 sidecar):** the C5 has native WiFi and BLE (no P4-style remoting gap), so its link to the Tab5 is unaffected by this amendment — that design decision in the Phase 6 spec should be revisited only if it assumed ESP-NOW specifically; if so, apply the same WiFi/BLE dual-transport pattern there for consistency, since the Tab5 side of that link has the identical P4 constraint.

### 4.6 Feature Module Contract (`shared/feature_contract`)

Establishes now, in the foundation, how every feature in every later phase plugs into the system:

- **`FeatureModule` descriptor:** id, display name, category, target-device affinity (`TAB5_NATIVE` / `CARDPUTER_ADV` / `C5_NODE`), a parameter schema (drives an auto-generated LVGL input form — text field, frequency field, dropdown, etc.), and a telemetry/result renderer.
- **Tab5-native features** (Phase 2 WiFi/BLE, Phase 3 NFC/RFID2/RF433) implement both the descriptor and the execution logic in one module.
- **Remote features** (Phase 4 IR/CC1101/nRF24, Phase 5 LoRa/GNSS, Phase 6 5GHz) are split: the Tab5 side holds only the descriptor (parameter form + result rendering); the actual execution logic lives in the satellite's firmware. The descriptor builds a command payload from user input, ships it over `c2proto`, and renders whatever telemetry comes back.
- **Static registration only** — all modules compiled directly into the firmware image and registered into a `FeatureRegistry` at startup, populating the app-launcher grid by category. No dynamic/runtime plugin loading at this scale.
- **Capability negotiation at connect time:** when a satellite connects (or reconnects), it reports the feature IDs/versions it actually supports in firmware. The Tab5 UI grays out or hides any feature tile the connected satellite doesn't currently support, preventing a mismatched-firmware situation where a command is sent to a satellite that silently can't handle it — important given the goal of full feature parity across every donor firmware.

## 5. Data Flow Example (Illustrative — no real feature is built in this phase)

To validate the contract end-to-end, Phase 1 includes one trivial "ping" feature:

1. User taps the "Ping Satellite" tile on the Tab5 UI.
2. Tab5's `FeatureModule` descriptor builds a `CMD_START_FEATURE` message with feature-id = `ping`, no parameters.
3. Sent over the ESP-NOW control channel to Cardputer-ADV.
4. Cardputer-ADV's remote dispatcher receives it, invokes the local `ping` handler (a no-op that just measures round-trip and reads local battery/uptime), and returns `RESP_TELEMETRY`.
5. Tab5 renders the returned telemetry (round-trip time, satellite battery %, uptime) in the app view.

This proves the descriptor/dispatcher split, the ESP-NOW round trip, and the UI rendering path all work before any real feature is built on top in Phase 2+.

## 6. Testing Strategy

- **Build verification:** top-level build script produces both `tab5.bin` and `cardputer-adv.bin` from a single invocation; CI-style compile check on every change.
- **On-device HAL verification:** each HAL interface (`IDisplay`, `ITouch`, `IRadio`, `INFC`, `IRF433`, `IC2Link`, `IPower`) gets a minimal on-device smoke test exercising it in isolation before the UI/protocol layers are built on top.
- **Protocol verification:** `shared/c2proto` framing/crypto logic is unit-testable host-side (native PlatformIO test environment, following Poseidon's precedent of host-native Unity tests for portable logic like its `kerberos_core`), independent of radio hardware.
- **End-to-end verification:** the ping feature (Section 5) run on real hardware is the integration test that closes out this phase — see Definition of Done below.

## 7. Risks / Open Questions

- **SD card vs C6 SDIO bus sharing on Tab5** — expected to be separate SDIO hosts (matching Espressif's P4-Function-EV-Board reference design) but not yet confirmed against the actual Tab5 board; first item to verify in hardware bring-up, since a conflict here would affect the HAL design for both `IDisplay`'s asset loading and `IC2Link`'s bulk-transfer channel.
- **esp-hosted WiFiRemote maturity under Arduino-ESP32 v3.x** — actively developed by Espressif but relatively new; version-mismatch issues between host and co-processor firmware have been reported in the wider community (P4↔C6 RPC timeouts). Foundation bring-up should pin specific tested versions of both the P4 and C6 firmware rather than tracking latest.
- **CC1101/nRF24 mutual exclusivity on the hydra hat** (confirmed via UniGeek) means Phase 4's feature-module design must expose only one of the two as active at a time on Cardputer-ADV — the capability-negotiation mechanism in Section 4.6 should reflect this as a runtime constraint, not just a static capability list.
- **LVGL performance at 1280×720 on ESP32-P4** — expected to be comfortable given the PPA and 32MB PSRAM, but not yet measured; worth a basic frame-rate check during bring-up before committing to heavier UI polish (animations, transitions) in later phases.

## 8. Definition of Done

Given all target hardware is in hand, this phase closes with real on-device verification:

1. Multi-target build produces both `tab5.bin` and `cardputer-adv.bin` from one top-level build.
2. LVGL boots on the physical Tab5; touch is responsive; `lv_keyboard` verified end-to-end (type into a field, confirm correct captured text).
3. esp-hosted WiFiRemote confirmed live: Tab5 associates to a test AP and obtains an IP address via the C6 co-processor.
4. SD card + C6 SDIO bus-sharing question resolved on real hardware (confirmed separate hosts, or a mitigation designed if not).
5. PSK provisioning flow completed physically between the two real devices (QR displayed on Tab5, key entered on Cardputer-ADV via keyboard or SD file); key persists across reboot on both sides.
6. ESP-NOW control channel and on-demand WiFi bulk socket both verified on real hardware: a command round-trips, a test file transfers over the bulk channel, and the socket tears down cleanly afterward.
7. Cardputer-ADV's existing standalone local-menu operation confirmed unmodified — remote-control support is additive.
8. The end-to-end "ping" feature (Section 5) demonstrated live: tap on Tab5 → command reaches Cardputer-ADV → executes → telemetry returns and renders on the Tab5.
9. Tab5-side `INFC` and `IRF433` HAL drivers bring up and successfully detect their respective HY2.0 units (full feature logic is Phase 3 scope; this phase only proves the HAL can talk to the hardware).

## 9. Explicitly Out of Scope for This Phase

- Any actual offensive/defensive feature (WiFi attacks, BLE attacks, Sub-GHz, NFC operations, IR, LoRa, etc.) — all deferred to Phases 2–7.
- ESP32-C5 satellite firmware (Phase 6).
- Licensing/attribution cleanup for redistribution (explicitly deferred per project owner).
- Interactive per-session pairing, key rotation, or multi-satellite fleet management beyond the single Cardputer-ADV pairing.
- Camera (MIPI-CSI) usage on the Tab5.
