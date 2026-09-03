# Quarky

Quarky is a unified offensive/defensive security suite for two devices:

- **M5Stack Tab5** (`firmware/tab5/`) — an ESP32-P4 touch-driven command
  center. Runs the LVGL UI, and hosts most of the actual feature radios and
  hardware (WiFi/BLE via an onboard ESP32-C6 co-processor, plus NFC/RFID2/
  RF433/IR/CC1101 via external add-on units).
- **M5Stack Cardputer-ADV** (`firmware/cardputer-adv/`) — an ESP32-S3
  satellite the Tab5 controls over a WiFi/BLE C2 link, for features that
  need a second device on the target's own footprint.

This document is a high-level orientation. For what each screen actually
does — buttons, status text, real limits, and real gotchas found on
hardware — see the per-category guides in [`docs/features/`](docs/features/).

**Use every feature in this project only against systems, accounts, and
devices you own or are explicitly authorized to test.**

## Getting around the UI

The Tab5 launcher shows one tile per feature category that currently has at
least one working screen, plus three always-present tiles: **Pair
Satellite**, **Settings**, and (if enabled at build time) a developer-only
keyboard test tile. Tapping a category tile opens a grid of that category's
screens.

| Category | Guide |
|---|---|
| Utility | [`docs/features/utility.md`](docs/features/utility.md) |
| WiFi | [`docs/features/wifi.md`](docs/features/wifi.md) |
| BLE | [`docs/features/ble.md`](docs/features/ble.md) |
| NFC / RFID2 | [`docs/features/nfc.md`](docs/features/nfc.md) |
| RF433 | [`docs/features/rf433.md`](docs/features/rf433.md) |
| IR | [`docs/features/ir.md`](docs/features/ir.md) |
| Sub-GHz (CC1101) | [`docs/features/subghz.md`](docs/features/subghz.md) |

Non-category screens:

| Screen | Guide |
|---|---|
| Pair Satellite | [`docs/features/pairing.md`](docs/features/pairing.md) |
| Settings (Theme) | [`docs/features/settings.md`](docs/features/settings.md) |

A few individual screens are documented in their own standalone guide
instead of their category's, because they need more depth than a shared
category doc can give cleanly:

- [`docs/features/evil-portal.md`](docs/features/evil-portal.md) — WiFi Evil Portal
- [`docs/features/ble-bad-kb.md`](docs/features/ble-bad-kb.md) — BLE Bad-KB
- [`docs/features/fast-pair-attacks.md`](docs/features/fast-pair-attacks.md) — Fast Pair Exploit & WhisperPair

Each category guide links out to these rather than duplicating them.

## Real hardware this project targets

- **Tab5 onboard:** ESP32-P4 (main SoC, display/touch/UI), ESP32-C6
  co-processor via esp-hosted (WiFi + BLE — no external unit needed for the
  WiFi/BLE categories).
- **Tab5 HY2.0 PORT.A** (one physical connector, three mutually-exclusive
  modes arbitrated at runtime — only one is actually usable per boot,
  matching whichever unit is plugged in): NFC unit (ST25R3916), RFID2 unit
  (WS1850S), RF433 receiver/transmitter, or the IR unit (M5Stack Unit IR).
  See [`docs/features/nfc.md`](docs/features/nfc.md) and
  [`docs/features/rf433.md`](docs/features/rf433.md) for how the arbiter
  behaves when two categories want the port at once.
- **Tab5 rear M-Bus connector:** an M5Stack CC1101 module, independent of
  PORT.A — this is what the Sub-GHz category drives.
- **Cardputer-ADV:** paired over WiFi or BLE C2 from the Tab5's **Pair
  Satellite** screen; currently used by the Utility category's Ping
  feature, with more satellite-affinity features planned for a future
  phase.

## Project status

This project ships in phases; not everything in the roadmap is built yet.
As of this writing: Foundation, NFC/RF433/IR, and CC1101-via-M-Bus are the
phases with real, on-device feature screens (documented above). WiFi/BLE
2.4GHz features and Chameleon Ultra integration are further along than
their original phase numbering suggests — check `docs/phases/` for the
authoritative, per-phase build record (architecture decisions, real
hardware findings, and known limitations) behind whatever's currently in
the app.

## Building and flashing

```bash
# Tab5
cd firmware/tab5 && pio run -t upload --upload-port <tab5 serial port>

# Cardputer-ADV
cd firmware/cardputer-adv && pio run -t upload --upload-port <cardputer-adv serial port>
```

Both boards use native USB-CDC serial, which re-enumerates on every reset —
re-check the port name after each flash or reboot rather than assuming it's
stable.

**Pairing:** open **Pair Satellite** on the Tab5 to generate/display a PSK
(see [`docs/features/pairing.md`](docs/features/pairing.md)). The
Cardputer-ADV needs the same key hardcoded and reflashed before the two
devices can talk — mismatched keys silently drop every C2 frame rather than
erroring, by design (HMAC authentication).

## Contributing / extending

Every feature is a `FeatureModule` registered against a shared
`FeatureRegistry` (`shared/feature_contract`) — see
[`docs/phases/phase-1-foundation.md`](docs/phases/phase-1-foundation.md)
for the pattern a new module follows. Full design specs, implementation
plans, and phase-by-phase build documentation (architecture decisions, real
hardware findings, known limitations) live under `docs/superpowers/` and
`docs/phases/` respectively — read the relevant phase doc before extending
a category, since several real hardware constraints (the PORT.A arbiter,
DMA-budget limits with WiFi/BLE active, PSRAM-vs-internal-RAM allocation
rules) aren't obvious from the code alone.
