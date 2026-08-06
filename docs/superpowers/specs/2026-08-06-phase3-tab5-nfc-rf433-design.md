# Phase 3: Tab5-Native NFC/RFID2/RF433 Peripherals — Design

**Status:** Draft for review
**Date:** 2026-08-06
**Depends on:** Phase 1 Foundation — `INFC`/`NfcPN532` and `IRF433`/`Rf433Gpio` HAL (detection-only, Task 18), LVGL shell, `lv_keyboard`, `IStorage`/SD.
**Scope:** Full feature logic for the Tab5's three HY2.0 units confirmed in hand: NFC unit, RFID2 unit, RF433R (receiver) + RF433T (transmitter) unit. IR is explicitly excluded (Cardputer-ADV only, per owner direction) and Phase 1's peripheral-allocation decision.

## 1. Feature Inventory and Source Mapping

### NFC / RFID (NFC unit + RFID2 unit, both PN532-based per Phase 1's HAL)
| Feature | Donor reference | Notes |
|---|---|---|
| Tag read (UID, type detection) | Bruce `src/modules/rfid/`, UniGeek `screens/module/PN532*Screen` | Baseline — both units use this identically, differentiated by which `NfcPN532` instance (I2C address) is called |
| MIFARE Classic key recovery (dictionary, nested, darkside attacks) | UniGeek `utils/nfc/` | Most technically involved feature in this phase — nested/darkside attacks are timing-sensitive against the PN532's own firmware, port UniGeek's implementation closely rather than re-deriving |
| Mifare/Chameleon Ultra-style emulation | Bruce `ESP-ChameleonUltra` (BLE-bridge to external hardware) | **Re-scoped**: Bruce's version bridges to a separate Chameleon Ultra device over BLE, which isn't in this project's hardware inventory. Tab5-native emulation is limited to what the PN532 itself can emulate as a card (MIFARE Classic UID/blocks) — full arbitrary tag emulation is out of scope for this phase, revisit only if a Chameleon Ultra is added to the kit later. |
| Amiibo read/write | Bruce `ESP-Amiibolink` | Straightforward NTAG215 read/write, no crypto-attack complexity |
| SRIX tag tool | Bruce | ISO 14443-B, less common but low-effort port |
| "Tag-o-matic" generic tag manager (save/load/browse tag library) | Bruce | UI/storage feature more than radio feature — a tag library browser backed by SD |
| EMV/APDU reader | Bruce `emv_reader.hpp`, `apdu.cpp`, `BER-TLV` | Read-only card-data extraction (PAN, expiry, etc. where unencrypted) — no payment/transaction logic |
| 125kHz / T5577 clean, HID Prox | UniGeek `chameleon/` (via BLE-bridge, same caveat as emulation above); Bruce `rfid125.cpp` | Bruce's `rfid125.cpp` is the relevant reference since it doesn't require external hardware — confirm the RFID2 unit's PN532 variant actually supports 125kHz (some PN532 modules are 13.56MHz-only; if the RFID2 unit is 13.56MHz-only, 125kHz work is out of scope for this phase — verify against the unit's actual datasheet during implementation, don't assume) |

### RF433 (RF433R + RF433T units — simple GPIO, not CC1101/SPI)
| Feature | Donor reference | Notes |
|---|---|---|
| Fixed-code scan/capture | Bruce `rf_scan.cpp` (via `M5_RF_MODULE`/`rc-switch` path, not the CC1101 path) | This is Bruce's *simple* RF433 mode, not its CC1101 spectrum/waterfall mode — matches the RF433R/T unit's actual capability (basic OOK/ASK receive+transmit, no SPI radio chip) |
| Replay | Bruce `rf_send.cpp`/`emit.cpp` (`rc-switch` path) | |
| Fixed-code bruteforce (Came/Nice/Linear/Chamberlain/Holtek/Ansonic) | Bruce `rf_bruteforce.cpp`, UniGeek `utils/rf/` protocol decoders | Both donors' brute-force logic targets exactly this class of fixed/rolling low-complexity code, well-matched to a GPIO-only 433MHz unit |
| Protocol decode/identify | UniGeek `utils/rf/M5RF433Util.*` — **direct match**, UniGeek has an M5-RF433-specific utility already, likely the closest possible reference for this exact hardware unit | Prioritize this over Bruce's generic rc-switch path if the pin/API shape matches the actual M5 RF433R/T unit |

### Explicitly out of scope for Phase 3 (belongs elsewhere)
- Full CC1101-class spectrum analysis, waterfall, wideband 300–900MHz scanning, jamming — that's the hydra hat's CC1101, which lives on Cardputer-ADV (Phase 4). The RF433R/T unit is receive/transmit only at a fixed ~433MHz band with no SPI radio chip, so it cannot do any of this regardless of software.
- KeeLoq rolling-code replay-plus-one (UniGeek's CC1101-based feature) — same reasoning, needs the CC1101's capability, not RF433R/T's.

## 2. Architecture

### 2.1 Module Structure

```
firmware/tab5/src/features/
├── nfc/
│   ├── nfc_read.{h,cpp}              # shared by NFC unit + RFID2 unit, parameterized by NfcPN532 instance
│   ├── nfc_mifare_crack.{h,cpp}      # dictionary/nested/darkside
│   ├── nfc_amiibo.{h,cpp}
│   ├── nfc_srix.{h,cpp}
│   ├── nfc_tag_library.{h,cpp}       # SD-backed save/load/browse
│   ├── nfc_emv_read.{h,cpp}
│   └── nfc_common.{h,cpp}            # tag-type detection, UID formatting
└── rf433/
    ├── rf433_scan.{h,cpp}
    ├── rf433_replay.{h,cpp}
    ├── rf433_bruteforce.{h,cpp}
    ├── rf433_protocol_decode.{h,cpp}  # ported from UniGeek's M5RF433Util as primary reference
    └── rf433_common.{h,cpp}
```

Two units (NFC, RFID2) share the same feature code — every NFC feature module takes an `INFC&` parameter (or is instantiated twice, once per unit) rather than being duplicated. The Phase 1 launcher UI should let the user pick which physical unit (NFC vs RFID2) to target before entering a feature screen, since both are simultaneously connected and a tag could be presented to either.

### 2.2 UI Pattern

- Tag/signal read features show a result card (UID, type, key data if cracked) with a "Save to library" action writing to SD via `nfc_tag_library`.
- MIFARE key recovery shows live progress (keys tried / keyspace, elapsed time) since dictionary/nested attacks can take real wall-clock time — reuse the Phase 2 streaming-feature UI pattern (progress + Stop button).
- RF433 scan/replay follows the same list-and-select pattern as Phase 2's WiFi AP scan: captured signals populate an `lv_list`, tap to replay or save.

### 2.3 Data Format

Tag library and RF433 captures are saved to `/quarky/captures/nfc/` and `/quarky/captures/rf433/` on SD, using the same top-level `/quarky/captures/` convention established in Phase 2 — one consistent capture-storage layout across all Tab5-native feature phases.

## 3. Risks / Open Questions

- **RFID2 unit's actual frequency support (13.56MHz vs 125kHz) is unconfirmed.** The 125kHz/T5577/HID-Prox feature row above is contingent on this — first implementation task should be reading the RFID2 unit's actual datasheet/model number and confirming which frequency band(s) it supports before committing to porting Bruce's `rfid125.cpp` logic.
- **MIFARE nested/darkside attack timing** depends on the PN532's own firmware response latency, which can differ between M5Stack's PN532 unit and whatever PN532 module the Bruce/UniGeek donor projects were tested against — expect some tuning of retry/timeout constants during bring-up rather than a direct drop-in port.
- **RF433R/T unit's exact GPIO pin assignment on Tab5's HY2.0 ports** was left as a placeholder in the Phase 1 plan (Task 18) — must be resolved before any Phase 3 task starts, since every feature in this phase depends on it.

## 4. Testing Strategy

- Host-native tests for pure-logic pieces: protocol decode/identify (given a captured raw timing sequence, decode to the correct brand/protocol — table-driven test against known sample captures), tag-library file format read/write round-trip, MIFARE dictionary-attack keyspace iteration logic (without needing real hardware to test the iteration order itself).
- On-device verification against real tags/fobs and a real 433MHz remote (garage door, doorbell, etc., used only against your own equipment) for read/scan/replay features.
- MIFARE crack features verified against a test card with a known, deliberately weak key (common practice: a blank/test MIFARE Classic card programmed with a default key) rather than an unknown target, so success/failure is unambiguous during development.

## 5. Definition of Done

1. NFC unit and RFID2 unit both independently confirmed working for baseline tag read (UID + type).
2. RFID2 unit's frequency capability confirmed and the 125kHz feature row resolved (implemented or explicitly dropped with a note here).
3. MIFARE key recovery succeeds against a known-weak test card.
4. RF433 scan, replay, and bruteforce all verified against real 433MHz hardware you own.
5. Tag/signal library save-and-reload round-trips correctly from the Tab5 UI.
