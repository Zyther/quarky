# Phase 3: Tab5-Native NFC/RFID2/RF433/IR Peripherals — Design

**Status:** Draft for review
**Date:** 2026-08-06 (IR added, NFC chip-identity corrected 2026-08-18; RFID2 chip-protocol corrected and 125kHz row resolved 2026-08-19 — see notes below)
**Depends on:** Phase 1 Foundation — `INFC`/`NfcPN532` and `IRF433`/`Rf433Gpio` HAL (detection-only, Task 18), LVGL shell, `lv_keyboard`, `IStorage`/SD.

> **Naming note, not a factual claim (added 2026-08-19):** the HAL class really is called `NfcPN532` and the file really is `hal/nfc_pn532.cpp` — a Phase-1-era name from when both units were believed to be PN532-based. **Neither Tab5 unit is a PN532** (see the two corrections below). The class is a generic presence-probe/bus-owner today, not a PN532 protocol implementation, and its own header comment says so. The name is left alone deliberately (renaming it is a Phase-1 HAL change, out of scope here), but do not read it as evidence of anything about either unit's protocol.
**Scope:** Full feature logic for the Tab5's HY2.0 units: NFC unit, RFID2 unit, RF433R (receiver) + RF433T (transmitter) unit, and (added 2026-08-18) a dedicated I2C IR receiver/transmitter unit.

**IR added (2026-08-18):** originally excluded from this phase (Cardputer-ADV only, per the project owner's original peripheral-allocation direction). The owner redirected IR here instead, having ordered a dedicated I2C IR receiver/transmitter HY2.0 unit for the Tab5 — see Phase 5's spec for the now-removed Cardputer-ADV-side IR section this superseded. Section 1 below adds an IR feature table; Section 2 adds it to the module structure; this phase's implementation plan should treat IR as a fourth peripheral alongside NFC/RFID2/RF433, not a bolt-on.

**NFC unit chip identity corrected (2026-08-18):** this spec originally assumed both the NFC unit and RFID2 unit are PN532-based, sharing the same `NfcPN532`-driven feature code (see the now-corrected Section 2.1 below). Real-hardware research recorded in Phase 1's Task 18 (see `firmware/tab5/src/hal/nfc_pn532.cpp`'s header comment) found this is wrong for the NFC unit: it identifies at I2C `0x50` as an **ST25R3916** (M5Stack's current "Unit NFC" / "NFC Universal Unit"), a different chip family from RFID2's **WS1850S** at `0x28` (which this note originally called "a PN532-register-compatible clone chip — that half of the original assumption holds"; **that surviving half has since been refuted too — see the 2026-08-19 correction immediately below**). ST25R3916 has its own register map and command protocol, unrelated to PN532 framing, so the two units need genuinely separate low-level driver code. ~~Only the RFID2 unit can realistically reuse PN532-oriented donor logic (Bruce/UniGeek's PN532 modules) directly.~~ **RETRACTED 2026-08-19 — this clause is wrong and is the same stale premise the parenthetical above retracts. The RFID2 unit reuses the donors' MFRC522 modules (`RFID2.cpp` / `MFRC522Screen.cpp`), not their PN532 modules; the donors' PN532 modules drive a different device at `0x24` and are used by NEITHER Tab5 unit. See the 2026-08-19 correction below.** This phase's implementation plan must budget for a real ST25R3916 driver (ST's own datasheet/reference code, not a PN532 port) for the NFC-unit half of every feature row in the table below that currently assumes shared code.

**RFID2 unit chip protocol corrected — WS1850S is MFRC522/PN512-protocol, NOT PN532 (2026-08-19, Phase 3 Task 3):** the 2026-08-18 note above corrected the NFC unit's identity but left "RFID2's WS1850S is PN532-register-compatible" standing, and Task 3's brief was written on that basis (implement a PN532 host-controller frame layer, verify with `GetFirmwareVersion`/`0x02`). Reading the real donor source refuted it unanimously:

- **Bruce** — `src/modules/rfid/RFID2.cpp`, the module named after this exact unit — defines `RFID2_I2C_ADDRESS 0x28` (`:17`) and drives it with `MFRC522DriverI2C` / `PCD_Init()` / `PCD_GetVersion()` (`:22`, `:32`, `:34`). Bruce's real PN532 code is a *separate* module talking to a *different* device at `0x24`.
- **UniGeek** — `screens/module/MFRC522Screen.cpp` (its `title()` literally returns "M5 RFID 2") sets `I2C_ADDRESS = 0x28` (`:13`) and uses the `MFRC522_I2C` library (`platformio.ini:64`). Its PN532 screens likewise target `0x24` and display "Transport: I2C (0x24)".
- **M5Stack's own current library**, `github.com/m5stack/M5Unit-RFID`, declares `class UnitWS1850S : public UnitMFRC522` and documents the part as "**PN512-compatible silicon**" (`src/unit/pn512_register.hpp`). **PN512, not PN532** — PN512 is an MFRC522-family register-mapped reader IC; PN532 is a firmware-driven host-controller part with an entirely different interface. That one-digit difference is the most likely origin of this spec's original assumption.

Consequences for this phase: the RFID2 half of every feature row below ports from **MFRC522-family** donor code (Bruce's `RFID2.cpp` + `RFID_MFRC522v2`, UniGeek's `MFRC522Screen.cpp` + `MFRC522_I2C`), *not* from the donors' PN532 modules — and that is the better outcome, since it is a closer port than the PN532 path would have been. `features/nfc/ws1850s_driver.{h,cpp}` (delivered by Task 3) implements the MFRC522 register interface against NXP's MFRC522 datasheet rev 3.9 with both donors cross-cited line by line; its header comment is the durable write-up. The bring-up readback is `VersionReg` (`0x37`), which is what both donors use for this unit.

**Confirmed on real hardware 2026-08-19** (RFID2 unit on PORT.A, `'2'` serial trigger): `VersionReg (0x37) = 0x15`, stable across a control re-read, and a real PN532 `GetFirmwareVersion` frame sent to the same address got **no response** — so the protocol-family correction above is verified at register level on this hardware, not only in donor source. Note `0x15` is **not** a value any donor library names (their known list is `0x12 / 0x88 / 0x89 / 0xb2 / 0x90 / 0x91 / 0x92`), which is exactly why the driver asserts no expected value and tests only the donors' own "`0x00`/`0xFF` means the bus answered, not the chip" condition. **`0x15` is now the citable observed value for this unit** — do not "tighten" any check to it without noting it came from this one observation rather than from a datasheet.

## 1. Feature Inventory and Source Mapping

### NFC / RFID (NFC unit = ST25R3916 @ 0x50, RFID2 unit = WS1850S @ 0x28 — see the chip-identity corrections above; **NEITHER unit is a PN532**)

*(That heading read "NOT both PN532" until 2026-08-19. Accurate when written, but it invites the reading "not both — so one of them is", which is exactly the wrong half-truth this spec has now had to retract twice. Neither unit is a PN532: the NFC unit is ST25R3916, the RFID2 unit is MFRC522/PN512-family.)*
| Feature | Donor reference | Notes |
|---|---|---|
| Tag read (UID, type detection) | Bruce `src/modules/rfid/RFID2.cpp` + `RFID_MFRC522v2`, UniGeek `screens/module/MFRC522Screen.cpp` + `MFRC522_I2C` (RFID2/WS1850S path — **MFRC522-family, not the donors' `PN532*Screen` code, see the 2026-08-19 correction**); ST's own ST25R3916 discovery-loop reference code (NFC-unit path — donor projects don't cover this chip) | **Two separate low-level implementations, one shared UI/result-card layer.** RFID2 can port the donors' MFRC522 code fairly directly (WS1850S is MFRC522/PN512-register-compatible). The NFC unit needs a real ST25R3916 driver written against ST's own datasheet/app notes — there is no donor reference for this chip in Bruce/Poseidon/UniGeek, so budget real implementation time, not a port. |
| MIFARE Classic key recovery (dictionary, nested, darkside attacks) | UniGeek `utils/nfc/` | RFID2-unit feature (MIFARE Classic is a WS1850S/**MFRC522**-class operation — *corrected 2026-08-19, this cell said "PN532-class"*). **This is the row the protocol correction helps most:** UniGeek's `utils/nfc/` attacks are written directly against `MFRC522_I2C`, the same library its `MFRC522Screen.cpp` drives this exact unit with, so they port onto `ws1850s_driver` rather than onto a PN532 abstraction that would have had to be invented first. Most technically involved feature in this phase — nested/darkside attacks are timing-sensitive against the chip's own firmware, port UniGeek's implementation closely rather than re-deriving. Whether the ST25R3916 NFC unit can do the same MIFARE-layer attacks depends on its own command set — confirm during implementation rather than assuming parity with RFID2. |
| Mifare/Chameleon Ultra-style emulation | Bruce `ESP-ChameleonUltra` (BLE-bridge to external hardware) | **Re-scoped**: Bruce's version bridges to a separate Chameleon Ultra device over BLE, which isn't in this project's hardware inventory. Tab5-native emulation is limited to what the RFID2 unit's chip can itself emulate as a card (MIFARE Classic UID/blocks) — full arbitrary tag emulation is out of scope for this phase, revisit only if a Chameleon Ultra is added to the kit later (see Phase 4). |
| Amiibo read/write | Bruce `ESP-Amiibolink` | Straightforward NTAG215 read/write, no crypto-attack complexity. NTAG215 is an ISO 14443-A/NFC-Forum-Type-2 tag — confirm which of the two units (or both) can address it during implementation; ST25R3916 is a general NFC reader IC so should support it, don't assume without checking. |
| SRIX tag tool | Bruce | ISO 14443-B, less common but low-effort port |
| "Tag-o-matic" generic tag manager (save/load/browse tag library) | Bruce | UI/storage feature more than radio feature — a tag library browser backed by SD, shared across both units' read results |
| EMV/APDU reader | Bruce `emv_reader.hpp`, `apdu.cpp`, `BER-TLV` | Read-only card-data extraction (PAN, expiry, etc. where unencrypted) — no payment/transaction logic |
| ~~125kHz / T5577 clone, HID Prox~~ **DROPPED 2026-08-19 — confirmed not physically possible with this hardware** | ~~UniGeek `chameleon/`; Bruce `rfid125.cpp`~~ | **Resolved, closing this spec's open question (Task 3, Step 3).** Neither Tab5 unit can do 125kHz LF, and the confirmation is from primary sources, not inference: **(a)** M5Stack's own Unit RFID2 documentation (`docs.m5stack.com/en/unit/rfid2`, retrieved 2026-08-19) specifies "Operating Freq: **13.56MHz**", "Supported Prot: ISO/IEC 14443 Type A/Type B"; **(b)** WS1850S is MFRC522/PN512-family silicon (see the 2026-08-19 correction above), and NXP's MFRC522 datasheet rev 3.9 Sec 2 opens "The MFRC522 is a highly integrated reader/writer IC for contactless communication **at 13.56 MHz**" — there is no LF signal path in the part at all; **(c)** the NFC unit's ST25R3916 is likewise a 13.56MHz-only NFC reader IC (DS12484 Rev 3, already cited in `st25r3916_driver.cpp`). 125kHz is not a firmware capability — it needs a different carrier oscillator, analog front end and physically different antenna, none of which either unit has. **(d)** Corroborating the "dedicated LF hardware" point: Bruce's `rfid125.cpp` is not a 13.56MHz-chip driver at all — it opens a `HardwareSerial` at 9600 baud (`rfid125.cpp:49`) against a separate UART LF reader module. Revisit only if such a dedicated 125kHz unit is added to the kit; nothing in this phase implements this row. |

### IR (dedicated I2C IR receiver/transmitter HY2.0 unit — added 2026-08-18, replaces the original Cardputer-ADV-side IR plan; exact chip TBD until the unit arrives, see Section 3 risk)
| Feature | Donor reference | Notes |
|---|---|---|
| TV-B-Gone | Bruce `src/modules/ir/`, Poseidon `ir_tvbgone.cpp` | Static code-database transmit, simplest feature in this section |
| IR receive/decode/learn | Bruce (`IRremoteESP8266` fork), Poseidon `ir_learn.cpp`/`ir_learn_decode.cpp` | Unlike the original Cardputer-ADV plan (transmit-only LED, receive capability unconfirmed), this dedicated unit is receive+transmit by design — this row is no longer conditional on confirming receive hardware exists, only on confirming the unit's actual I2C protocol once it's in hand |
| Universal remote / multi-profile clone (Samsung/LG/Sony) | Poseidon `ir_clone.cpp/.h`, UniGeek Flipper-IRDB-compatible database | UniGeek's Flipper-IRDB compatibility is the most valuable single piece to port here — gives access to a large existing community remote database instead of hand-building one |
| IR jammer | Bruce | Continuous-noise transmit |

Donor projects' IR code (`IRremoteESP8266`-based) universally assumes a GPIO-bit-banged transmit LED and a GPIO receive diode read via a hardware timer/RMT peripheral, not an I2C-attached IR transceiver IC — none of the three donors' IR modules were written against an I2C IR unit. Expect this section's implementation to need a real driver written against the actual unit's datasheet (once in hand), with donor code reused only for the higher-level pieces that don't depend on the transport (protocol timing databases, Flipper-IRDB parsing, code-database formats) — the same "separate low-level driver, shared upper layers" shape as the NFC-unit situation above.

### RF433 (RF433R + RF433T units — simple GPIO, not CC1101/SPI)
| Feature | Donor reference | Notes |
|---|---|---|
| Fixed-code scan/capture | Bruce `rf_scan.cpp` (via `M5_RF_MODULE`/`rc-switch` path, not the CC1101 path) | This is Bruce's *simple* RF433 mode, not its CC1101 spectrum/waterfall mode — matches the RF433R/T unit's actual capability (basic OOK/ASK receive+transmit, no SPI radio chip) |
| Replay | Bruce `rf_send.cpp`/`emit.cpp` (`rc-switch` path) | |
| Fixed-code bruteforce (Came/Nice/Linear/Chamberlain/Holtek/Ansonic) | Bruce `rf_bruteforce.cpp`, UniGeek `utils/rf/` protocol decoders | Both donors' brute-force logic targets exactly this class of fixed/rolling low-complexity code, well-matched to a GPIO-only 433MHz unit |
| Protocol decode/identify | UniGeek `utils/rf/M5RF433Util.*` — **direct match**, UniGeek has an M5-RF433-specific utility already, likely the closest possible reference for this exact hardware unit | Prioritize this over Bruce's generic rc-switch path if the pin/API shape matches the actual M5 RF433R/T unit |

### Explicitly out of scope for Phase 3 (belongs elsewhere)
- Full CC1101-class spectrum analysis, waterfall, wideband 300–900MHz scanning, jamming — that's the hydra hat's CC1101, which lives on Cardputer-ADV (Phase 5). The RF433R/T unit is receive/transmit only at a fixed ~433MHz band with no SPI radio chip, so it cannot do any of this regardless of software.
- KeeLoq rolling-code replay-plus-one (UniGeek's CC1101-based feature) — same reasoning, needs the CC1101's capability, not RF433R/T's.

## 2. Architecture

### 2.1 Module Structure

```
firmware/tab5/src/features/
├── nfc/
│   ├── nfc_read.{h,cpp}               # dispatches to whichever chip driver the target unit uses
│   ├── nfc_mifare_crack.{h,cpp}       # dictionary/nested/darkside -- RFID2 (WS1850S) only, see Section 1
│   ├── nfc_amiibo.{h,cpp}
│   ├── nfc_srix.{h,cpp}
│   ├── nfc_tag_library.{h,cpp}        # SD-backed save/load/browse, shared across both units' results
│   ├── nfc_emv_read.{h,cpp}
│   ├── nfc_common.{h,cpp}             # tag-type detection, UID formatting -- chip-agnostic
│   ├── ws1850s_driver.{h,cpp}         # RFID2 unit's real low-level chip driver (MFRC522/PN512-register-compatible -- NOT PN532, see the 2026-08-19 correction)
│   └── st25r3916_driver.{h,cpp}       # NFC unit's real low-level chip driver (own register map, no donor precedent)
├── rf433/
│   ├── rf433_scan.{h,cpp}
│   ├── rf433_replay.{h,cpp}
│   ├── rf433_bruteforce.{h,cpp}
│   ├── rf433_protocol_decode.{h,cpp}  # ported from UniGeek's M5RF433Util as primary reference
│   └── rf433_common.{h,cpp}
└── ir/
    ├── ir_tvbgone.{h,cpp}
    ├── ir_learn.{h,cpp}
    ├── ir_clone.{h,cpp}               # Flipper-IRDB-compatible, ported from UniGeek
    ├── ir_jammer.{h,cpp}
    └── ir_common.{h,cpp}              # the unit's real low-level I2C driver, once its chip is known
```

The two NFC-family units (NFC, RFID2) share their feature-logic and UI layer (`nfc_read.cpp` etc.) but NOT their low-level chip driver — `nfc_read.cpp` and friends dispatch to `ws1850s_driver.cpp` or `st25r3916_driver.cpp` depending on which physical unit (`INFC` instance) the user targeted, rather than one shared driver parameterized by I2C address the way the original (incorrect, PN532-for-both) design assumed. The Phase 1 launcher UI should let the user pick which physical unit (NFC vs RFID2) to target before entering a feature screen, since both are simultaneously connected and a tag could be presented to either.

### 2.2 UI Pattern

- Tag/signal read features show a result card (UID, type, key data if cracked) with a "Save to library" action writing to SD via `nfc_tag_library`.
- MIFARE key recovery shows live progress (keys tried / keyspace, elapsed time) since dictionary/nested attacks can take real wall-clock time — reuse the Phase 2 streaming-feature UI pattern (progress + Stop button).
- RF433 scan/replay follows the same list-and-select pattern as Phase 2's WiFi AP scan: captured signals populate an `lv_list`, tap to replay or save.

### 2.3 Data Format

Tag library, RF433 captures, and (added 2026-08-18) IR code captures are saved to `/quarky/captures/nfc/`, `/quarky/captures/rf433/`, and `/quarky/captures/ir/` on SD, using the same top-level `/quarky/captures/` convention established in Phase 2 — one consistent capture-storage layout across all Tab5-native feature phases.

## 3. Risks / Open Questions

- **NFC unit needs a real ST25R3916 driver with no donor-project precedent** (added 2026-08-18, see the chip-identity correction above) — this is now the single biggest unknown in this phase, bigger than the pre-existing risks below. First implementation task for the NFC-unit half of this phase should be a spike: bring up ST25R3916 register-level communication (chip ID read, basic field-detect) against ST's own datasheet/reference code, before committing to any specific feature's driver logic.
- **IR unit's exact chip/protocol is unknown until the unit physically arrives** (added 2026-08-18) — this phase's IR tasks cannot start real implementation until then. Once in hand, first step is the same shape as the NFC-unit spike above: identify the chip (I2C address census, datasheet/markings lookup), confirm receive+transmit both work at a bare-protocol level, before building feature logic on top.
- ~~**RFID2 unit's actual frequency support (13.56MHz vs 125kHz) is unconfirmed.**~~ **CLOSED 2026-08-19 (Task 3, Step 3): 13.56MHz only, confirmed from M5Stack's own unit documentation and NXP's MFRC522 datasheet; 125kHz is not physically possible on either Tab5 unit and that feature row is dropped.** See the struck-through row in Section 1 for the full citation trail.
- **MIFARE nested/darkside attack timing** depends on the WS1850S's own firmware response latency, which can differ between M5Stack's unit and whatever MFRC522-class module the Bruce/UniGeek donor projects were tested against — expect some tuning of retry/timeout constants during bring-up rather than a direct drop-in port.
- **RF433R/T unit's exact GPIO pin assignment is partially resolved**, not fully open like the original text of this risk implied: real-hardware testing during Phase 1 confirmed `TAB5_RF433T_PIN = GPIO53` (independent-listener-verified, high confidence) and set `TAB5_RF433R_PIN = GPIO53` as a same-connector hypothesis, NOT independently confirmed — an earlier `loop()`-polling receive test on GPIO53/54 found no correlated signal, but is now understood as a likely false negative from sampling far too slow for real OOK receive-pulse timing (hundreds of microseconds per bit), not evidence the pin is wrong. This phase's first RF433 task should be a proper interrupt- or timer-driven (not `loop()`-polling) receive test against GPIO53 to actually confirm or refute the hypothesis before building scan/bruteforce features on top of it. See `.superpowers/sdd/2026-08-06-tab5-foundation-plan/progress.md`'s 2026-08-09 entries for the full real-hardware trail.

## 4. Testing Strategy

- Host-native tests for pure-logic pieces: protocol decode/identify (given a captured raw timing sequence, decode to the correct brand/protocol — table-driven test against known sample captures), tag-library file format read/write round-trip, MIFARE dictionary-attack keyspace iteration logic (without needing real hardware to test the iteration order itself), IR code-database/Flipper-IRDB parsing.
- On-device verification against real tags/fobs, a real 433MHz remote (garage door, doorbell, etc., used only against your own equipment), and (once the unit is in hand) a real IR remote/device for read/scan/replay features.
- MIFARE crack features verified against a test card with a known, deliberately weak key (common practice: a blank/test MIFARE Classic card programmed with a default key) rather than an unknown target, so success/failure is unambiguous during development.

## 5. Definition of Done

1. NFC unit and RFID2 unit both independently confirmed working for baseline tag read (UID + type), each via its own real chip driver (ST25R3916 / WS1850S respectively).
2. ~~RFID2 unit's frequency capability confirmed and the 125kHz feature row resolved (implemented or explicitly dropped with a note here).~~ **MET 2026-08-19 (Task 3, Step 3): 13.56MHz only, confirmed from primary sources; the 125kHz/T5577/HID-Prox row is explicitly DROPPED** (Section 1 row and Section 3 risk both updated with the citation trail). Note this item needed no hardware to close — it is a datasheet question, and it is now closed independently of item 1's on-hardware readback.
3. MIFARE key recovery succeeds against a known-weak test card.
4. RF433R's receive pin hypothesis (GPIO53) confirmed or refuted via a real interrupt/timer-driven test; RF433 scan, replay, and bruteforce all verified against real 433MHz hardware you own.
5. Tag/signal library save-and-reload round-trips correctly from the Tab5 UI.
6. IR unit's chip/protocol identified once in hand; TV-B-Gone, receive/decode, universal-remote clone, and jammer all verified against real IR remotes/devices you own.
