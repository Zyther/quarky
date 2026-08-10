# Phase 8: Remaining Feature Sweep — Design

**Status:** Draft for review
**Date:** 2026-08-06
**Depends on:** Phases 1–7 all landed — this phase explicitly allocates each remaining donor feature to whichever device/pattern fits once the full architecture is proven out, per the program roadmap.
**Scope:** Everything from Bruce/Poseidon/UniGeek not already covered by Phases 2–7: BadUSB/HID injection, FIDO2/KERBEROS, scripting engine, network-layer attacks, defensive/anomaly detection, GPS/wardriving polish, and smaller utility features.

## 1. Feature Inventory, Device Allocation, and Source Mapping

### BadUSB / HID Injection
| Feature | Donor reference | Allocation | Notes |
|---|---|---|---|
| USB HID DuckyScript injection | Bruce (`lib/Bad_Usb_Lib`, USBHID/USBHIDKeyboard fork + CH9329 support), Poseidon `badusb.cpp`, UniGeek (DuckyScript 3.0 interpreter) | **Cardputer-ADV** | Cardputer-ADV has native ESP32-S3 USB OTG (confirmed in Phase 1 research); Tab5's ESP32-P4 also has USB, but Cardputer-ADV is the natural "plug into a target machine" form factor, matching all three donors' actual field usage pattern |
| BLE HID injection ("Bad-BLE") | Bruce `ducky_typer.cpp` | **Already Phase 2** (Tab5-native, contingent on the Phase 2 BLE-HID risk spike) — cross-reference, not new work here |
| "Live Keyboard" USB passthrough | Poseidon | **Cardputer-ADV** | Alongside DuckyScript injection, same USB hardware path |
| USB mass storage emulation | UniGeek | **Cardputer-ADV** | Same native USB OTG dependency |

### FIDO2/U2F (KERBEROS)
| Feature | Donor reference | Allocation | Notes |
|---|---|---|---|
| CTAP1/U2F + CTAP2 stack, resident/non-resident passkeys, on-device approval | Poseidon `lib/kerberos_core` (hand-written, host-unit-tested, verified against `python-fido2`) — the single most mature/tested piece of code across all three donors | **Cardputer-ADV** | Poseidon's own README repeatedly marks this "not yet stable" despite functional verification — inherit that caution. `lib/kerberos_core` is portable (has its own native Unity tests per Poseidon's `test/test_kerberos_*` — confirmed in Phase 1 research), so it can likely be vendored close to as-is into `firmware/cardputer-adv/lib/kerberos_core` with its existing test suite carried over, rather than rewritten. USB-dependent (needs native USB-C HID+CDC), so Cardputer-ADV, not Tab5. |
| WebAuthn/FIDO2 passkey (alternate implementation) | UniGeek (`pico-fido`-referenced) | **Superseded by Poseidon's KERBEROS** | UniGeek's version references `pico-fido` (AGPL) as an inspiration rather than shipping a complete independent stack; Poseidon's `kerberos_core` is both more complete and already unit-tested — use it as the single implementation, don't duplicate effort porting both |

### Scripting Engine
| Feature | Donor reference | Allocation | Notes |
|---|---|---|---|
| JS interpreter (mquickjs) with GPIO/display bindings | Bruce `src/modules/bjs_interpreter/` | **Tab5** | Scripting is a UI/orchestration-layer concern, natural fit for the command-center device; bindings would target Tab5's own `FeatureRegistry`/HAL rather than raw GPIO, giving scripts access to trigger any registered feature module (Tab5-native or remote) — a more powerful integration than Bruce's original GPIO-only binding surface |
| Lua interpreter | UniGeek `utils/interpreter/` | **Superseded by Bruce's JS engine** | Two scripting languages isn't warranted; Bruce's mquickjs is the more actively maintained/complete of the two references — pick one, and JS's broader familiarity plus Bruce's existing GPIO/display binding precedent makes it the better base to extend |

### Network-Layer Attacks (post-association, not radio-specific)
| Feature | Donor reference | Allocation | Notes |
|---|---|---|---|
| Responder (LLMNR/NBT-NS → NTLM capture) | Bruce `responder.cpp`, Poseidon `net_responder.cpp`, UniGeek | **Tab5** | Runs over an established WiFi connection (via Phase 2's `IRadio`), not radio-specific — natural Tab5 placement alongside Phase 2's WiFi suite |
| Port scan, ping, DNS, SSDP/UPnP scan, LAN recon | Bruce `scan_hosts.h`/`tcp_utils.cpp`, Poseidon `net_lanrecon.cpp`/`net_ssdp.cpp`, UniGeek | **Tab5** | Same reasoning |
| DHCP starvation, rogue DHCP, ARP spoof/poison/MAC-flood | Bruce (`src/modules/ethernet/`: `ARPoisoner`, `ARPSpoofer`, `ARPScanner`, `MACFlooding`, `DHCPStarvation`), Poseidon `net_dhcp.cpp` | **Tab5** | |
| WPAD/Autodiscover abuse, network hijacking (chained MITM) | Poseidon (`SaltyJack` suite, credited port from RaspyJack/Evil-M5Project) | **Tab5** | |
| SOCKS4 proxy, reverse TCP tunnel, telnet honeypot, WireGuard tunneling, SSH (LibSSH-ESP32) | Bruce, Poseidon | **Tab5** | These are heavier on flash/RAM (Bruce's own `LITE_VERSION`/`env_light` build strips exactly these for size-constrained boards) — Tab5's 16MB flash/32MB PSRAM comfortably fits all of them, unlike the donors' original size-constrained hardware, so no build-variant stripping is needed here |
| Printer detection/raw print/prank, CCTV sniffer, cast bomb, Bonjour/mDNS spam | Poseidon `net_cctv.cpp`, UniGeek | **Tab5** | Lower-priority utility/prank features, implement last within this group |

### Defensive / Anomaly Detection
| Feature | Donor reference | Allocation | Notes |
|---|---|---|---|
| Defensive monitor (7-class WiFi+BLE anomaly detector: deauth-flood, broadcast-deauth, evil-twin, beacon-spam, WiFi-karma, BLE-spoof, BLE-flood) | Poseidon `defensive_monitor.cpp` | **Tab5** | Passive-only, built on the same scan/sniff primitives as Phase 2's offensive features — natural to build once Phase 2's WiFi/BLE scan infrastructure exists, since it's largely the same data pipeline pointed at anomaly classification instead of attack execution |
| Surveillance Hunter (Flock/Raven ALPR + body-cam detector via Plume OUI tables) | Poseidon `defensive_monitor.cpp`/OUI matching | **Tab5** | OUI-table-driven, reuses Phase 2's vendor-DB infrastructure |
| SATCOM Tracker (baked TLE database, az/el tracking, no-internet) | Poseidon `feat_satcom.cpp`/`satcom.cpp` | **Tab5** | Pure computation + baked data, no radio dependency at all — could ship early/cheaply relative to its neighbors in this table since it needs nothing from any other phase except UI |

### Remaining Utilities and Games
| Feature | Donor reference | Allocation | Notes |
|---|---|---|---|
| QR/barcode generation, TOTP | Bruce, UniGeek | **Tab5** | Phase 1 already has a QR *encoder* (pairing screen) — reuse directly |
| Flashlight, stopwatch, dice/coin/calculator, MAC randomizer, world clock | UniGeek, Poseidon | **Tab5** | Trivial, low-priority, implement opportunistically |
| Password manager | UniGeek | **Tab5** | Needs secure storage design (NVS encryption or similar) — don't treat as trivial despite simple UI |
| On-device games (Wordle, Flappy Bird, etc.) | UniGeek | **Explicitly deprioritized** | Not a security feature; only worth doing if genuinely desired as a bonus, not counted toward "full feature parity" |
| FM radio transmit (Si4713) | Bruce | **Not applicable** | No Si4713 hardware in this program's device inventory (Bruce's own support is board-gated to specific boards with that chip populated) — explicitly out of scope unless that hardware is added later |

## 2. Architecture Notes

- This phase is intentionally the least architecturally novel — its job is applying the `FeatureModule`/`FeatureRegistry` pattern (Phase 1) to a long tail of remaining features, mostly on the Tab5 (network/scripting/defensive/utility) with a smaller Cardputer-ADV group (BadUSB, KERBEROS — both tied to physical USB hardware).
- KERBEROS's `lib/kerberos_core` vendoring (Section 1) is the one piece of this phase worth calling out architecturally: it should land as a near-verbatim vendored library with its existing native test suite intact, not a rewrite — matching this program's general principle (established across Phases 2–6) of porting proven donor logic rather than re-deriving it, applied here to the most rigorously-tested single component across all three donor codebases.
- The scripting engine's GPIO/display bindings (Bruce's original) should be redesigned as `FeatureRegistry` bindings (Section 1) — this is a deliberate improvement over the direct port pattern used elsewhere in this phase, since Bruce's original binding surface predates this program's feature-module contract and a straight port would bypass it.

## 3. Risks / Open Questions

- **This phase's scope is the least certain of the whole program** — by design, it's "whatever's left," and its actual task list should be finalized only after Phases 2–7 reveal which architectural patterns really stuck and which needed adjustment. Treat this document as a starting allocation, not a locked plan.
- **KERBEROS's "not yet stable" status** (Poseidon's own assessment) should carry forward as-is; don't represent this feature as more mature than its own donor project claims.
- **Password manager secure storage** needs a real design decision (encrypted NVS partition vs. SD-based encrypted vault) before implementation — not addressed in this draft, flagged for the eventual Phase 8 implementation plan.

## 4. Testing Strategy

- Host-native tests for all pure-logic pieces: DuckyScript parser, TLE orbital math (SATCOM tracker), OUI table lookups, QR/barcode encoders, TOTP generation, KERBEROS's existing test suite (carried over from Poseidon largely unmodified).
- On-device verification for USB HID injection against a real target machine you own, network attacks against your own test network/lab equipment (consistent with every other phase's authorized-testing framing), KERBEROS verified against `python-fido2` the same way Poseidon's own donor testing did.

## 5. Definition of Done

1. Every feature in Section 1 is either implemented and verified, or explicitly marked deprioritized/out-of-scope with a reason (as several already are in this draft).
2. `lib/kerberos_core` vendored with its native test suite passing unmodified on this program's build.
3. Scripting engine's `FeatureRegistry` bindings verified by writing at least one script that triggers a Phase 2+ feature module (e.g. a script-triggered WiFi scan) — proves the binding redesign (Section 2) actually works, not just compiles.
4. Password manager secure-storage design decision made and documented before implementation begins.
