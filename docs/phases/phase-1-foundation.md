# Phase 1: Foundation

**Status:** Complete. Both devices boot stably, both C2 transports (WiFi socket + BLE GATT) are verified end-to-end on real hardware, and the first working feature module (ping) proves the whole contract.

This document describes what Phase 1 actually delivered and why it's built the way it is. For the full planning/verification trail (every task, every real-hardware finding, every fix round), see:
- Design spec: [`docs/superpowers/specs/2026-08-06-tab5-foundation-design.md`](../superpowers/specs/2026-08-06-tab5-foundation-design.md)
- Implementation plan (21 tasks): [`docs/superpowers/plans/2026-08-06-tab5-foundation-plan.md`](../superpowers/plans/2026-08-06-tab5-foundation-plan.md)
- Bring-up log (Definition of Done walkthrough): `docs/superpowers/specs/2026-08-06-tab5-foundation-bringup-log.md` (Task 21)

## What this phase built

Two firmwares that can boot independently and talk to each other over either of two radio transports:

- **Tab5** (`firmware/tab5/`) — the ESP32-P4 command center. LVGL touch UI shell (status bar + launcher grid + screen stack), WiFi via the ESP32-C6 co-processor (esp-hosted), BLE via raw ESP-IDF NimBLE (NimBLE-Arduino doesn't build for P4 — see below), PSK-based pairing flow (hex + QR display, NVS-persisted), SD card, and HAL stubs for NFC/RFID2/RF433 peripherals on the external HY2.0 port.
- **Cardputer-ADV** (`firmware/cardputer-adv/`) — the ESP32-S3 satellite. Device HAL skeleton (keyboard I2C probe, display stub), WiFi client + BLE client mirroring Tab5's two transports, a standalone local menu (works with zero Tab5 connection), and a command dispatcher that routes incoming C2 commands to registered feature modules.
- **`shared/c2proto`** — the wire protocol both firmwares link against: a framed, HMAC-SHA256-authenticated message format (`c2proto::Frame`) with PSK-based auth.
- **`shared/feature_contract`** — the `FeatureModule`/`FeatureRegistry` pattern every Phase 2+ feature will plug into: a module declares an id, category, device affinity, and (as of Task 20) optional start/stop callbacks; the registry drives both the Tab5 launcher UI and the Cardputer-ADV command dispatcher off the same declarations.

## Architecture decisions worth knowing

**Dual transport, not ESP-NOW.** The original design used ESP-NOW as the control channel. Real verification (checking which `.a` files export `esp_now_init`) found ESP-NOW has no implementation at all for the ESP32-P4 in the installed framework — esp-hosted's WiFi remoting to the C6 doesn't proxy the ESP-NOW API surface. Redesigned around two persistent-connection transports instead, selected by whichever radio is free: a WiFi TCP socket (Tab5 = AP + server, Cardputer-ADV = client) and BLE GATT via NimBLE (Tab5 = GATT server, Cardputer-ADV = client). `IC2Link`'s interface is deliberately simple — `send()`, `set_receive_handler()`, `is_connected()` — with no per-message peer addressing, since both transports are single persistent connections rather than ESP-NOW's connectionless peer model.

**BLE on the P4 needed a raw ESP-IDF approach.** NimBLE-Arduino (the library Cardputer-ADV uses normally) does not compile for ESP32-P4 at all (maintainer-confirmed upstream). The underlying ESP-IDF NimBLE host stack is already linkable in the installed framework though, so Tab5's BLE server is written directly against the raw `ble_gap_*`/`ble_gatts_*` C API — no NimBLE-Arduino, no esp-nimble-cpp. Cardputer-ADV's BLE client is normal NimBLE-Arduino (works fine on S3).

**Feature descriptor/executor split.** A feature module has two halves: a Tab5-side "descriptor" (UI tile + command-sending logic) and, for satellite-affinity features, a Cardputer-ADV-side "executor" (`on_start`/`on_stop` callbacks the command dispatcher invokes). Task 20's ping feature is the reference implementation every Phase 2+ feature follows.

## Real hardware findings

Everything below was found only once real devices were in hand — none of it was discoverable from source/datasheets alone, and several would have silently produced a "looks done but doesn't work" state without live hardware testing.

**Tab5:**
- The ESP32-C6 radio co-processor was in a continuous brownout crash-loop: wrong SDIO pins (generic eval-board definition instead of Tab5's actual wiring) **and** no power at all (gated by an I2C IO-expander bit nothing was asserting). Fixed with a custom in-repo board definition plus a small IO-expander driver.
- The MIPI-DSI display was never actually implemented (backlight-only stub) prior to hardware bring-up. Once implemented, the panel initially rendered black despite every software-level check reporting success — the physical panel is an ST7121, misdetected as the visually-identical ST7123 by the community BSP this project's reference used (both ACK the same I2C probe address; only a firmware-version register read tells them apart). Fixed using the correct 32-command init table, cross-checked against M5Stack's own M5GFX driver source.
- Touch was assumed to be a standalone GT911 controller at a separate I2C address — it doesn't exist. The ST7121 is a TDDI part with the touch engine integrated into the same silicon, answering at the display's own I2C address. Confirmed working via real taps on hardware once the correct controller was targeted.
- The external HY2.0 peripheral port (where NFC/RFID2/RF433 modules connect) was completely dead — same root cause class as the C6: a different bit on the same IO-expander, unasserted. Fixed by reusing the existing IO-expander driver. The connected NFC unit turned out to be an ST25R3916, not the PN532 originally assumed.

**Cardputer-ADV:**
- Crash-looped every ~225ms on first boot, before `setup()` ever ran: the board's ESP32-S3FN8 chip has zero embedded PSRAM, but the build claimed `-DBOARD_HAS_PSRAM`. That flag isn't just an app-level hint — it's Arduino-ESP32's actual switch for compiling PSRAM bring-up in or out. Root cause traced to the object-code level (disassembly of the crashing ELF); fix was removing the one false flag.
- The BLE transport connected and subscribed correctly but replies never arrived back at Tab5. Two independent bugs, both required fixing: (1) `C2LinkBle::poll()` had an early return that made the receive-drain loop unreachable once connected, so inbound frames were queued but never dispatched — a structural bug that would have broken any future BLE-carried feature, not just ping; (2) Tab5's BLE characteristic only declared write-with-response support while Cardputer-ADV replied with write-without-response, so the server silently dropped every reply per BLE spec (no error observable on either side).
- Cardputer-ADV's own `Serial.printf` output has never once been observed over its USB serial in this project (only `ESP_LOG`-tagged traffic comes through) — an unresolved, real toolchain quirk. All Cardputer-side behavior has to be inferred from Tab5's serial output plus whether the overall round trip succeeds.

## Known limitations / deferred work

- **Cardputer-ADV's display and keyboard are HAL skeleton stubs, not real drivers.** `Device::init()` hardcodes `display_ready_ = true` with no real ST7789 panel init call, and keyboard readiness is only a TCA8418 I2C address ACK, not real key-matrix scanning. The physical screen is currently black with no backlight enabled at all. This is intentional, deferred scope (real driver bring-up was always planned for Phase 5-6, not Phase 1) — confirmed with the user rather than treated as a bug.
- **Pairing screen UX needs rework before it's relied on again.** The Tab5 pairing screen displays the PSK as both hex text and a QR code, but the Cardputer-ADV has no camera to scan it — the QR half of the flow is unusable as designed. Separately, the on-screen back button is positioned directly on top of the QR code and is small enough to be hard to tap reliably. Not fixed in Phase 1 (out of scope for a foundation-phase hardware bring-up task), but any future Tab5 UI work — especially phases that touch pairing or add other QR-driven flows — should account for this rather than copy the current layout.
- **RFID2 (WS1850S) confirmed on real hardware (2026-08-09)** — detected at I2C 0x28 with a real unit plugged into HY2.0 PORT.A, matching the doc-only prediction.
- **RF433T confirmed on real hardware (2026-08-09); RF433R still a hypothesis.** RF433R and RF433T were both confirmed to use the same physical HY2.0 PORT.A connector as NFC/RFID2 (swapped one at a time, not simultaneous) — the port isn't I2C-exclusive as Task 18 originally concluded. `TAB5_RF433T_PIN=53` is now a real, independently-verified value: a distinctive on/off blink driven on GPIO53 produced real 433.920MHz activity observed by a second device (Poseidon) acting as an independent listener, precisely correlated with the test window. `TAB5_RF433R_PIN` is set to the same pin (53) as a reasoned hypothesis, not an independent confirmation — an earlier receive-side test (polling `digitalRead()` from `loop()` during a real remote-control button press) found no correlated signal, but this is now understood to likely be a false negative from sampling too slowly for genuine OOK receive-pulse timing, not real evidence against the pin. A proper interrupt- or timer-driven receive test is Phase 3 scope.
- **The pioarduino platform is unpinned** (tracks the git branch head, not a tagged release) for both firmwares — an upstream update could silently change behavior without the `static_assert`s (which only cover 7 specific C6 pins) catching it.
- **C6 co-processor firmware is older than the host expects** (1.4.1 vs. an expected 2.12.11) — works today, flagged as worth a deliberate upgrade-or-pin decision before it becomes load-bearing.
- **A headless serial debug trigger exists on Tab5** (`k`/`p` over Serial, gated behind `-DQUARKY_SERIAL_DEBUG`, off by default) that drives the pairing screen and ping-send without touching the physical UI — useful pattern for verifying future touch-driven features without camera/vision access during automated bring-up.

## How to build, flash, and pair

```bash
# Tab5
cd firmware/tab5 && pio run -t upload --upload-port <tab5 serial port>

# Cardputer-ADV
cd firmware/cardputer-adv && pio run -t upload --upload-port <cardputer-adv serial port>
```

Both boards use native USB-CDC serial, which re-enumerates on every reset — re-check the port name after each flash/reboot rather than assuming it's stable.

**Pairing:** open "Pair Satellite" on the Tab5 UI (or send `k` over serial with `QUARKY_SERIAL_DEBUG` enabled) to generate/display a PSK. Hardcode the same 16-byte key into Cardputer-ADV's `test_psk` array in `firmware/cardputer-adv/src/main.cpp` and reflash. Both C2 links on both devices must use the same key or every frame is silently dropped (HMAC mismatch, by design — this exact gap was a real bug found and fixed during Task 20).

## Verification

Task 21's bring-up log (`docs/superpowers/specs/2026-08-06-tab5-foundation-bringup-log.md`) walks every Definition-of-Done item from the design spec against what was actually verified on real hardware, with pass/fail and notes per item.
