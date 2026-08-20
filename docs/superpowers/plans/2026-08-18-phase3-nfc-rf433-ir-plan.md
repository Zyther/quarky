# Phase 3: Tab5-Native NFC/RFID2/RF433/IR Peripherals — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build real feature logic for the Tab5's four HY2.0 peripherals — NFC unit (ST25R3916), RFID2 unit (WS1850S), RF433R/T (GPIO OOK), and a dedicated I2C IR receiver/transmitter unit — on top of Phase 1's detection-only HAL (`INFC`/`NfcPN532`, `IRF433`/`Rf433Gpio`).

**Architecture:** Each peripheral family gets its own `firmware/tab5/src/features/<family>/` directory (`nfc/`, `rf433/`, `ir/`), following the exact `FeatureModule`/`FeatureRegistry` + LVGL `build_sub_screen()`/`ScreenStack` + `poll()`-driven pattern every Phase 2 feature already uses. The two NFC-family units share feature/UI code but NOT low-level chip drivers — `ws1850s_driver.cpp` and `st25r3916_driver.cpp` are separate, since real hardware research (Phase 1 Task 18) found they are genuinely different chips, not two instances of the same one. Tasks are sequenced risk-first: every real technical unknown (RF433R's actual receive pin, ST25R3916 register-level communication, RFID2's frequency capability, the IR unit's chip identity) gets its own spike task before any feature is built on top of it.

**Tech Stack:** Arduino-ESP32 framework (raw `Wire`/`Wire1` I2C, `attachInterrupt`/`micros()` for RF433 receive timing), LVGL 9.5, `IStorage`/`StorageSD` for SD capture storage, `FeatureModule`/`FeatureRegistry` (`shared/feature_contract`).

## Global Constraints

- LVGL screens: every feature screen is built via `build_sub_screen(title, &content)` (`firmware/tab5/src/ui/screen_scaffold.h`) and tears down via an `LV_EVENT_DELETE` handler that nulls every static widget pointer and cancels in-flight state — see `firmware/tab5/src/features/ble/ble_bad_kb.cpp` for the canonical shape.
- Launcher tiles: every feature calls `g_registry.register_module({id, name, category, Affinity::TAB5_NATIVE, start, nullptr})` from its own `register_module()`, called once from `main.cpp`'s `setup()`. `Category::NFC`, `Category::RF433`, and `Category::IR` already exist in `shared/feature_contract/src/feature_module.h` — no enum changes needed.
- Long-running work (MIFARE dictionary attacks, RF433 bruteforce, IR clone-library scans) is `poll()`-driven from `main.cpp`'s `loop()`, never a blocking loop inside a click handler — matches `wifi_connect.cpp`'s real-hardware crash lesson (blocking a click handler starves the loop-watchdog feed) already documented project-wide. No single `poll()` call may block longer than ~50ms without a disclosed, commented exception.
- Cross-task shared state (an ISR/interrupt handler writing state a `poll()` on the main task reads) needs either `volatile` (single-word scalars) or a `portMUX_TYPE` critical section (multi-field structs/buffers) — same house rule as `hal/c2link_ble.cpp`'s `s_rx_mux` and `features/ble/ble_scan.cpp`'s `s_devices_mux`.
- SD captures: `extern StorageSD storage;` (declared `firmware/tab5/src/hal/storage_sd.h`, defined `main.cpp:116`) is the single global `IStorage` instance every feature already uses (see `wifi_pmkid.cpp`). Tag/signal/IR captures save under `/quarky/captures/nfc/`, `/quarky/captures/rf433/`, `/quarky/captures/ir/` respectively, matching Phase 2's established `/quarky/captures/<category>/` convention.
- Real sources only for protocol/register-level work — no fabricated register maps, command bytes, or pin values. Every spike task below names what to go verify against (a real datasheet, a real donor implementation, a real interrupt-timing capture from actual hardware) rather than prescribing register values that would have to be invented. This is the same discipline this project has followed since Phase 1's display/touch/SDIO hotfixes and Phase 2's BLE HID spike.
- **Standing hardware-checkpoint instruction (2026-08-18, project owner):** any task step that requires a physical peripheral to be connected, swapped, or otherwise physically manipulated (RF433T vs RF433R vs NFC vs RFID2 all share the Tab5's single HY2.0 PORT.A socket and are swapped by hand; the IR unit is a brand-new physical unit) must PAUSE and wait for the project owner's explicit confirmation before proceeding — never just ask in passing narration and continue. Tasks 1, 2, 3, and 15–19 below have explicit `**PAUSE FOR HARDWARE**` checkpoints for this reason; do not skip them even if a previous task already established the physical connector state, since the owner swaps units by hand between tasks.
- Existing HAL instances (`main.cpp`): `NfcPN532 nfc_unit(TAB5_NFC_I2C_ADDR)` (0x50, ST25R3916), `NfcPN532 rfid2_unit(TAB5_RFID2_I2C_ADDR)` (0x28, WS1850S), `Rf433Gpio rf433`. Despite the class name, `NfcPN532::detect()` is a bare I2C presence probe today (Phase 1 Task 18) — real protocol logic is entirely this phase's job.

---

## Task 1: RF433R receive-pin confirmation spike

**Files:**
- Create: `firmware/tab5/src/features/rf433/rf433_common.h`
- Create: `firmware/tab5/src/features/rf433/rf433_common.cpp`
- Modify: `firmware/tab5/src/main.cpp` (temporary serial-debug trigger, gated behind `QUARKY_SERIAL_DEBUG` matching every other spike in this project — e.g. Task 20's ping-feature trigger)

**Interfaces:**
- Produces: `namespace Rf433Common { struct EdgeSample { uint32_t timestamp_us; bool level; }; bool capture_start(); void capture_stop(); size_t capture_read(EdgeSample *out, size_t max); }` — a real, reusable interrupt-driven edge-timing capture used by every later RF433 task (scan, protocol decode, bruteforce), not spike-only throwaway code.

**Context:** Phase 1's ledger (`.superpowers/sdd/2026-08-06-tab5-foundation-plan/progress.md`, 2026-08-09 entries) already confirmed `TAB5_RF433T_PIN = GPIO53` via an independent 433.92MHz listener. `TAB5_RF433R_PIN = GPIO53` is only a same-connector hypothesis — a same-day `loop()`-polling receive test found nothing, but `loop()`-based `digitalRead()` polling is far too slow and irregular to catch real OOK receive-pulse timing (typically hundreds of microseconds per bit), so that result is understood as a likely false negative, not real evidence against the pin. This task runs the *right* kind of test: a GPIO interrupt handler (`attachInterrupt`, `CHANGE` mode) recording `micros()`-timestamped edges into a ring buffer — the same real, industry-standard technique `rc-switch` (the library Bruce's `rf_scan.cpp` is built on, per the Phase 3 spec's own donor citation) uses for OOK receive.

- [ ] **Step 1: Implement the edge-capture ring buffer**

```cpp
// firmware/tab5/src/features/rf433/rf433_common.h
#pragma once
#include <cstdint>
#include <cstddef>

namespace Rf433Common {

struct EdgeSample {
    uint32_t timestamp_us;
    bool level; // GPIO level AFTER the edge (i.e. the level this sample transitioned TO)
};

// Starts capturing GPIO edges on TAB5_RF433R_PIN via attachInterrupt(CHANGE).
// Safe to call while a capture is already running (no-op, matching this
// project's established "refuse rather than lie" idempotent-start
// convention, e.g. BleHidSpike::start()). Returns false if the pin isn't
// configured as INPUT (Rf433Gpio::init() must have run first -- see
// main.cpp's setup() ordering).
bool capture_start();

// Stops the interrupt handler. Safe to call whether or not a capture is
// running.
void capture_stop();

// Copies up to max samples out of the ring buffer into out, in the order
// they were captured, and clears the buffer. Returns the number actually
// copied. Call from poll() on the main task -- see this function's .cpp for
// the portMUX_TYPE critical section that makes this safe against the
// interrupt handler (which runs on... an ISR context, not a FreeRTOS task,
// so this needs an ISR-safe critical section, not just the plain
// portENTER_CRITICAL this project's other cross-TASK state uses -- use
// portENTER_CRITICAL_ISR from inside the handler and the matching
// non-ISR portENTER_CRITICAL from capture_read(), which is the standard
// ESP-IDF pairing for a buffer shared between an ISR and a task).
size_t capture_read(EdgeSample *out, size_t max);

} // namespace Rf433Common
```

```cpp
// firmware/tab5/src/features/rf433/rf433_common.cpp
#include "rf433_common.h"
#include "../../../boards/tab5/pins_config.h"
#include <Arduino.h>

namespace Rf433Common {
namespace {

constexpr size_t kRingSize = 512; // generous headroom for one OOK burst --
                                   // typical fixed-code remotes send a few
                                   // hundred edges per press (repeated ~4-8x)
volatile EdgeSample s_ring[kRingSize];
volatile size_t s_head = 0; // ISR-owned write index, wraps
volatile size_t s_count = 0; // number of valid unread samples, capped at kRingSize
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
bool s_capturing = false;

void IRAM_ATTR isr_edge() {
    portENTER_CRITICAL_ISR(&s_mux);
    s_ring[s_head] = {micros(), (bool)digitalRead(TAB5_RF433R_PIN)};
    s_head = (s_head + 1) % kRingSize;
    if (s_count < kRingSize) s_count++;
    portEXIT_CRITICAL_ISR(&s_mux);
}

} // namespace

bool capture_start() {
    if (s_capturing) return true;
    pinMode(TAB5_RF433R_PIN, INPUT);
    portENTER_CRITICAL(&s_mux);
    s_head = 0;
    s_count = 0;
    portEXIT_CRITICAL(&s_mux);
    attachInterrupt(digitalPinToInterrupt(TAB5_RF433R_PIN), isr_edge, CHANGE);
    s_capturing = true;
    Serial.println("quarky-tab5: [rf433] edge capture started on GPIO"
                    + String(TAB5_RF433R_PIN));
    return true;
}

void capture_stop() {
    if (!s_capturing) return;
    detachInterrupt(digitalPinToInterrupt(TAB5_RF433R_PIN));
    s_capturing = false;
}

size_t capture_read(EdgeSample *out, size_t max) {
    portENTER_CRITICAL(&s_mux);
    // Read out oldest-first: s_head is the next WRITE slot, so with a full
    // buffer the oldest sample is at s_head itself; with a partially-filled
    // buffer (s_count < kRingSize) the oldest is at index 0 (capture_start()
    // reset s_head to 0, so the first s_count writes landed at 0..s_count-1
    // in order -- no wraparound has happened yet).
    size_t n = s_count < max ? s_count : max;
    size_t start = (s_count < kRingSize) ? 0 : s_head;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(start + i) % kRingSize];
    }
    s_count = 0; // consumed
    portEXIT_CRITICAL(&s_mux);
    return n;
}

} // namespace Rf433Common
```

- [ ] **Step 2: Wire a serial-debug spike trigger**

In `main.cpp`, inside the existing `#ifdef QUARKY_SERIAL_DEBUG` serial-command block (see the `'h'`/`'j'` BLE HID triggers for the exact pattern), add a new case (pick an unused letter, e.g. `'r'`) that calls `Rf433Common::capture_start()`, then on a *second* press of the same key calls `Rf433Common::capture_stop()` and dumps every captured `EdgeSample`'s `timestamp_us` delta from the previous sample plus its `level` to `Serial`, one line per edge.

- [ ] **Step 3: PAUSE FOR HARDWARE, then run the real test**

**PAUSE FOR HARDWARE:** before this step, stop and ask the project owner to confirm the RF433R unit (receiver) is physically plugged into the Tab5's HY2.0 PORT.A socket (swap out whatever unit is currently connected). Wait for their explicit confirmation before proceeding — do not just narrate the request and continue.

Once confirmed: flash, trigger the capture via the serial command, have the project owner press a real 433MHz remote control (garage door, doorbell, etc. — their own equipment) several times during the ~10-20 second capture window, stop the capture, and inspect the dumped edge timings.

**Acceptance criterion:** a real OOK/ASK fixed-code remote produces edges clustered into a small number of distinct pulse-width buckets (typically two: a "short" pulse in the hundreds-of-microseconds range and a "long" pulse some small integer multiple of it — this is the defining structural signature of PWM/PPM-encoded fixed-code RF, and is what distinguishes a real signal from noise), repeating in a near-identical burst pattern several times in a row (most remotes repeat the same code 4-8x per button press). Constant, evenly-spaced chatter with no repeating structure (the same failure signature Phase 1's `loop()`-polling test saw) means GPIO53 is NOT the receive pin, and this task's result is "hypothesis disproven" — in that case, do not guess a different pin; report back and stop, since no further real-hardware evidence exists to guess from (the same honest-escalation-over-forced-fix principle this project has followed throughout).

- [ ] **Step 4: Update pins_config.h's confidence annotation and commit**

If Step 3 confirms real, structured signal timing: update `TAB5_RF433R_PIN`'s comment in `boards/tab5/pins_config.h` from "HYPOTHESIS, not independently confirmed" to "CONFIRMED (real interrupt-driven receive test, <today's date>, real remote — see this plan's Task 1)", matching `TAB5_RF433T_PIN`'s existing confirmed-annotation style exactly. Update `Rf433Gpio::init()`'s log line the same way. Commit `rf433_common.h/.cpp`, the `main.cpp` debug trigger, and the `pins_config.h`/`rf433_gpio.cpp` annotation updates together.

**Model:** Opus — this is real-hardware interrupt/timing work with a genuine chance of finding the hypothesis wrong, matching this project's established tiering for spikes with real technical risk (e.g. Phase 2's BLE central-connect spike, BLE HID spike).

---

## Task 2: ST25R3916 register-level bring-up spike (NFC unit)

**Files:**
- Create: `firmware/tab5/src/features/nfc/st25r3916_driver.h`
- Create: `firmware/tab5/src/features/nfc/st25r3916_driver.cpp`

**Interfaces:**
- Produces: `namespace St25r3916 { bool init(); bool read_chip_id(uint8_t *id_out); bool field_on(); void field_off(); }` — the foundation every later NFC-unit feature task builds its actual tag-protocol logic on top of.

**Context:** This is the single biggest unknown in this phase (see the spec's Section 3 risk, added 2026-08-18). No donor project (Bruce/Poseidon/UniGeek) has ANY ST25R3916 code — they are all PN532-oriented. **Before writing any register-poke code, obtain ST's real ST25R3916 datasheet and reference driver** (ST publishes both openly: the datasheet at st.com, and reference C code as part of their "ST25R3916 discovery kit" / RFAL (RF Abstraction Layer) library, commonly mirrored on GitHub as `st25rfal002` or similar — verify the exact repository name/URL at implementation time rather than trusting a name recalled here, since library naming from ST has shifted over time). Do not invent register addresses, command bytes, or bit-field meanings — every constant in this driver must trace to that real source, cited in a comment the same way `nfc_pn532.cpp`'s header comment cites M5Stack's own documentation for the chip-identity finding this task is a direct continuation of.

- [ ] **Step 1: Research and cite the real chip-ID readback sequence**

Read ST's real ST25R3916 datasheet section on device identification (typically an IC Identity register, readable via the chip's direct/indirect register-access command framing over I2C). Write the citation (datasheet revision/section, or reference-driver file/function name) as a comment at the top of `st25r3916_driver.cpp`, in the same style as `nfc_pn532.cpp`'s existing header comment.

- [ ] **Step 2: Implement `init()` and `read_chip_id()`**

```cpp
// firmware/tab5/src/features/nfc/st25r3916_driver.h
#pragma once
#include <cstdint>

// Real ST25R3916 register-level driver for the Tab5's NFC unit (I2C 0x50 on
// Wire1 -- see hal/nfc_pn532.cpp's header comment for the chip-identity
// research this continues). NOT PN532 framing -- despite the HAL class
// being named NfcPN532 for interface-contract reasons, this file's protocol
// is entirely ST25R3916's own, built from ST's real datasheet/reference
// driver (cited in st25r3916_driver.cpp), not ported from any donor project.
namespace St25r3916 {

// Brings the chip out of reset / into a known register state and confirms
// I2C communication works via read_chip_id() internally. Must be called
// before field_on()/read_chip_id() are meaningful. Idempotent.
bool init();

// Reads the chip's IC Identity register. Returns false on any I2C failure.
// *id_out receives the raw register value regardless of whether it matches
// the datasheet's documented ST25R3916 identity value -- callers (this
// task's own verification step) compare it themselves so a mismatch is
// visible rather than silently swallowed.
bool read_chip_id(uint8_t *id_out);

// Enables/disables the RF field (required before any tag can be detected --
// analogous to a PN532's RFConfiguration + field-on sequence, but this
// chip's own real command for it, per the cited datasheet section).
bool field_on();
void field_off();

} // namespace St25r3916
```

The `.cpp`'s actual register addresses/command bytes are written from the real cited source during implementation — do not fill this in from memory here; that is precisely the fabrication risk this task exists to avoid.

- [ ] **Step 3: PAUSE FOR HARDWARE, then verify against real hardware**

**PAUSE FOR HARDWARE:** confirm with the project owner that the NFC unit (not RFID2) is the one currently connected to PORT.A before running this step. Wait for explicit confirmation.

Flash and run `read_chip_id()` via a serial-debug trigger (same `QUARKY_SERIAL_DEBUG` pattern as Task 1). **Acceptance criterion:** the returned register value matches the datasheet's documented ST25R3916 IC-identity value exactly (not "something non-zero came back" — a real, specific, cited expected value). Log both the expected and actual value.

- [ ] **Step 4: Commit**

Commit `st25r3916_driver.h/.cpp` and the serial-debug trigger together, with the real-hardware chip-ID readback result recorded in the commit message or a follow-up ledger entry.

**Model:** Opus — largest single unknown in this phase, genuine research + protocol-implementation risk, matching this project's tiering for its highest-risk spikes.

---

## Task 3: RFID2 (WS1850S) frequency-capability confirmation + ~~PN532~~ **MFRC522**-protocol bring-up

> **PREMISE CORRECTED 2026-08-19 — read this before reading the rest of this task.** This task's Context and Step 1 below state that WS1850S is PN532-register-compatible. **It is not.** Executing the task refuted that from the donor source it pointed at: Bruce's `RFID2.cpp` and UniGeek's `MFRC522Screen.cpp` both drive this exact unit at `0x28` with **MFRC522** libraries, and M5Stack's own library documents WS1850S as "**PN512**-compatible silicon" (PN512, an MFRC522-family register-mapped part — not PN532). Confirmed on real hardware: the delivered driver's MFRC522 framing reads `VersionReg (0x37) = 0x15`, while a real PN532 `GetFirmwareVersion` frame got no response. **What was actually built is an MFRC522 register driver** with the four signatures below unchanged. The original text is left intact below as the historical record of what was asked for; see the Phase 3 spec's 2026-08-19 correction and `features/nfc/ws1850s_driver.cpp`'s header for the full citation trail.

**Files:**
- Create: `firmware/tab5/src/features/nfc/ws1850s_driver.h`
- Create: `firmware/tab5/src/features/nfc/ws1850s_driver.cpp`

**Interfaces:**
- Produces: `namespace Ws1850sDriver { bool init(); bool get_firmware_version(uint8_t out[4]); bool field_on(); void field_off(); }` — the RFID2-unit-specific protocol layer every later RFID2 feature builds on.
- Consumes: nothing from Task 2 (separate chip, separate driver — see the spec's Section 2.1 "two separate low-level implementations" note).

**Context:** WS1850S is PN532-register-compatible (Phase 1's real-hardware finding), so this task's protocol layer is a much closer port of Bruce/UniGeek's real PN532 code than Task 2's ST25R3916 work — the standard PN532 host-controller frame format (preamble `0x00`, start code `0x00 0xFF`, TFI `0xD4` host-to-PN532 / `0xD5` PN532-to-host, followed by command/response bytes and a checksum) is genuinely public, standardized, and identical across every PN532-based donor implementation, not something specific to guess at. `GetFirmwareVersion` (command byte `0x02`) is the standard PN532 bring-up command every donor project already uses for exactly this purpose.

- [ ] **Step 1: Implement the PN532 frame layer and `get_firmware_version()`**

```cpp
// firmware/tab5/src/features/nfc/ws1850s_driver.h
#pragma once
#include <cstdint>

// PN532-register-compatible driver for the Tab5's RFID2 unit (WS1850S, I2C
// 0x28 on Wire1) -- see hal/nfc_pn532.cpp's header comment for the
// chip-identity research. Uses the standard PN532 host-controller frame
// format (public/standardized across every PN532 implementation), ported
// from Bruce/UniGeek's real PN532 code rather than re-derived -- see this
// file's own citation comments for exact donor file references.
namespace Ws1850sDriver {

bool init();

// Sends GetFirmwareVersion (PN532 command 0x02) and reads the 4-byte
// response (IC, Ver, Rev, Support). Returns false on any I2C/framing
// failure. The real acceptance test for "this chip really does speak
// PN532 framing" -- if this fails, the WS1850S-is-PN532-compatible
// assumption itself needs re-examining before any further work here.
bool get_firmware_version(uint8_t out[4]);

bool field_on();
void field_off();

} // namespace Ws1850sDriver
```

Port the actual frame-building/parsing code from Bruce's or UniGeek's real PN532 source (cite the exact donor file in a comment), adapted to this project's `Wire1`/`ensureExternalI2CBegun()`-style bus access (see `nfc_pn532.cpp` for the existing external-bus pattern to reuse, including the `EXT_5V_EN` power-gate sequencing already solved there).

- [ ] **Step 2: PAUSE FOR HARDWARE, then verify GetFirmwareVersion on real hardware**

**PAUSE FOR HARDWARE:** confirm with the project owner that the RFID2 unit is the one currently connected to PORT.A. Wait for explicit confirmation.

Flash, trigger `get_firmware_version()` via a serial-debug command, confirm a real, non-error response comes back (log all 4 bytes).

- [ ] **Step 3: Confirm frequency capability from the real datasheet, resolve the 125kHz spec risk**

Look up WS1850S's actual datasheet/product page (M5Stack's own Unit RFID2 documentation, already cited in `nfc_pn532.cpp`'s header comment as the source for the chip identity itself, is the right starting point). **Acceptance criterion:** a definitive answer, backed by a real citation, on whether the RFID2 unit supports 125kHz LF operation at all (WS1850S, like PN532, is fundamentally a 13.56MHz NFC/RFID reader IC — the spec's own risk note already flags this as unlikely, but this task must confirm it from the real datasheet rather than leave it assumed).

- [ ] **Step 4: Update the Phase 3 spec and commit**

Update `docs/superpowers/specs/2026-08-06-phase3-tab5-nfc-rf433-design.md`'s 125kHz/T5577/HID-Prox feature row with the confirmed answer (either "confirmed supported, proceed" or "confirmed NOT supported, row dropped" — either way, close the open question rather than leave the spec's placeholder language in place). Commit `ws1850s_driver.h/.cpp`, the serial trigger, and the spec update together.

**Model:** Opus — real protocol bring-up with a genuine "does the compatibility assumption actually hold" question, though lower risk than Task 2 since PN532 framing is well-documented and directly portable.

---

## Task 4: NFC common module + baseline tag read (both units)

**Files:**
- Create: `firmware/tab5/src/features/nfc/nfc_common.h`
- Create: `firmware/tab5/src/features/nfc/nfc_common.cpp`
- Create: `firmware/tab5/src/features/nfc/nfc_read.h`
- Create: `firmware/tab5/src/features/nfc/nfc_read.cpp`
- Modify: `firmware/tab5/src/main.cpp` (register two launcher tiles, one per unit)

**Interfaces:**
- Consumes: `St25r3916::init/field_on/field_off` (Task 2), `Ws1850sDriver::init/field_on/field_off` (Task 3).
- Produces: `namespace NfcCommon { struct TagInfo { uint8_t uid[10]; uint8_t uid_len; char type_name[24]; }; const char *format_uid(const uint8_t *uid, uint8_t len, char *out, size_t out_len); }` (shared UID formatting/type-name buffer, used by every later NFC feature task including the tag library); `namespace NfcRead { void register_module_nfc_unit(); void register_module_rfid2_unit(); void poll(); }`.

**Context:** Per the spec's Section 2.1 (corrected 2026-08-18), the two units share this UI/feature layer but dispatch to their own chip driver underneath. Since the two units' actual tag-detection/anticollision sequences differ by chip, `nfc_read.cpp` needs a real polling-loop implementation per chip — for the RFID2/WS1850S path, port the real **MFRC522 PICC anticollision sequence** — `PICC_RequestA()` (sends REQA, collects the ATQA) followed by `PICC_ReadCardSerial()` / `PICC_Select()` (the cascade-level anticollision loop that yields the UID), driven over the FIFO/`PCD_Transceive` path. Real donor references to port from: Bruce `src/modules/rfid/RFID2.cpp:39-53` (`PICC_IsNewCardPresent()` → `PICC_RequestA(bufferATQA, &bufferSize)`) and `:60` (`PICC_ReadCardSerial()`); library-side implementations at `MFRC522_I2C.cpp:493` (`PICC_RequestA`), `:556` (`PICC_Select`), `:1792` (`PICC_ReadCardSerial`). **CORRECTED 2026-08-19 — this sentence previously said "port the real PN532 `InListPassiveTarget` command (standard PN532 command `0x4A`)". That instruction was WRONG and would have sent this task down a dead end: WS1850S is MFRC522/PN512-protocol silicon (proven on hardware, see Task 3's correction banner), and MFRC522-family chips have NO `InListPassiveTarget` command — no host-controller command set at all. Tag detection there is FIFO/transceive plus PICC anticollision, a completely different protocol shape. Note this also means the RFID2 path does NOT "return the UID directly" the way the old text promised; it is a multi-step exchange the donor libraries already implement.** For the ST25R3916/NFC-unit path, use whatever real tag-detection sequence Task 2's cited reference driver provides (do not invent one here — if Task 2's driver doesn't yet expose a passive-target-detect call, that's this task's cue to extend `st25r3916_driver.h`'s interface, still sourced from the same real ST reference material).

- [ ] **Step 1: Implement `NfcCommon::format_uid()` and `TagInfo`**

Straightforward hex-formatting helper (`"04:A3:F1:..."` style, matching this project's existing `ble_addr_to_str()`-style formatting convention in `features/ble/ble_common.cpp` for the same kind of byte-array-to-hex-string task) — no hardware dependency, can be unit-style-tested on-device via a fixed test UID before any real tag is involved.

- [ ] **Step 2: Implement the two `register_module_*` entry points and shared screen**

Follow `ble_bad_kb.cpp`'s exact `build_screen()`/`LV_EVENT_DELETE` teardown shape: a screen with a "Scan" trigger (or auto-poll while open, whichever the real per-chip polling call shape naturally supports without blocking `loop()` — resolve during implementation based on what Tasks 2/3's `field_on()`/detect calls actually cost in wall-clock time per attempt) and a result card showing `TagInfo` once a tag is found. Two `FeatureModule` registrations (`"nfc_read_nfc"` / `"NFC: Tag Read"` and `"nfc_read_rfid2"` / `"RFID2: Tag Read"`, both `Category::NFC`) sharing the same underlying screen-building code parameterized by which driver to call.

- [ ] **Step 3: PAUSE FOR HARDWARE, then verify against a real tag**

**PAUSE FOR HARDWARE:** ask the project owner to have a real NFC/RFID tag (any MIFARE Classic, NTAG, or similar tag they have) ready, and confirm which unit (NFC or RFID2) is currently connected to PORT.A, since only one can be tested at a time on the single physical socket. Wait for explicit confirmation before testing each unit; test both units (one at a time, pausing between swaps) if the owner has tags compatible with each.

**Acceptance criterion:** presenting a real tag to the unit under test produces a correct, non-garbage UID logged to serial and shown on the result card.

- [ ] **Step 4: Commit**

**Model:** Sonnet — integration work on top of Tasks 2/3's already-proven protocol layers, following an established in-repo UI pattern closely.

---

## Task 5: RF433 scan/capture feature

**Files:**
- Create: `firmware/tab5/src/features/rf433/rf433_scan.h`
- Create: `firmware/tab5/src/features/rf433/rf433_scan.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `Rf433Common::capture_start/capture_stop/capture_read/EdgeSample` (Task 1).
- Produces: `namespace Rf433Scan { void register_module(); void poll(); }`; a captured-signal struct (`struct CapturedSignal { EdgeSample edges[kMaxEdgesPerSignal]; size_t edge_count; uint32_t captured_at_ms; }`) later tasks (replay, protocol decode, bruteforce, tag library) consume.

**Context:** Follows the spec's Section 2.2 UI pattern: "RF433 scan/replay follows the same list-and-select pattern as Phase 2's WiFi AP scan" — use `wifi_scan.cpp`'s real `lv_list` populate-on-poll()-tick shape as the direct template (already a proven, real-hardware-verified pattern in this codebase) rather than re-deriving a list UI from scratch.

- [ ] **Step 1: Implement capture-to-list-entry logic**

`poll()` calls `Rf433Common::capture_read()` each tick while the screen is open and a capture is armed; a full burst (edges separated by a long idle gap, e.g. >10ms with no new edge — a real, standard end-of-burst heuristic used by rc-switch-style receivers) closes out one `CapturedSignal` entry and adds it to the on-screen `lv_list`.

- [ ] **Step 2: Build the screen (list + Start/Stop capture)**

- [ ] **Step 3: PAUSE FOR HARDWARE, then verify against a real remote**

**PAUSE FOR HARDWARE:** confirm RF433R is connected; wait for explicit confirmation. Verify a real remote press produces a new list entry with a plausible edge count (tens to low hundreds, matching Task 1's acceptance-test observations).

- [ ] **Step 4: Commit**

**Model:** Sonnet.

---

## Task 6: RF433 replay feature

**Files:**
- Create: `firmware/tab5/src/features/rf433/rf433_replay.h`
- Create: `firmware/tab5/src/features/rf433/rf433_replay.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `Rf433Scan::CapturedSignal` (Task 5) or a signal loaded from the tag library (Task 10, if sequenced after — otherwise this task supports replaying a just-captured-this-session signal only, and gains library-load support once Task 10 lands).
- Produces: `namespace Rf433Replay { void transmit(const Rf433Scan::CapturedSignal &sig); void register_module(); }`.

**Context:** `transmit()` bit-bangs `TAB5_RF433T_PIN` (GPIO53, confirmed Phase 1) via `digitalWrite()` + `delayMicroseconds()` reproducing each edge's timing from the captured `EdgeSample` array — the same real technique Bruce's `rf_send.cpp`/`emit.cpp` (via `rc-switch`) uses, per the spec's own donor citation.

- [ ] **Step 1: Implement `transmit()`**

- [ ] **Step 2: Wire a "Replay" action into the scan-result list from Task 5**

- [ ] **Step 3: PAUSE FOR HARDWARE, then verify end-to-end**

**PAUSE FOR HARDWARE:** ask the project owner to swap PORT.A to RF433T (transmitter) for this test, and have their real remote-controlled device (garage door, etc.) present to observe whether the replay actually triggers it. Wait for explicit confirmation before proceeding — this test involves actually triggering a real device the owner controls, so confirm they're ready to observe the result, not just that the unit is plugged in.

**Acceptance criterion:** replaying a captured signal from Task 5 triggers the same real-world response the original remote press did (the door opens, the doorbell rings, etc.) — a genuinely convincing end-to-end signal, not just "no crash."

- [ ] **Step 4: Commit**

**Model:** Sonnet.

---

## Task 7: RF433 protocol decode

**Files:**
- Create: `firmware/tab5/src/features/rf433/rf433_protocol_decode.h`
- Create: `firmware/tab5/src/features/rf433/rf433_protocol_decode.cpp`
- Test: `firmware/tab5/test/test_rf433_protocol_decode.cpp` (host-native, per the spec's Testing Strategy — table-driven against known sample captures)

**Interfaces:**
- Consumes: `Rf433Scan::CapturedSignal` (Task 5).
- Produces: `namespace Rf433ProtocolDecode { struct DecodedCode { char protocol_name[24]; uint64_t code; uint8_t bit_length; }; bool decode(const Rf433Scan::CapturedSignal &sig, DecodedCode *out); }` — consumed by Task 8 (bruteforce) and the tag library (Task 10)'s display of a decoded-vs-raw signal.

**Context:** Per the spec, prioritize UniGeek's `utils/rf/M5RF433Util.*` as the primary reference since it's cited as a direct match for this exact hardware unit — port its real pulse-width-to-bit decode tables and brand/protocol identification logic (Came/Nice/Linear/Chamberlain/Holtek/Ansonic timing signatures), citing the exact donor file, rather than inventing timing thresholds. This is genuinely host-native-testable: given a fixed array of `EdgeSample`s (either captured once from real hardware and hardcoded as a test fixture, or taken directly from UniGeek's own test/sample data if it ships any), decode should deterministically produce the same `DecodedCode` every time — no hardware needed to test the decode logic itself, only to obtain the initial sample fixture.

- [ ] **Step 1: Port the protocol timing tables and decode logic from UniGeek's `M5RF433Util`**

- [ ] **Step 2: Write host-native tests against real captured sample fixtures**

Capture at least one real signal via Task 5 first (reuse a capture the project owner already provided during Task 5/6's hardware checkpoints — no NEW hardware pause needed here if a suitable sample already exists from those sessions; only pause again if a fresh capture is genuinely needed), hardcode its edge array as a test fixture, assert `decode()` returns the expected protocol/code.

- [ ] **Step 3: Run the host-native test suite**

Run: `cd firmware/tab5/test && pio test -e native` (or this project's established host-test invocation — confirm the exact command from an existing host-tested module, e.g. `shared/feature_contract`'s `platformio.ini` `[env:native]`, if Tab5 doesn't already have its own native test environment; add one following that same shape if it doesn't exist yet).
Expected: PASS.

- [ ] **Step 4: Commit**

**Model:** Haiku/cheapest tier — table-driven, host-testable, mechanical port with a clear correctness gate (the test suite itself).

---

## Task 8: RF433 bruteforce

**Files:**
- Create: `firmware/tab5/src/features/rf433/rf433_bruteforce.h`
- Create: `firmware/tab5/src/features/rf433/rf433_bruteforce.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `Rf433Replay::transmit()` (Task 6), `Rf433ProtocolDecode::DecodedCode` (Task 7).
- Produces: `namespace Rf433Bruteforce { void register_module(); void poll(); }`.

**Context:** Port Bruce's `rf_bruteforce.cpp` / UniGeek's equivalent fixed-code keyspace-iteration logic (Came/Nice/Linear/Chamberlain/Holtek/Ansonic, per the spec) — `poll()`-driven (one or a small bounded batch of codes transmitted per tick, matching the Global Constraints' ~50ms budget and this project's already-established streaming-feature UI pattern: progress bar + Stop button, same as the spec's Section 2.2 calls for MIFARE key recovery).

- [ ] **Step 1: Implement keyspace iteration + per-tick transmit batching**
- [ ] **Step 2: Build the progress/Stop-button screen**
- [ ] **Step 3: PAUSE FOR HARDWARE, then verify against a real known-brand device**

**PAUSE FOR HARDWARE:** confirm RF433T connected and the project owner has a real fixed-code device of one of the supported brands ready to observe. Wait for confirmation.

- [ ] **Step 4: Commit**

**Model:** Sonnet.

---

## Task 9: MIFARE Classic key recovery (RFID2 unit)

**Files:**
- Create: `firmware/tab5/src/features/nfc/nfc_mifare_crack.h`
- Create: `firmware/tab5/src/features/nfc/nfc_mifare_crack.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `Ws1850sDriver::*` (Task 3), `NfcCommon::TagInfo/format_uid` (Task 4).
- Produces: `namespace NfcMifareCrack { void register_module(); void poll(); }`.

**Context:** RFID2-unit only (MIFARE Classic is a WS1850S/**MFRC522**-class operation — *corrected 2026-08-19, this said "PN532-class"; see Task 3's correction banner. This is good news for this task: UniGeek's `utils/nfc/` attacks are written directly against `MFRC522_I2C`, the same library its `MFRC522Screen.cpp` drives this unit with, so they port onto `ws1850s_driver` rather than onto a PN532 abstraction that would have had to be invented first* — the spec explicitly flags the ST25R3916/NFC-unit path as unconfirmed for this, don't assume parity). Port UniGeek's dictionary/nested/darkside attack implementation closely per the spec's explicit guidance ("timing-sensitive against the chip's own firmware, port... closely rather than re-deriving"). `poll()`-driven with a progress UI (keys tried / keyspace, elapsed time), same streaming pattern as Task 8.

- [ ] **Step 1: Port the dictionary-attack key list and per-tick key-try loop**
- [ ] **Step 2: Port the nested-attack and darkside-attack logic**

Expect real retry/timeout constant tuning during hardware bring-up per the spec's own risk note (WS1850S's firmware response latency may differ from whatever MFRC522-class module UniGeek tested against — *"PN532 module" corrected 2026-08-19*) — this is expected engineering work for this task, not a sign the port is wrong.

- [ ] **Step 3: Build the progress-card screen**
- [ ] **Step 4: PAUSE FOR HARDWARE, then verify against a known-weak test card**

**PAUSE FOR HARDWARE:** confirm RFID2 connected, and that the project owner has a MIFARE Classic test card programmed with a known-weak/default key ready (per the spec's Testing Strategy — "a blank/test MIFARE Classic card programmed with a default key" specifically so success/failure is unambiguous, never an unknown/real target card). Wait for confirmation.

**Acceptance criterion:** the known key is recovered correctly.

- [ ] **Step 5: Commit**

**Model:** Opus — timing-sensitive port against real chip firmware behavior, matching the spec's own flagged risk and this project's tiering for timing-sensitive real-hardware ports (e.g. Task 16's Fast Pair crypto exploit in Phase 2 Plan 2).

---

## Task 10: NFC/RFID2 tag library (SD-backed save/load/browse)

**Files:**
- Create: `firmware/tab5/src/features/nfc/nfc_tag_library.h`
- Create: `firmware/tab5/src/features/nfc/nfc_tag_library.cpp`
- Modify: `firmware/tab5/src/main.cpp`
- Test: `firmware/tab5/test/test_nfc_tag_library.cpp` (host-native round-trip test, per spec's Testing Strategy)

**Interfaces:**
- Consumes: `extern StorageSD storage;` (`hal/storage_sd.h`), `NfcCommon::TagInfo` (Task 4).
- Produces: `namespace NfcTagLibrary { bool save(const NfcCommon::TagInfo &tag); int list(char names_out[][64], int max_names); bool load(const char *name, NfcCommon::TagInfo *out); void register_module(); }`.

**Context:** UI/storage feature more than radio feature per the spec. Uses `IStorage::write_capture_file()`/`read_file()`/`list_files()` (already defined, see the interface dump above) exactly as `wifi_evil_portal.cpp`'s template picker already does for the same "save/list/load small files" shape — a direct, proven in-repo precedent to follow rather than re-derive. Saves under `/quarky/captures/nfc/`.

- [ ] **Step 1: Implement `save()`/`list()`/`load()` using a simple fixed-layout binary or text record format for `TagInfo`**
- [ ] **Step 2: Write a host-native round-trip test** (`save()` then `load()` returns the identical `TagInfo` — this needs `IStorage` mockable/fake-able for host-native testing; if `StorageSD` can't run host-native, write a minimal in-memory `IStorage` test double implementing the same interface, following whatever mocking pattern this project's existing host-native tests already use for HAL interfaces — check `shared/feature_contract/test/` for precedent before inventing a new one)
- [ ] **Step 3: Run tests, expect PASS**
- [ ] **Step 4: Build the browse-library screen (list + tap-to-view-and-optionally-load)**
- [ ] **Step 5: PAUSE FOR HARDWARE, then verify a real save/reload round-trip from the Tab5 UI itself** (not just the host-native test — the spec's DoD item 5 requires this)

**PAUSE FOR HARDWARE:** confirm SD card is present and either NFC or RFID2 unit connected with a real tag to scan-then-save. Wait for confirmation.

- [ ] **Step 6: Commit**

**Model:** Haiku/cheapest tier — mechanical SD I/O following an already-proven in-repo pattern.

---

## Task 11: Amiibo read/write (NTAG215)

**Files:**
- Create: `firmware/tab5/src/features/nfc/nfc_amiibo.h`
- Create: `firmware/tab5/src/features/nfc/nfc_amiibo.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: whichever of `St25r3916::*` / `Ws1850sDriver::*` the implementation confirms can address NTAG215 (Task 11's own first step resolves this — the spec flags it as "confirm during implementation... don't assume without checking").
- Produces: `namespace NfcAmiibo { void register_module(); }`.

**Context:** Port Bruce's `ESP-Amiibolink` (straightforward NTAG215 read/write per the spec, no crypto-attack complexity).

- [ ] **Step 1: Confirm which unit(s) can address NTAG215** (ISO 14443-A/NFC-Forum-Type-2) — quick real-hardware check, likely both units given they're both general 13.56MHz NFC readers, but confirm rather than assume.
- [ ] **Step 2: Port Bruce's Amiibo read/write logic**
- [ ] **Step 3: Build the screen (read result card + write action)**
- [ ] **Step 4: PAUSE FOR HARDWARE, then verify against a real Amiibo/NTAG215 tag**

**PAUSE FOR HARDWARE:** confirm unit connected and a real NTAG215/Amiibo tag available. Wait for confirmation.

- [ ] **Step 5: Commit**

**Model:** Sonnet.

---

## Task 12: SRIX tag tool

**Files:**
- Create: `firmware/tab5/src/features/nfc/nfc_srix.h`
- Create: `firmware/tab5/src/features/nfc/nfc_srix.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Produces: `namespace NfcSrix { void register_module(); }`.

**Context:** ISO 14443-B, low-effort port per the spec ("less common but low-effort port"). Port Bruce's SRIX module directly.

- [ ] **Step 1: Port Bruce's SRIX read/dump logic**
- [ ] **Step 2: Build the screen**
- [ ] **Step 3: PAUSE FOR HARDWARE, then verify against a real SRIX tag if the project owner has one** (if not available, note this in the task report as untested-for-lack-of-hardware rather than silently skipping the checkpoint discipline — a real, disclosed gap, not a fabricated pass)
- [ ] **Step 4: Commit**

**Model:** Haiku/cheapest tier.

---

## Task 13: EMV/APDU reader

**Files:**
- Create: `firmware/tab5/src/features/nfc/nfc_emv_read.h`
- Create: `firmware/tab5/src/features/nfc/nfc_emv_read.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Produces: `namespace NfcEmvRead { void register_module(); }`.

**Context:** Read-only card-data extraction (PAN, expiry, etc. where unencrypted) — no payment/transaction logic, per the spec. Port Bruce's `emv_reader.hpp`/`apdu.cpp`/BER-TLV parsing logic.

- [ ] **Step 1: Port the APDU command layer and BER-TLV parser**
- [ ] **Step 2: Build the result-card screen**
- [ ] **Step 3: PAUSE FOR HARDWARE, then verify against a real contactless payment card (your own)**
- [ ] **Step 4: Commit**

**Model:** Sonnet.

---

## Task 14: 125kHz/T5577/HID Prox — conditional

**Files:** TBD, contingent entirely on Task 3's Step 3 confirmed answer.

**Context:** This task's SCOPE is not fixed yet — it is directly gated on Task 3's real-datasheet confirmation of whether either unit supports 125kHz LF operation at all.
- If Task 3 confirmed NO 125kHz support on either unit: this task's entire deliverable is confirming the spec's DoD item 2 already reflects that ("resolved... implemented or explicitly dropped with a note here" — Task 3 already updates the spec directly, so this task may already be satisfied by Task 3 alone; the controller should verify this before dispatching any implementer for this task, and skip dispatch entirely if so).
- If Task 3 confirmed real 125kHz support: write this task's brief at that point, sized the same way as Task 11 (Amiibo) — port Bruce's `rfid125.cpp` against whichever unit's confirmed capability, with its own hardware-checkpoint-gated verification step.

**Do not dispatch this task's implementation until Task 3 is complete and its answer is known** — this is a real, planned contingency, not a placeholder being deferred out of laziness.

---

## Task 15: IR unit chip identification spike

**Files:**
- Create: `firmware/tab5/src/hal/ir_unit.h` (new HAL interface — IR was never part of Phase 1's Tab5 HAL scope, greenfield per the spec)
- Create: `firmware/tab5/src/hal/ir_unit.cpp`
- Modify: `firmware/tab5/src/main.cpp` (instantiate + wire into `setup()`, same pattern as `nfc_unit`/`rfid2_unit`/`rf433`)

**Interfaces:**
- Produces: `class IIrUnit { public: virtual ~IIrUnit() = default; virtual bool detect() = 0; };` plus a concrete class once the chip is identified (name TBD — this task's own output).

**Context:** The IR unit's exact chip/protocol is completely unknown until it physically arrives (ordered same day as this plan's authoring, 2026-08-18). This task cannot start real implementation before then.

- [ ] **Step 1: PAUSE FOR HARDWARE — do not proceed past this point until the IR unit has physically arrived AND the project owner explicitly confirms it's connected to the Tab5.** This is the first and most important checkpoint in this entire plan's IR section — unlike Tasks 1-14, there is no prior confirmed connector state to build on here at all.

- [ ] **Step 2: Census the external I2C bus**

Reuse `nfc_scan_external_i2c_bus()` (`hal/nfc_pn532.cpp`, already exists) or extend `labelForExternalI2CAddr()` with the new address once found — this immediately tells you the I2C address, and the address alone is often enough to identify the chip family via a web/datasheet lookup (the same technique that resolved the NFC/RFID2 chip identities in Phase 1).

- [ ] **Step 3: Identify the chip from its I2C address + any markings/model info the project owner can read off the physical unit**

Ask the project owner for the unit's product name/model number and any visible chip markings if the address alone doesn't uniquely identify it (M5Stack sells several IR-capable units under different names — do not guess which one this is without checking).

- [ ] **Step 4: Confirm bare presence + (if a public datasheet exists) one real register/command exchange**, mirroring Task 2's `read_chip_id()`-style acceptance test — a real, cited, falsifiable check, not just "I2C ACK'd."

- [ ] **Step 5: Document findings and commit the HAL skeleton**

Write the chip identity, address, and citation into `ir_unit.h`'s header comment, matching `nfc_pn532.cpp`'s established citation style. Commit.

**Model:** Opus — real hardware-identification research under total initial uncertainty, same risk class as Task 2.

---

## Task 16: IR common driver + TV-B-Gone

**Files:**
- Create: `firmware/tab5/src/features/ir/ir_common.h`
- Create: `firmware/tab5/src/features/ir/ir_common.cpp`
- Create: `firmware/tab5/src/features/ir/ir_tvbgone.h`
- Create: `firmware/tab5/src/features/ir/ir_tvbgone.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: whatever concrete class/API Task 15 produced.
- Produces: `namespace IrCommon { bool transmit_raw(const uint16_t *pulse_widths_us, size_t count); }` (or the real shape Task 15's chip's actual transmit API demands — this signature is a starting assumption, not fixed, since it depends entirely on Task 15's findings); `namespace IrTvbGone { void register_module(); void poll(); }`.

**Context:** Simplest IR feature per the spec (static code-database transmit). Port Bruce's `src/modules/ir/` or Poseidon's `ir_tvbgone.cpp` code-database and transmit-loop logic, adapted to Task 15's real transmit API (donor code assumes a bit-banged GPIO LED, not an I2C transceiver — the adaptation layer is `ir_common.cpp`, isolating every other IR feature task from needing to know the physical unit's real protocol).

- [ ] **Step 1: Implement `ir_common.cpp`'s transmit adaptation layer against Task 15's real API**
- [ ] **Step 2: Port the TV-B-Gone code database and per-tick transmit loop**
- [ ] **Step 3: Build the screen**
- [ ] **Step 4: PAUSE FOR HARDWARE, then verify against a real TV**

**PAUSE FOR HARDWARE:** confirm IR unit connected, project owner has a real TV in range and ready to observe. Wait for confirmation.

- [ ] **Step 5: Commit**

**Model:** Sonnet.

---

## Task 17: IR receive/decode/learn

**Files:**
- Create: `firmware/tab5/src/features/ir/ir_learn.h`
- Create: `firmware/tab5/src/features/ir/ir_learn.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `IrCommon::*` (Task 16), extended with whatever real receive API Task 15's chip provides.
- Produces: `namespace IrLearn { struct LearnedCode { uint16_t pulse_widths_us[kMaxPulses]; size_t count; }; void register_module(); void poll(); }` — consumed by Task 18 (clone) and any future IR tag-library-style save.

**Context:** Unlike the original Cardputer-ADV plan (transmit-only, receive unconfirmed), this dedicated unit is receive+transmit by design per the spec (added 2026-08-18) — this row is no longer conditional on confirming receive hardware exists, only on Task 15 having confirmed the real receive protocol. Port Bruce's `IRremoteESP8266`-fork-based decode logic or Poseidon's `ir_learn.cpp`/`ir_learn_decode.cpp`, adapted the same way Task 16 adapted transmit.

- [ ] **Step 1: Implement the receive-capture path against Task 15's real API**
- [ ] **Step 2: Port protocol decode logic (NEC, RC5, etc. — whatever the ported donor code covers)**
- [ ] **Step 3: Build the screen (capture + decoded-result display)**
- [ ] **Step 4: PAUSE FOR HARDWARE, then verify against a real remote**

**PAUSE FOR HARDWARE:** confirm IR unit connected, project owner has a real IR remote ready to point at it. Wait for confirmation.

- [ ] **Step 5: Commit**

**Model:** Opus — protocol-timing-sensitive against a still-freshly-characterized chip from Task 15, matching this project's tiering for timing-sensitive work on newly-bridged hardware.

---

## Task 18: Universal remote / multi-profile clone (Flipper-IRDB)

**Files:**
- Create: `firmware/tab5/src/features/ir/ir_clone.h`
- Create: `firmware/tab5/src/features/ir/ir_clone.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `IrLearn::LearnedCode` (Task 17), `IrCommon::transmit_raw` (Task 16).
- Produces: `namespace IrClone { void register_module(); }`.

**Context:** Port UniGeek's Flipper-IRDB-compatible database/parsing logic per the spec ("most valuable single piece to port here — gives access to a large existing community remote database"). This is largely transport-independent (parsing a known file format into pulse-width sequences) so most of this task's logic is host-testable without hardware, using Task 7's precedent for host-native table-driven tests against real sample `.ir`/Flipper-format files.

- [ ] **Step 1: Port the Flipper-IRDB format parser, with a host-native test against a real sample file**
- [ ] **Step 2: Build the profile-browse-and-send screen**
- [ ] **Step 3: PAUSE FOR HARDWARE, then verify at least one profile against a real device**
- [ ] **Step 4: Commit**

**Model:** Sonnet.

---

## Task 19: IR jammer

**Files:**
- Create: `firmware/tab5/src/features/ir/ir_jammer.h`
- Create: `firmware/tab5/src/features/ir/ir_jammer.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `IrCommon::transmit_raw` (Task 16).
- Produces: `namespace IrJammer { void register_module(); void poll(); }`.

**Context:** Continuous-noise transmit per the spec. Simplest remaining IR task.

- [ ] **Step 1: Implement continuous noise-pattern transmit, poll()-driven with a Stop button (never a blocking loop, per Global Constraints)**
- [ ] **Step 2: PAUSE FOR HARDWARE, then verify real IR noise is observable (e.g. via a phone camera, which can usually see IR LEDs)**
- [ ] **Step 3: Commit**

**Model:** Haiku/cheapest tier.

---

## Task 20: Definition-of-Done bring-up log + Phase 3 documentation

**Files:**
- Create: `docs/phases/phase-3-nfc-rf433-ir.md`
- Modify: `.superpowers/sdd/2026-08-18-phase3-nfc-rf433-ir-plan/progress.md` (the SDD ledger this plan's execution will already be writing to)

**Context:** Per `CLAUDE.md`'s standing process ("every phase ends with documentation, not just a Definition-of-Done check") — this task is both the spec's Section 5 Definition-of-Done walkthrough (every item, pass/fail + notes, against real hardware results already gathered across Tasks 1-19) AND the durable phase-documentation distillation, matching `docs/phases/phase-1-foundation.md`'s established depth and shape (architecture decisions and why, real-hardware findings with root causes, known limitations/deferred work — e.g. if Task 14's 125kHz row was dropped, if any IR feature had to be scoped down based on Task 15's real chip capabilities, if the RF433R pin hypothesis from Task 1 was confirmed or led to a stop-and-report).

- [ ] **Step 1: Walk every Definition-of-Done item from the spec (Section 5, as corrected 2026-08-18) against this plan's actual task results, pass/fail + notes**
- [ ] **Step 2: Write `docs/phases/phase-3-nfc-rf433-ir.md`** covering: the NFC/RFID2 chip-split architecture decision and why (Task 2/3's findings), the RF433R pin resolution (Task 1), the IR unit's actual identity and any scope changes that resulted (Task 15), known limitations/deferred work, build/flash/use instructions for what this phase delivered.
- [ ] **Step 3: Commit**

**Model:** Direct (documentation task, no implementer dispatch needed — matches Phase 1's Task 21 precedent, "controller had full context from the ledger").

---

## Task 21: SD file format interop (.sub / .ir)

> **Numbered non-sequentially, added 2026-08-20 — deliberate, not a numbering mistake** (matches this project's established precedent for Phase 10's non-sequential number, per this file's own header history). Inserted after the original Task 1-20 sequence was drafted, once real-hardware testing surfaced a genuine gap: this plan as originally written never gave RF433 a tag-library equivalent to Task 10's NFC one, and never gave either RF433 or IR a way to load a signal from an arbitrary SD file rather than only a signal captured in the current session. **Sequencing: execute this task after Task 7, before Task 8** (Task 7's own decode logic is a natural building block here, and this is mostly host-testable like Task 7, so it fits before diving into the hardware-heavy remaining tasks) — despite its number.

**Files:**
- Create: `firmware/tab5/src/features/rf433/rf433_sub_format.h` / `.cpp` — Flipper's real "Flipper SubGhz RAW File" text format (`Filetype:`/`Frequency:`/`Preset:`/`Protocol:`/`RAW_Data:` lines) ↔ `Rf433Scan::CapturedSignal`, both directions (read AND write — write matters for real interop, not just consumption).
- Extend: whatever Task 18 built for `.ir` parsing (Flipper-IRDB format) — confirm/extend it to load a single standalone `.ir` file from an arbitrary SD path, not only the bundled community database it was originally scoped for.
- Test: `firmware/tab5/test/test_rf433_sub_format.cpp` — host-native, round-trip (`write` then `read` returns the identical `CapturedSignal`, and a real sample `.sub` file — see Context below — parses into the expected edge sequence).

**Interfaces:**
- Consumes: `Rf433Scan::CapturedSignal`/`EdgeSample` (Task 5), Task 18's `.ir` parser output type (whatever concrete type that task produces).
- Produces: `namespace Rf433SubFormat { bool write(const Rf433Scan::CapturedSignal &sig, const char *path); bool read(const char *path, Rf433Scan::CapturedSignal *out); }` (or the real shape the Flipper format's actual field set demands — this signature is a starting assumption, not fixed, same caveat this plan already uses elsewhere for tasks gated on real-world research); an equivalent standalone-file load path for `.ir`, shape TBD by what Task 18 already exposes.

**Context — read before starting, this task has a real, disclosed research gap:** neither of this project's two donor checkouts (`~/src/unigeek-main`, `~/src/firmware` [Bruce]) contains a real `.sub`/`.ir` sample file or a parser for the raw FILE format itself — both only have protocol/pulse-DECODE logic (UniGeek's `SubGhzDecoders.cpp`, ported from Flipper Zero's `lib/subghz/protocols/`, GPLv3 — see the licensing note below) operating on already-parsed pulse-duration arrays, not the surrounding text-file format that wraps them. Confirmed by a direct search of both checkouts before writing this task. The Flipper file formats themselves are real and are publicly documented by Flipper Devices (their own firmware repository and file-format documentation), but this task's implementer must research and cite that real source directly at implementation time — do not invent field names, delimiters, or the RAW_Data pulse-encoding convention from memory. If no citable real source can be found within reasonable scope, report BLOCKED and disclose the gap (per this plan's established Task 12/SRIX precedent: "a real, disclosed gap, not a fabricated pass") rather than guess at a format that would silently fail to interoperate with real Flipper files or real other tools.

**Licensing note, surfaced for the project owner, not decided here:** UniGeek's `SubGhzDecoders.cpp` (the brand/protocol pulse-decode logic — Came/Nice/Chamberlain/Holtek/Ansonic/etc. — that Task 7 already ports from, separately from this task's file-format work) is itself ported from Flipper Zero firmware and is GPLv3-licensed per its own header comment. This is the first GPLv3-sourced content flowing into this codebase (every other donor port so far has been public-domain or a from-scratch reimplementation citing a datasheet). Whether that's acceptable for this project's overall licensing posture is the project owner's call, not an implementation detail — flag it, do not silently proceed as if it were equivalent to this project's other donor sourcing.

- [ ] **Step 1: Research and cite the real Flipper `.sub` RAW file format**, then implement `Rf433SubFormat::write()`/`read()`.
- [ ] **Step 2: Extend Task 18's `.ir` parser for standalone-file loading, if it doesn't already support an arbitrary path**
- [ ] **Step 3: Write host-native round-trip tests** (write→read identity; and a real sample `.sub` file if one can be sourced/cited — a fresh real capture from this session's own hardware testing, re-encoded to the real Flipper format, is an acceptable fixture if no external sample is available)
- [ ] **Step 4: Run the host-native test suite, expect PASS**
- [ ] **Step 5: PAUSE FOR HARDWARE, then verify a real round-trip from the Tab5 UI itself** — save a real capture in `.sub` format, confirm it's readable, confirm a signal loaded from a `.sub` file (not just a same-session capture) can still drive `Rf433Replay::transmit()` correctly.
- [ ] **Step 6: Commit**

**Model:** Sonnet for the parsing/round-trip logic; escalate to Opus if the real file-format research proves to have genuine ambiguity (e.g. multiple incompatible format revisions in the wild) rather than a single citable spec.

---

## Task 22: Generic SD file browser + wire into RF433/IR replay

> **Also numbered non-sequentially, added 2026-08-20, same reasoning as Task 21.** **Sequencing: execute after Task 21** (needs Task 21's `.sub`/`.ir` load functions to have something real to wire a "load this" action to), **before Task 8.**

**Files:**
- Create: a reusable SD file-browser UI component — exact location/name is this task's own call (e.g. `firmware/tab5/src/ui/file_browser.h`/`.cpp`), generalizing the picker pattern rather than duplicating it per-feature.
- Modify: `firmware/tab5/src/features/rf433/rf433_scan.cpp` / `rf433_replay.cpp` — add a "Load from SD" path alongside the existing "replay a session capture" path.
- Modify: whichever IR feature ends up owning replay/clone (Task 18, `ir_clone.cpp`) — same "browse and load" capability, generalized rather than reimplemented.

**Interfaces:** TBD by implementation, but must follow the established `IStorage::list_files()`/`read_file()` abstraction (`hal/storage_sd.h`) — no new storage abstraction invented.

**Context:** `firmware/tab5/src/features/wifi/wifi_evil_portal.cpp`'s template picker is this codebase's real, proven, already-cited-elsewhere-in-this-plan precedent for "list files via `IStorage`, let the user pick one, load it" — but it is scoped to one fixed folder and one file type. This task generalizes that pattern to arbitrary directories (the user needs to navigate into `/quarky/captures/rf433/`, `/quarky/captures/ir/`, or wherever they've copied external `.sub`/`.ir` files onto the SD card) rather than reinventing file-listing UI from scratch. Read `wifi_evil_portal.cpp`'s picker in full before designing this — the goal is one shared component two features call into, not two similar-but-different pickers.

- [ ] **Step 1: Build the generic file-browser component** (directory listing + navigate-into-subdirectory + pick-a-file, backed by `IStorage`)
- [ ] **Step 2: Wire "Load from SD" into the RF433 Scan/Replay screen**, using Task 21's `Rf433SubFormat::read()`
- [ ] **Step 3: Wire the same capability into the IR clone/replay screen**, using Task 21's extended `.ir` standalone-file loader
- [ ] **Step 4: PAUSE FOR HARDWARE, then verify both** — browse to and load a real `.sub` file, confirm replay works identically to a session-captured signal; browse to and load a real standalone `.ir` file (not from the bundled database), confirm it transmits correctly.
- [ ] **Step 5: Commit**

**Model:** Sonnet.

---

## Task 23: Battery percentage stub — real HAL + status-bar wiring

> **Numbered non-sequentially, added 2026-08-20** — same reasoning as Tasks 21/22. Not NFC/RF433/IR peripheral work; the project owner asked for it to be tacked onto this phase's close-out rather than opened as a separate phase, given it's small, self-contained, and this is a natural point to fold it in. **Sequencing: independent of every other task in this plan (touches unrelated files: `ui/shell.cpp` + a new battery HAL module) — run whenever convenient, but before Task 20**, since Task 20's DoD/documentation pass should cover this fix too.

**Files:**
- Create: `firmware/tab5/src/hal/battery.h` / `.cpp` (new HAL module — battery/power monitoring was never part of Phase 1's HAL scope, greenfield, same situation Task 15 is in for the IR unit).
- Modify: `firmware/tab5/src/ui/shell.cpp` — wire a real periodic update into the status bar instead of the permanent hardcoded stub.
- Modify: `firmware/tab5/src/main.cpp` — instantiate + poll, same pattern as every other HAL instance.

**Context:** Confirmed directly (2026-08-20) that `Shell::build()` (`ui/shell.cpp:70-73`) creates the status bar's battery label with a **permanently hardcoded** `"Battery: --%"` string — `battery_label` is a plain local variable, not even stored the way `Shell::status_bar_` is, and nothing anywhere in the tree ever calls `lv_label_set_text()` on it again. This is not a wiring bug on top of working battery-sense hardware; it is a complete stub — no HAL module, no ADC/fuel-gauge/PMIC read exists anywhere in this codebase for battery state. Confirmed by a repo-wide grep for `battery`/`Battery`/`BAT_` before writing this task: the only other hits are unrelated (BLE battery-service *spoofing* in `ble_sourapple.cpp`/`ble_findmy.cpp`/`ble_fastpair_exploit.cpp` — those fake a battery level in an advertised payload, they don't read this device's own).

**Real hardware unknown, spike-class, matching Task 2/Task 15's risk tier:** the Tab5's actual battery-monitoring hardware (a dedicated fuel-gauge/coulomb-counter IC such as commonly used in M5Stack products, a simple ADC voltage-divider pin, or a PMIC register readable over the existing internal I2C bus alongside the already-discovered IO-expanders) is not yet identified in this codebase. Research and cite the real answer the same way Phase 1 resolved the display panel/touch controller/IO-expander identities and Task 2 resolved the NFC chip identity — M5Stack's own official Tab5 documentation/schematic is the right starting point (the same source class already cited repeatedly in `pins_config.h`), not assumption. Do not invent a register map or ADC scaling formula — if a real, citable source can't be found within reasonable scope, report BLOCKED and disclose the gap rather than guess (this plan's established Task-12/SRIX precedent).

- [ ] **Step 1: Identify the real battery-monitoring hardware** (chip/pin, I2C address if applicable) from a real, citable source (M5Stack's own Tab5 documentation/schematic, cross-checked against any other real source the way this project already double/triple-sources its hardware findings).
- [ ] **Step 2: Implement the HAL read** (`hal/battery.h`/`.cpp`), citing the real register/ADC-scaling source for every constant, matching this project's established citation style (see `st25r3916_driver.cpp`'s header for the depth of citation expected).
- [ ] **Step 3: Wire a real periodic update into the status bar** — store the label the way `Shell::status_bar_` already is (a static member, or an equivalent retrievable handle) so a `poll()`-driven update can reach it; pick a sane refresh interval (battery percentage does not need per-tick updates — every few seconds is plenty, document the choice) and wire it into `main.cpp`'s `loop()` the same way every other HAL/feature poll is.
- [ ] **Step 4: PAUSE FOR HARDWARE, then verify against real battery state** — confirm the displayed percentage is plausible with the Tab5 running on battery, and (if practical) that it actually changes as the battery charges/discharges rather than reading a frozen or trivially-wrong value.
- [ ] **Step 5: Commit**

**Model:** Opus for Step 1 (real hardware-identification research under initial uncertainty, same risk class as Task 2/15) — Sonnet is fine for Steps 2-3 once the chip/interface is known.
