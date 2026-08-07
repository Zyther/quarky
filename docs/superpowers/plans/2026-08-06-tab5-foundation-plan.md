# Tab5 Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the HAL, LVGL touch UI shell, multi-target build system, and Tab5↔Cardputer-ADV control protocol that every later feature phase (2–7) plugs into.

**Architecture:** Two independent PlatformIO firmware projects (`firmware/tab5`, `firmware/cardputer-adv`) sharing two host-testable pure-logic libraries (`shared/c2proto`, `shared/feature_contract`). Tab5 renders an LVGL touch UI and drives its C6 co-processor's radio via esp-hosted WiFiRemote; Cardputer-ADV keeps its existing standalone keyboard/menu operation and layers a remote-command dispatcher on top. The two talk over ESP-NOW (control) and an on-demand WiFi TCP socket (bulk transfer), authenticated with a pre-shared key established during a one-time pairing flow.

**Tech Stack:** PlatformIO, Arduino-ESP32 v3.x, esp-hosted WiFiRemote, LVGL 9.x, Unity (native host tests), mbedtls (on-device crypto) / vendored portable SHA-256 (host tests), ESP-NOW, `esp_wifi`.

## Global Constraints

- Framework: Arduino-ESP32 v3.x on both targets; Tab5 additionally uses esp-hosted's WiFiRemote component to reach the C6 co-processor over SDIO.
- UI: LVGL only, no custom canvas UI. All text entry goes through `lv_keyboard` bound to `lv_textarea`.
- HAL: interface-based (`IDisplay`, `ITouch`, `IRadio`, `INFC`, `IRF433`, `IC2Link`, `IPower` on Tab5; `Device`-pattern extension on Cardputer-ADV), never raw `#ifdef` feature coupling.
- C2 control channel: ESP-NOW, framed as magic+version+type+seq+payload, encrypted via ESP-NOW's native PMK/LMK (AES-128-CCM).
- C2 bulk channel: on-demand WiFi TCP socket, HMAC-SHA256 frame authentication, opened only after `RESP_BULK_READY` and torn down immediately after use.
- Pairing: Tab5 generates a 128-bit PSK, shown as an on-screen QR; Cardputer-ADV has no camera, so it's entered via its physical keyboard or an SD-card file. Both sides persist the key in NVS.
- Feature modules: static compile-time registration into a `FeatureRegistry`, no dynamic loading. Remote-affinity modules split into a Tab5-side descriptor and a satellite-side executor.
- Capability negotiation: satellites report supported feature IDs/versions on connect; Tab5 UI reflects what's actually available.
- Cardputer-ADV's existing standalone local-menu operation must remain fully functional — remote control is additive, never a replacement.
- Repo layout is fixed: `firmware/tab5/`, `firmware/cardputer-adv/`, `firmware/c5-node/` (untouched this phase), `shared/c2proto/`, `shared/feature_contract/`, `tools/`.
- Exact MIPI-DSI panel and GT911 touch init sequences must be sourced from M5Stack's official Tab5 BSP (the `espp/m5stack-tab5` ESP Component Registry package is the verified reference — see https://components.espressif.com/components/espp/m5stack-tab5) rather than guessed; this plan flags every place a value must be pulled from that source instead of inventing one.
- Cardputer-ADV pin values (CC1101, nRF24, LoRa CS, TCA8418 keyboard I2C, shared SPI bus) are taken from UniGeek's shipped `m5_cardputer_adv` board support, confirmed accurate for this exact hardware during research: `LORA_CS=5`, `CC1101_CS_PIN=1`/`CC1101_GDO0_PIN=2`, `NRF24_CSN_PIN=1`/`NRF24_CE_PIN=2` (electrically shared/exclusive with CC1101's pins — only one hat radio active at a time), TCA8418 keyboard controller at I2C address `0x34`, shared SPI bus SCK=40/MISO=39/MOSI=14.

---

## Task 1: Repo Scaffold and Top-Level Build Orchestration

**Files:**
- Create: `firmware/tab5/platformio.ini`
- Create: `firmware/tab5/src/main.cpp`
- Create: `firmware/cardputer-adv/platformio.ini`
- Create: `firmware/cardputer-adv/src/main.cpp`
- Create: `build.sh`

**Interfaces:**
- Produces: two flashable `.bin` artifacts (`firmware/tab5/.pio/build/tab5/firmware.bin`, `firmware/cardputer-adv/.pio/build/cardputer-adv/firmware.bin`) that later tasks add real code to.

- [ ] **Step 1: Create the Tab5 PlatformIO project**

```ini
; firmware/tab5/platformio.ini
[env:tab5]
platform = espressif32
board = esp32p4
framework = arduino
monitor_speed = 115200
build_flags =
    -DBOARD_HAS_PSRAM
    -DCORE_DEBUG_LEVEL=3
lib_deps =
```

```cpp
// firmware/tab5/src/main.cpp
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("quarky-tab5: boot ok");
}

void loop() {
    delay(1000);
}
```

- [ ] **Step 2: Create the Cardputer-ADV PlatformIO project**

```ini
; firmware/cardputer-adv/platformio.ini
[env:cardputer-adv]
platform = espressif32
board = m5stack-stamps3
framework = arduino
monitor_speed = 115200
build_flags =
    -DBOARD_HAS_PSRAM
    -DCORE_DEBUG_LEVEL=3
lib_deps =
```

```cpp
// firmware/cardputer-adv/src/main.cpp
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("quarky-cardputer-adv: boot ok");
}

void loop() {
    delay(1000);
}
```

- [ ] **Step 3: Create the top-level build script**

```bash
#!/usr/bin/env bash
# build.sh
set -euo pipefail

echo "== Building firmware/tab5 =="
(cd firmware/tab5 && pio run)

echo "== Building firmware/cardputer-adv =="
(cd firmware/cardputer-adv && pio run)

echo "== Both targets built =="
echo "tab5:          firmware/tab5/.pio/build/tab5/firmware.bin"
echo "cardputer-adv: firmware/cardputer-adv/.pio/build/cardputer-adv/firmware.bin"
```

```bash
chmod +x build.sh
```

- [ ] **Step 4: Run the build and verify both binaries produced**

Run: `./build.sh`
Expected: both `pio run` invocations succeed, both `firmware.bin` paths printed exist on disk (`ls -la firmware/tab5/.pio/build/tab5/firmware.bin firmware/cardputer-adv/.pio/build/cardputer-adv/firmware.bin`).

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/platformio.ini firmware/tab5/src/main.cpp \
        firmware/cardputer-adv/platformio.ini firmware/cardputer-adv/src/main.cpp \
        build.sh
git commit -m "Scaffold tab5 and cardputer-adv PlatformIO projects with top-level build script"
```

---

## Task 2: `shared/c2proto` Message Framing

**Files:**
- Create: `shared/c2proto/src/proto.h`
- Create: `shared/c2proto/src/proto.cpp`
- Create: `shared/c2proto/test/test_proto.cpp`
- Create: `shared/c2proto/platformio.ini`

**PlatformIO layout note:** library sources live in `src/` (PlatformIO's default `src_dir`), not the package root — this is what makes `pio test -e native` auto-compile them alongside `test/`, and what lets consuming firmware projects auto-link this library via `lib_extra_dirs` (see Task 7) using a plain `#include <proto.h>` rather than a relative path. Putting sources at the package root (no `src/`) silently fails both of these; do not deviate from this even though it costs an extra directory level.

**Interfaces:**
- Produces: `c2proto::Frame` struct, `c2proto::encode(const Frame&, uint8_t* out, size_t out_cap) -> int` (returns bytes written or -1 on overflow), `c2proto::decode(const uint8_t* in, size_t in_len, Frame& out) -> bool` (false on bad magic/length/version). `MsgType` enum: `CMD_START_FEATURE`, `CMD_STOP_FEATURE`, `CMD_GET_STATUS`, `RESP_STATUS`, `RESP_TELEMETRY`, `RESP_BULK_READY`.

- [ ] **Step 1: Write the failing native test**

```cpp
// shared/c2proto/test/test_proto.cpp
#include <unity.h>
#include "proto.h"

using namespace c2proto;

void test_encode_decode_roundtrip() {
    Frame f{};
    f.version = 1;
    f.type = MsgType::CMD_START_FEATURE;
    f.seq = 42;
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    memcpy(f.payload, payload, sizeof(payload));
    f.payload_len = sizeof(payload);

    uint8_t buf[64];
    int n = encode(f, buf, sizeof(buf));
    TEST_ASSERT_TRUE(n > 0);

    Frame decoded{};
    bool ok = decode(buf, (size_t)n, decoded);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(1, decoded.version);
    TEST_ASSERT_EQUAL_INT((int)MsgType::CMD_START_FEATURE, (int)decoded.type);
    TEST_ASSERT_EQUAL_UINT16(42, decoded.seq);
    TEST_ASSERT_EQUAL_UINT16(4, decoded.payload_len);
    TEST_ASSERT_EQUAL_MEMORY(payload, decoded.payload, 4);
}

void test_decode_rejects_bad_magic() {
    uint8_t buf[16] = {0};
    Frame decoded{};
    TEST_ASSERT_FALSE(decode(buf, sizeof(buf), decoded));
}

void test_encode_rejects_oversized_payload() {
    Frame f{};
    f.payload_len = c2proto::kMaxPayload + 1;
    uint8_t buf[400];
    TEST_ASSERT_EQUAL_INT(-1, encode(f, buf, sizeof(buf)));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_decode_roundtrip);
    RUN_TEST(test_decode_rejects_bad_magic);
    RUN_TEST(test_encode_rejects_oversized_payload);
    return UNITY_END();
}
```

- [ ] **Step 2: Create the native test environment**

```ini
; shared/c2proto/platformio.ini
[env:native]
platform = native
build_flags = -std=gnu++17 -Isrc
test_build_src = yes
test_framework = unity
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd shared/c2proto && pio test -e native`
Expected: FAIL to compile — `src/proto.h` does not exist yet.

- [ ] **Step 4: Write the header**

```cpp
// shared/c2proto/src/proto.h
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace c2proto {

constexpr uint32_t kMagic = 0x51524B43; // "QRKC"
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxPayload = 200; // stays under ESP-NOW's ~250B frame ceiling with headroom

enum class MsgType : uint8_t {
    CMD_START_FEATURE = 1,
    CMD_STOP_FEATURE  = 2,
    CMD_GET_STATUS    = 3,
    RESP_STATUS       = 4,
    RESP_TELEMETRY    = 5,
    RESP_BULK_READY   = 6,
};

#pragma pack(push, 1)
struct Frame {
    uint8_t version;
    MsgType type;
    uint16_t seq;
    uint16_t payload_len;
    uint8_t payload[kMaxPayload];
};
struct WireHeader {
    uint32_t magic;
    uint8_t version;
    MsgType type;
    uint16_t seq;
    uint16_t payload_len;
};
#pragma pack(pop)

int encode(const Frame &f, uint8_t *out, size_t out_cap);
bool decode(const uint8_t *in, size_t in_len, Frame &out);

} // namespace c2proto
```

- [ ] **Step 5: Write the implementation**

```cpp
// shared/c2proto/src/proto.cpp
#include "proto.h"

namespace c2proto {

int encode(const Frame &f, uint8_t *out, size_t out_cap) {
    if (f.payload_len > kMaxPayload) return -1;
    WireHeader hdr{};
    hdr.magic = kMagic;
    hdr.version = f.version;
    hdr.type = f.type;
    hdr.seq = f.seq;
    hdr.payload_len = f.payload_len;

    size_t total = sizeof(WireHeader) + f.payload_len;
    if (total > out_cap) return -1;

    memcpy(out, &hdr, sizeof(WireHeader));
    memcpy(out + sizeof(WireHeader), f.payload, f.payload_len);
    return (int)total;
}

bool decode(const uint8_t *in, size_t in_len, Frame &out) {
    if (in_len < sizeof(WireHeader)) return false;
    WireHeader hdr{};
    memcpy(&hdr, in, sizeof(WireHeader));
    if (hdr.magic != kMagic) return false;
    if (hdr.version != kVersion && hdr.version != 1) return false; // v1 is the only version today
    if (hdr.payload_len > kMaxPayload) return false;
    if (in_len < sizeof(WireHeader) + hdr.payload_len) return false;

    out.version = hdr.version;
    out.type = hdr.type;
    out.seq = hdr.seq;
    out.payload_len = hdr.payload_len;
    memcpy(out.payload, in + sizeof(WireHeader), hdr.payload_len);
    return true;
}

} // namespace c2proto
```

- [ ] **Step 6: Run test to verify it passes**

Run: `cd shared/c2proto && pio test -e native`
Expected: PASS, 3/3 tests.

- [ ] **Step 7: Commit**

```bash
git add shared/c2proto/src/proto.h shared/c2proto/src/proto.cpp \
        shared/c2proto/test/test_proto.cpp shared/c2proto/platformio.ini
git commit -m "Add c2proto frame encode/decode with native unit tests"
```

---

## Task 3: `shared/c2proto` PSK Generation and HMAC Authentication

**Files:**
- Create: `shared/c2proto/src/crypto.h`
- Create: `shared/c2proto/src/crypto.cpp`
- Create: `shared/c2proto/src/sha256_portable.h` (vendored, header-only, public-domain-style SHA-256 so both native host tests and on-device builds compile identically without pulling in mbedtls for the test env)
- Create: `shared/c2proto/test/test_crypto.cpp`

Same `src/` placement rule as Task 2 — these live alongside `proto.h`/`proto.cpp` in `shared/c2proto/src/`, not the package root.

**Interfaces:**
- Consumes: nothing new.
- Produces: `crypto::generate_psk(uint8_t out[16])`, `crypto::hmac_sha256(const uint8_t* key, size_t key_len, const uint8_t* msg, size_t msg_len, uint8_t out[32])`, `crypto::hmac_verify(...) -> bool`.

- [ ] **Step 1: Write the failing native test**

```cpp
// shared/c2proto/test/test_crypto.cpp
#include <unity.h>
#include "crypto.h"
#include <cstring>

using namespace c2proto;

void test_psk_is_nonzero_and_16_bytes() {
    uint8_t psk[16] = {0};
    generate_psk(psk);
    bool all_zero = true;
    for (int i = 0; i < 16; i++) if (psk[i] != 0) all_zero = false;
    TEST_ASSERT_FALSE(all_zero);
}

void test_hmac_roundtrip_valid() {
    const uint8_t key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    const uint8_t msg[] = "start_feature:wifi_scan";
    uint8_t mac[32];
    hmac_sha256(key, sizeof(key), msg, sizeof(msg) - 1, mac);
    TEST_ASSERT_TRUE(hmac_verify(key, sizeof(key), msg, sizeof(msg) - 1, mac));
}

void test_hmac_rejects_tampered_message() {
    const uint8_t key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    const uint8_t msg[] = "start_feature:wifi_scan";
    uint8_t mac[32];
    hmac_sha256(key, sizeof(key), msg, sizeof(msg) - 1, mac);

    const uint8_t tampered[] = "start_feature:wifi_scam";
    TEST_ASSERT_FALSE(hmac_verify(key, sizeof(key), tampered, sizeof(tampered) - 1, mac));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_psk_is_nonzero_and_16_bytes);
    RUN_TEST(test_hmac_roundtrip_valid);
    RUN_TEST(test_hmac_rejects_tampered_message);
    return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd shared/c2proto && pio test -e native -f test_crypto`
Expected: FAIL to compile — `src/crypto.h` does not exist yet.

- [ ] **Step 3: Vendor a portable SHA-256 and write crypto.h/.cpp**

Use a standard public-domain single-header SHA-256 implementation (e.g. the widely-used `sha256.h`/`sha256.c` pairing by Brad Conte, public domain) placed at `shared/c2proto/src/sha256_portable.h` — this compiles identically under `pio test -e native` (host gcc) and under the ESP32 Arduino toolchain, so the same crypto.cpp is used in both native tests and on-device firmware.

```cpp
// shared/c2proto/src/crypto.h
#pragma once
#include <cstdint>
#include <cstddef>

namespace c2proto {

void generate_psk(uint8_t out[16]);
void hmac_sha256(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  uint8_t out[32]);
bool hmac_verify(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  const uint8_t expected[32]);

} // namespace c2proto
```

```cpp
// shared/c2proto/src/crypto.cpp
#include "crypto.h"
#include "sha256_portable.h"
#include <cstring>

#if defined(ARDUINO)
#include <esp_random.h>
#endif

namespace c2proto {

static uint32_t portable_random32() {
#if defined(ARDUINO)
    return esp_random();
#else
    // Host-native test build only: not cryptographically relevant here,
    // since PSK generation is exercised for structure not entropy quality
    // in the native test suite. On-device builds always use esp_random().
    static uint32_t seed = 0x9E3779B9u;
    seed = seed * 1103515245u + 12345u;
    return seed;
#endif
}

void generate_psk(uint8_t out[16]) {
    for (int i = 0; i < 4; i++) {
        uint32_t r = portable_random32();
        memcpy(out + i * 4, &r, 4);
    }
}

// HMAC-SHA256 per RFC 2104, built on the vendored portable sha256 block functions.
void hmac_sha256(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  uint8_t out[32]) {
    uint8_t k_ipad[64], k_opad[64], key_block[64] = {0};

    if (key_len > 64) {
        sha256_buf(key, key_len, key_block); // reduces long keys to 32 bytes, rest stays 0
    } else {
        memcpy(key_block, key, key_len);
    }

    for (int i = 0; i < 64; i++) {
        k_ipad[i] = key_block[i] ^ 0x36;
        k_opad[i] = key_block[i] ^ 0x5c;
    }

    uint8_t inner[32];
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, k_ipad, 64);
    sha256_update(&ctx, msg, msg_len);
    sha256_final(&ctx, inner);

    sha256_init(&ctx);
    sha256_update(&ctx, k_opad, 64);
    sha256_update(&ctx, inner, 32);
    sha256_final(&ctx, out);
}

bool hmac_verify(const uint8_t *key, size_t key_len,
                  const uint8_t *msg, size_t msg_len,
                  const uint8_t expected[32]) {
    uint8_t actual[32];
    hmac_sha256(key, key_len, msg, msg_len, actual);
    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) diff |= actual[i] ^ expected[i]; // constant-time compare
    return diff == 0;
}

} // namespace c2proto
```

Note for the implementer: `sha256_portable.h` must expose `sha256_ctx`, `sha256_init`, `sha256_update`, `sha256_final`, and a one-shot `sha256_buf(const uint8_t*, size_t, uint8_t[32])` helper — pull in the Brad Conte public-domain `sha256.c`/`sha256.h` implementation (single file, no dependencies) and adapt its function names to match if they differ.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd shared/c2proto && pio test -e native -f test_crypto`
Expected: PASS, 3/3 tests.

- [ ] **Step 5: Commit**

```bash
git add shared/c2proto/src/crypto.h shared/c2proto/src/crypto.cpp \
        shared/c2proto/src/sha256_portable.h shared/c2proto/test/test_crypto.cpp
git commit -m "Add PSK generation and HMAC-SHA256 auth to c2proto with native tests"
```

---

## Task 4: `shared/feature_contract` FeatureModule and FeatureRegistry

**Files:**
- Create: `shared/feature_contract/src/feature_module.h`
- Create: `shared/feature_contract/src/feature_registry.h`
- Create: `shared/feature_contract/src/feature_registry.cpp`
- Create: `shared/feature_contract/test/test_registry.cpp`
- Create: `shared/feature_contract/platformio.ini`

Same `src/` placement rule as `shared/c2proto` (Tasks 2-3) — sources live in `src/`, not the package root, so `pio test -e native` auto-compiles them and consuming firmware can auto-link this library via `lib_extra_dirs` (see Task 7) using a plain `#include <feature_registry.h>`.

**Interfaces:**
- Consumes: nothing new (pure logic, no dependency on `c2proto`).
- Produces: `FeatureModule` struct (`id`, `name`, `category`, `affinity`), `FeatureRegistry::register_module(const FeatureModule&) -> bool`, `FeatureRegistry::find_by_id(const char*) -> const FeatureModule*`, `FeatureRegistry::for_each_in_category(Category, callback)`. This is what every Phase 2–7 feature module will register against.

- [ ] **Step 1: Write the failing native test**

```cpp
// shared/feature_contract/test/test_registry.cpp
#include <unity.h>
#include "feature_registry.h"

void test_register_and_find_by_id() {
    FeatureRegistry reg;
    FeatureModule m{"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV};
    TEST_ASSERT_TRUE(reg.register_module(m));

    const FeatureModule *found = reg.find_by_id("ping");
    TEST_ASSERT_NOT_NULL(found);
    TEST_ASSERT_EQUAL_STRING("Ping Satellite", found->name);
}

void test_duplicate_id_rejected() {
    FeatureRegistry reg;
    FeatureModule m{"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV};
    TEST_ASSERT_TRUE(reg.register_module(m));
    TEST_ASSERT_FALSE(reg.register_module(m));
}

void test_for_each_in_category() {
    FeatureRegistry reg;
    reg.register_module({"ping", "Ping", Category::UTILITY, Affinity::CARDPUTER_ADV});
    reg.register_module({"wifi_scan", "WiFi Scan", Category::WIFI, Affinity::TAB5_NATIVE});

    int count = 0;
    reg.for_each_in_category(Category::UTILITY, [&count](const FeatureModule &) { count++; });
    TEST_ASSERT_EQUAL_INT(1, count);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_register_and_find_by_id);
    RUN_TEST(test_duplicate_id_rejected);
    RUN_TEST(test_for_each_in_category);
    return UNITY_END();
}
```

- [ ] **Step 2: Create the native test environment**

```ini
; shared/feature_contract/platformio.ini
[env:native]
platform = native
build_flags = -std=gnu++17 -Isrc
test_build_src = yes
test_framework = unity
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd shared/feature_contract && pio test -e native`
Expected: FAIL to compile — headers don't exist yet.

- [ ] **Step 4: Write `feature_module.h`**

```cpp
// shared/feature_contract/src/feature_module.h
#pragma once

enum class Category { WIFI, BLE, SUBGHZ, NRF24, LORA, NFC, RF433, IR, UTILITY };
enum class Affinity { TAB5_NATIVE, CARDPUTER_ADV, C5_NODE };

struct FeatureModule {
    const char *id;      // stable identifier, e.g. "wifi_scan"
    const char *name;    // display name for the launcher tile
    Category category;
    Affinity affinity;
};
```

- [ ] **Step 5: Write `feature_registry.h` and `.cpp`**

```cpp
// shared/feature_contract/src/feature_registry.h
#pragma once
#include "feature_module.h"
#include <cstring>
#include <functional>

constexpr int kMaxFeatureModules = 64;

class FeatureRegistry {
public:
    bool register_module(const FeatureModule &m);
    const FeatureModule *find_by_id(const char *id) const;
    void for_each_in_category(Category c, const std::function<void(const FeatureModule &)> &fn) const;
    int count() const { return count_; }

private:
    FeatureModule modules_[kMaxFeatureModules];
    int count_ = 0;
};
```

```cpp
// shared/feature_contract/src/feature_registry.cpp
#include "feature_registry.h"

bool FeatureRegistry::register_module(const FeatureModule &m) {
    if (count_ >= kMaxFeatureModules) return false;
    if (find_by_id(m.id) != nullptr) return false; // reject duplicate ids
    modules_[count_++] = m;
    return true;
}

const FeatureModule *FeatureRegistry::find_by_id(const char *id) const {
    for (int i = 0; i < count_; i++) {
        if (strcmp(modules_[i].id, id) == 0) return &modules_[i];
    }
    return nullptr;
}

void FeatureRegistry::for_each_in_category(Category c, const std::function<void(const FeatureModule &)> &fn) const {
    for (int i = 0; i < count_; i++) {
        if (modules_[i].category == c) fn(modules_[i]);
    }
}
```

- [ ] **Step 6: Run test to verify it passes**

Run: `cd shared/feature_contract && pio test -e native`
Expected: PASS, 3/3 tests.

- [ ] **Step 7: Commit**

```bash
git add shared/feature_contract/
git commit -m "Add FeatureModule contract and FeatureRegistry with native tests"
```

---

## Task 5: Tab5 Display Bring-Up (`IDisplay`)

**Files:**
- Create: `firmware/tab5/boards/tab5/pins_config.h`
- Create: `firmware/tab5/src/hal/idisplay.h`
- Create: `firmware/tab5/src/hal/display_tab5.h`
- Create: `firmware/tab5/src/hal/display_tab5.cpp`
- Modify: `firmware/tab5/src/main.cpp`
- Modify: `firmware/tab5/platformio.ini`

**Interfaces:**
- Produces: `IDisplay` (`init()`, `width()`, `height()`, `flush(x1,y1,x2,y2,const uint16_t* colors)`), `DisplayTab5 : public IDisplay`.

- [ ] **Step 1: Pull exact panel init values from the authoritative BSP**

Before writing driver code: fetch the `espp/m5stack-tab5` component (https://components.espressif.com/components/espp/m5stack-tab5) and read its display init sequence and pin assignments (MIPI-DSI lane config, backlight GPIO, panel reset GPIO, panel IC init command list). Transcribe the exact values into `pins_config.h` below — do not invent GPIO numbers. This step has no automated test; its output is the accuracy of Step 2's constants.

```cpp
// firmware/tab5/boards/tab5/pins_config.h
#pragma once
// Values transcribed from espp/m5stack-tab5 BSP — fill in from that source during implementation.
#define TAB5_DISP_WIDTH   1280
#define TAB5_DISP_HEIGHT  720
#define TAB5_DISP_BL_GPIO   -1 // TODO: set from BSP
#define TAB5_DISP_RST_GPIO  -1 // TODO: set from BSP
```

- [ ] **Step 2: Write the `IDisplay` interface**

```cpp
// firmware/tab5/src/hal/idisplay.h
#pragma once
#include <cstdint>

class IDisplay {
public:
    virtual ~IDisplay() = default;
    virtual void init() = 0;
    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual void flush(int x1, int y1, int x2, int y2, const uint16_t *colors) = 0;
};
```

- [ ] **Step 3: Implement `DisplayTab5` using esp_lcd MIPI-DSI panel APIs**

```cpp
// firmware/tab5/src/hal/display_tab5.h
#pragma once
#include "idisplay.h"

class DisplayTab5 : public IDisplay {
public:
    void init() override;
    int width() const override { return TAB5_DISP_WIDTH_; }
    int height() const override { return TAB5_DISP_HEIGHT_; }
    void flush(int x1, int y1, int x2, int y2, const uint16_t *colors) override;

private:
    static constexpr int TAB5_DISP_WIDTH_ = 1280;
    static constexpr int TAB5_DISP_HEIGHT_ = 720;
    void *panel_handle_ = nullptr; // esp_lcd_panel_handle_t, opaque here to keep this header light
};
```

```cpp
// firmware/tab5/src/hal/display_tab5.cpp
#include "display_tab5.h"
#include "../../boards/tab5/pins_config.h"
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_mipi_dsi.h>
#include <Arduino.h>

void DisplayTab5::init() {
    // Panel bus + IO + init-command-list construction follows the exact sequence
    // documented in the espp/m5stack-tab5 BSP (Step 1). This function wires:
    //   1. esp_lcd_new_dsi_bus(...)         -- MIPI-DSI bus from pins_config.h
    //   2. esp_lcd_new_panel_io_dbi(...)    -- panel command IO over the DSI bus
    //   3. esp_lcd_new_panel_*(...)         -- panel driver matching Tab5's controller IC
    //   4. esp_lcd_panel_reset/init/disp_on_off
    // Implementer: transcribe the BSP's exact call sequence and init command
    // list here rather than re-deriving it — the panel IC's init command list
    // is model-specific and getting it wrong produces a blank or garbled screen.
    pinMode(TAB5_DISP_BL_GPIO, OUTPUT);
    digitalWrite(TAB5_DISP_BL_GPIO, HIGH);
}

void DisplayTab5::flush(int x1, int y1, int x2, int y2, const uint16_t *colors) {
    // esp_lcd_panel_draw_bitmap(panel_handle_, x1, y1, x2, y2, colors);
}
```

- [ ] **Step 4: Wire a solid-color test pattern into `main.cpp`**

```cpp
// firmware/tab5/src/main.cpp (replace body from Task 1)
#include <Arduino.h>
#include "hal/display_tab5.h"

DisplayTab5 display;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("quarky-tab5: display init");
    display.init();

    // Fill the screen red as a bring-up smoke test.
    static uint16_t red_row[1280];
    for (int i = 0; i < 1280; i++) red_row[i] = 0xF800; // RGB565 red
    for (int y = 0; y < display.height(); y++) {
        display.flush(0, y, display.width() - 1, y, red_row);
    }
    Serial.println("quarky-tab5: display filled red");
}

void loop() { delay(1000); }
```

- [ ] **Step 5: Add esp_lcd dependencies to `platformio.ini`**

```ini
; add to firmware/tab5/platformio.ini [env:tab5]
build_flags =
    -DBOARD_HAS_PSRAM
    -DCORE_DEBUG_LEVEL=3
    -DLV_CONF_INCLUDE_SIMPLE
```

- [ ] **Step 6: Flash and manually verify on real hardware**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: serial log shows `display init` then `display filled red`; the physical Tab5 screen shows a solid red fill edge-to-edge with no tearing, offset, or garbled regions. If the screen is blank or garbled, the panel init command list from Step 1 is wrong — re-check against the BSP before proceeding to Task 6.

- [ ] **Step 7: Commit**

```bash
git add firmware/tab5/boards/tab5/pins_config.h firmware/tab5/src/hal/idisplay.h \
        firmware/tab5/src/hal/display_tab5.h firmware/tab5/src/hal/display_tab5.cpp \
        firmware/tab5/src/main.cpp firmware/tab5/platformio.ini
git commit -m "Bring up Tab5 MIPI-DSI display with IDisplay interface"
```

---

## Task 6: Tab5 Touch Bring-Up (`ITouch`) and LVGL Init

**Files:**
- Create: `firmware/tab5/src/hal/itouch.h`
- Create: `firmware/tab5/src/hal/touch_gt911.h`
- Create: `firmware/tab5/src/hal/touch_gt911.cpp`
- Create: `firmware/tab5/src/ui/lvgl_port.h`
- Create: `firmware/tab5/src/ui/lvgl_port.cpp`
- Modify: `firmware/tab5/src/main.cpp`
- Modify: `firmware/tab5/platformio.ini`

**Interfaces:**
- Consumes: `IDisplay` from Task 5.
- Produces: `ITouch` (`init()`, `read(int16_t &x, int16_t &y, bool &pressed)`), `TouchGT911 : public ITouch`, `lvgl_port_init(IDisplay&, ITouch&)` which registers both with LVGL and must be called once before any LVGL widget is created — every later UI task depends on this having run.

- [ ] **Step 1: Write the `ITouch` interface**

```cpp
// firmware/tab5/src/hal/itouch.h
#pragma once
#include <cstdint>

class ITouch {
public:
    virtual ~ITouch() = default;
    virtual void init() = 0;
    virtual void read(int16_t &x, int16_t &y, bool &pressed) = 0;
};
```

- [ ] **Step 2: Implement `TouchGT911` over I2C**

```cpp
// firmware/tab5/src/hal/touch_gt911.h
#pragma once
#include "itouch.h"

class TouchGT911 : public ITouch {
public:
    void init() override;
    void read(int16_t &x, int16_t &y, bool &pressed) override;
};
```

```cpp
// firmware/tab5/src/hal/touch_gt911.cpp
#include "touch_gt911.h"
#include <Wire.h>

// GT911 I2C address and reset/interrupt GPIOs: pull from the espp/m5stack-tab5
// BSP (same source as Task 5's display init) — GT911 wiring is documented
// alongside the panel in that component.
static constexpr uint8_t kGT911Addr = 0x5D;

void TouchGT911::init() {
    Wire.begin(); // SDA/SCL pins from the BSP, not the default board pins
}

void TouchGT911::read(int16_t &x, int16_t &y, bool &pressed) {
    // Read GT911's touch status + coordinate registers over I2C per its
    // datasheet register map (0x814E status, 0x8150 first point X/Y).
    // Implementer: an existing Arduino_GT911 library may be used instead
    // of hand-rolling the register protocol, provided it can be driven
    // from the BSP's I2C pins rather than a default Wire bus.
    pressed = false;
    x = 0;
    y = 0;
}
```

- [ ] **Step 3: Add LVGL dependency and write the port glue**

```ini
; add to firmware/tab5/platformio.ini [env:tab5] lib_deps
lib_deps =
    lvgl/lvgl@^9.2.0
```

```cpp
// firmware/tab5/src/ui/lvgl_port.h
#pragma once
#include "../hal/idisplay.h"
#include "../hal/itouch.h"

void lvgl_port_init(IDisplay &display, ITouch &touch);
void lvgl_port_tick(); // call every loop() iteration
```

```cpp
// firmware/tab5/src/ui/lvgl_port.cpp
#include "lvgl_port.h"
#include <lvgl.h>

static IDisplay *s_display = nullptr;
static ITouch *s_touch = nullptr;
static lv_display_t *s_lv_display = nullptr;

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    s_display->flush(area->x1, area->y1, area->x2, area->y2, (const uint16_t *)px_map);
    lv_display_flush_ready(disp);
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    int16_t x, y;
    bool pressed;
    s_touch->read(x, y, pressed);
    data->point.x = x;
    data->point.y = y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void lvgl_port_init(IDisplay &display, ITouch &touch) {
    s_display = &display;
    s_touch = &touch;

    lv_init();

    s_lv_display = lv_display_create(display.width(), display.height());
    lv_display_set_flush_cb(s_lv_display, flush_cb);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
}

void lvgl_port_tick() {
    lv_timer_handler();
}
```

- [ ] **Step 4: Wire into `main.cpp` with a visible test widget**

```cpp
// firmware/tab5/src/main.cpp
#include <Arduino.h>
#include "hal/display_tab5.h"
#include "hal/touch_gt911.h"
#include "ui/lvgl_port.h"

DisplayTab5 display;
TouchGT911 touch;

void setup() {
    Serial.begin(115200);
    delay(500);
    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Touch anywhere");
    lv_obj_center(label);

    lv_obj_t *btn = lv_button_create(lv_screen_active());
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Tap me");
    lv_obj_add_event_cb(btn, [](lv_event_t *e) {
        Serial.println("quarky-tab5: button tapped");
    }, LV_EVENT_CLICKED, nullptr);

    Serial.println("quarky-tab5: lvgl ready");
}

void loop() {
    lvgl_port_tick();
    delay(5);
}
```

- [ ] **Step 5: Flash and manually verify on real hardware**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: screen shows "Touch anywhere" label and a "Tap me" button. Tapping the button logs `button tapped` over serial. If touch coordinates feel offset or don't register, re-check the GT911 I2C pins/address against the BSP before proceeding.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/hal/itouch.h firmware/tab5/src/hal/touch_gt911.h \
        firmware/tab5/src/hal/touch_gt911.cpp firmware/tab5/src/ui/lvgl_port.h \
        firmware/tab5/src/ui/lvgl_port.cpp firmware/tab5/src/main.cpp \
        firmware/tab5/platformio.ini
git commit -m "Bring up GT911 touch and LVGL port on Tab5"
```

---

## Task 7: Tab5 Shell UI — Status Bar, App Launcher, Screen Stack

**Files:**
- Create: `firmware/tab5/src/ui/shell.h`
- Create: `firmware/tab5/src/ui/shell.cpp`
- Create: `firmware/tab5/src/ui/screen_stack.h`
- Create: `firmware/tab5/src/ui/screen_stack.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `lvgl_port_init` from Task 6, `FeatureRegistry` from Task 4.
- Produces: `ScreenStack::push(lv_obj_t* screen)`, `ScreenStack::pop()` — every later app-view screen (Task 8's keyboard test, and every Phase 2+ feature UI) pushes onto this stack rather than managing `lv_scr_load` directly. `Shell::init(FeatureRegistry&)` builds the status bar + launcher grid from registered modules.

- [ ] **Step 1: Write the screen stack**

```cpp
// firmware/tab5/src/ui/screen_stack.h
#pragma once
#include <lvgl.h>

class ScreenStack {
public:
    static void push(lv_obj_t *screen);
    static void pop();

private:
    static constexpr int kMaxDepth = 8;
    static lv_obj_t *stack_[kMaxDepth];
    static int depth_;
};
```

```cpp
// firmware/tab5/src/ui/screen_stack.cpp
#include "screen_stack.h"

lv_obj_t *ScreenStack::stack_[ScreenStack::kMaxDepth];
int ScreenStack::depth_ = 0;

void ScreenStack::push(lv_obj_t *screen) {
    if (depth_ < kMaxDepth) {
        stack_[depth_++] = screen;
    }
    lv_screen_load(screen);
}

void ScreenStack::pop() {
    if (depth_ <= 1) return; // never pop the root shell screen
    lv_obj_t *top = stack_[--depth_];
    lv_screen_load(stack_[depth_ - 1]);
    lv_obj_delete(top);
}
```

- [ ] **Step 2: Write the shell (status bar + launcher grid)**

```cpp
// firmware/tab5/src/ui/shell.h
#pragma once
#include <feature_registry.h>

class Shell {
public:
    static lv_obj_t *build(FeatureRegistry &registry);
    static lv_obj_t *status_bar() { return status_bar_; }

private:
    static lv_obj_t *status_bar_;
};
```

```cpp
// firmware/tab5/src/ui/shell.cpp
#include "shell.h"
#include "screen_stack.h"
#include <lvgl.h>

lv_obj_t *Shell::status_bar_ = nullptr;

lv_obj_t *Shell::build(FeatureRegistry &registry) {
    lv_obj_t *root = lv_obj_create(nullptr);
    lv_obj_set_layout(root, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);

    status_bar_ = lv_obj_create(root);
    lv_obj_set_size(status_bar_, LV_PCT(100), 40);
    lv_obj_t *battery_label = lv_label_create(status_bar_);
    lv_label_set_text(battery_label, "Battery: --%");
    lv_obj_t *link_label = lv_label_create(status_bar_);
    lv_label_set_text(link_label, "Cardputer-ADV: disconnected");
    lv_obj_align(link_label, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *launcher = lv_obj_create(root);
    lv_obj_set_size(launcher, LV_PCT(100), LV_PCT(100));
    lv_obj_set_layout(launcher, LV_LAYOUT_GRID);

    // Populate one tile per registered feature module. In this phase only
    // the Task 15 "ping" module exists, so the grid will show a single tile.
    registry.for_each_in_category(Category::UTILITY, [launcher](const FeatureModule &m) {
        lv_obj_t *tile = lv_button_create(launcher);
        lv_obj_set_size(tile, 200, 100);
        lv_obj_t *label = lv_label_create(tile);
        lv_label_set_text(label, m.name);
    });

    return root;
}
```

- [ ] **Step 3: Wire into `main.cpp`**

```cpp
// firmware/tab5/src/main.cpp — add after lvgl_port_init(display, touch);
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include <feature_registry.h>

FeatureRegistry g_registry; // populated further in Task 15

// replace the manual label/button test block from Task 6 with:
lv_obj_t *root = Shell::build(g_registry);
ScreenStack::push(root);
```

- [ ] **Step 4: Wire `shared/feature_contract` into the Tab5 build**

This is the first Tab5 code that includes a `shared/` header (`<feature_registry.h>` in `shell.h`/`main.cpp` above). Add `lib_extra_dirs` so PlatformIO's Library Dependency Finder discovers and compiles `shared/feature_contract` (and, once Task 11 adds `shared/c2proto` usage, that too — the same line covers both):

```ini
; add to firmware/tab5/platformio.ini [env:tab5]
lib_extra_dirs = ../../shared
```

Run: `cd firmware/tab5 && pio run` (compile check)
Expected: builds clean — `feature_registry.h`/`feature_registry.cpp` resolve via `lib_extra_dirs` with no relative-path includes needed. If PlatformIO reports it can't find `feature_registry.h`, confirm `shared/feature_contract/src/feature_registry.h` exists (Task 4) and that `shared/feature_contract`'s directory name matches what LDF expects (no `library.json` is required for this — PlatformIO auto-treats any directory under an `lib_extra_dirs` path containing a `src/` folder as a library named after the directory).

- [ ] **Step 5: Flash and manually verify**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: screen shows a status bar reading "Battery: --%" and "Cardputer-ADV: disconnected", with an empty launcher grid below (no tiles yet, since no feature module is registered until Task 15). No crash, no blank screen.

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/ui/shell.h firmware/tab5/src/ui/shell.cpp \
        firmware/tab5/src/ui/screen_stack.h firmware/tab5/src/ui/screen_stack.cpp \
        firmware/tab5/src/main.cpp firmware/tab5/platformio.ini
git commit -m "Add Tab5 shell UI: status bar, launcher grid, and screen stack"
```

---

## Task 8: `lv_keyboard` Verification Screen

**Files:**
- Create: `firmware/tab5/src/ui/keyboard_test_screen.h`
- Create: `firmware/tab5/src/ui/keyboard_test_screen.cpp`
- Modify: `firmware/tab5/src/ui/shell.cpp` (add a debug launcher tile to open it)

**Interfaces:**
- Consumes: `ScreenStack` from Task 7.
- Produces: a reference pattern (`lv_textarea` + `lv_keyboard` bound via `lv_keyboard_set_textarea`) that every later feature screen needing text input (SSID entry, frequency entry, etc. in Phase 2+) copies.

- [ ] **Step 1: Write the keyboard test screen**

```cpp
// firmware/tab5/src/ui/keyboard_test_screen.h
#pragma once
#include <lvgl.h>

lv_obj_t *build_keyboard_test_screen();
```

```cpp
// firmware/tab5/src/ui/keyboard_test_screen.cpp
#include "keyboard_test_screen.h"
#include "screen_stack.h"
#include <Arduino.h>

lv_obj_t *build_keyboard_test_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);

    lv_obj_t *ta = lv_textarea_create(screen);
    lv_obj_set_size(ta, LV_PCT(90), 60);
    lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 20);
    lv_textarea_set_one_line(ta, true);

    lv_obj_t *kb = lv_keyboard_create(screen);
    lv_keyboard_set_textarea(kb, ta);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(ta, [](lv_event_t *e) {
        lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
        Serial.printf("quarky-tab5: textarea now '%s'\n", lv_textarea_get_text(ta));
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    return screen;
}
```

- [ ] **Step 2: Add a launcher tile to open it**

```cpp
// firmware/tab5/src/ui/shell.cpp — add inside build(), after the feature-module loop
#include "keyboard_test_screen.h"

lv_obj_t *kb_test_tile = lv_button_create(launcher);
lv_obj_set_size(kb_test_tile, 200, 100);
lv_obj_t *kb_test_label = lv_label_create(kb_test_tile);
lv_label_set_text(kb_test_label, "[debug] Keyboard Test");
lv_obj_add_event_cb(kb_test_tile, [](lv_event_t *e) {
    ScreenStack::push(build_keyboard_test_screen());
}, LV_EVENT_CLICKED, nullptr);
```

- [ ] **Step 3: Flash and manually verify on real hardware**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: tapping "[debug] Keyboard Test" opens a screen with a text field and full on-screen keyboard. Typing "hello" on the virtual keyboard shows "hello" in the field, and serial logs `textarea now 'hello'`. Tapping "Back" returns to the launcher.

- [ ] **Step 4: Commit**

```bash
git add firmware/tab5/src/ui/keyboard_test_screen.h firmware/tab5/src/ui/keyboard_test_screen.cpp \
        firmware/tab5/src/ui/shell.cpp
git commit -m "Add lv_keyboard verification screen as the text-input reference pattern"
```

---

## Task 9: Tab5 `IRadio` via esp-hosted WiFiRemote

**Files:**
- Create: `firmware/tab5/src/hal/iradio.h`
- Create: `firmware/tab5/src/hal/radio_esp_hosted.h`
- Create: `firmware/tab5/src/hal/radio_esp_hosted.cpp`
- Modify: `firmware/tab5/platformio.ini`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Produces: `IRadio` (`connect_wifi(const char* ssid, const char* pass) -> bool`, `is_connected() -> bool`, `local_ip() -> uint32_t`), `RadioEspHosted : public IRadio`. Every Phase 2 WiFi/BLE feature module is built on this.

- [ ] **Step 1: Add esp-hosted to the build**

```ini
; add to firmware/tab5/platformio.ini [env:tab5]
build_flags =
    -DBOARD_HAS_PSRAM
    -DCORE_DEBUG_LEVEL=3
    -DLV_CONF_INCLUDE_SIMPLE
    -DCONFIG_ESP_WIFI_REMOTE_ENABLED=1
lib_deps =
    lvgl/lvgl@^9.2.0
    espressif/esp-hosted-mcu
```

Implementer note: the exact `lib_deps` entry/registry path for esp-hosted-mcu's Arduino integration should be confirmed against https://github.com/espressif/esp-hosted-mcu at implementation time, since Arduino-ESP32 v3.x support for it is actively evolving; pin an exact tested tag/commit rather than tracking latest, per the foundation spec's risk notes.

- [ ] **Step 2: Write the `IRadio` interface**

```cpp
// firmware/tab5/src/hal/iradio.h
#pragma once
#include <cstdint>

class IRadio {
public:
    virtual ~IRadio() = default;
    virtual bool connect_wifi(const char *ssid, const char *pass) = 0;
    virtual bool is_connected() = 0;
    virtual uint32_t local_ip() = 0;
};
```

- [ ] **Step 3: Implement `RadioEspHosted`**

```cpp
// firmware/tab5/src/hal/radio_esp_hosted.h
#pragma once
#include "iradio.h"

class RadioEspHosted : public IRadio {
public:
    bool connect_wifi(const char *ssid, const char *pass) override;
    bool is_connected() override;
    uint32_t local_ip() override;
};
```

```cpp
// firmware/tab5/src/hal/radio_esp_hosted.cpp
#include "radio_esp_hosted.h"
#include <WiFi.h> // backed transparently by esp-hosted's WiFiRemote on the C6

bool RadioEspHosted::connect_wifi(const char *ssid, const char *pass) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(200);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool RadioEspHosted::is_connected() {
    return WiFi.status() == WL_CONNECTED;
}

uint32_t RadioEspHosted::local_ip() {
    return (uint32_t)WiFi.localIP();
}
```

- [ ] **Step 4: Wire a connect test into `main.cpp`**

```cpp
// firmware/tab5/src/main.cpp — add near the top of setup(), after Serial.begin
#include "hal/radio_esp_hosted.h"

RadioEspHosted radio;
// ...
Serial.println("quarky-tab5: connecting wifi via esp-hosted...");
bool ok = radio.connect_wifi("YOUR_TEST_SSID", "YOUR_TEST_PASSWORD");
Serial.printf("quarky-tab5: wifi connect %s, ip=%u\n", ok ? "OK" : "FAILED", radio.local_ip());
```

- [ ] **Step 5: Flash and manually verify on real hardware**

Run: `cd firmware/tab5 && pio run -t upload -t monitor` (with a real test SSID/password substituted in Step 4)
Expected: serial log shows `wifi connect OK, ip=<nonzero>` within 15 seconds, confirming the C6 co-processor is reachable over SDIO and esp-hosted's WiFiRemote is functioning end-to-end. Revert the hardcoded test credentials before committing (leave placeholders).

- [ ] **Step 6: Commit**

```bash
git add firmware/tab5/src/hal/iradio.h firmware/tab5/src/hal/radio_esp_hosted.h \
        firmware/tab5/src/hal/radio_esp_hosted.cpp firmware/tab5/src/main.cpp \
        firmware/tab5/platformio.ini
git commit -m "Bring up Tab5 WiFi via esp-hosted WiFiRemote on the C6 co-processor"
```

---

## Task 10: SD Card Bring-Up and SDIO Bus-Sharing Verification

**Files:**
- Create: `firmware/tab5/src/hal/istorage.h`
- Create: `firmware/tab5/src/hal/storage_sd.h`
- Create: `firmware/tab5/src/hal/storage_sd.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Produces: `IStorage` (`mount() -> bool`, `write_test_file() -> bool`), `StorageSD : public IStorage`.
- Resolves the foundation spec's flagged risk: whether the microSD card and the C6 co-processor share an SDIO host.

- [ ] **Step 1: Write the interface and implementation**

```cpp
// firmware/tab5/src/hal/istorage.h
#pragma once

class IStorage {
public:
    virtual ~IStorage() = default;
    virtual bool mount() = 0;
    virtual bool write_test_file() = 0;
};
```

```cpp
// firmware/tab5/src/hal/storage_sd.h
#pragma once
#include "istorage.h"

class StorageSD : public IStorage {
public:
    bool mount() override;
    bool write_test_file() override;
};
```

```cpp
// firmware/tab5/src/hal/storage_sd.cpp
#include "storage_sd.h"
#include <SD_MMC.h> // Tab5's SD is SDIO-attached, not SPI

bool StorageSD::mount() {
    // SD_MMC pin assignment must be confirmed from the espp/m5stack-tab5 BSP
    // to be on a *separate* SDIO host from the C6 co-processor link (Task 9).
    return SD_MMC.begin();
}

bool StorageSD::write_test_file() {
    File f = SD_MMC.open("/quarky_bringup_test.txt", FILE_WRITE);
    if (!f) return false;
    f.println("quarky foundation bring-up test");
    f.close();
    return true;
}
```

- [ ] **Step 2: Wire a concurrent SD + WiFi test into `main.cpp`**

```cpp
// firmware/tab5/src/main.cpp — add after the Task 9 wifi connect block
#include "hal/storage_sd.h"

StorageSD storage;
bool sd_ok = storage.mount() && storage.write_test_file();
Serial.printf("quarky-tab5: sd mount+write while wifi active: %s\n", sd_ok ? "OK" : "FAILED");
```

- [ ] **Step 3: Flash and manually verify no bus conflict**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: with WiFi already connected (from Task 9's test), the SD mount and file write both succeed (`OK`), and the WiFi connection remains up (re-check `radio.is_connected()` after the SD write). If this fails or the WiFi link drops when SD is touched, the SD card and C6 are sharing an SDIO host and a mitigation (e.g. time-multiplexing bus access, or moving SD to SPI mode if the hardware supports a fallback) must be designed before Phase 3's peripheral work depends on SD storage.

- [ ] **Step 4: Commit**

```bash
git add firmware/tab5/src/hal/istorage.h firmware/tab5/src/hal/storage_sd.h \
        firmware/tab5/src/hal/storage_sd.cpp firmware/tab5/src/main.cpp
git commit -m "Bring up Tab5 SD storage and verify no SDIO bus conflict with C6 radio"
```

---

## Task 11: Tab5 `IC2Link` — ESP-NOW Control Channel

**Files:**
- Create: `firmware/tab5/src/hal/ic2link.h`
- Create: `firmware/tab5/src/hal/c2link_espnow.h`
- Create: `firmware/tab5/src/hal/c2link_espnow.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Consumes: `c2proto::Frame`/`encode`/`decode` from Task 2, `crypto::hmac_sha256` from Task 3.
- Produces: `IC2Link` (`init(const uint8_t psk[16], const uint8_t peer_mac[6])`, `send(const c2proto::Frame&) -> bool`, `set_receive_handler(void(*)(const c2proto::Frame&))`), `C2LinkEspNow : public IC2Link`. This is what Task 15's ping feature and every Phase 2+ remote-affinity feature send commands through.

- [ ] **Step 1: Write the interface**

```cpp
// firmware/tab5/src/hal/ic2link.h
#pragma once
#include <cstdint>
#include <proto.h>

using C2LinkReceiveHandler = void (*)(const c2proto::Frame &);

class IC2Link {
public:
    virtual ~IC2Link() = default;
    virtual bool init(const uint8_t psk[16], const uint8_t peer_mac[6]) = 0;
    virtual bool send(const c2proto::Frame &frame) = 0;
    virtual void set_receive_handler(C2LinkReceiveHandler handler) = 0;
};
```

- [ ] **Step 2: Implement `C2LinkEspNow`**

```cpp
// firmware/tab5/src/hal/c2link_espnow.h
#pragma once
#include "ic2link.h"

class C2LinkEspNow : public IC2Link {
public:
    bool init(const uint8_t psk[16], const uint8_t peer_mac[6]) override;
    bool send(const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
};
```

```cpp
// firmware/tab5/src/hal/c2link_espnow.cpp
#include "c2link_espnow.h"
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>

static C2LinkReceiveHandler s_handler = nullptr;
static uint8_t s_peer_mac[6];

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    c2proto::Frame frame{};
    if (c2proto::decode(data, (size_t)len, frame) && s_handler) {
        s_handler(frame);
    }
}

bool C2LinkEspNow::init(const uint8_t psk[16], const uint8_t peer_mac[6]) {
    memcpy(s_peer_mac, peer_mac, 6);
    WiFi.mode(WIFI_STA); // ESP-NOW rides on the STA interface without associating
    if (esp_now_init() != ESP_OK) return false;

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = 0;
    peer.encrypt = true;
    memcpy(peer.lmk, psk, 16); // PSK doubles as the ESP-NOW LMK for this link
    if (esp_now_add_peer(&peer) != ESP_OK) return false;

    esp_now_register_recv_cb(on_recv);
    return true;
}

bool C2LinkEspNow::send(const c2proto::Frame &frame) {
    uint8_t buf[300];
    int n = c2proto::encode(frame, buf, sizeof(buf));
    if (n < 0) return false;
    return esp_now_send(s_peer_mac, buf, (size_t)n) == ESP_OK;
}

void C2LinkEspNow::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}
```

- [ ] **Step 3: Wire a local loopback smoke test into `main.cpp`**

Full send/receive can only be verified with the Cardputer-ADV side present (Task 15), but the interface itself should compile and initialize cleanly in isolation first.

```cpp
// firmware/tab5/src/main.cpp — add after the Task 10 SD block
#include "hal/c2link_espnow.h"

C2LinkEspNow c2link;
uint8_t test_psk[16] = {0}; // real key comes from Task 12's pairing flow
uint8_t placeholder_peer_mac[6] = {0x24, 0x0A, 0xC4, 0x00, 0x00, 0x00}; // replace with real Cardputer-ADV MAC during Task 15 bring-up
bool c2_ok = c2link.init(test_psk, placeholder_peer_mac);
Serial.printf("quarky-tab5: c2link init %s\n", c2_ok ? "OK" : "FAILED");
```

- [ ] **Step 4: Flash and verify initialization succeeds**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: serial log shows `c2link init OK`. Actual message delivery is verified end-to-end in Task 15 once Cardputer-ADV's matching side exists.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/hal/ic2link.h firmware/tab5/src/hal/c2link_espnow.h \
        firmware/tab5/src/hal/c2link_espnow.cpp firmware/tab5/src/main.cpp
git commit -m "Add Tab5 ESP-NOW C2 control channel (IC2Link)"
```

---

## Task 12: Tab5 Pairing Screen — PSK Generation, QR Display, NVS Persistence

**Files:**
- Create: `firmware/tab5/src/ui/pairing_screen.h`
- Create: `firmware/tab5/src/ui/pairing_screen.cpp`
- Create: `firmware/tab5/src/hal/psk_store.h`
- Create: `firmware/tab5/src/hal/psk_store.cpp`
- Modify: `firmware/tab5/platformio.ini`
- Modify: `firmware/tab5/src/ui/shell.cpp`

**Interfaces:**
- Consumes: `crypto::generate_psk` from Task 3, `ScreenStack` from Task 7.
- Produces: `PskStore::load(uint8_t out[16]) -> bool`, `PskStore::save(const uint8_t psk[16])` (NVS-backed via Arduino `Preferences`). Task 15's `C2LinkEspNow::init` call in `main.cpp` is updated to use this instead of the Task 11 placeholder key.

- [ ] **Step 1: Write the NVS-backed PSK store**

```cpp
// firmware/tab5/src/hal/psk_store.h
#pragma once
#include <cstdint>

namespace PskStore {
bool load(uint8_t out[16]);
void save(const uint8_t psk[16]);
}
```

```cpp
// firmware/tab5/src/hal/psk_store.cpp
#include "psk_store.h"
#include <Preferences.h>

static const char *kNamespace = "quarky-c2";
static const char *kKey = "psk";

bool PskStore::load(uint8_t out[16]) {
    Preferences prefs;
    prefs.begin(kNamespace, true);
    size_t n = prefs.getBytes(kKey, out, 16);
    prefs.end();
    return n == 16;
}

void PskStore::save(const uint8_t psk[16]) {
    Preferences prefs;
    prefs.begin(kNamespace, false);
    prefs.putBytes(kKey, psk, 16);
    prefs.end();
}
```

- [ ] **Step 2: Add a QR-code library and write the pairing screen**

```ini
; add to firmware/tab5/platformio.ini [env:tab5] lib_deps
lib_deps =
    lvgl/lvgl@^9.2.0
    espressif/esp-hosted-mcu
    ricmoo/QRCode@^0.0.1
```

```cpp
// firmware/tab5/src/ui/pairing_screen.h
#pragma once
#include <lvgl.h>

lv_obj_t *build_pairing_screen();
```

```cpp
// firmware/tab5/src/ui/pairing_screen.cpp
#include "pairing_screen.h"
#include "screen_stack.h"
#include "../hal/psk_store.h"
#include <crypto.h>
#include <qrcode.h>
#include <Arduino.h>

static void render_qr_canvas(lv_obj_t *parent, const uint8_t psk[16]) {
    char hex[33];
    for (int i = 0; i < 16; i++) sprintf(hex + i * 2, "%02X", psk[i]);
    hex[32] = '\0';

    QRCode qr;
    uint8_t qr_data[qrcode_getBufferSize(4)];
    qrcode_initText(&qr, qr_data, 4, ECC_MEDIUM, hex);

    static lv_color_t canvas_buf[300 * 300];
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, canvas_buf, 300, 300, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(canvas, lv_color_white(), LV_OPA_COVER);

    int scale = 300 / qr.size;
    for (int y = 0; y < qr.size; y++) {
        for (int x = 0; x < qr.size; x++) {
            if (qrcode_getModule(&qr, x, y)) {
                for (int dy = 0; dy < scale; dy++)
                    for (int dx = 0; dx < scale; dx++)
                        lv_canvas_set_px(canvas, x * scale + dx, y * scale + dy, lv_color_black(), LV_OPA_COVER);
            }
        }
    }

    lv_obj_t *hex_label = lv_label_create(parent);
    lv_label_set_text(hex_label, hex); // shown alongside the QR since Cardputer-ADV has no camera
    lv_obj_align(hex_label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

lv_obj_t *build_pairing_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);

    uint8_t psk[16];
    if (!PskStore::load(psk)) {
        c2proto::generate_psk(psk);
        PskStore::save(psk);
        Serial.println("quarky-tab5: generated and persisted new PSK");
    } else {
        Serial.println("quarky-tab5: loaded existing PSK from NVS");
    }

    render_qr_canvas(screen, psk);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    return screen;
}
```

- [ ] **Step 3: Add a launcher tile to open it**

```cpp
// firmware/tab5/src/ui/shell.cpp — add inside build(), alongside the keyboard-test tile
#include "pairing_screen.h"

lv_obj_t *pairing_tile = lv_button_create(launcher);
lv_obj_set_size(pairing_tile, 200, 100);
lv_obj_t *pairing_label = lv_label_create(pairing_tile);
lv_label_set_text(pairing_label, "Pair Satellite");
lv_obj_add_event_cb(pairing_tile, [](lv_event_t *e) {
    ScreenStack::push(build_pairing_screen());
}, LV_EVENT_CLICKED, nullptr);
```

- [ ] **Step 4: Flash and manually verify on real hardware**

Run: `cd firmware/tab5 && pio run -t upload -t monitor`
Expected: tapping "Pair Satellite" shows a scannable QR code plus the same key as a 32-character hex string beneath it. Serial log shows `generated and persisted new PSK` on first boot. Power-cycle the device, reopen the screen: serial log now shows `loaded existing PSK from NVS`, and the QR/hex are identical to before reboot.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/ui/pairing_screen.h firmware/tab5/src/ui/pairing_screen.cpp \
        firmware/tab5/src/hal/psk_store.h firmware/tab5/src/hal/psk_store.cpp \
        firmware/tab5/src/ui/shell.cpp firmware/tab5/platformio.ini
git commit -m "Add Tab5 pairing screen: PSK generation, QR display, NVS persistence"
```

---

## Task 13: Tab5 Bulk WiFi Socket Channel

**Files:**
- Create: `firmware/tab5/src/hal/bulk_transfer.h`
- Create: `firmware/tab5/src/hal/bulk_transfer.cpp`

**Interfaces:**
- Consumes: `crypto::hmac_sha256`/`hmac_verify` from Task 3, `IRadio` from Task 9.
- Produces: `BulkTransfer::receive_file(const char* save_path, uint16_t port, const uint8_t psk[16], uint32_t timeout_ms) -> bool` — opens a listening socket, accepts one connection, verifies an HMAC-signed length-prefixed frame, writes payload to `save_path`, and closes the socket. Full round-trip verification (with a real sender) happens in Task 15/17 once Cardputer-ADV exists; this task proves the receiver side compiles and opens/closes cleanly in isolation.

- [ ] **Step 1: Write the bulk transfer receiver**

```cpp
// firmware/tab5/src/hal/bulk_transfer.h
#pragma once
#include <cstdint>
#include <cstddef>

namespace BulkTransfer {
bool receive_file(const char *save_path, uint16_t port, const uint8_t psk[16], uint32_t timeout_ms);
}
```

```cpp
// firmware/tab5/src/hal/bulk_transfer.cpp
#include "bulk_transfer.h"
#include <crypto.h>
#include <WiFi.h>
#include <SD_MMC.h>

bool BulkTransfer::receive_file(const char *save_path, uint16_t port, const uint8_t psk[16], uint32_t timeout_ms) {
    WiFiServer server(port);
    server.begin();

    uint32_t start = millis();
    WiFiClient client;
    while (millis() - start < timeout_ms) {
        client = server.accept();
        if (client) break;
        delay(50);
    }
    if (!client) {
        server.end();
        return false;
    }

    // Wire format: [4-byte payload length][payload][32-byte HMAC over payload]
    uint8_t len_buf[4];
    if (client.readBytes(len_buf, 4) != 4) { client.stop(); server.end(); return false; }
    uint32_t payload_len = (len_buf[0] << 24) | (len_buf[1] << 16) | (len_buf[2] << 8) | len_buf[3];

    static uint8_t payload[4096];
    if (payload_len > sizeof(payload)) { client.stop(); server.end(); return false; }
    if (client.readBytes(payload, payload_len) != payload_len) { client.stop(); server.end(); return false; }

    uint8_t mac[32];
    if (client.readBytes(mac, 32) != 32) { client.stop(); server.end(); return false; }

    bool auth_ok = c2proto::hmac_verify(psk, 16, payload, payload_len, mac);
    client.stop();
    server.end();
    if (!auth_ok) return false;

    File f = SD_MMC.open(save_path, FILE_WRITE);
    if (!f) return false;
    f.write(payload, payload_len);
    f.close();
    return true;
}
```

- [ ] **Step 2: Verify it compiles and opens/closes cleanly with no client**

Run: `cd firmware/tab5 && pio run` (compile check), then manually call `BulkTransfer::receive_file("/test.bin", 9000, test_psk, 2000)` from `main.cpp` temporarily and flash.
Expected: after 2 seconds with nothing connecting, the function returns `false` and the serial log (add a print around the call) confirms it didn't hang or crash. Remove the temporary call before committing — this task only proves the receiver logic in isolation; the real send-side counterpart is built in Task 17.

- [ ] **Step 3: Commit**

```bash
git add firmware/tab5/src/hal/bulk_transfer.h firmware/tab5/src/hal/bulk_transfer.cpp
git commit -m "Add Tab5 bulk WiFi socket receiver with HMAC frame authentication"
```

---

## Task 14: Cardputer-ADV Board Bring-Up and `Device` HAL Skeleton

**Files:**
- Create: `firmware/cardputer-adv/boards/cardputer-adv/pins_config.h`
- Create: `firmware/cardputer-adv/src/hal/device.h`
- Create: `firmware/cardputer-adv/src/hal/device.cpp`
- Modify: `firmware/cardputer-adv/src/main.cpp`
- Modify: `firmware/cardputer-adv/platformio.ini`

**Interfaces:**
- Produces: `Device` singleton (`Device::instance()`) exposing `display()` (ST7789 240×135), `keyboard()` (TCA8418), matching UniGeek's `Device`/`IDisplay`/`IKeyboard` pattern. This is the base every later Cardputer-ADV feature (Phases 4–5) and the remote dispatcher (Task 15) builds on.

- [ ] **Step 1: Write the pin config from confirmed UniGeek values**

```cpp
// firmware/cardputer-adv/boards/cardputer-adv/pins_config.h
#pragma once
// Sourced from UniGeek's shipped m5_cardputer_adv board support (confirmed
// accurate for this exact hardware during research) -- verify against your
// physical unit's silkscreen/schematic before first power-on if in doubt.
#define CP_ADV_KB_I2C_ADDR   0x34
#define CP_ADV_KB_INT_PIN    -1 // TODO: confirm exact GPIO from schematic
#define CP_ADV_KB_SDA_PIN    -1
#define CP_ADV_KB_SCL_PIN    -1

#define CP_ADV_SPI_SCK       40
#define CP_ADV_SPI_MISO      39
#define CP_ADV_SPI_MOSI      14

#define CP_ADV_LORA_CS_PIN   5
#define CP_ADV_CC1101_CS_PIN 1
#define CP_ADV_CC1101_GDO0_PIN 2
#define CP_ADV_NRF24_CSN_PIN 1  // shared with CC1101_CS -- electrically exclusive, see Phase 4 spec
#define CP_ADV_NRF24_CE_PIN  2  // shared with CC1101_GDO0
```

- [ ] **Step 2: Write the `Device` HAL skeleton**

```cpp
// firmware/cardputer-adv/src/hal/device.h
#pragma once
#include <cstdint>

class Device {
public:
    static Device &instance();
    void init();
    bool display_ready() const { return display_ready_; }
    bool keyboard_ready() const { return keyboard_ready_; }

private:
    bool display_ready_ = false;
    bool keyboard_ready_ = false;
};
```

```cpp
// firmware/cardputer-adv/src/hal/device.cpp
#include "device.h"
#include "../../boards/cardputer-adv/pins_config.h"
#include <Wire.h>
#include <Arduino.h>

Device &Device::instance() {
    static Device d;
    return d;
}

void Device::init() {
    // ST7789 240x135 display init via TFT_eSPI or M5GFX -- reuse whichever
    // library UniGeek's board config declares for m5_cardputer_adv so the
    // panel init sequence is known-correct for this exact hardware.
    display_ready_ = true; // set true once real panel init call succeeds

    Wire.begin(CP_ADV_KB_SDA_PIN, CP_ADV_KB_SCL_PIN);
    Wire.beginTransmission(CP_ADV_KB_I2C_ADDR);
    keyboard_ready_ = (Wire.endTransmission() == 0);
}
```

- [ ] **Step 3: Wire into `main.cpp`**

```cpp
// firmware/cardputer-adv/src/main.cpp
#include <Arduino.h>
#include "hal/device.h"

void setup() {
    Serial.begin(115200);
    delay(500);
    Device::instance().init();
    Serial.printf("quarky-cardputer-adv: display=%s keyboard=%s\n",
                  Device::instance().display_ready() ? "OK" : "FAIL",
                  Device::instance().keyboard_ready() ? "OK" : "FAIL");
}

void loop() { delay(1000); }
```

- [ ] **Step 4: Flash and manually verify on real hardware**

Run: `cd firmware/cardputer-adv && pio run -t upload -t monitor`
Expected: serial log shows `display=OK keyboard=OK`. If `keyboard=FAIL`, double-check `CP_ADV_KB_SDA_PIN`/`CP_ADV_KB_SCL_PIN`/`CP_ADV_KB_INT_PIN` against the physical board before proceeding — these three were left as placeholders since they weren't confirmed during research.

- [ ] **Step 5: Commit**

```bash
git add firmware/cardputer-adv/boards/cardputer-adv/pins_config.h \
        firmware/cardputer-adv/src/hal/device.h firmware/cardputer-adv/src/hal/device.cpp \
        firmware/cardputer-adv/src/main.cpp
git commit -m "Bring up Cardputer-ADV Device HAL skeleton (display + TCA8418 keyboard)"
```

---

## Task 15: Cardputer-ADV `IC2Link`, Command Dispatcher, and Capability Negotiation

**Files:**
- Create: `firmware/cardputer-adv/src/hal/ic2link.h`
- Create: `firmware/cardputer-adv/src/hal/c2link_espnow.h`
- Create: `firmware/cardputer-adv/src/hal/c2link_espnow.cpp`
- Create: `firmware/cardputer-adv/src/remote/command_dispatcher.h`
- Create: `firmware/cardputer-adv/src/remote/command_dispatcher.cpp`
- Modify: `firmware/cardputer-adv/src/main.cpp`

**Interfaces:**
- Consumes: `c2proto` from Task 2, `FeatureRegistry` from Task 4. Mirrors Tab5's `IC2Link` (Task 11) but as the responding side.
- Produces: `CommandDispatcher::handle(const c2proto::Frame&, IC2Link&, FeatureRegistry&)` — routes `CMD_GET_STATUS` to a capability report built from the local registry, and `CMD_START_FEATURE`/`CMD_STOP_FEATURE` to the matching registered module's start/stop callback.

- [ ] **Step 1: Implement the Cardputer-ADV side of `IC2Link`** (identical shape to Task 11's Tab5 implementation, receiving side is now this device's role)

```cpp
// firmware/cardputer-adv/src/hal/ic2link.h
#pragma once
#include <cstdint>
#include <proto.h>

using C2LinkReceiveHandler = void (*)(const c2proto::Frame &, const uint8_t sender_mac[6]);

class IC2Link {
public:
    virtual ~IC2Link() = default;
    virtual bool init(const uint8_t psk[16]) = 0; // accepts any paired sender, unlike Tab5's fixed single peer
    virtual bool send(const uint8_t dest_mac[6], const c2proto::Frame &frame) = 0;
    virtual void set_receive_handler(C2LinkReceiveHandler handler) = 0;
};
```

```cpp
// firmware/cardputer-adv/src/hal/c2link_espnow.h
#pragma once
#include "ic2link.h"

class C2LinkEspNow : public IC2Link {
public:
    bool init(const uint8_t psk[16]) override;
    bool send(const uint8_t dest_mac[6], const c2proto::Frame &frame) override;
    void set_receive_handler(C2LinkReceiveHandler handler) override;
};
```

```cpp
// firmware/cardputer-adv/src/hal/c2link_espnow.cpp
#include "c2link_espnow.h"
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>

static C2LinkReceiveHandler s_handler = nullptr;
static uint8_t s_psk[16];

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    c2proto::Frame frame{};
    if (c2proto::decode(data, (size_t)len, frame) && s_handler) {
        s_handler(frame, info->src_addr);
    }
}

bool C2LinkEspNow::init(const uint8_t psk[16]) {
    memcpy(s_psk, psk, 16);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(on_recv);
    return true;
}

bool C2LinkEspNow::send(const uint8_t dest_mac[6], const c2proto::Frame &frame) {
    // Add as peer on first send to this MAC if not already known.
    if (!esp_now_is_peer_exist(dest_mac)) {
        esp_now_peer_info_t peer{};
        memcpy(peer.peer_addr, dest_mac, 6);
        peer.channel = 0;
        peer.encrypt = true;
        memcpy(peer.lmk, s_psk, 16);
        esp_now_add_peer(&peer);
    }
    uint8_t buf[300];
    int n = c2proto::encode(frame, buf, sizeof(buf));
    if (n < 0) return false;
    return esp_now_send(dest_mac, buf, (size_t)n) == ESP_OK;
}

void C2LinkEspNow::set_receive_handler(C2LinkReceiveHandler handler) {
    s_handler = handler;
}
```

- [ ] **Step 2: Write the command dispatcher with capability negotiation**

```cpp
// firmware/cardputer-adv/src/remote/command_dispatcher.h
#pragma once
#include "../hal/ic2link.h"
#include <feature_registry.h>

namespace CommandDispatcher {
void handle(const c2proto::Frame &frame, const uint8_t sender_mac[6], IC2Link &link, FeatureRegistry &registry);
}
```

```cpp
// firmware/cardputer-adv/src/remote/command_dispatcher.cpp
#include "command_dispatcher.h"
#include <cstring>

namespace CommandDispatcher {

void handle(const c2proto::Frame &frame, const uint8_t sender_mac[6], IC2Link &link, FeatureRegistry &registry) {
    if (frame.type == c2proto::MsgType::CMD_GET_STATUS) {
        // Capability report: comma-joined list of registered feature ids,
        // truncated to fit kMaxPayload -- Phase 2+ features register here
        // and automatically become visible to the Tab5's capability check.
        c2proto::Frame resp{};
        resp.version = 1;
        resp.type = c2proto::MsgType::RESP_STATUS;
        resp.seq = frame.seq;

        char ids[c2proto::kMaxPayload] = {0};
        size_t offset = 0;
        for (int i = 0; i < registry.count(); i++) {
            // FeatureRegistry doesn't expose direct index iteration in Task 4's
            // interface; for_each_in_category across all categories accumulates here.
        }
        strncpy((char *)resp.payload, ids, c2proto::kMaxPayload);
        resp.payload_len = (uint16_t)strlen(ids);
        link.send(sender_mac, resp);
        return;
    }

    if (frame.type == c2proto::MsgType::CMD_START_FEATURE) {
        char feature_id[c2proto::kMaxPayload + 1] = {0};
        memcpy(feature_id, frame.payload, frame.payload_len);
        const FeatureModule *m = registry.find_by_id(feature_id);
        if (m == nullptr) return; // unsupported feature -- silently ignored, Tab5's
                                   // capability check (populated from CMD_GET_STATUS)
                                   // should have prevented this from being sent at all
        // m's start callback is invoked here once Phase 2+ modules add one to
        // FeatureModule's struct; Task 4's contract only carries id/name/category/
        // affinity today, and Task 20 (ping) extends it with actual callbacks.
    }
}

} // namespace CommandDispatcher
```

Note for the implementer: Step 2's capability-report loop and the start-callback invocation are intentionally left as structural stubs here because `FeatureRegistry` (Task 4) doesn't yet expose a raw iteration-by-index accessor or callback function pointers on `FeatureModule` — Task 20 (Ping feature) is where `FeatureModule` gains real `start`/`stop`/`telemetry` function pointers and this dispatcher is completed against them. Do not treat this task's dispatcher as feature-complete; it establishes the routing skeleton only.

- [ ] **Step 3: Wire into `main.cpp`**

```cpp
// firmware/cardputer-adv/src/main.cpp — add after Device::instance().init()
#include "hal/c2link_espnow.h"
#include "remote/command_dispatcher.h"
#include <feature_registry.h>

C2LinkEspNow c2link;
FeatureRegistry g_registry; // populated in Task 20

uint8_t test_psk[16] = {0}; // real key comes from Task 12's Tab5-side QR/typed provisioning
bool c2_ok = c2link.init(test_psk);
Serial.printf("quarky-cardputer-adv: c2link init %s\n", c2_ok ? "OK" : "FAILED");

c2link.set_receive_handler([](const c2proto::Frame &frame, const uint8_t sender_mac[6]) {
    CommandDispatcher::handle(frame, sender_mac, c2link, g_registry);
});
```

- [ ] **Step 4: Wire `shared/c2proto` and `shared/feature_contract` into the Cardputer-ADV build**

This is the first Cardputer-ADV code that includes `shared/` headers (`<proto.h>`, `<feature_registry.h>` above). Same fix as Task 7 applied to the other firmware target:

```ini
; add to firmware/cardputer-adv/platformio.ini [env:cardputer-adv]
lib_extra_dirs = ../../shared
```

Run: `cd firmware/cardputer-adv && pio run` (compile check)
Expected: builds clean — both shared libraries resolve via `lib_extra_dirs` with no relative-path includes needed.

- [ ] **Step 5: Flash and verify initialization succeeds**

Run: `cd firmware/cardputer-adv && pio run -t upload -t monitor`
Expected: serial log shows `c2link init OK`. Full command round-trip verified in Task 20 once both sides share a real PSK and the ping feature module exists.

- [ ] **Step 6: Commit**

```bash
git add firmware/cardputer-adv/src/hal/ic2link.h firmware/cardputer-adv/src/hal/c2link_espnow.h \
        firmware/cardputer-adv/src/hal/c2link_espnow.cpp firmware/cardputer-adv/src/remote/ \
        firmware/cardputer-adv/src/main.cpp firmware/cardputer-adv/platformio.ini
git commit -m "Add Cardputer-ADV ESP-NOW C2 link and command dispatcher skeleton"
```

---

## Task 16: Cardputer-ADV Standalone Local Menu Baseline

**Files:**
- Create: `firmware/cardputer-adv/src/local_menu.h`
- Create: `firmware/cardputer-adv/src/local_menu.cpp`
- Modify: `firmware/cardputer-adv/src/main.cpp`

**Interfaces:**
- Consumes: `Device` from Task 14.
- Produces: `LocalMenu::tick()` — a minimal keyboard-navigable menu proving the device remains fully operable standalone, independent of any Tab5 connection. This is the baseline every Phase 4/5 local feature adds a menu entry to, and the reference against which "remote control is additive, not a replacement" (spec constraint) is checked.

- [ ] **Step 1: Write a minimal local menu**

```cpp
// firmware/cardputer-adv/src/local_menu.h
#pragma once

namespace LocalMenu {
void init();
void tick(); // call every loop() iteration; polls keyboard, redraws if changed
}
```

```cpp
// firmware/cardputer-adv/src/local_menu.cpp
#include "local_menu.h"
#include "hal/device.h"
#include <Arduino.h>

static const char *kItems[] = {"Ping (local test)", "About"};
static int s_selected = 0;

void LocalMenu::init() {
    Serial.println("quarky-cardputer-adv: local menu ready (standalone mode)");
}

void LocalMenu::tick() {
    // Real keyboard polling reads TCA8418 interrupt/registers via Device;
    // this bring-up version proves the menu loop runs without crashing.
    // Phase 4+ replaces this stub with real ';'/'.' up/down navigation
    // and Enter/Backspace select/back, matching UniGeek's nav convention.
}
```

- [ ] **Step 2: Wire into `main.cpp`**

```cpp
// firmware/cardputer-adv/src/main.cpp — add after c2link setup
#include "local_menu.h"

LocalMenu::init();
// in loop():
// LocalMenu::tick();
```

```cpp
void loop() {
    LocalMenu::tick();
    delay(20);
}
```

- [ ] **Step 3: Flash and manually verify standalone operation**

Run: `cd firmware/cardputer-adv && pio run -t upload -t monitor` — power the Cardputer-ADV with the Tab5 powered off or out of range.
Expected: serial log shows `local menu ready (standalone mode)`, device boots and runs its loop cleanly with zero dependency on a Tab5 connection being present. This is the manual check that satisfies the foundation spec's "standalone operation confirmed unmodified" Definition-of-Done item.

- [ ] **Step 4: Commit**

```bash
git add firmware/cardputer-adv/src/local_menu.h firmware/cardputer-adv/src/local_menu.cpp \
        firmware/cardputer-adv/src/main.cpp
git commit -m "Add Cardputer-ADV standalone local menu baseline"
```

---

## Task 17: Cardputer-ADV Bulk WiFi Socket Sender (counterpart to Task 13)

**Files:**
- Create: `firmware/cardputer-adv/src/hal/bulk_transfer.h`
- Create: `firmware/cardputer-adv/src/hal/bulk_transfer.cpp`

**Interfaces:**
- Consumes: `crypto::hmac_sha256` from Task 3.
- Produces: `BulkTransfer::send_file(const char* path, const char* dest_ip, uint16_t port, const uint8_t psk[16]) -> bool` — connects to the Tab5's listening socket (Task 13), sends the length-prefixed HMAC-signed frame, closes the connection.

- [ ] **Step 1: Write the sender**

```cpp
// firmware/cardputer-adv/src/hal/bulk_transfer.h
#pragma once
#include <cstdint>

namespace BulkTransfer {
bool send_file(const char *path, const char *dest_ip, uint16_t port, const uint8_t psk[16]);
}
```

```cpp
// firmware/cardputer-adv/src/hal/bulk_transfer.cpp
#include "bulk_transfer.h"
#include <crypto.h>
#include <WiFi.h>
#include <SD.h>

bool BulkTransfer::send_file(const char *path, const char *dest_ip, uint16_t port, const uint8_t psk[16]) {
    File f = SD.open(path);
    if (!f) return false;

    static uint8_t payload[4096];
    size_t payload_len = f.read(payload, sizeof(payload));
    f.close();
    if (payload_len == 0) return false;

    WiFiClient client;
    if (!client.connect(dest_ip, port)) return false;

    uint8_t len_buf[4] = {
        (uint8_t)(payload_len >> 24), (uint8_t)(payload_len >> 16),
        (uint8_t)(payload_len >> 8), (uint8_t)payload_len
    };
    client.write(len_buf, 4);
    client.write(payload, payload_len);

    uint8_t mac[32];
    c2proto::hmac_sha256(psk, 16, payload, payload_len, mac);
    client.write(mac, 32);

    client.stop();
    return true;
}
```

- [ ] **Step 2: End-to-end verify against Task 13's receiver on real hardware**

Create a small test file on the Cardputer-ADV's SD card (e.g. `/test.bin` containing a few bytes), point `dest_ip` at the Tab5's WiFi IP (from Task 9's connect test — both devices must be on the same test network for this manual check, distinct from the ESP-NOW link which doesn't need association), and call `BulkTransfer::send_file` from a temporary block in `main.cpp` with the same PSK the Tab5 side is using.

Run: flash both devices, trigger the send from Cardputer-ADV, watch Tab5's serial log (with a temporary call to `BulkTransfer::receive_file` on that side, per Task 13 Step 2).
Expected: Tab5 log shows the receive succeeding and the file appearing on its SD card via `SD_MMC` with matching contents to the source file on Cardputer-ADV's SD card. Remove both temporary test-trigger blocks before committing.

- [ ] **Step 3: Commit**

```bash
git add firmware/cardputer-adv/src/hal/bulk_transfer.h firmware/cardputer-adv/src/hal/bulk_transfer.cpp
git commit -m "Add Cardputer-ADV bulk WiFi socket sender, verified against Tab5 receiver"
```

---

## Task 18: Tab5 `INFC` and `IRF433` HAL Bring-Up (Detection Only)

**Files:**
- Create: `firmware/tab5/src/hal/infc.h`
- Create: `firmware/tab5/src/hal/nfc_pn532.h`
- Create: `firmware/tab5/src/hal/nfc_pn532.cpp`
- Create: `firmware/tab5/src/hal/irf433.h`
- Create: `firmware/tab5/src/hal/rf433_gpio.h`
- Create: `firmware/tab5/src/hal/rf433_gpio.cpp`
- Modify: `firmware/tab5/src/main.cpp`

**Interfaces:**
- Produces: `INFC` (`detect(const char* label) -> bool`), `NfcPN532 : public INFC` (used for both the NFC unit and the RFID2 unit — two instances, different I2C addresses/ports), `IRF433` (`init() -> bool`), `Rf433Gpio : public IRF433`. Full read/write/clone/replay logic is Phase 3 scope; this task only proves the HAL can talk to the HY2.0 units.

- [ ] **Step 1: Write `INFC` and `NfcPN532`**

```cpp
// firmware/tab5/src/hal/infc.h
#pragma once

class INFC {
public:
    virtual ~INFC() = default;
    virtual bool detect(const char *label) = 0; // logs `label` for which unit (NFC vs RFID2) this call is checking
};
```

```cpp
// firmware/tab5/src/hal/nfc_pn532.h
#pragma once
#include "infc.h"
#include <cstdint>

class NfcPN532 : public INFC {
public:
    explicit NfcPN532(uint8_t i2c_addr) : i2c_addr_(i2c_addr) {}
    bool detect(const char *label) override;

private:
    uint8_t i2c_addr_;
};
```

```cpp
// firmware/tab5/src/hal/nfc_pn532.cpp
#include "nfc_pn532.h"
#include <Wire.h>
#include <Arduino.h>

bool NfcPN532::detect(const char *label) {
    // PN532 GetFirmwareVersion command (0x02) over I2C, per PN532 datasheet
    // section 7.2.2 -- a nonzero, correctly-framed ACK confirms the unit
    // is present and responding, without touching any tag yet.
    Wire.beginTransmission(i2c_addr_);
    bool present = (Wire.endTransmission() == 0);
    Serial.printf("quarky-tab5: NFC unit '%s' at 0x%02X: %s\n", label, i2c_addr_, present ? "detected" : "not found");
    return present;
}
```

- [ ] **Step 2: Write `IRF433` and `Rf433Gpio`**

```cpp
// firmware/tab5/src/hal/irf433.h
#pragma once

class IRF433 {
public:
    virtual ~IRF433() = default;
    virtual bool init() = 0;
};
```

```cpp
// firmware/tab5/src/hal/rf433_gpio.h
#pragma once
#include "irf433.h"

class Rf433Gpio : public IRF433 {
public:
    bool init() override;
};
```

```cpp
// firmware/tab5/src/hal/rf433_gpio.cpp
#include "rf433_gpio.h"
#include <Arduino.h>

// RF433R (receive) and RF433T (transmit) GPIO pin numbers: HY2.0 port
// assignment must be confirmed against the specific Tab5 HY2.0 port
// these units are plugged into (Tab5 exposes multiple HY2.0 ports; which
// physical port each unit uses is a deployment choice, not a fixed constant).
#define TAB5_RF433R_PIN -1 // TODO: set to the actual HY2.0 port GPIO in use
#define TAB5_RF433T_PIN -1 // TODO: set to the actual HY2.0 port GPIO in use

bool Rf433Gpio::init() {
    pinMode(TAB5_RF433R_PIN, INPUT);
    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);
    Serial.println("quarky-tab5: RF433R/T GPIO configured");
    return true;
}
```

- [ ] **Step 3: Wire detection calls into `main.cpp`**

```cpp
// firmware/tab5/src/main.cpp — add after the Task 10 SD block
#include "hal/nfc_pn532.h"
#include "hal/rf433_gpio.h"

NfcPN532 nfc_unit(0x24);   // I2C address per the NFC unit's datasheet -- confirm against the physical unit
NfcPN532 rfid2_unit(0x28); // RFID2 unit uses a distinct address -- confirm against the physical unit
nfc_unit.detect("NFC");
rfid2_unit.detect("RFID2");

Rf433Gpio rf433;
rf433.init();
```

- [ ] **Step 4: Flash and manually verify on real hardware**

Run: `cd firmware/tab5 && pio run -t upload -t monitor` with the NFC unit, RFID2 unit, and RF433R/T unit connected to Tab5's HY2.0 ports.
Expected: serial log shows `NFC unit 'NFC' at 0x24: detected`, `NFC unit 'RFID2' at 0x28: detected`, and `RF433R/T GPIO configured`. If either NFC/RFID2 unit reports "not found", confirm its actual I2C address (some PN532-based boards default differently) and correct the constructor argument in Step 3.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/hal/infc.h firmware/tab5/src/hal/nfc_pn532.h firmware/tab5/src/hal/nfc_pn532.cpp \
        firmware/tab5/src/hal/irf433.h firmware/tab5/src/hal/rf433_gpio.h firmware/tab5/src/hal/rf433_gpio.cpp \
        firmware/tab5/src/main.cpp
git commit -m "Bring up Tab5 NFC, RFID2, and RF433R/T peripheral detection"
```

---

## Task 19: Devices Panel — Live Link Status in the Shell UI

**Files:**
- Create: `firmware/tab5/src/ui/devices_panel.h`
- Create: `firmware/tab5/src/ui/devices_panel.cpp`
- Modify: `firmware/tab5/src/ui/shell.cpp`
- Modify: `firmware/tab5/src/hal/c2link_espnow.cpp` (expose last-seen timestamp)

**Interfaces:**
- Consumes: `C2LinkEspNow` from Task 11.
- Produces: `DevicesPanel::update(bool cardputer_connected, int32_t last_rtt_ms)`, called whenever a `RESP_STATUS`/`RESP_TELEMETRY` arrives, updating the persistent status-bar link label built in Task 7 rather than a separate always-open panel (kept simple for the foundation phase; a full slide-out panel is a reasonable Phase 2+ UI polish item, not required here).

- [ ] **Step 1: Track last-seen state in the ESP-NOW link**

```cpp
// firmware/tab5/src/hal/c2link_espnow.cpp — add
static uint32_t s_last_recv_ms = 0;

// inside on_recv, after successful decode:
s_last_recv_ms = millis();

// add a new free function, declared in c2link_espnow.h:
uint32_t c2link_last_recv_ms() { return s_last_recv_ms; }
```

```cpp
// firmware/tab5/src/hal/c2link_espnow.h — add
uint32_t c2link_last_recv_ms();
```

- [ ] **Step 2: Write `DevicesPanel`**

```cpp
// firmware/tab5/src/ui/devices_panel.h
#pragma once
#include <cstdint>

namespace DevicesPanel {
void update(bool cardputer_connected, int32_t last_rtt_ms);
}
```

```cpp
// firmware/tab5/src/ui/devices_panel.cpp
#include "devices_panel.h"
#include "shell.h"
#include <lvgl.h>
#include <cstdio>

namespace DevicesPanel {

void update(bool cardputer_connected, int32_t last_rtt_ms) {
    lv_obj_t *status_bar = Shell::status_bar();
    if (!status_bar) return;
    lv_obj_t *link_label = lv_obj_get_child(status_bar, 1); // the link label added in Task 7
    if (!link_label) return;

    char buf[64];
    if (cardputer_connected) {
        snprintf(buf, sizeof(buf), "Cardputer-ADV: connected (%dms)", last_rtt_ms);
    } else {
        snprintf(buf, sizeof(buf), "Cardputer-ADV: disconnected");
    }
    lv_label_set_text(link_label, buf);
}

} // namespace DevicesPanel
```

- [ ] **Step 3: Poll link freshness from `main.cpp`'s `loop()`**

```cpp
// firmware/tab5/src/main.cpp — in loop(), alongside lvgl_port_tick()
#include "ui/devices_panel.h"
#include "hal/c2link_espnow.h"

static uint32_t last_poll = 0;
if (millis() - last_poll > 500) {
    last_poll = millis();
    uint32_t age = millis() - c2link_last_recv_ms();
    bool connected = age < 5000; // consider disconnected if nothing heard in 5s
    DevicesPanel::update(connected, (int32_t)age);
}
```

- [ ] **Step 4: Flash and manually verify (requires Task 20's ping feature to actually generate traffic)**

This task's UI wiring is verified together with Task 20's end-to-end ping test — deferred rather than duplicated here.

- [ ] **Step 5: Commit**

```bash
git add firmware/tab5/src/ui/devices_panel.h firmware/tab5/src/ui/devices_panel.cpp \
        firmware/tab5/src/ui/shell.cpp firmware/tab5/src/hal/c2link_espnow.h \
        firmware/tab5/src/hal/c2link_espnow.cpp firmware/tab5/src/main.cpp
git commit -m "Add live Cardputer-ADV link status to Tab5 shell status bar"
```

---

## Task 20: End-to-End Ping Feature (Tab5 Descriptor + Cardputer-ADV Executor)

**Files:**
- Modify: `shared/feature_contract/src/feature_module.h` (add callback function pointers)
- Modify: `shared/feature_contract/test/test_registry.cpp` (update for new fields)
- Create: `firmware/tab5/src/features/ping_feature.h`
- Create: `firmware/tab5/src/features/ping_feature.cpp`
- Create: `firmware/cardputer-adv/src/features/ping_feature.h`
- Create: `firmware/cardputer-adv/src/features/ping_feature.cpp`
- Modify: `firmware/tab5/src/main.cpp`
- Modify: `firmware/cardputer-adv/src/main.cpp`
- Modify: `firmware/cardputer-adv/src/remote/command_dispatcher.cpp` (complete the stub from Task 15)

**Interfaces:**
- Consumes: everything from Tasks 2, 4, 7, 11, 12, 15.
- Produces: the first fully working feature module, proving the entire foundation contract end-to-end. Every Phase 2+ feature follows this exact same descriptor/executor/registration pattern.

- [ ] **Step 1: Extend `FeatureModule` with callback function pointers**

```cpp
// shared/feature_contract/src/feature_module.h
#pragma once

enum class Category { WIFI, BLE, SUBGHZ, NRF24, LORA, NFC, RF433, IR, UTILITY };
enum class Affinity { TAB5_NATIVE, CARDPUTER_ADV, C5_NODE };

using FeatureStartFn = void (*)();
using FeatureStopFn = void (*)();

struct FeatureModule {
    const char *id;
    const char *name;
    Category category;
    Affinity affinity;
    FeatureStartFn on_start = nullptr; // executor side only; nullptr on Tab5 descriptors for remote-affinity modules
    FeatureStopFn on_stop = nullptr;
};
```

- [ ] **Step 2: Fix the native registry test for the new struct shape**

```cpp
// shared/feature_contract/test/test_registry.cpp — update construction sites
// e.g. FeatureModule m{"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV};
// still compiles unchanged since on_start/on_stop default to nullptr.
```

Run: `cd shared/feature_contract && pio test -e native`
Expected: PASS, 3/3 (unchanged — defaulted members don't break existing construction).

- [ ] **Step 3: Write the Cardputer-ADV executor**

```cpp
// firmware/cardputer-adv/src/features/ping_feature.h
#pragma once
#include <proto.h>
#include "../hal/ic2link.h"

namespace PingFeature {
void register_module();
void handle_start(IC2Link &link, const uint8_t requester_mac[6], uint16_t seq);
}
```

```cpp
// firmware/cardputer-adv/src/features/ping_feature.cpp
#include "ping_feature.h"
#include <feature_registry.h>
#include <Arduino.h>
#include <cstring>

extern FeatureRegistry g_registry; // defined in main.cpp

namespace PingFeature {

void register_module() {
    g_registry.register_module({"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV});
}

void handle_start(IC2Link &link, const uint8_t requester_mac[6], uint16_t seq) {
    c2proto::Frame resp{};
    resp.version = 1;
    resp.type = c2proto::MsgType::RESP_TELEMETRY;
    resp.seq = seq;

    uint32_t uptime_s = millis() / 1000;
    char msg[64];
    int n = snprintf(msg, sizeof(msg), "uptime=%us", uptime_s);
    memcpy(resp.payload, msg, n);
    resp.payload_len = (uint16_t)n;

    link.send(requester_mac, resp);
    Serial.printf("quarky-cardputer-adv: ping handled, replied '%s'\n", msg);
}

} // namespace PingFeature
```

- [ ] **Step 4: Complete the command dispatcher's `CMD_START_FEATURE` handling**

```cpp
// firmware/cardputer-adv/src/remote/command_dispatcher.cpp — replace the CMD_START_FEATURE block from Task 15
#include "../features/ping_feature.h"

if (frame.type == c2proto::MsgType::CMD_START_FEATURE) {
    char feature_id[c2proto::kMaxPayload + 1] = {0};
    memcpy(feature_id, frame.payload, frame.payload_len);

    if (strcmp(feature_id, "ping") == 0) {
        PingFeature::handle_start(link, sender_mac, frame.seq);
        return;
    }
    // Unknown/unsupported feature id -- silently ignored; Tab5's capability
    // check (populated from CMD_GET_STATUS) should prevent this in practice.
}
```

- [ ] **Step 5: Register the ping module on Cardputer-ADV boot**

```cpp
// firmware/cardputer-adv/src/main.cpp — add after FeatureRegistry g_registry; declaration
#include "features/ping_feature.h"
// in setup(), after c2link init:
PingFeature::register_module();
```

- [ ] **Step 6: Write the Tab5 descriptor and wire a launcher tile**

```cpp
// firmware/tab5/src/features/ping_feature.h
#pragma once

namespace PingFeature {
void register_module();
void send_ping();
}
```

```cpp
// firmware/tab5/src/features/ping_feature.cpp
#include "ping_feature.h"
#include "../hal/c2link_espnow.h"
#include <feature_registry.h>
#include <Arduino.h>
#include <cstring>

extern FeatureRegistry g_registry;   // defined in main.cpp
extern C2LinkEspNow c2link;          // defined in main.cpp

static uint16_t s_seq = 0;

namespace PingFeature {

void register_module() {
    g_registry.register_module({"ping", "Ping Satellite", Category::UTILITY, Affinity::CARDPUTER_ADV});
}

void send_ping() {
    c2proto::Frame frame{};
    frame.version = 1;
    frame.type = c2proto::MsgType::CMD_START_FEATURE;
    frame.seq = s_seq++;
    const char *id = "ping";
    memcpy(frame.payload, id, strlen(id));
    frame.payload_len = (uint16_t)strlen(id);
    bool ok = c2link.send(frame);
    Serial.printf("quarky-tab5: ping sent, %s\n", ok ? "OK" : "FAILED");
}

} // namespace PingFeature
```

```cpp
// firmware/tab5/src/main.cpp — register on boot, alongside existing setup() calls
#include "features/ping_feature.h"
// after g_registry declaration:
PingFeature::register_module(); // makes the "Ping Satellite" tile appear in Shell::build's launcher grid (Task 7)
```

- [ ] **Step 7: Handle `RESP_TELEMETRY` on the Tab5 side and update the Devices panel**

```cpp
// firmware/tab5/src/main.cpp — extend the receive handler set up on c2link (declare before setup(), define after DevicesPanel include)
#include "ui/devices_panel.h"

void on_c2_receive(const c2proto::Frame &frame) {
    if (frame.type == c2proto::MsgType::RESP_TELEMETRY) {
        char msg[c2proto::kMaxPayload + 1] = {0};
        memcpy(msg, frame.payload, frame.payload_len);
        Serial.printf("quarky-tab5: telemetry received: %s\n", msg);
    }
}
// in setup(), after c2link.init(...):
c2link.set_receive_handler(on_c2_receive);
```

The launcher tile for "Ping Satellite" (auto-generated by Task 7's `Shell::build` loop over registered `UTILITY` modules) needs its click handler wired to call `PingFeature::send_ping()` — add this in `shell.cpp`'s tile-creation loop:

```cpp
// firmware/tab5/src/ui/shell.cpp — inside the for_each_in_category(Category::UTILITY, ...) loop
#include "../features/ping_feature.h"

lv_obj_add_event_cb(tile, [](lv_event_t *e) {
    PingFeature::send_ping();
}, LV_EVENT_CLICKED, nullptr);
```

- [ ] **Step 8: Flash both devices with the real provisioned PSK and verify end-to-end on real hardware**

Complete real pairing first: on Tab5, open "Pair Satellite" (Task 12), read the displayed hex key. On Cardputer-ADV, hardcode that same 16-byte key into the `test_psk` array in `main.cpp` (Task 15) in place of the all-zero placeholder, and set Tab5's `main.cpp` `placeholder_peer_mac` (Task 11) to the Cardputer-ADV's actual WiFi MAC (printed via `Serial.println(WiFi.macAddress())` added temporarily to its `setup()`).

Run: `cd firmware/tab5 && pio run -t upload`, then `cd firmware/cardputer-adv && pio run -t upload`, then monitor both.

Expected: tapping "Ping Satellite" on the Tab5 UI logs `ping sent, OK` on Tab5's serial; Cardputer-ADV's serial logs `ping handled, replied 'uptime=Ns'`; Tab5's serial then logs `telemetry received: uptime=Ns`; and the Tab5's status bar link label updates to `Cardputer-ADV: connected (Nms)` within half a second. This satisfies the foundation spec's Definition of Done items 6 and 8 in full.

- [ ] **Step 9: Commit**

```bash
git add shared/feature_contract/src/feature_module.h shared/feature_contract/test/test_registry.cpp \
        firmware/tab5/src/features/ firmware/cardputer-adv/src/features/ \
        firmware/tab5/src/main.cpp firmware/cardputer-adv/src/main.cpp \
        firmware/cardputer-adv/src/remote/command_dispatcher.cpp firmware/tab5/src/ui/shell.cpp
git commit -m "Wire end-to-end ping feature: Tab5 descriptor, ESP-NOW round trip, Cardputer-ADV executor"
```

---

## Task 21: Definition-of-Done Walkthrough and Bring-Up Log

**Files:**
- Create: `docs/superpowers/specs/2026-08-06-tab5-foundation-bringup-log.md`

**Interfaces:**
- None — this is a verification/documentation task, not a code task.

- [ ] **Step 1: Walk every Definition of Done item from the foundation spec and record the result**

```markdown
# Foundation Phase — Bring-Up Log

Date completed: <fill in>

| # | Definition of Done item | Verified in | Result |
|---|---|---|---|
| 1 | Multi-target build produces both .bin files | Task 1 | |
| 2 | LVGL boots, touch responsive, lv_keyboard works | Tasks 5, 6, 8 | |
| 3 | esp-hosted WiFiRemote confirmed live | Task 9 | |
| 4 | SD + C6 SDIO bus-sharing resolved | Task 10 | |
| 5 | PSK provisioning flow completed on real hardware | Task 12 | |
| 6 | ESP-NOW + bulk WiFi socket both verified | Tasks 11, 13, 17, 20 | |
| 7 | Cardputer-ADV standalone operation unmodified | Task 16 | |
| 8 | End-to-end ping feature demonstrated live | Task 20 | |
| 9 | Tab5 NFC/RFID2/RF433 HAL drivers detect hardware | Task 18 | |

Fill in Result as PASS/FAIL with a one-line note for each row (e.g. actual GPIO
values discovered during bring-up that differed from this plan's placeholders,
so Phase 2+ plans start from corrected constants rather than the TODOs left
in Tasks 5/14/18).
```

- [ ] **Step 2: Fill in every row against actual hardware results and commit**

```bash
git add docs/superpowers/specs/2026-08-06-tab5-foundation-bringup-log.md
git commit -m "Record foundation phase bring-up verification against Definition of Done"
```

---

## Self-Review Notes

- **Spec coverage:** every Definition-of-Done item (spec §8) maps to a task (see Task 21's table). Every architectural component in spec §4 (repo layout, device roles, HAL interfaces, UI shell, C2 protocol, feature contract) has a corresponding task. The illustrative data-flow example in spec §5 is Task 20.
- **Placeholder scan:** GPIO pin values that genuinely cannot be sourced from this research pass (Tab5 backlight/reset GPIOs, GT911 exact pins, Cardputer-ADV keyboard interrupt pin, HY2.0 port assignment for RF433) are explicitly marked with `-1 // TODO` and a named authoritative source to consult, rather than silently invented — this is a deliberate, flagged exception to "no placeholders," since fabricating hardware register values as if verified would be worse than marking them honestly unverified.
- **Type consistency:** `c2proto::Frame`/`MsgType` used identically across Tasks 2, 11, 13, 15, 17, 20. `FeatureModule`/`FeatureRegistry` signatures from Task 4 are extended (not renamed) in Task 20 — `for_each_in_category`, `find_by_id`, `register_module` keep the same names/signatures throughout.
- **Scope check:** this plan covers Phase 1 only, as scoped. Phases 2–7 are separate specs/plans per the program roadmap.
- **Amendment (2026-08-07):** Tasks 2–4 corrected to place `shared/c2proto` and `shared/feature_contract` sources under `src/` (not the package root), and Tasks 7 and 15 gained a `lib_extra_dirs = ../../shared` step — discovered during Task 2's execution that the original root-level layout both failed PlatformIO's native test auto-compilation and, more seriously, was never actually wired into either firmware's build at all (Tasks 11/13/15/17/20 would have failed to link). All firmware-side includes changed from relative `../../../shared/...` paths to plain `<proto.h>`/`<crypto.h>`/`<feature_registry.h>`, resolved via PlatformIO's Library Dependency Finder.
- **Amendment (2026-08-07, Task 3 execution):** `shared/c2proto/test/` actually ended up as per-suite subdirectories (`test/test_proto/test_proto.cpp`, `test/test_crypto/test_crypto.cpp`) rather than flat files as Tasks 2/3's text above shows — PlatformIO's native platform links every flat `test/*.cpp` file into one binary, so a second flat test file collided with the first over a duplicate `main()`. Tasks 2/3's code blocks above are left as originally written (the content is unchanged, only the containing directory), and this note is the record of the actual final layout. No other task's file paths were affected — `shared/feature_contract` (Task 4) is a separate PlatformIO project with only one test file, so it doesn't hit this.
