// Pure format/storage half of the Flipper ".nfc" exporter -- deliberately
// free of Arduino/LVGL/FeatureRegistry dependencies so it builds for
// `pio test -e native`, exactly like rf433_sub_format.cpp (Task 21) and
// nfc_tag_library.cpp (Task 10) before it. Every real citation for the file
// format itself lives in nfc_flipper_format.h's header comment; the comments
// below cite line-by-line only where a specific decision needs it.
#include "nfc_flipper_format.h"

#include <cstdio>
#include <cstring>

namespace NfcFlipperFormat {

const char kNfcDir[] = "/quarky/captures/nfc";
const char kDeviceTypeIso14443_4a[] = "ISO14443-4A";

namespace {

// [SOURCE] flipper_format_stream_i.h:4-7 -- ':' delimiter, '#' comment,
// '\n' EOL. Every literal below is one of the real key strings cited in this
// module's header comment, not a paraphrase.
constexpr char kKeyFiletype[]   = "Filetype";
constexpr char kKeyVersion[]    = "Version";
constexpr char kKeyDeviceType[] = "Device type";
constexpr char kKeyUid[]        = "UID";
constexpr char kKeyAtqa[]       = "ATQA";
constexpr char kKeySak[]        = "SAK";
constexpr char kKeyT0[]         = "T0";
constexpr char kKeyTa1[]        = "TA(1)";
constexpr char kKeyTb1[]        = "TB(1)";
constexpr char kKeyTc1[]        = "TC(1)";
constexpr char kKeyT1Tk[]       = "T1...Tk";

constexpr char kFiletypeValue[] = "Flipper NFC device"; // [SOURCE] nfc_device.c:9

// [SOURCE] nfc_device.c's nfc_device_save() builds this comment by iterating
// nfc_devices[]; the list and its order come from
// lib/nfc/protocols/nfc_device_defs.c's nfc_devices[] initialiser and each
// protocol's own <PROTO>_PROTOCOL_NAME macro (all read directly, dev branch,
// 2026-08-23). Purely informational -- Flipper's own parser skips '#' lines
// entirely -- but written so a file from this project is byte-shaped like a
// real one rather than merely parseable.
constexpr char kDeviceTypeListComment[] =
    "Device type can be ISO14443-3A, ISO14443-3B, ISO14443-4A, ISO14443-4B, "
    "ISO15693-3, FeliCa, NTAG/Ultralight, Mifare Classic, Mifare Plus, "
    "Mifare DESFire, SLIX, ST25TB";

// [SOURCE] ISO/IEC 14443-4 / rfal_isoDep.h:135-137 (RFAL_ISODEP_ATS_T0_TA1 /
// _TB1 / _TC1), the same three presence bits st25r3916_driver.cpp already
// uses to walk an ATS, and the same ones iso14443_4a_save() tests before
// emitting each optional key.
constexpr uint8_t kAtsT0Ta1 = 0x10U;
constexpr uint8_t kAtsT0Tb1 = 0x20U;
constexpr uint8_t kAtsT0Tc1 = 0x40U;

// [SOURCE] nfc_common.h NFC_LSB_ATQA_FORMAT_VERSION = 2 -- the oldest version
// still loadable by real Flipper firmware, and the boundary at which ATQA
// byte order changed.
constexpr int kMinSupportedVersion = 2;
constexpr int kLsbAtqaVersion = 2;

// ── bounded text appending ────────────────────────────────────────────────
// One cursor struct instead of scattered snprintf bounds checks: every
// append() is capacity-checked and sets a sticky failure flag, so encode()
// checks overflow once at the end rather than after each of ~12 writes.
struct Out {
    char *buf;
    size_t cap;
    size_t len;
    bool ok;
};

void append(Out *o, const char *s) {
    if (!o->ok) return;
    const size_t n = std::strlen(s);
    if (o->len + n + 1 > o->cap) { // +1 keeps room for the final NUL
        o->ok = false;
        return;
    }
    std::memcpy(o->buf + o->len, s, n);
    o->len += n;
}

void append_line(Out *o, const char *key, const char *value) {
    append(o, key);
    append(o, ": "); // [SOURCE] flipper_format_stream_write_key(): key, ':', ' '
    append(o, value);
    append(o, "\n");
}

void append_comment(Out *o, const char *text) {
    append(o, "# "); // [SOURCE] flipper_format_stream_write_comment_cstr()
    append(o, text);
    append(o, "\n");
}

// [SOURCE] flipper_format_stream_write_value_line()'s FlipperStreamValueHex
// case: each byte "%02X", single-space separated.
void append_hex_line(Out *o, const char *key, const uint8_t *data, size_t n) {
    if (!o->ok) return;
    append(o, key);
    append(o, ": ");
    char b[4];
    for (size_t i = 0; i < n; i++) {
        std::snprintf(b, sizeof(b), i + 1 < n ? "%02X " : "%02X", data[i]);
        append(o, b);
    }
    append(o, "\n");
}

// ── text parsing ──────────────────────────────────────────────────────────

bool next_line(const char *text, size_t len, size_t *pos,
               const char **line_start, size_t *line_len) {
    if (*pos >= len) return false;
    const size_t start = *pos;
    size_t i = start;
    while (i < len && text[i] != '\n') i++;
    size_t end = i;
    if (end > start && text[end - 1] == '\r') end--; // defensive CRLF trim
    *line_start = text + start;
    *line_len = end - start;
    *pos = (i < len) ? i + 1 : i;
    return true;
}

// If `line` is "<key>: <value>", points *val_start/*val_len at the value
// (leading spaces/tabs stripped) and returns true. A '#' comment line never
// matches, since the key comparison is anchored at the line's first byte.
bool line_key_value(const char *line, size_t line_len, const char *key,
                    const char **val_start, size_t *val_len) {
    const size_t klen = std::strlen(key);
    if (line_len < klen + 1) return false;
    if (std::memcmp(line, key, klen) != 0) return false;
    if (line[klen] != ':') return false;
    size_t i = klen + 1;
    while (i < line_len && (line[i] == ' ' || line[i] == '\t')) i++;
    *val_start = line + i;
    *val_len = line_len - i;
    return true;
}

bool hex_nibble(char c, uint8_t *out) {
    if (c >= '0' && c <= '9') { *out = static_cast<uint8_t>(c - '0'); return true; }
    if (c >= 'A' && c <= 'F') { *out = static_cast<uint8_t>(c - 'A' + 10); return true; }
    if (c >= 'a' && c <= 'f') { *out = static_cast<uint8_t>(c - 'a' + 10); return true; }
    return false;
}

// Parses "AA BB CC" into out[]. Returns the byte count, or -1 on any
// malformed token (odd digit count, non-hex character, or more bytes than
// max_out). Whitespace-tolerant beyond the single space the writer emits.
int parse_hex_bytes(const char *s, size_t len, uint8_t *out, size_t max_out) {
    size_t i = 0;
    size_t n = 0;
    while (i < len) {
        while (i < len && (s[i] == ' ' || s[i] == '\t')) i++;
        if (i >= len) break;
        uint8_t hi = 0;
        uint8_t lo = 0;
        if (i + 1 >= len || !hex_nibble(s[i], &hi) || !hex_nibble(s[i + 1], &lo)) return -1;
        // A third hex digit glued to the pair is a malformed token, not two
        // separate bytes -- reject rather than silently resynchronising.
        if (i + 2 < len && s[i + 2] != ' ' && s[i + 2] != '\t') return -1;
        if (n >= max_out) return -1;
        out[n++] = static_cast<uint8_t>((hi << 4) | lo);
        i += 2;
    }
    return static_cast<int>(n);
}

// Finds the first "<key>: ..." line anywhere in the text and parses its value
// as hex bytes. Key lookup is by scan, not by fixed line position, matching
// flipper_format's own seek-to-key behavior (and rf433_sub_format.cpp's own
// header-parsing precedent).
int find_hex_key(const char *text, size_t len, const char *key,
                 uint8_t *out, size_t max_out) {
    size_t pos = 0;
    const char *line = nullptr;
    size_t line_len = 0;
    while (next_line(text, len, &pos, &line, &line_len)) {
        const char *v = nullptr;
        size_t vlen = 0;
        if (line_key_value(line, line_len, key, &v, &vlen)) {
            return parse_hex_bytes(v, vlen, out, max_out);
        }
    }
    return -1;
}

bool find_str_key(const char *text, size_t len, const char *key,
                  const char **val, size_t *val_len) {
    size_t pos = 0;
    const char *line = nullptr;
    size_t line_len = 0;
    while (next_line(text, len, &pos, &line, &line_len)) {
        if (line_key_value(line, line_len, key, val, val_len)) return true;
    }
    return false;
}

bool str_equals(const char *s, size_t len, const char *expected) {
    const size_t elen = std::strlen(expected);
    return len == elen && std::memcmp(s, expected, elen) == 0;
}

// True if `rec`'s ATS is self-consistent: either absent, or a real ATS whose
// first byte (TL) equals its own total length, which is the same validity
// gate st25r3916_driver.cpp's iso14443_4_activate() applies before ever
// handing an ATS out ([REF] rfal_isoDep.cpp:936-938).
bool ats_consistent(const Iso14443_4aRecord &rec) {
    if (rec.ats_len == 0) return true;
    if (rec.ats_len > kMaxAtsLen) return false;
    return rec.ats[0] == rec.ats_len;
}

bool uid_len_valid(uint8_t n) { return n == 4 || n == 7 || n == 10; }

} // namespace

bool encode(const Iso14443_4aRecord &rec, char *buf, size_t buf_size, size_t *out_len) {
    if (buf == nullptr || buf_size == 0) return false;
    if (!uid_len_valid(rec.uid_len)) return false;
    if (!ats_consistent(rec)) return false;

    Out o{buf, buf_size, 0, true};

    char vbuf[16];
    append_line(&o, kKeyFiletype, kFiletypeValue);
    std::snprintf(vbuf, sizeof(vbuf), "%d", kFormatVersion);
    append_line(&o, kKeyVersion, vbuf);
    append_comment(&o, kDeviceTypeListComment);
    append_line(&o, kKeyDeviceType, kDeviceTypeIso14443_4a);
    append_comment(&o, "UID is common for all formats");
    append_hex_line(&o, kKeyUid, rec.uid, rec.uid_len);

    // [SOURCE] iso14443_3a_save(): "ISO14443-3A specific data" comment, then
    // ATQA written REVERSED ("Save ATQA in MSB order for correct companion
    // apps display", `{data->atqa[1], data->atqa[0]}`), then SAK.
    append_comment(&o, "ISO14443-3A specific data");
    const uint8_t atqa_msb[2] = {rec.atqa[1], rec.atqa[0]};
    append_hex_line(&o, kKeyAtqa, atqa_msb, 2);
    append_hex_line(&o, kKeySak, &rec.sak, 1);

    // [SOURCE] iso14443_4a_save(): the "ISO14443-4A specific data" comment is
    // written unconditionally, but every key under it only `if
    // (ats_data->tl > 1)` and then only for the interface bytes T0's own
    // presence bits announce. ats_len is TL (see ats_consistent()), so
    // ats_len > 1 is literally the same test.
    append_comment(&o, "ISO14443-4A specific data");
    if (rec.ats_len > 1) {
        const uint8_t t0 = rec.ats[1];
        append_hex_line(&o, kKeyT0, &t0, 1);
        size_t idx = 2;
        if ((t0 & kAtsT0Ta1) != 0 && idx < rec.ats_len) {
            append_hex_line(&o, kKeyTa1, &rec.ats[idx], 1);
            idx++;
        }
        if ((t0 & kAtsT0Tb1) != 0 && idx < rec.ats_len) {
            append_hex_line(&o, kKeyTb1, &rec.ats[idx], 1);
            idx++;
        }
        if ((t0 & kAtsT0Tc1) != 0 && idx < rec.ats_len) {
            append_hex_line(&o, kKeyTc1, &rec.ats[idx], 1);
            idx++;
        }
        if (idx < rec.ats_len) {
            append_hex_line(&o, kKeyT1Tk, &rec.ats[idx], rec.ats_len - idx);
        }
    }

    if (!o.ok) return false;
    o.buf[o.len] = '\0'; // append() always reserved this byte
    if (out_len != nullptr) *out_len = o.len;
    return true;
}

bool decode(const char *text, size_t len, Iso14443_4aRecord *out) {
    if (text == nullptr || out == nullptr || len == 0) return false;

    const char *v = nullptr;
    size_t vlen = 0;

    if (!find_str_key(text, len, kKeyFiletype, &v, &vlen)) return false;
    if (!str_equals(v, vlen, kFiletypeValue)) return false;

    if (!find_str_key(text, len, kKeyVersion, &v, &vlen)) return false;
    int version = 0;
    {
        // Small, explicit digit walk rather than atoi over a non-NUL-
        // terminated slice.
        if (vlen == 0 || vlen > 6) return false;
        for (size_t i = 0; i < vlen; i++) {
            if (v[i] < '0' || v[i] > '9') return false;
            version = version * 10 + (v[i] - '0');
        }
    }
    if (version < kMinSupportedVersion) return false;

    if (!find_str_key(text, len, kKeyDeviceType, &v, &vlen)) return false;
    if (!str_equals(v, vlen, kDeviceTypeIso14443_4a)) return false;

    Iso14443_4aRecord rec{};

    const int uid_n = find_hex_key(text, len, kKeyUid, rec.uid, sizeof(rec.uid));
    if (uid_n <= 0 || !uid_len_valid(static_cast<uint8_t>(uid_n))) return false;
    rec.uid_len = static_cast<uint8_t>(uid_n);

    uint8_t atqa_raw[2] = {0, 0};
    if (find_hex_key(text, len, kKeyAtqa, atqa_raw, sizeof(atqa_raw)) != 2) return false;
    // [SOURCE] iso14443_3a_load(): versions newer than
    // NFC_LSB_ATQA_FORMAT_VERSION store ATQA MSB-first and are swapped on
    // load; a version-2 file is already in the LSB/wire order this project
    // uses everywhere else.
    if (version > kLsbAtqaVersion) {
        rec.atqa[0] = atqa_raw[1];
        rec.atqa[1] = atqa_raw[0];
    } else {
        rec.atqa[0] = atqa_raw[0];
        rec.atqa[1] = atqa_raw[1];
    }

    if (find_hex_key(text, len, kKeySak, &rec.sak, 1) != 1) return false;

    // Rebuild the TL-first wire-layout ATS from the decomposed keys. The
    // whole block is optional: Flipper omits it whenever tl <= 1, so its
    // absence means "this card's ATS carried nothing beyond TL", not a
    // malformed file.
    uint8_t t0 = 0;
    if (find_hex_key(text, len, kKeyT0, &t0, 1) == 1) {
        uint8_t ats[kMaxAtsLen];
        size_t n = 1; // ats[0] (TL) filled in at the end
        ats[n++] = t0;
        uint8_t b = 0;
        if ((t0 & kAtsT0Ta1) != 0) {
            if (find_hex_key(text, len, kKeyTa1, &b, 1) != 1) return false;
            ats[n++] = b;
        }
        if ((t0 & kAtsT0Tb1) != 0) {
            if (find_hex_key(text, len, kKeyTb1, &b, 1) != 1) return false;
            ats[n++] = b;
        }
        if ((t0 & kAtsT0Tc1) != 0) {
            if (find_hex_key(text, len, kKeyTc1, &b, 1) != 1) return false;
            ats[n++] = b;
        }
        const int tk = find_hex_key(text, len, kKeyT1Tk, &ats[n], kMaxAtsLen - n);
        if (tk > 0) {
            n += static_cast<size_t>(tk);
        } else if (tk == 0) {
            return false; // key present but empty -- malformed, not absent
        }
        ats[0] = static_cast<uint8_t>(n); // TL counts itself
        std::memcpy(rec.ats, ats, n);
        rec.ats_len = static_cast<uint8_t>(n);
    }

    *out = rec;
    return true;
}

bool build_path(const Iso14443_4aRecord &rec, char *path, size_t path_len) {
    if (path == nullptr || rec.uid_len == 0 || rec.uid_len > sizeof(rec.uid)) {
        return false;
    }
    char hex[2 * sizeof(rec.uid) + 1];
    for (uint8_t i = 0; i < rec.uid_len; i++) {
        std::snprintf(hex + (i * 2), 3, "%02X", rec.uid[i]);
    }
    hex[rec.uid_len * 2] = '\0';
    std::snprintf(path, path_len, "%s/%s.nfc", kNfcDir, hex);
    return true;
}

bool write(IStorage &storage, const char *path, const Iso14443_4aRecord &rec) {
    // Plain stack buffer, unlike rf433_sub_format.cpp's heap-allocated one:
    // kMaxEncodedTextBytes is 1 KB here, not the 128 KB-1 MB that module
    // deals in, so there is nothing to move off the stack.
    char buf[kMaxEncodedTextBytes];
    size_t len = 0;
    if (!encode(rec, buf, sizeof(buf), &len)) return false;
    return storage.write_capture_file(path, reinterpret_cast<const uint8_t *>(buf), len);
}

bool read(IStorage &storage, const char *path, Iso14443_4aRecord *out) {
    char buf[kMaxEncodedTextBytes];
    size_t len = 0;
    if (!storage.read_file(path, reinterpret_cast<uint8_t *>(buf), sizeof(buf), &len)) {
        return false;
    }
    // IStorage::read_file() caps *out_len at the buffer size when the real
    // file is longer (hal/istorage.h's own contract) -- a file that fills the
    // buffer exactly may therefore be truncated, and a truncated .nfc file
    // has no safe partial interpretation (unlike a .sub capture, which
    // rf433_sub_format.cpp can flag and keep). Reject it.
    if (len == 0 || len == sizeof(buf)) return false;
    return decode(buf, len, out);
}

} // namespace NfcFlipperFormat
