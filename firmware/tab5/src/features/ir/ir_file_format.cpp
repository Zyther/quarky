#include "ir_file_format.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace IrFileFormat {

namespace {

constexpr char kFiletypeLine[] = "Filetype: IR signals file";
constexpr char kVersionLine[] = "Version: 1";

// Same small line-scanning helpers as rf433_sub_format.cpp -- duplicated
// rather than shared across the two format modules (different translation
// units, no existing shared text-parsing utility in this codebase, and each
// is small enough that introducing a new shared header for three ~5-line
// functions would be more coupling than the duplication itself).
bool next_line(const char *text, size_t len, size_t *pos, const char **line_start, size_t *line_len) {
    if (*pos >= len) return false;
    size_t start = *pos;
    size_t i = start;
    while (i < len && text[i] != '\n') i++;
    size_t end = i;
    if (end > start && text[end - 1] == '\r') end--; // defensive CRLF trim, not spec-mandated
    *line_start = text + start;
    *line_len = end - start;
    *pos = (i < len) ? i + 1 : i;
    return true;
}

bool line_equals(const char *line, size_t line_len, const char *expected) {
    size_t expected_len = std::strlen(expected);
    return line_len == expected_len && std::memcmp(line, expected, expected_len) == 0;
}

bool starts_with(const char *line, size_t line_len, const char *prefix, size_t prefix_len) {
    return line_len >= prefix_len && std::memcmp(line, prefix, prefix_len) == 0;
}

// Value substring after a "key:" prefix, skipping one optional leading space
// (real sample lines are "key: value", single space after the colon).
void value_after(const char *line, size_t line_len, size_t prefix_len, const char **val, size_t *val_len) {
    size_t i = prefix_len;
    if (i < line_len && line[i] == ' ') i++;
    *val = line + i;
    *val_len = line_len - i;
}

void copy_bounded(char *dst, size_t dst_size, const char *src, size_t src_len) {
    size_t n = (src_len < dst_size - 1) ? src_len : dst_size - 1;
    std::memcpy(dst, src, n);
    dst[n] = '\0';
}

// Bounded strtol/strtoul: same real bug class and same fix as
// rf433_sub_format.cpp's parse_long_bounded() (see its comment for the full
// reasoning) -- decode()'s own contract (ir_file_format.h) says text[0..len)
// "need not be NUL-terminated", so calling strtol/strtoul directly on the
// last token in a value substring with no trailing delimiter would scan
// past the caller's declared bound. Copies at most kMaxTokenLen-1 bytes into
// a local NUL-terminated buffer first. kMaxTokenLen=8 comfortably covers
// this format's own token shapes: a 2-hex-digit byte ("FF") and an unsigned
// `data` timing (uint16_t, max 5 decimal digits).
constexpr size_t kMaxTokenLen = 8;
bool parse_token_bounded(const char *p, size_t max_len, int base, bool is_signed, long *out_signed,
                          unsigned long *out_unsigned, size_t *consumed) {
    char tmp[kMaxTokenLen];
    size_t n = (max_len < kMaxTokenLen - 1) ? max_len : kMaxTokenLen - 1;
    std::memcpy(tmp, p, n);
    tmp[n] = '\0';
    char *endp = nullptr;
    long signed_v = 0;
    unsigned long unsigned_v = 0;
    if (is_signed) {
        signed_v = std::strtol(tmp, &endp, base);
    } else {
        unsigned_v = std::strtoul(tmp, &endp, base);
    }
    if (endp == tmp) return false;
    size_t used = static_cast<size_t>(endp - tmp);
    // Same reasoning as rf433_sub_format.cpp's parse_long_bounded(): if the
    // copied buffer was exhausted by digits AND the real input had at least
    // one more byte, that next byte might continue the same digit run past
    // what this module supports -- reject rather than silently return a
    // truncated value split across two bogus tokens.
    if (used == n && n < max_len) {
        char next = p[n];
        bool next_is_digit = (base == 16)
            ? ((next >= '0' && next <= '9') || (next >= 'a' && next <= 'f') || (next >= 'A' && next <= 'F'))
            : (next >= '0' && next <= '9');
        if (next_is_digit) return false; // token longer than this module supports
    }
    if (is_signed) {
        *out_signed = signed_v;
    } else {
        *out_unsigned = unsigned_v;
    }
    *consumed = used;
    return true;
}

// Space-separated 2-hex-digit byte tokens, e.g. "EE 87 00 00" -- real
// spec's own address/command syntax.
size_t parse_hex_bytes(const char *val, size_t val_len, uint8_t *out, size_t max_out) {
    size_t count = 0;
    size_t i = 0;
    while (i < val_len && count < max_out) {
        while (i < val_len && (val[i] == ' ' || val[i] == '\t')) i++;
        if (i >= val_len) break;
        long v = 0;
        size_t consumed = 0;
        if (!parse_token_bounded(val + i, val_len - i, 16, true, &v, nullptr, &consumed)) break; // non-hex -- stop
        out[count++] = static_cast<uint8_t>(v & 0xFF);
        i += consumed;
    }
    return count;
}

// Space-separated UNSIGNED decimal timings -- real spec's own `data` syntax
// (no sign, unlike `.sub`'s RAW_Data; alternating mark/space is implicit
// from position).
size_t parse_unsigned_list(const char *val, size_t val_len, uint16_t *out, size_t max_out, bool *truncated) {
    size_t count = 0;
    size_t i = 0;
    while (i < val_len) {
        while (i < val_len && (val[i] == ' ' || val[i] == '\t')) i++;
        if (i >= val_len) break;
        unsigned long v = 0;
        size_t consumed = 0;
        if (!parse_token_bounded(val + i, val_len - i, 10, false, nullptr, &v, &consumed)) break; // non-numeric -- stop
        if (count < max_out) {
            out[count++] = static_cast<uint16_t>(v);
        } else if (truncated != nullptr) {
            *truncated = true;
        }
        i += consumed;
    }
    return count;
}

} // namespace

size_t decode(const char *text, size_t len, IrSignal *out, size_t max_signals, bool *out_truncated) {
    if (out_truncated != nullptr) *out_truncated = false;
    if (text == nullptr || out == nullptr || max_signals == 0) return 0;

    size_t pos = 0;
    const char *line = nullptr;
    size_t line_len = 0;

    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kFiletypeLine)) return 0;
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kVersionLine)) return 0;

    size_t out_count = 0;
    size_t signals_seen = 0; // total signals finalized so far, capped at
                              // kMaxSignalsPerFile below -- distinct from
                              // out_count, which is separately capped by
                              // max_signals
    bool have_current = false;
    // static, not stack-local -- sizeof(IrSignal) is ~2KB; this runs on the
    // LVGL/Arduino task's stack once Task 22 wires this in, same reasoning
    // as every static buffer in rf433_sub_format.cpp. Not reentrant, same
    // real, disclosed, single-caller-today constraint as those. Fields not
    // explicitly set on this call may retain stale content from a previous
    // call, which is fine: `current = IrSignal{};` below resets it in full
    // before each new signal starts, and finalize()'s own copy into out[]
    // happens only after the caller-visible fields are populated.
    static IrSignal current{};
    current = IrSignal{};

    auto finalize = [&]() {
        if (!have_current) return;
        if (out_count < max_signals) {
            out[out_count++] = current;
        } else if (out_truncated != nullptr) {
            *out_truncated = true;
        }
        signals_seen++;
        current = IrSignal{};
        have_current = false;
    };

    while (next_line(text, len, &pos, &line, &line_len)) {
        if (line_len == 0) continue; // tolerate blank lines
        if (line[0] == '#') {
            // Real sample file's own separator convention. Treated as an
            // explicit boundary: close out whatever signal was in progress.
            finalize();
            continue;
        }

        if (starts_with(line, line_len, "name:", 5)) {
            // A `name:` line always starts a NEW signal -- close the
            // previous one first even if no `#` line preceded this one
            // (defensive: not every real file necessarily has `#` between
            // every pair, only the cited sample does).
            finalize();
            if (signals_seen >= kMaxSignalsPerFile) {
                // Enforced internal-processing bound (kMaxSignalsPerFile,
                // ir_file_format.h) -- stop scanning further signals
                // regardless of max_signals/out_count, so a pathological
                // file with far more signals than either doesn't make
                // decode() keep processing lines indefinitely.
                if (out_truncated != nullptr) *out_truncated = true;
                break;
            }
            const char *val;
            size_t val_len;
            value_after(line, line_len, 5, &val, &val_len);
            copy_bounded(current.name, sizeof(current.name), val, val_len);
            have_current = true;
            continue;
        }

        if (!have_current) continue; // field line with no signal in progress -- ignore

        if (starts_with(line, line_len, "type:", 5)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 5, &val, &val_len);
            if (val_len == 6 && std::memcmp(val, "parsed", 6) == 0) {
                current.type = SignalType::kParsed;
            } else if (val_len == 3 && std::memcmp(val, "raw", 3) == 0) {
                current.type = SignalType::kRaw;
            }
        } else if (starts_with(line, line_len, "protocol:", 9)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 9, &val, &val_len);
            copy_bounded(current.protocol, sizeof(current.protocol), val, val_len);
        } else if (starts_with(line, line_len, "address:", 8)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 8, &val, &val_len);
            current.address_len = parse_hex_bytes(val, val_len, current.address, kMaxAddressBytes);
        } else if (starts_with(line, line_len, "command:", 8)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 8, &val, &val_len);
            current.command_len = parse_hex_bytes(val, val_len, current.command, kMaxAddressBytes);
        } else if (starts_with(line, line_len, "frequency:", 10)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 10, &val, &val_len);
            char tmp[16];
            copy_bounded(tmp, sizeof(tmp), val, val_len);
            current.frequency_hz = static_cast<uint32_t>(std::strtoul(tmp, nullptr, 10));
        } else if (starts_with(line, line_len, "duty_cycle:", 11)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 11, &val, &val_len);
            char tmp[32];
            copy_bounded(tmp, sizeof(tmp), val, val_len);
            current.duty_cycle = std::strtof(tmp, nullptr);
        } else if (starts_with(line, line_len, "data:", 5)) {
            const char *val;
            size_t val_len;
            value_after(line, line_len, 5, &val, &val_len);
            bool this_signal_truncated = false;
            current.data_count = parse_unsigned_list(val, val_len, current.data, kMaxRawSamples, &this_signal_truncated);
            current.truncated = this_signal_truncated;
            if (this_signal_truncated && out_truncated != nullptr) *out_truncated = true;
        }
        // Unknown key -- ignore (forward-compat with fields this module
        // doesn't model).
    }
    finalize();
    return out_count;
}

bool encode(const IrSignal *signals, size_t count, char *buf, size_t buf_size, size_t *out_len) {
    if (signals == nullptr || buf == nullptr || buf_size == 0 || count == 0) return false;

    int written = std::snprintf(buf, buf_size, "%s\n%s\n", kFiletypeLine, kVersionLine);
    if (written < 0 || static_cast<size_t>(written) >= buf_size) return false;
    size_t pos = static_cast<size_t>(written);

    for (size_t s = 0; s < count; s++) {
        const IrSignal &sig = signals[s];

        written = std::snprintf(buf + pos, buf_size - pos, "#\nname: %s\ntype: %s\n", sig.name,
                                 sig.type == SignalType::kParsed ? "parsed" : "raw");
        if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
        pos += static_cast<size_t>(written);

        if (sig.type == SignalType::kParsed) {
            written = std::snprintf(buf + pos, buf_size - pos, "protocol: %s\naddress:", sig.protocol);
            if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
            pos += static_cast<size_t>(written);

            for (size_t i = 0; i < sig.address_len; i++) {
                written = std::snprintf(buf + pos, buf_size - pos, " %02X", sig.address[i]);
                if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
                pos += static_cast<size_t>(written);
            }
            written = std::snprintf(buf + pos, buf_size - pos, "\ncommand:");
            if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
            pos += static_cast<size_t>(written);

            for (size_t i = 0; i < sig.command_len; i++) {
                written = std::snprintf(buf + pos, buf_size - pos, " %02X", sig.command[i]);
                if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
                pos += static_cast<size_t>(written);
            }
            written = std::snprintf(buf + pos, buf_size - pos, "\n");
            if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
            pos += static_cast<size_t>(written);
        } else {
            written = std::snprintf(buf + pos, buf_size - pos, "frequency: %u\nduty_cycle: %f\ndata:",
                                     static_cast<unsigned>(sig.frequency_hz), static_cast<double>(sig.duty_cycle));
            if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
            pos += static_cast<size_t>(written);

            for (size_t i = 0; i < sig.data_count; i++) {
                written = std::snprintf(buf + pos, buf_size - pos, " %u", static_cast<unsigned>(sig.data[i]));
                if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
                pos += static_cast<size_t>(written);
            }
            written = std::snprintf(buf + pos, buf_size - pos, "\n");
            if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
            pos += static_cast<size_t>(written);
        }
    }

    if (out_len != nullptr) *out_len = pos;
    return true;
}

bool write(IStorage &storage, const char *path, const IrSignal *signals, size_t count) {
    // Static, not stack-local -- same reasoning as Rf433SubFormat::write()'s
    // identically-sized buffer: generous enough for this project's own
    // learn/clone use (a handful of signals, not a 50+-button community
    // database written in one call -- a real, disclosed practical
    // constraint on write(), not a silent failure, since encode() returning
    // false on overflow is checked and propagated here).
    static char buf[8192];
    size_t len = 0;
    if (!encode(signals, count, buf, sizeof(buf), &len)) return false;
    return storage.write_capture_file(path, reinterpret_cast<const uint8_t *>(buf), len);
}

size_t read(IStorage &storage, const char *path, IrSignal *out, size_t max_signals, bool *out_truncated) {
    static char buf[8192];
    size_t len = 0;
    if (!storage.read_file(path, reinterpret_cast<uint8_t *>(buf), sizeof(buf), &len)) {
        if (out_truncated != nullptr) *out_truncated = false;
        return 0;
    }
    size_t n = decode(buf, len, out, max_signals, out_truncated);
    // IStorage::read_file()'s own documented contract (hal/istorage.h):
    // *out_len is capped at max_len if the real file was longer. len ==
    // sizeof(buf) is therefore this call's only signal that the file may
    // have been truncated -- ORed into *out_truncated alongside decode()'s
    // own (unrelated) truncation reasons rather than adding a second flag.
    if (len == sizeof(buf) && out_truncated != nullptr) *out_truncated = true;
    return n;
}

} // namespace IrFileFormat
