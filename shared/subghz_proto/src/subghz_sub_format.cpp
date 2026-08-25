#include "subghz_sub_format.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

// ===========================================================================
// See subghz_sub_format.h for the full design note (why this is a parallel,
// generalized implementation rather than a migration of rf433_sub_format.cpp,
// and why this module defines its own EdgeSample). The merge/signed-duration
// and header-parsing algorithms below are the SAME real logic
// rf433_sub_format.cpp already established and cited (Flipper's own .sub RAW
// format documentation) -- generalized to take frequency/preset as
// parameters and to write into a caller-supplied edges_out buffer instead of
// a fixed CapturedSignal, not re-derived from scratch.
// ===========================================================================

namespace SubghzProto {

namespace {

constexpr char kFiletypeLine[] = "Filetype: Flipper SubGhz RAW File";
constexpr char kVersionLine[] = "Version: 1";
constexpr char kProtocolLine[] = "Protocol: RAW";
constexpr char kRawDataPrefix[] = "RAW_Data:";
constexpr size_t kRawDataPrefixLen = sizeof(kRawDataPrefix) - 1;
constexpr size_t kMaxValuesPerLine = 512; // real spec's own per-line cap

// ── WRITE direction: EdgeSample[] -> signed RAW_Data durations ────────────

struct LevelPulse {
    uint32_t duration;
    bool level;
};

// Merges consecutive EdgeSamples that share the same held level into one
// true pulse (summing their durations) -- same real fix rf433_sub_format.cpp's
// own merge_pulses() applies, for the same reason: a real capture front end
// can produce runs of consecutive same-level samples from receiver squelch/
// glitch chatter, which must be folded together before being treated as
// alternating HIGH/LOW pulses. duration[i] (the gap between edges[i-1] and
// edges[i]) is held at edges[i-1].level, matching this project's established
// EdgeSample convention (rf433_common.h: "level AFTER the edge").
std::vector<LevelPulse> merge_pulses(const EdgeSample *edges, size_t edge_count) {
    std::vector<LevelPulse> out;
    if (edge_count < 2) return out;

    uint32_t merged_duration = 0;
    bool merged_level = false;
    bool have_pulse = false;

    for (size_t i = 1; i < edge_count; i++) {
        uint32_t gap = static_cast<uint32_t>(edges[i].timestamp_us - edges[i - 1].timestamp_us);
        bool held_level = edges[i - 1].level;

        if (!have_pulse) {
            merged_duration = gap;
            merged_level = held_level;
            have_pulse = true;
        } else if (held_level == merged_level) {
            merged_duration += gap;
        } else {
            out.push_back(LevelPulse{merged_duration, merged_level});
            merged_duration = gap;
            merged_level = held_level;
        }
    }
    if (have_pulse) out.push_back(LevelPulse{merged_duration, merged_level});
    return out;
}

// Builds strictly-alternating SIGNED durations (positive = HIGH, negative =
// LOW) from merged pulses. Beyond merge_pulses()'s alternation guarantee,
// the real .sub spec additionally requires RAW_Data to START positive
// (first segment always HIGH). A real capture can start at either level, so
// if the first merged pulse is LOW it is dropped rather than emitted with
// the wrong sign -- same real, disclosed, unrepresentable-per-spec case
// rf433_sub_format.cpp's own build_signed_durations() documents (an
// unrecorded rising edge, and whatever HIGH dwell preceded it, happened
// before capture armed, with no way to know how long that dwell was).
std::vector<int32_t> build_signed_durations(const EdgeSample *edges, size_t edge_count) {
    std::vector<int32_t> out;
    std::vector<LevelPulse> merged = merge_pulses(edges, edge_count);
    if (merged.empty()) return out;

    size_t start = merged[0].level ? 0 : 1;
    out.reserve(merged.size() - start);
    for (size_t i = start; i < merged.size(); i++) {
        int64_t signed_dur = merged[i].level ? static_cast<int64_t>(merged[i].duration)
                                              : -static_cast<int64_t>(merged[i].duration);
        out.push_back(static_cast<int32_t>(signed_dur));
    }
    return out;
}

// ── READ direction: text helpers ───────────────────────────────────────────
// Identical helpers to rf433_sub_format.cpp's own (same real parsing rules,
// same real bounded-strtol bug fix for a possibly-non-NUL-terminated last
// token -- see that file's own parse_long_bounded() comment for the full
// reasoning), duplicated rather than shared for the same reason
// rf433_sub_format.cpp's own merge_pulses() is not shared with
// rf433_protocol_decode.cpp's build_durations(): different translation
// units, anonymous-namespace statics, no existing shared header for them.

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

// Bounded strtol -- decode_sub()'s own contract says text[0..len) "need not
// be NUL-terminated", so calling strtol directly on the last token in such a
// buffer would scan past the caller's declared bound. Copies at most
// kMaxTokenLen-1 bytes into a local NUL-terminated buffer before parsing.
constexpr size_t kMaxTokenLen = 16; // "-2147483648" (11 chars) + NUL + slack
bool parse_long_bounded(const char *p, size_t max_len, long *out, size_t *consumed) {
    char tmp[kMaxTokenLen];
    size_t n = (max_len < kMaxTokenLen - 1) ? max_len : kMaxTokenLen - 1;
    std::memcpy(tmp, p, n);
    tmp[n] = '\0';
    char *endp = nullptr;
    long v = std::strtol(tmp, &endp, 10);
    if (endp == tmp) return false; // non-numeric token
    size_t used = static_cast<size_t>(endp - tmp);
    if (used == n && n < max_len) {
        char next = p[n];
        if (next >= '0' && next <= '9') return false; // token longer than this module supports
    }
    *out = v;
    *consumed = used;
    return true;
}

// Bounded strtoul for the Frequency: field's value (unsigned Hz per the real
// spec). Same bounded-copy reasoning as parse_long_bounded() above.
bool parse_ulong_bounded(const char *p, size_t max_len, unsigned long *out) {
    constexpr size_t kMaxFreqTokenLen = 16; // "4294967295" (10 chars) + NUL + slack
    char tmp[kMaxFreqTokenLen];
    size_t n = (max_len < kMaxFreqTokenLen - 1) ? max_len : kMaxFreqTokenLen - 1;
    std::memcpy(tmp, p, n);
    tmp[n] = '\0';
    char *endp = nullptr;
    unsigned long v = std::strtoul(tmp, &endp, 10);
    if (endp == tmp) return false;
    *out = v;
    return true;
}

} // namespace

bool encode_sub(uint32_t freq_hz, const char *preset, const EdgeSample *edges, size_t edge_count,
                 char *buf, size_t buf_size, size_t *out_len) {
    if (buf == nullptr || buf_size == 0 || preset == nullptr || edges == nullptr) return false;
    if (edge_count < 2) return false;

    std::vector<int32_t> durations = build_signed_durations(edges, edge_count);
    if (durations.empty()) return false;

    int written = std::snprintf(buf, buf_size, "%s\n%s\nFrequency: %u\nPreset: %s\n%s\n%s",
                                 kFiletypeLine, kVersionLine, static_cast<unsigned>(freq_hz), preset,
                                 kProtocolLine, kRawDataPrefix);
    if (written < 0 || static_cast<size_t>(written) >= buf_size) return false;
    size_t pos = static_cast<size_t>(written);

    for (size_t i = 0; i < durations.size(); i++) {
        if (i > 0 && (i % kMaxValuesPerLine) == 0) {
            written = std::snprintf(buf + pos, buf_size - pos, "\n%s", kRawDataPrefix);
            if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
            pos += static_cast<size_t>(written);
        }
        written = std::snprintf(buf + pos, buf_size - pos, " %ld", static_cast<long>(durations[i]));
        if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
        pos += static_cast<size_t>(written);
    }
    written = std::snprintf(buf + pos, buf_size - pos, "\n");
    if (written < 0 || pos + static_cast<size_t>(written) >= buf_size) return false;
    pos += static_cast<size_t>(written);

    if (out_len != nullptr) *out_len = pos;
    return true;
}

bool decode_sub(const char *text, size_t len, uint32_t *freq_hz_out, EdgeSample *edges_out,
                 size_t edges_capacity, size_t *edge_count_out) {
    if (text == nullptr || freq_hz_out == nullptr || edges_out == nullptr || edges_capacity == 0 ||
        edge_count_out == nullptr) {
        return false;
    }

    size_t pos = 0;
    const char *line = nullptr;
    size_t line_len = 0;

    // Filetype/Version are always the first two lines, real spec's own
    // fixed field order.
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kFiletypeLine)) return false;
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kVersionLine)) return false;

    // Remaining header lines scanned for by key rather than assumed to sit
    // at fixed positions, so both real file shapes parse (standard-preset,
    // and "RAW file, custom preset" which inserts Custom_preset_module:/
    // Custom_preset_data: lines between Preset: and Protocol:) -- see this
    // file's header comment.
    constexpr int kMaxHeaderLines = 6; // Frequency, Preset, Custom_preset_module,
                                        // Custom_preset_data, Protocol, +1 slack
    bool saw_frequency = false;
    bool saw_preset = false;
    bool found_protocol = false;
    unsigned long parsed_freq = 0;
    for (int header_lines = 0; header_lines < kMaxHeaderLines; header_lines++) {
        if (!next_line(text, len, &pos, &line, &line_len)) break;
        if (line_equals(line, line_len, kProtocolLine)) {
            found_protocol = true;
            break;
        }
        if (starts_with(line, line_len, "Protocol:", 9)) return false; // non-RAW protocol -- out of scope
        if (starts_with(line, line_len, "Frequency:", 10)) {
            size_t vpos = 10;
            while (vpos < line_len && line[vpos] == ' ') vpos++;
            if (!parse_ulong_bounded(line + vpos, line_len - vpos, &parsed_freq)) return false;
            saw_frequency = true;
            continue;
        }
        if (starts_with(line, line_len, "Preset:", 7)) { saw_preset = true; continue; }
        if (starts_with(line, line_len, "Custom_preset_module:", 21)) continue;
        if (starts_with(line, line_len, "Custom_preset_data:", 19)) continue;
        return false; // unrecognized header line -- malformed
    }
    if (!found_protocol || !saw_frequency || !saw_preset) return false;

    // One or more RAW_Data: lines, concatenated -- real spec's own
    // continuation convention. Bounded to at most edges_capacity - 1 values
    // (a duration is the gap between two edges), since this module has no
    // fixed internal cap of its own -- see this file's header comment.
    const size_t max_durations = edges_capacity - 1;
    std::vector<int32_t> durations;
    durations.reserve(max_durations < 512 ? max_durations : 512);
    bool saw_raw_data = false;
    bool have_prev_sign = false;
    bool prev_positive = false;

    while (next_line(text, len, &pos, &line, &line_len)) {
        if (line_len == 0) continue; // tolerate trailing blank lines
        if (!starts_with(line, line_len, kRawDataPrefix, kRawDataPrefixLen)) continue;
        saw_raw_data = true;

        size_t i = kRawDataPrefixLen;
        while (i < line_len) {
            while (i < line_len && (line[i] == ' ' || line[i] == '\t')) i++;
            if (i >= line_len) break;

            long v = 0;
            size_t consumed = 0;
            if (!parse_long_bounded(line + i, line_len - i, &v, &consumed)) return false; // non-numeric/unsupported token
            int32_t narrowed = static_cast<int32_t>(v);
            if (v < std::numeric_limits<int32_t>::min() || v > std::numeric_limits<int32_t>::max()) {
                return false; // out of this module's representable range
            }
            if (narrowed == 0) return false; // real spec: "values must be non-zero"

            bool positive = narrowed > 0;
            have_prev_sign = true;
            prev_positive = positive;
            (void)prev_positive; // sign-alternation is not enforced on read (see header comment)

            if (durations.size() < max_durations) {
                durations.push_back(narrowed);
            }
            // else: silently dropped, capped at edges_capacity -- see header comment.
            i += consumed;
        }
    }
    (void)have_prev_sign;
    if (!saw_raw_data || durations.empty()) return false;

    // Reconstruct EdgeSample[] from n signed durations -> n+1 edges. Same
    // real first-edge/last-edge reconstruction rf433_sub_format.cpp's own
    // decode() establishes: duration[0]'s sign gives edges[0].level directly
    // (there is no absolute timestamp in a .sub file, so edges[0].timestamp_us
    // is an arbitrary synthetic origin, 0 -- only RELATIVE timing between
    // edges is ever meaningful). For k = 1..n-1: edges[k] = {sum of
    // |duration[0..k-1]|, sign(duration[k])}. The final edge has no
    // duration[n] to read a sign from -- only the fact a transition
    // happened -- so edges[n].level = !sign(duration[n-1]).
    size_t n = durations.size();
    size_t edge_count = n + 1;
    if (edge_count > edges_capacity) edge_count = edges_capacity; // defensive; unreachable given the cap above

    uint32_t t = 0;
    for (size_t k = 0; k < edge_count; k++) {
        if (k == 0) {
            edges_out[0] = EdgeSample{0u, durations[0] > 0};
        } else if (k < n) {
            t += static_cast<uint32_t>(std::abs(durations[k - 1]));
            edges_out[k] = EdgeSample{t, durations[k] > 0};
        } else { // k == n
            t += static_cast<uint32_t>(std::abs(durations[n - 1]));
            edges_out[k] = EdgeSample{t, !(durations[n - 1] > 0)};
        }
    }

    *freq_hz_out = static_cast<uint32_t>(parsed_freq);
    *edge_count_out = edge_count;
    return true;
}

} // namespace SubghzProto
