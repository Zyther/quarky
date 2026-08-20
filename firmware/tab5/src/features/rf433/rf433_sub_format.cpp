#include "rf433_sub_format.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace Rf433SubFormat {

const char kPresetName[] = "FuriHalSubGhzPresetOok650Async";

namespace {

constexpr char kFiletypeLine[] = "Filetype: Flipper SubGhz RAW File";
constexpr char kVersionLine[] = "Version: 1";
constexpr char kProtocolLine[] = "Protocol: RAW";
constexpr char kRawDataPrefix[] = "RAW_Data:";
constexpr size_t kRawDataPrefixLen = sizeof(kRawDataPrefix) - 1;

// ── WRITE direction: EdgeSample[] -> signed RAW_Data durations ────────────

struct LevelPulse {
    uint32_t duration;
    bool level;
};

// Identical merge loop to rf433_protocol_decode.cpp's build_durations()
// (same fix, same reasoning: this project's ISR-timestamped capture can
// produce runs of consecutive EdgeSamples sharing the same .level from
// receiver squelch/glitch chatter, which must be folded into one true pulse
// before being handed out as a duration), except this version keeps the
// LEVEL each merged pulse was held at, not just its magnitude -- the
// unsigned build_durations() never needed to preserve sign because its
// callers (the brand decoders) derive level from output-array index parity
// instead. RAW_Data needs the level directly, as its sign. Duplicated
// rather than shared: different return shape, and build_durations() is a
// static helper in an anonymous namespace, not exposed for reuse.
size_t merge_pulses(const Rf433Scan::CapturedSignal &sig, LevelPulse *out, size_t max_out) {
    if (sig.edge_count < 2) return 0;

    size_t out_count = 0;
    uint32_t merged_duration = 0;
    bool merged_level = false;
    bool have_pulse = false;

    for (size_t i = 1; i < sig.edge_count; i++) {
        uint32_t gap = static_cast<uint32_t>(sig.edges[i].timestamp_us - sig.edges[i - 1].timestamp_us);
        bool held_level = sig.edges[i - 1].level;

        if (!have_pulse) {
            merged_duration = gap;
            merged_level = held_level;
            have_pulse = true;
        } else if (held_level == merged_level) {
            merged_duration += gap;
        } else {
            if (out_count >= max_out) return out_count;
            out[out_count++] = LevelPulse{merged_duration, merged_level};
            merged_duration = gap;
            merged_level = held_level;
        }
    }
    if (have_pulse && out_count < max_out) {
        out[out_count++] = LevelPulse{merged_duration, merged_level};
    }
    return out_count;
}

// Builds strictly-alternating SIGNED durations (positive = HIGH, negative =
// LOW) from sig.edges. Beyond merge_pulses()'s alternation guarantee, the
// real .sub spec additionally requires RAW_Data to START positive (first
// segment always HIGH). This project's capture can start at either level --
// whatever TAB5_RF433R_PIN happened to read the instant capture_start()
// armed the interrupt, not something this project controls -- so if the
// FIRST merged pulse is LOW, it is dropped rather than emitted with the
// wrong sign. This mirrors build_durations()'s own "no leading duration for
// edges[0]" reasoning one level further: a leading LOW segment records only
// dead air before the real signal starts, with no known dwell time
// preceding it (nothing in this CapturedSignal records how long the pin was
// already LOW before capture began), so treating it as unrepresentable
// costs no real information beyond what build_durations() already accepts
// losing for the same reason.
//
// static, not stack-local, for the same reason rf433_protocol_decode.cpp's
// decode() keeps its duration buffer static: ~4KB (kMaxDurations *
// sizeof(LevelPulse)) is fine off the stack but means this function is NOT
// reentrant -- a real constraint, not a live bug, given this project calls
// it from a single task's poll/save path today.
size_t build_signed_durations(const Rf433Scan::CapturedSignal &sig, int32_t *out, size_t max_out) {
    static LevelPulse merged[kMaxDurations];
    size_t merged_count = merge_pulses(sig, merged, kMaxDurations);
    if (merged_count == 0) return 0;

    size_t start = merged[0].level ? 0 : 1;
    size_t n = 0;
    for (size_t i = start; i < merged_count && n < max_out; i++) {
        int64_t signed_dur = merged[i].level ? static_cast<int64_t>(merged[i].duration)
                                              : -static_cast<int64_t>(merged[i].duration);
        out[n++] = static_cast<int32_t>(signed_dur);
    }
    return n;
}

// ── READ direction: text helpers ───────────────────────────────────────────

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

} // namespace

bool encode(const Rf433Scan::CapturedSignal &sig, char *buf, size_t buf_size, size_t *out_len) {
    if (buf == nullptr || buf_size == 0) return false;

    int32_t durations[kMaxDurations];
    size_t n = build_signed_durations(sig, durations, kMaxDurations);
    if (n == 0) return false;

    // Real spec's own guidance: up to 512 values per RAW_Data: line.
    // kMaxDurations (511) is always <= that, so exactly one RAW_Data: line
    // is always sufficient for this project's own captures -- multi-line
    // splitting is real spec behavior this WRITE side never needs to
    // exercise (decode(), below, still parses multiple RAW_Data: lines, for
    // reading real external files that do use them).
    static_assert(kMaxDurations <= 512, "encode() assumes one RAW_Data line suffices");

    int written = std::snprintf(buf, buf_size, "%s\n%s\nFrequency: %u\nPreset: %s\n%s\n%s",
                                 kFiletypeLine, kVersionLine, static_cast<unsigned>(kFrequencyHz),
                                 kPresetName, kProtocolLine, kRawDataPrefix);
    if (written < 0 || static_cast<size_t>(written) >= buf_size) return false;
    size_t pos = static_cast<size_t>(written);

    for (size_t i = 0; i < n; i++) {
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

bool decode(const char *text, size_t len, Rf433Scan::CapturedSignal *out) {
    if (text == nullptr || out == nullptr) return false;

    size_t pos = 0;
    const char *line = nullptr;
    size_t line_len = 0;

    // Header: Filetype, Version, Frequency, Preset, Protocol -- real spec's
    // own field order. Frequency/Preset are consumed (line present) but not
    // value-validated -- see this file's header comment.
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kFiletypeLine)) return false;
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kVersionLine)) return false;
    if (!next_line(text, len, &pos, &line, &line_len) || !starts_with(line, line_len, "Frequency:", 10)) return false;
    if (!next_line(text, len, &pos, &line, &line_len) || !starts_with(line, line_len, "Preset:", 7)) return false;
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kProtocolLine)) return false;

    // One or more RAW_Data: lines, concatenated -- real spec's own
    // continuation convention for captures over 512 values.
    static int32_t durations[kMaxDurations];
    size_t n = 0;
    bool truncated = false;
    bool saw_raw_data = false;

    while (next_line(text, len, &pos, &line, &line_len)) {
        if (line_len == 0) continue; // tolerate trailing blank lines
        if (!starts_with(line, line_len, kRawDataPrefix, kRawDataPrefixLen)) continue;
        saw_raw_data = true;

        size_t i = kRawDataPrefixLen;
        while (i < line_len) {
            while (i < line_len && (line[i] == ' ' || line[i] == '\t')) i++;
            if (i >= line_len) break;

            char *endp = nullptr;
            long v = std::strtol(line + i, &endp, 10);
            if (endp == line + i) return false; // non-numeric token -- malformed
            if (v == 0) return false;            // real spec: "values must be non-zero"

            if (n < kMaxDurations) {
                durations[n++] = static_cast<int32_t>(v);
            } else {
                truncated = true;
            }
            i = static_cast<size_t>(endp - line);
        }
    }
    if (!saw_raw_data || n == 0) return false;

    // Reconstruct EdgeSample[] from n signed durations -> n+1 edges (a
    // duration is the gap BETWEEN two edges, same relationship
    // build_durations() uses in the forward direction).
    //
    // First-edge decision (this module's own call, same as Task 7 had to
    // make its own for the forward direction): duration[0]'s sign directly
    // gives edges[0]'s level (duration[0] is HELD at edges[0].level, exactly
    // how build_durations() defines "duration[i] held at edges[i].level").
    // There is no absolute timestamp in a .sub file -- RAW_Data records only
    // relative durations -- so edges[0].timestamp_us is set to 0, an
    // arbitrary synthetic origin. This is not a loss of information: only
    // RELATIVE timing between edges is ever meaningful anywhere in this
    // project's RF433 code (capture, decode, or replay).
    //
    // For k = 1..n-1: edges[k] = {sum of |duration[0..k-1]|, sign(duration[k])}.
    // The final edge, edges[n], has no duration[n] to read a sign from --
    // only the fact that a transition happened, flipping the level relative
    // to duration[n-1] -- so edges[n].level = !sign(duration[n-1]).
    size_t edge_count = n + 1;
    if (edge_count > Rf433Scan::kMaxEdgesPerSignal) {
        // Defensive only -- unreachable given n is already capped at
        // kMaxDurations == kMaxEdgesPerSignal - 1 above.
        edge_count = Rf433Scan::kMaxEdgesPerSignal;
        truncated = true;
    }

    Rf433Scan::CapturedSignal result{};
    uint32_t t = 0;
    for (size_t k = 0; k < edge_count; k++) {
        if (k == 0) {
            result.edges[0] = Rf433Common::EdgeSample{0u, durations[0] > 0};
        } else if (k < n) {
            t += static_cast<uint32_t>(std::abs(durations[k - 1]));
            result.edges[k] = Rf433Common::EdgeSample{t, durations[k] > 0};
        } else { // k == n
            t += static_cast<uint32_t>(std::abs(durations[n - 1]));
            result.edges[k] = Rf433Common::EdgeSample{t, !(durations[n - 1] > 0)};
        }
    }
    result.edge_count = edge_count;
    result.captured_at_ms = 0;
    result.capture_id = 0;
    result.truncated = truncated;

    *out = result;
    return true;
}

bool write(IStorage &storage, const char *path, const Rf433Scan::CapturedSignal &sig) {
    // Static: kMaxDurations (511) signed values at up to 12 chars each
    // (" -2147483648") plus a small fixed header is ~6.5KB -- fine off the
    // stack, not fine to keep growing this function's own frame with.
    static char buf[8192];
    size_t len = 0;
    if (!encode(sig, buf, sizeof(buf), &len)) return false;
    return storage.write_capture_file(path, reinterpret_cast<const uint8_t *>(buf), len);
}

bool read(IStorage &storage, const char *path, Rf433Scan::CapturedSignal *out) {
    static char buf[8192];
    size_t len = 0;
    if (!storage.read_file(path, reinterpret_cast<uint8_t *>(buf), sizeof(buf), &len)) return false;
    return decode(buf, len, out);
}

} // namespace Rf433SubFormat
