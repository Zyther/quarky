#include "rf433_sub_format.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

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
// edges[0]" reasoning one level further: EdgeSample.level is the level
// AFTER a transition (rf433_common.h), so a LOW first merged pulse means
// the first detected transition was FALLING -- an unrecorded rising edge
// (and whatever HIGH dwell preceded it) happened before capture armed, with
// no way to know how long that dwell was. That is not "no real information
// to lose" (dead air) -- it is genuinely unrepresentable per the real
// spec's own must-start-positive rule, so it is dropped rather than emitted
// with a fabricated/wrong sign, the same real constraint build_durations()
// already accepts losing for its own "no leading duration for edges[0]"
// case.
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

// Bounded strtol: decode()'s own contract (rf433_sub_format.h) says text[0..len)
// "need not be NUL-terminated" -- calling std::strtol directly on the last
// token in such a buffer (no trailing newline) would scan past the caller's
// declared bound looking for a non-digit, reading whatever bytes happen to
// sit after it (review-reported real bug: with Rf433SubFormat::read()'s
// static working buffer, those bytes can be stale content left over from a
// PREVIOUS, longer read). Copies at most kMaxTokenLen-1 bytes -- ample for
// any real int32_t RAW_Data token, sign included -- into a local
// NUL-terminated buffer before parsing, so strtol never sees memory beyond
// what the caller actually bounded. Returns false for a non-numeric token,
// OR if the digit run doesn't fit kMaxTokenLen-1 bytes and evidently
// continues past what was copied (a token this module can't represent
// rather than one it should silently mis-parse).
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

} // namespace

bool encode(const Rf433Scan::CapturedSignal &sig, char *buf, size_t buf_size, size_t *out_len) {
    if (buf == nullptr || buf_size == 0) return false;

    // static, not stack-local -- same reasoning as build_signed_durations()'s
    // own `merged` buffer just above: ~2KB (kMaxDurations * sizeof(int32_t))
    // is fine off the stack, not fine added to a caller's frame once this
    // runs on the LVGL/Arduino task's stack (Task 22). Not reentrant, same
    // real, disclosed, single-caller-today constraint as every other static
    // buffer in this file.
    static int32_t durations[kMaxDurations];
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

bool decode(const char *text, size_t len, Rf433Scan::CapturedSignal *out, bool *out_non_alternating) {
    if (out_non_alternating != nullptr) *out_non_alternating = false;
    if (text == nullptr || out == nullptr) return false;

    size_t pos = 0;
    const char *line = nullptr;
    size_t line_len = 0;

    // Filetype/Version are always the first two lines, real spec's own
    // fixed field order.
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kFiletypeLine)) return false;
    if (!next_line(text, len, &pos, &line, &line_len) || !line_equals(line, line_len, kVersionLine)) return false;

    // Remaining header lines (Frequency, Preset, and -- only for the real
    // spec's "RAW file, custom preset" file shape -- Custom_preset_module/
    // Custom_preset_data) are scanned for by key rather than assumed to sit
    // at fixed positions, so both real file shapes parse: the standard-preset
    // shape is just Frequency/Preset/Protocol; the custom-preset shape
    // inserts Custom_preset_module:/Custom_preset_data: lines between Preset:
    // and Protocol: (real spec's own documented example -- see this file's
    // header comment). None of these values are validated -- see this file's
    // top comment for why. A bounded scan (kMaxHeaderLines) guards against
    // spinning through an entire malformed file with no Protocol: line at all.
    constexpr int kMaxHeaderLines = 6; // Frequency, Preset, Custom_preset_module,
                                        // Custom_preset_data, Protocol, +1 slack
    bool saw_frequency = false;
    bool saw_preset = false;
    bool found_protocol = false;
    for (int header_lines = 0; header_lines < kMaxHeaderLines; header_lines++) {
        if (!next_line(text, len, &pos, &line, &line_len)) break;
        if (line_equals(line, line_len, kProtocolLine)) {
            found_protocol = true;
            break;
        }
        if (starts_with(line, line_len, "Protocol:", 9)) return false; // non-RAW protocol -- out of scope
        if (starts_with(line, line_len, "Frequency:", 10)) { saw_frequency = true; continue; }
        if (starts_with(line, line_len, "Preset:", 7)) { saw_preset = true; continue; }
        if (starts_with(line, line_len, "Custom_preset_module:", 21)) continue;
        if (starts_with(line, line_len, "Custom_preset_data:", 19)) continue;
        return false; // unrecognized header line -- malformed
    }
    if (!found_protocol || !saw_frequency || !saw_preset) return false;

    // One or more RAW_Data: lines, concatenated -- real spec's own
    // continuation convention for captures over 512 values.
    static int32_t durations[kMaxDurations];
    size_t n = 0;
    bool truncated = false;
    bool saw_raw_data = false;
    bool non_alternating = false;
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
            // Real spec: "values must be non-zero." Checked on the NARROWED
            // value (and the pre-narrow range rejected outright below) so a
            // value like 4294967296 -- non-zero as a 64-bit `long` on this
            // host, but which wraps to 0 when narrowed to int32_t -- can't
            // sneak past the zero-check and silently become a zero-length
            // duration.
            if (v < std::numeric_limits<int32_t>::min() || v > std::numeric_limits<int32_t>::max()) {
                return false; // out of this module's representable range
            }
            if (narrowed == 0) return false;

            bool positive = narrowed > 0;
            if (have_prev_sign && positive == prev_positive) non_alternating = true;
            have_prev_sign = true;
            prev_positive = positive;

            if (n < kMaxDurations) {
                durations[n++] = narrowed;
            } else {
                truncated = true;
            }
            i += consumed;
        }
    }
    if (!saw_raw_data || n == 0) return false;
    if (out_non_alternating != nullptr) *out_non_alternating = non_alternating;

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

    // static, not stack-local -- ~4KB (sizeof(CapturedSignal)), same
    // reasoning as every other buffer in this file (see build_signed_durations()'s
    // own comment): fine off the stack, not fine added to a caller's frame
    // once this runs on the LVGL/Arduino task's stack (Task 22). Entries at
    // indices >= edge_count may hold stale data from a previous call, same
    // as `durations` above -- never read, since callers only ever access
    // edges[0..edge_count).
    static Rf433Scan::CapturedSignal result{};
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
    if (!decode(buf, len, out)) return false;
    // IStorage::read_file()'s own documented contract (hal/istorage.h):
    // *out_len is capped at max_len if the real file was longer. len ==
    // sizeof(buf) is therefore this call's only signal that the file may
    // have been truncated -- flagged via the existing CapturedSignal::truncated
    // field rather than adding a new one (this project's established idiom).
    if (len == sizeof(buf)) out->truncated = true;
    return true;
}

} // namespace Rf433SubFormat
