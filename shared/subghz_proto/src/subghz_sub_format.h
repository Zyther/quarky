#pragma once

#include <cstddef>
#include <cstdint>

// ===========================================================================
// Flipper "SubGhz RAW File" (.sub) format read/write, GENERALIZED for
// CC1101's variable frequency/preset (Phase 10 Task 2).
//
// This is a PARALLEL implementation to firmware/tab5/src/features/rf433/
// rf433_sub_format.{h,cpp} (Phase 3 Task 21), not a migration of it --
// Task 21's own module stays exactly as-is (RF433 is permanently fixed at
// 433.92MHz, one preset string) and is NOT touched by this task. Read that
// header's own top comment first: it already documents, from Flipper
// Devices' own real file-format documentation
// (https://github.com/flipperdevices/flipperzero-firmware/blob/dev/
// documentation/file_formats/SubGhzFileFormats.md, dev branch), the real
// field grammar (Filetype:/Version:/Frequency:/Preset:/Protocol:/RAW_Data:),
// both real header shapes (standard preset vs. the "RAW file, custom preset"
// shape that inserts Custom_preset_module:/Custom_preset_data: lines between
// Preset: and Protocol:), the signed-duration/must-start-positive/
// interleaving rules, and the multi-line RAW_Data: splitting convention (up
// to 512 values per line) -- all of that real research is reused here
// unchanged, just parameterized instead of hardcoded, since it is the exact
// same real file format.
//
// WHAT'S DIFFERENT HERE, AND WHY: rf433_sub_format.cpp hardcodes
// kFrequencyHz (433920000, RF433's one real confirmed center frequency) and
// kPresetName ("FuriHalSubGhzPresetOok650Async", the only preset RF433's
// fixed bit-bang GPIO output can represent). CC1101 is a real
// variable-frequency radio (855-925MHz, per this phase's spec) with more
// than one real preset, so encode_sub()/decode_sub() take both as
// parameters instead.
//
// RAW IS A FIRST-CLASS PATH (explicit project-owner requirement,
// 2026-08-25): encode_sub()/decode_sub() work directly off raw edge/duration
// arrays and are NOT gated on a successful SubghzProto::decode() protocol
// match -- an undecoded capture must still be fully encodable as a real
// "Protocol: RAW" .sub file. This mirrors rf433_sub_format.cpp's own scope
// exactly (it is Protocol:RAW only -- it never even attempts protocol-keyed
// writing) and every real donor project's own behavior (raw capture/replay
// is always the fallback for a signal no decoder recognizes).
//
// CROSS-BOUNDARY TYPE DECISION: this library must stay independent of the
// firmware/tab5 tree (host-testable, no Arduino dependency -- see this
// directory's platformio.ini). Rather than take a dependency on
// firmware/tab5's Rf433Common::EdgeSample (which would break that
// independence) or on Rf433Scan::CapturedSignal (fixed-size, RF433-specific,
// and equally out of bounds), this module defines its own plain
// SubghzProto::EdgeSample -- same two fields, same semantics
// (timestamp_us + "level AFTER the edge"), zero dependency either way. This
// follows the same established pattern shared/feature_contract and
// shared/c2proto already use: both define their own plain, dependency-free
// types (FeatureModule, c2proto::Frame) rather than reaching into
// firmware/tab5 for equivalents, even though firmware/tab5 has its own
// versions of similar concepts.
// ===========================================================================

namespace SubghzProto {

// Plain, dependency-free edge sample -- see this file's header comment for
// why this is its own type rather than a reuse of Rf433Common::EdgeSample.
struct EdgeSample {
    uint32_t timestamp_us;
    bool level; // GPIO/radio level AFTER the edge (the level this sample transitioned TO)
};

// Pure in-memory conversion, no SD I/O -- host-testable (see
// test/test_subghz_sub_format.cpp). Encodes edges[0..edge_count) into real
// Flipper ".sub" RAW text (header + one or more RAW_Data: lines, per the
// real spec's 512-values-per-line convention) into buf, NUL-terminating.
//
// freq_hz is written verbatim as the Frequency: field (Hz, unsigned decimal
// -- CC1101's real variable frequency, unlike rf433_sub_format.cpp's fixed
// 433920000). preset is written verbatim as the Preset: field (must not be
// null) -- same "written for real-Flipper-tooling compatibility, not acted
// on" caveat rf433_sub_format.h documents for kPresetName applies here too.
//
// Returns false if edges is null, edge_count < 2, buf/buf_size are
// invalid, preset is null, every derived segment turns out to be the
// unrepresentable leading-LOW case (see .cpp -- same real "must start
// positive" spec rule rf433_sub_format.cpp already established), or the
// encoded text would not fit in buf_size.
bool encode_sub(uint32_t freq_hz, const char *preset, const EdgeSample *edges, size_t edge_count,
                 char *buf, size_t buf_size, size_t *out_len);

// Pure in-memory conversion, no SD I/O -- host-testable. Parses real Flipper
// ".sub" RAW text (text[0..len), need not be NUL-terminated) into
// *freq_hz_out and edges_out[0..*edge_count_out). edges_out is a
// CALLER-SUPPLIED buffer of capacity edges_capacity -- this module has no
// fixed internal edge cap of its own (unlike rf433_sub_format.cpp's
// kMaxEdgesPerSignal-derived constants, which exist because that module owns
// a fixed-size CapturedSignal; this one does not), so the caller's own real
// capture-buffer size is the only real bound that applies. If the file's
// real RAW_Data content would produce more edges than edges_capacity, the
// excess is silently dropped (edges_out is filled up to edges_capacity and
// *edge_count_out is capped there) -- there is no truncation-flag output
// parameter in this signature (unlike rf433_sub_format.cpp's
// CapturedSignal::truncated), so callers that need to detect this should
// size edges_capacity generously or compare *edge_count_out against what
// they expected.
//
// Returns false if the text isn't a well-formed RAW .sub file this module
// supports: wrong Filetype/Version, non-RAW Protocol, no RAW_Data lines, a
// zero-valued (or out-of-int32_t-range) duration (real spec: "values must be
// non-zero"), or a malformed (non-numeric) token. Header parsing tolerates
// both real file shapes the spec documents (see this file's top comment).
// Does not enforce pairwise sign alternation on read (same real-spec-example
// self-contradiction rf433_sub_format.cpp's own decode() already found and
// documented -- each token's sign is consumed independently).
bool decode_sub(const char *text, size_t len, uint32_t *freq_hz_out, EdgeSample *edges_out,
                 size_t edges_capacity, size_t *edge_count_out);

} // namespace SubghzProto
