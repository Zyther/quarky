#pragma once

#include "../../hal/istorage.h"
#include "rf433_common.h"
#include "rf433_scan.h"
#include <cstddef>
#include <cstdint>

// ===========================================================================
// Flipper "SubGhz RAW File" (.sub) format read/write (Phase 3 Task 21 --
// numbered non-sequentially, inserted after Task 20 was drafted; see the
// plan's own Task 21 header for why).
//
// SOURCE: Flipper Devices' own firmware repository documentation,
// https://github.com/flipperdevices/flipperzero-firmware/blob/dev/documentation/file_formats/SubGhzFileFormats.md
// (dev branch, fetched by the controller 2026-08-20 -- this project's donor
// checkouts, ~/src/unigeek-main and ~/src/firmware (Bruce), contain only
// already-decoded pulse-timing logic (UniGeek's SubGhzDecoders.cpp, what
// Task 7's rf433_protocol_decode.cpp ports from) and no RAW *file* format or
// sample file at all -- confirmed by direct search before this task was
// written; see task-21-controller-notes.md for the full trail). This is
// Flipper's own public file-format documentation, not GPL-encumbered
// firmware source (contrast rf433_protocol_decode.h's GPLv3
// SubGhzDecoders.cpp note, which is about ported CODE).
//
// FORMAT, real spec verbatim:
//   Filetype: Flipper SubGhz RAW File
//   Version: 1
//   Frequency: 433920000
//   Preset: FuriHalSubGhzPresetOok650Async
//   Protocol: RAW
//   RAW_Data: 29262 361 -68 2635 -66 24113 -66 11 ...
// `Frequency` is an unsigned Hz value -- this project writes its own fixed,
// already-confirmed real center frequency (Phase 1's real-hardware listener
// test on TAB5_RF433R_PIN/TAB5_RF433T_PIN), kFrequencyHz below. `Preset` is a
// radio-preset name string; this project has no Flipper-compatible
// configurable-radio preset concept (bare GPIO OOK bit-bang, not a
// CC1101-class chip) -- kPresetName is written verbatim for real-Flipper-
// tooling compatibility only, and read() does not parse/validate it
// meaningfully (this project can't act on a different value anyway).
// `Protocol: RAW` is this module's whole scope -- Flipper's protocol-keyed
// (non-RAW) .sub files are a different format this project's capture/replay
// pipeline has no representation for, and are rejected by read().
//
// The real spec also documents a second header shape, "RAW file, custom
// preset": Preset becomes the literal `FuriHalSubGhzPresetCustom`, and two
// extra lines -- `Custom_preset_module:` (transceiver identifier, e.g.
// `CC1101`) and `Custom_preset_data:` (transceiver register bytes) -- are
// inserted between `Preset:` and `Protocol:`. This project's fixed
// bit-bang GPIO output never WRITES this shape (kPresetName is always the
// standard preset), but decode() reads it: header parsing scans for known
// keys rather than assuming fixed line positions, so a real external file
// using either shape parses.
//
// `RAW_Data` values are SIGNED microsecond durations: positive = level HIGH
// for that duration, negative = LOW. Must start positive (first segment is
// always HIGH) and must strictly alternate sign ("interleaved" -- two
// same-signed values in a row is invalid per the real spec). Values must be
// non-zero. Up to 512 values per RAW_Data: line; multiple RAW_Data: lines
// are used for longer captures (real spec's own guidance). UPDATED
// 2026-08-21: this project's own capacity ceiling (kMaxDurations, below) was
// raised past 512 (see Rf433Scan::kMaxEdgesPerSignal's own comment for the
// real-world capture that motivated it -- a genuine Tesla charge-port
// remote's real .sub file, 2395 duration values) -- write() now genuinely
// splits across multiple RAW_Data: lines too, matching the real spec's own
// convention read() already had to parse.
//
// CONVERSION REASONING (EdgeSample[] <-> signed RAW_Data), both directions
// documented in rf433_sub_format.cpp next to the code that implements them:
// the WRITE direction reuses rf433_protocol_decode.cpp's build_durations()
// merge-consecutive-same-level fix (see that function's own header comment)
// since the .sub spec's "interleaved" requirement needs the exact same
// strict-alternation guarantee that fix already established for a different
// reason; the READ direction's edge/timestamp reconstruction is this
// module's own first-edge decision, documented the same way Task 7 had to
// document its own.
// ===========================================================================

namespace Rf433SubFormat {

// This project's fixed, real-hardware-confirmed RF433 center frequency
// (Phase 1's real-hardware listener test). Written verbatim into every file
// this module produces.
constexpr uint32_t kFrequencyHz = 433920000u;

// Real Flipper preset name for standard 650kHz-bandwidth OOK -- the preset
// their own RAW .sub examples use. See this header's top comment for why
// this project writes it without being able to act on it.
extern const char kPresetName[];

// Maximum RAW_Data duration values this module emits/consumes for one
// CapturedSignal -- one fewer than kMaxEdgesPerSignal, identical reasoning
// to rf433_protocol_decode.cpp's kMaxDurations (a duration is the gap
// BETWEEN two edges). Always <= the real spec's 512-per-line cap.
constexpr size_t kMaxDurations = Rf433Scan::kMaxEdgesPerSignal - 1;

// Real worst-case text size for encode()/read()'s buffers: kMaxDurations
// values at up to 12 chars each ("-2147483648") + 1 leading space = 13
// bytes/value, plus a small fixed header (~150 bytes) and one extra
// "RAW_Data:\n" line prefix per 512 values once encode() splits (real spec's
// own per-line cap, see this header's own top comment). Rounded up
// generously past that worst case -- real captures use far fewer digits on
// average (the genuine Tesla remote capture that motivated raising
// kMaxDurations is 10991 bytes as a real .sub file, well under this).
// Scales with kMaxEdgesPerSignal (see that constant's own comment for the
// 4096->8192 real-PSRAM-headroom reasoning) -- kept proportional so this
// buffer never becomes the new bottleneck after a future edge-cap increase.
constexpr size_t kMaxEncodedTextBytes = 131072;

// ── Combine ("daisy-chain several signals into one .sub") sizing ──────────
//
// Added 2026-08-21: the project owner asked whether a COMBINED file (built
// from multiple captured signals via rf433_scan.cpp's "Select"/"Combine ->
// .sub" feature) should be allowed to exceed the per-signal edge cap
// (kMaxEdgesPerSignal) -- individual captures truncate at that real limit,
// but concatenating several of them is a genuinely different operation with
// no reason to inherit the SAME bound. kMaxCombinedEdges is the real worst
// case for that operation: kMaxChainSignals full-length signals back to
// back. At 8 signals * 8192 edges = 65536 edges, this costs ~512KB in
// PSRAM (32MB total on this hardware, confirmed via ESP.getPsramSize() --
// trivial) -- not an arbitrary separate guess, the actual sum of the parts.
constexpr size_t kMaxCombinedEdges = static_cast<size_t>(Rf433Scan::kMaxChainSignals) *
                                     Rf433Scan::kMaxEdgesPerSignal;
constexpr size_t kMaxCombinedDurations = kMaxCombinedEdges - 1;
// Scaled the same way kMaxEncodedTextBytes is for the single-signal case
// (13 bytes/value worst case + header + per-512-values line-prefix
// overhead), just for the larger combined edge count.
constexpr size_t kMaxCombinedEncodedTextBytes = 1048576;

// Pure in-memory conversion, no SD I/O -- host-testable (see
// test/test_rf433_sub_format.cpp). Encodes sig into real Flipper ".sub" RAW
// text (header + one RAW_Data: line) into buf, NUL-terminating. Returns
// false if sig has fewer than 2 edges, if every derived segment turns out to
// be the unrepresentable leading-LOW case (see .cpp), or if the encoded text
// would not fit in buf_size.
bool encode(const Rf433Scan::CapturedSignal &sig, char *buf, size_t buf_size, size_t *out_len);

// Pure in-memory conversion, no SD I/O -- host-testable. Parses real Flipper
// ".sub" RAW text (text[0..len), need not be NUL-terminated) into *out.
// Returns false if the text isn't a well-formed RAW .sub file this module
// supports: wrong Filetype/Version, non-RAW Protocol, no RAW_Data lines, a
// zero-valued (or out-of-int32_t-range) duration (spec: "values must be
// non-zero"), or a malformed (non-numeric) token. Header parsing tolerates
// both real file shapes the spec documents: the standard-preset shape
// (Frequency/Preset/Protocol) and the "RAW file, custom preset" shape,
// which inserts Custom_preset_module:/Custom_preset_data: lines between
// Preset: and Protocol: -- both are scanned for by key, not assumed to sit
// at fixed line positions. On success, *out is fully populated;
// captured_at_ms and capture_id are left 0 -- the .sub format carries
// neither, matching this project's own SD-round-trip precedent
// (rf433_scan.cpp's saved .raw files carry only edges too).
//
// *out_non_alternating (if non-null) is set true if the consumed RAW_Data
// values did not strictly alternate sign -- the real spec's own quoted
// example self-contradicts its "interleaved" rule (see this file's SOURCE
// comment and the .cpp's decode() comment), so this is deliberately a flag,
// not a rejection: parsing still succeeds, but a caller can distinguish
// "parsed cleanly" from "parsed, but the input didn't actually alternate."
bool decode(const char *text, size_t len, Rf433Scan::CapturedSignal *out,
            bool *out_non_alternating = nullptr);

// SD-backed convenience wrappers around encode()/decode(). Take an IStorage&
// (dependency injection) rather than reaching for this project's usual
// `extern StorageSD storage;` global (see wifi_pmkid.cpp/
// wifi_evil_portal.cpp for that existing idiom) specifically so this whole
// module -- including these wrappers -- stays free of any concrete
// SD_MMC/Arduino dependency and can be exercised in the host-native test
// build via a fake IStorage, not just encode()/decode() in isolation.
bool write(IStorage &storage, const char *path, const Rf433Scan::CapturedSignal &sig);
// If the real file on SD is larger than read()'s internal working buffer,
// the excess is silently unreadable through this call -- read() detects
// that case (the underlying IStorage::read_file() call reports exactly the
// buffer's capacity, its own documented signal that the file may have been
// longer) and reports it the same way this module already reports any
// other lossy read: out->truncated is set true, reusing this project's
// existing CapturedSignal::truncated idiom rather than adding a new flag.
bool read(IStorage &storage, const char *path, Rf433Scan::CapturedSignal *out);

// ── Raw-array entry points, for combined/chained signals ─────────────────
//
// Same real encoding logic as encode()/write() above (both now delegate to
// these), but operating directly on an edges[]/edge_count pair instead of
// requiring a CapturedSignal -- CapturedSignal's edges[] array is fixed at
// kMaxEdgesPerSignal, which is the right bound for one live capture but not
// for a combined file built from several of them (see kMaxCombinedEdges'
// own comment). edge_count may be up to kMaxCombinedEdges; behavior is
// otherwise identical to encode()/write() (neither of those took or used a
// "truncated" flag either -- that's this project's own CapturedSignal
// bookkeeping, not part of the real .sub text format itself, so there is
// nothing for these raw entry points to accept or emit for it).
bool encode_raw(const Rf433Common::EdgeSample *edges, size_t edge_count,
                 char *buf, size_t buf_size, size_t *out_len);
bool write_raw(IStorage &storage, const char *path, const Rf433Common::EdgeSample *edges,
               size_t edge_count);

} // namespace Rf433SubFormat
