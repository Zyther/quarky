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
// `RAW_Data` values are SIGNED microsecond durations: positive = level HIGH
// for that duration, negative = LOW. Must start positive (first segment is
// always HIGH) and must strictly alternate sign ("interleaved" -- two
// same-signed values in a row is invalid per the real spec). Values must be
// non-zero. Up to 512 values per RAW_Data: line; multiple RAW_Data: lines
// are used for longer captures (real spec's own guidance). This project's
// own capacity ceiling (kMaxDurations, below) is always <= 512, so write()
// never needs to split across lines -- read() still parses multiple
// RAW_Data: lines, for real interop with external files that do.
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
// zero-valued duration (spec: "values must be non-zero"), or a malformed
// (non-numeric) token. On success, *out is fully populated; captured_at_ms
// and capture_id are left 0 -- the .sub format carries neither, matching
// this project's own SD-round-trip precedent (rf433_scan.cpp's saved .raw
// files carry only edges too).
bool decode(const char *text, size_t len, Rf433Scan::CapturedSignal *out);

// SD-backed convenience wrappers around encode()/decode(). Take an IStorage&
// (dependency injection) rather than reaching for this project's usual
// `extern StorageSD storage;` global (see wifi_pmkid.cpp/
// wifi_evil_portal.cpp for that existing idiom) specifically so this whole
// module -- including these wrappers -- stays free of any concrete
// SD_MMC/Arduino dependency and can be exercised in the host-native test
// build via a fake IStorage, not just encode()/decode() in isolation.
bool write(IStorage &storage, const char *path, const Rf433Scan::CapturedSignal &sig);
bool read(IStorage &storage, const char *path, Rf433Scan::CapturedSignal *out);

} // namespace Rf433SubFormat
