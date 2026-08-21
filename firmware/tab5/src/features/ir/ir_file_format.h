#pragma once

#include "../../hal/istorage.h"
#include <cstddef>
#include <cstdint>

// ===========================================================================
// Flipper "IR signals file" (.ir) format read/write (Phase 3 Task 21).
//
// SOURCE: Flipper Devices' own firmware repository documentation,
// https://github.com/flipperdevices/flipperzero-firmware/blob/dev/documentation/file_formats/InfraredFileFormats.md
// (dev branch, fetched by the controller 2026-08-20 -- see
// task-21-controller-notes.md for the full citation and this project's donor
// checkouts' confirmed lack of this file format). Flipper's own public
// file-format documentation, not GPL-encumbered firmware source.
//
// SCOPE: Task 18 ("Universal remote / multi-profile clone (Flipper-IRDB)")
// is this format's eventual long-term owner, but it hasn't been built yet --
// gated behind Tasks 15/16/17, all hardware-blocked this session (the IR
// unit's chip identity is unknown until it physically arrives; Task 15 opens
// with its own PAUSE FOR HARDWARE). Per this task's controller notes'
// explicit ruling, Task 21 does not block on Task 18's non-existence:
// this module implements the real .ir format's read/write logic standalone
// now, deliberately independent of any IrLearn/IrClone type (none exist
// yet), structured so Task 18 can call into it later instead of duplicating
// it.
//
// FORMAT, real spec verbatim:
//   Filetype: IR signals file
//   Version: 1
// followed by one or more signals, each optionally preceded by a `#`
// comment line (the real sample file uses `#` between every signal,
// including before the first -- this module treats any line starting with
// '#' as a separator/comment and simply skips it, and separately detects a
// new signal starting whenever a `name:` line is seen while one is already
// in progress, so it does not depend on `#` being present between every
// pair). Two signal shapes:
//   Parsed (known protocol):
//     name: Button_1
//     type: parsed
//     protocol: NECext
//     address: EE 87 00 00
//     command: 5D A0 00 00
//   Raw (unrecognized/generic):
//     name: Button_2
//     type: raw
//     frequency: 38000
//     duty_cycle: 0.330000
//     data: 504 3432 502 483 500 484 510 502 502 482 501 485 509 1452 504 1458 509
// `frequency` is carrier Hz (typically 38000 for IR -- distinct from RF433's
// 433920000MHz-band value; do not confuse the two subsystems' framing).
// `duty_cycle` is a float, typically 0.33. `data` values are UNSIGNED
// microsecond timings between logic-level changes -- alternating mark/space
// is implicit from POSITION, not sign, unlike `.sub`'s signed RAW_Data --
// real spec's own stated max 1024 elements. All button names: printable
// ASCII only (real spec's own constraint; this module does not scan-enforce
// that beyond truncating to kNameMaxLen, an implementation bound the spec
// itself does not state).
// ===========================================================================

namespace IrFileFormat {

// Implementation-chosen bounds, NOT spec-stated, except kMaxRawSamples
// (the real spec's own 1024-element cap for `data`):
constexpr size_t kNameMaxLen = 40;
constexpr size_t kProtocolMaxLen = 24;  // matches this project's existing
                                         // DecodedCode::protocol_name[24]
                                         // convention (rf433_protocol_decode.h)
constexpr size_t kMaxAddressBytes = 8;  // real cited example uses 4 bytes;
                                         // sized generously above that -- not
                                         // a spec limit, other real protocols
                                         // in the wild use other widths
constexpr size_t kMaxRawSamples = 1024; // real spec's own stated max for `data`
constexpr size_t kMaxSignalsPerFile = 64; // enforced internal-processing bound:
                                           // decode() stops scanning further
                                           // signals once this many have been
                                           // parsed, independent of max_signals
                                           // (which separately controls how many
                                           // of those get copied into the
                                           // caller's out[] array)

enum class SignalType : uint8_t { kParsed, kRaw };

struct IrSignal {
    char name[kNameMaxLen];
    SignalType type;

    // type == kParsed
    char protocol[kProtocolMaxLen];
    uint8_t address[kMaxAddressBytes];
    size_t address_len;
    uint8_t command[kMaxAddressBytes];
    size_t command_len;

    // type == kRaw
    uint32_t frequency_hz;
    float duty_cycle;
    uint16_t data[kMaxRawSamples];
    size_t data_count;
    // True if this signal's `data` line had more values than kMaxRawSamples
    // -- excess dropped. Mirrors Rf433Scan::CapturedSignal::truncated's
    // naming/semantics.
    bool truncated;
};

// Pure in-memory parse, no SD I/O -- host-testable (see
// test/test_rf433_sub_format.cpp, which also covers this module). Parses
// text[0..len) (need not be NUL-terminated) into out[0..N), N = min(signals
// found, max_signals). Returns N (0 if the header doesn't match "Filetype:
// IR signals file" / "Version: 1" -- Version 1 only is supported, matching
// Rf433SubFormat's identical "reject unknown version rather than guess"
// discipline). *out_truncated (if non-null) is set true if the file had
// more signals than max_signals (extras dropped), more signals than
// kMaxSignalsPerFile (scanning stops there, an enforced internal bound --
// see kMaxSignalsPerFile above), or any individual raw signal's `data`
// exceeded kMaxRawSamples.
size_t decode(const char *text, size_t len, IrSignal *out, size_t max_signals, bool *out_truncated);

// Pure in-memory encode, no SD I/O -- host-testable. Encodes signals[0..count)
// into real .ir format text into buf, NUL-terminating. Returns false if it
// doesn't fit in buf_size or count == 0.
bool encode(const IrSignal *signals, size_t count, char *buf, size_t buf_size, size_t *out_len);

// SD-backed convenience wrappers, mirroring Rf433SubFormat::write()/read()'s
// IStorage-injection idiom for the same reason (keeps this module free of
// any concrete SD_MMC/Arduino dependency so it stays host-testable via a
// fake IStorage). This is the "load a single standalone .ir file from an
// arbitrary SD path" capability the plan's Task 21 text calls for --
// generalized to any path, not tied to a bundled community-database
// location a future Task 18 might also use.
//
// *out_truncated also picks up one more real case beyond decode()'s own:
// if the real file on SD was larger than read()'s internal working buffer,
// IStorage::read_file() caps *out_len at the buffer's capacity (its own
// documented contract) -- read() detects that (out_len == buffer capacity)
// and ORs it into *out_truncated too, so a caller can't mistake "read
// everything, decode()-truncated for other reasons" from "the file itself
// didn't fully fit in the read buffer" as "fully read."
size_t read(IStorage &storage, const char *path, IrSignal *out, size_t max_signals, bool *out_truncated);
bool write(IStorage &storage, const char *path, const IrSignal *signals, size_t count);

} // namespace IrFileFormat
