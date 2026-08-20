#pragma once

#include "rf433_scan.h"
#include <cstdint>

// ===========================================================================
// RF433 protocol decode (Phase 3 Task 7): turns a Task 5 CapturedSignal's raw
// edge timing into a recognized brand/protocol name + code value, the same
// way a Flipper Zero (or this donor's own CC1101Util::pollReceive()) labels a
// captured OOK/ASK frame instead of leaving it as an opaque pulse train.
//
// SOURCE, AND A REAL CITATION CORRECTION: the Phase 3 plan's own Task 7 text
// says to port this from UniGeek's `utils/rf/M5RF433Util.*`. That is wrong --
// checked directly, M5RF433Util.h/.cpp is RMT TX/RX plumbing only (drives the
// RMT peripheral for send/receive, delegates encoding to RCSwitchUtil) and
// contains no brand-specific timing tables at all. The controller confirmed
// the real file and ruled to port from it instead:
//
//   [SGD] ~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.h / .cpp
//         Header comment (SubGhzDecoders.h:1-14): "brand/manufacturer
//         protocol decoders ported from Flipper Zero firmware
//         (lib/subghz/protocols/*)... labels the Signal with the real
//         protocol name (CAME, Princeton, Nice FLO, Holtek, Linear, ...)".
//         Reference cited in that same header: Flipper Zero firmware,
//         https://github.com/flipperdevices/flipperzero-firmware,
//         lib/subghz/protocols/ -- GPLv3.
//
// LICENSING NOTE (surfaced, not resolved here -- project owner's call):
// SubGhzDecoders.cpp is itself a port of Flipper Zero's GPLv3-licensed
// lib/subghz/protocols/*.c decoders. Every prior donor port in this project
// (Bruce, UniGeek's own non-Flipper code, ST/NXP datasheets) has been either
// public-domain or an original reimplementation citing a datasheet -- this is
// the first GPLv3-sourced content flowing into this codebase. The project
// owner has been informed this file (and its .cpp) carry that provenance;
// this comment exists so the fact stays attached to the code, not so it can
// be missed later.
//
// SCOPE, CORRECTED (round-2 review finding): this port is scoped to the
// Phase 3 plan's own Task 7 Context line's seven named brands -- Came, Nice
// (FLO), Linear, Chamberlain, Holtek (both the 40-bit HT12 and the 12-bit
// HT12X variant), Ansonic -- all OOK, all in SubGhzDecoders.cpp's own
// kDecoders[] table (SubGhzDecoders.cpp:1813-1861). The remaining ~42
// decoders (2 FSK -- Honeywell Security, Marantec -- and ~40 OOK, including
// common real protocols like Princeton) are OUT OF SCOPE FOR THIS TASK, not
// unreachable by this hardware. An earlier version of this comment (and the
// commit message) incorrectly claimed the unported set was "largely
// FSK-only" -- that was never checked against the donor file and was wrong;
// verified by direct count, only 2 of 44 are FSK. See rf433_protocol_
// decode.cpp's header comment for the exact line ranges ported for each of
// the seven. A later task can port more of kDecoders[] the same way if a
// real capture needs one that is missing.
//
// REACHABILITY CAVEAT (documented, not fixed -- round-2 review ruling): Task
// 5's burst-splitting threshold (rf433_scan.cpp's kBurstGapThresholdUs,
// 25ms) means not every ported decoder's full documented sync window can
// ever arrive intact in one CapturedSignal. Chamberlain's real sync window
// is 35-43ms (ts=1000, DDIFF(d, ts*39) < td*20 => 35000-43000us window
// centered at 39000us in decode_chamberlain()'s Reset check) -- entirely
// above the 25ms threshold, so a genuine Chamberlain preamble is always
// split across two captured bursts before this decoder ever sees it whole;
// it is effectively unreachable via this project's current capture
// pipeline. Nice FLO's window (te_short*36 +/- te_delta*36 = 18.0-32.4ms)
// and Linear's window (te_short*42 +/- te_delta*15 = 15.75-26.25ms) straddle
// the same 25ms threshold rather than sitting entirely above it, so they are
// reachable only via intra-press repeat gaps that happen to land under the
// threshold, not their full documented sync window. This is a real gap
// between Task 5's capture layer and Task 7's decoders, left unfixed this
// round because fixing it means changing already real-hardware-verified
// Task 5 code with no way to re-verify against hardware this round
// (standing no-flash instruction) -- see the Task 7 fix-round report for the
// full reasoning.
// ===========================================================================

namespace Rf433ProtocolDecode {

struct DecodedCode {
    char protocol_name[24];
    uint64_t code;
    uint8_t bit_length;
};

// Attempts to decode `sig`'s captured edge timing against the ported brand
// decoder table. Tries both signal phases (the capture's starting level is
// unknown -- see rf433_protocol_decode.cpp's sample_level() for why) and
// every decoder in table order, returning on the first match. On success,
// fills *out and returns true; on no match, *out is left untouched and this
// returns false.
//
// sig.edge_count < 2 (fewer than one full pulse) always fails: see .cpp.
bool decode(const Rf433Scan::CapturedSignal &sig, DecodedCode *out);

} // namespace Rf433ProtocolDecode
