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
// SCOPE: SubGhzDecoders.cpp ports 44 Flipper brand decoders total (many FSK-
// only, e.g. Marantec/Honeywell Security, which this project's OOK-only
// RF433R front end can never feed real data to -- see rf433_common.h's
// header comment on why capture here is edge-interrupt/OOK, not RMT/FSK).
// This port is deliberately narrower: exactly the six brands the Phase 3
// plan's own Task 7 Context line names -- Came, Nice (FLO), Linear,
// Chamberlain, Holtek, Ansonic -- all OOK, all in SubGhzDecoders.cpp's own
// kDecoders[] table (SubGhzDecoders.cpp:1813-1861). See rf433_protocol_
// decode.cpp's header comment for the exact line ranges ported for each.
// Not a claim that these are the only six protocols worth having; a later
// task can port more of kDecoders[] the same way if a real capture needs one
// that is missing.
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
