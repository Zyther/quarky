#pragma once

#include <cstdint>

// ===========================================================================
// SubghzProto protocol decode (Phase 10 Task 2) -- CC1101's own named
// protocol scope (Phase 5 spec Section 1: Princeton, CAME, NiceFLO, Linear,
// Chamberlain, Holtek, Ansonic -- 7 protocols total).
//
// PROVENANCE, SPLIT ACROSS TWO SOURCES:
//
//   Six of the seven decoders here (Holtek, Holtek HT12X, CAME, Nice FLO,
//   Chamberlain, Ansonic, Linear) are EXTRACTED -- moved verbatim, not
//   retyped -- from this project's own firmware/tab5/src/features/rf433/
//   rf433_protocol_decode.cpp (Phase 3 Task 7), which already ported and
//   real-hardware-tested them. That file's own header comment documents the
//   original donor citation and its own real-hardware verification history;
//   see it for that record. As of this extraction, rf433_protocol_decode.cpp
//   becomes a thin consumer of this library (see that file's own updated
//   header comment).
//
//   The seventh, Princeton, is ported FRESH for this task from
//   ~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.cpp's real
//   decode_princeton() (confirmed at line 78 of that file as read for this
//   task, 2026-08-25 -- donor files can shift, so this was checked directly
//   rather than trusted from the plan's own citation, which happened to
//   match). Bit-timing constants (te_short=390, te_long=1170, te_delta=300)
//   and the double-frame (last_data == data) match requirement are copied
//   verbatim from that function, not re-derived.
//
// Both sources are themselves ports of Flipper Zero firmware
// (https://github.com/flipperdevices/flipperzero-firmware,
// lib/subghz/protocols/*, GPLv3) -- SubGhzDecoders.cpp's own header
// (SubGhzDecoders.h:1-14) documents this provenance directly. This project
// has already disclosed and accepted this exact licensing question (Phase 3
// Task 21's own history) -- not re-litigated here, see rf433_protocol_
// decode.h's own LICENSING NOTE for the original disclosure.
//
// ORDERING: the donor's real kDecoders[] table (SubGhzDecoders.cpp:
// 1813-1861) orders decoders most-specific-first so a loose-tolerance
// decoder can't grab a frame another would parse correctly. This library
// preserves the SAME RELATIVE ORDER the donor uses among these 7:
//   decode_holtek, decode_princeton, decode_holtek_ht12x, decode_came,
//   decode_nice_flo, decode_chamberlain, decode_ansonic, decode_linear
// (donor line numbers: holtek=1828, princeton=1842, holtek_ht12x=1846,
// came=1850, nice_flo=1851, chamberlain=1852, ansonic=1858, linear=1860).
// In particular, decode_holtek_ht12x MUST precede decode_came (donor's own
// comment, SubGhzDecoders.cpp:1846-1849): the two share IDENTICAL 320/640us
// bit timing and are distinguished only by preamble length -- without
// HT12X ordered ahead, a real HT12X frame would silently decode AS "CAME",
// a confidently wrong answer, not a "no match". See
// subghz_protocol_decode.cpp's own header comment for the full reasoning
// (carried over from rf433_protocol_decode.cpp's original).
// ===========================================================================

namespace SubghzProto {

// Match produced by a single decoder -- mirrors SubGhzDecoders::Match
// (SubGhzDecoders.h:30-35) exactly, including the `te` (short-pulse timing)
// field the donor's Match carries that rf433_protocol_decode.cpp's own
// narrower internal Match struct did not previously expose.
struct Match {
    const char *name = nullptr;
    uint64_t key = 0;
    uint8_t bits = 0;
    uint16_t te = 0;
};

// Attempts to decode a raw alternating-duration pulse train against the
// ported brand decoder table. Tries both signal phases (the capture's
// starting level is unknown -- see subghz_protocol_decode.cpp's
// sample_level() for why) and every decoder in table order, returning on
// the first match. On success, fills *out and returns true; on no match (or
// if out is null), *out is left untouched and this returns false.
//
// count < 8 always fails -- mirrors SubGhzDecoders::decode()'s own guard
// (SubGhzDecoders.cpp:1866): not enough durations for any decoder to
// self-sync, regardless of brand.
bool decode(const unsigned int *dur, uint16_t count, Match *out);

} // namespace SubghzProto
