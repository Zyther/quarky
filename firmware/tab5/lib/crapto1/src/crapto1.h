// ===========================================================================
// VENDORED THIRD-PARTY SOURCE -- provenance header added by this project.
// Everything below this block is byte-identical to the copy it was taken
// from; diff against the path named below to confirm (sha256 of the three
// source files at copy time is recorded in this task's report).
//
// [C1]  crapto1 -- the CRYPTO1 cipher and LFSR state-recovery math used by
//       essentially every open MIFARE Classic tool (mfoc / mfcuk / proxmark3
//       lineage).
//         Copyright (C) 2008-2014 bla <blapost@gmail.com>
//         License: GPLv2+   <-- COPYLEFT. This is NOT project-original code
//                               and is NOT under this repository's own terms.
//       The copyright/license lines above are quoted verbatim from the
//       original file headers immediately following this block -- they are
//       the primary citation, not a summary of one.
//
//       COPIED FROM: ~/src/unigeek-main/firmware/src/utils/crypto/
//       (crapto1.h, crapto1.c, crypto1.c) on 2026-08-21. UniGeek is one of
//       this program's three authorized donor firmwares (see CLAUDE.md);
//       Phase 3 Task 9's three attack ports (src/features/nfc/
//       nfc_mifare_crack.cpp) are written against exactly this copy, which
//       is why this copy -- rather than a fresh upstream pull -- is the one
//       vendored.
//
//       CANONICAL UPSTREAM. No single "official" crapto1 repository could be
//       confirmed to exist; the author (bla) distributed it through the
//       MIFARE-research tooling ecosystem rather than as a standalone
//       project, so NO upstream URL is asserted here. What WAS confirmed
//       directly, by reading the code, is that this copy matches the
//       long-standing canonical redistribution in nfc-tools/mfcuk:
//         https://github.com/nfc-tools/mfcuk/blob/master/src/crapto1.h
//         https://sources.debian.org/src/mfcuk/0.3.8+git20180720-1/src/crapto1.c/
//       Cross-checked 2026-08-21 and identical there: LF_POLY_ODD
//       (0x29CE5C), LF_POLY_EVEN (0x870804), BIT/BEBIT, parity(), filter()'s
//       five magic constants (0xf22c0 / 0x6c9c0 / 0x3c8b0 / 0x1e458 /
//       0x0d938 and the 0xEC57E80A output table), quicksort(), binsearch(),
//       update_contribution(), extend_table(), extend_table_simple(),
//       recover(), lfsr_recovery32(), lfsr_rollback_{bit,byte,word}(),
//       nonce_distance(), and all of crypto1.c (crypto1_create/destroy/
//       get_lfsr/bit/byte/word, prng_successor). mfcuk's header states
//       "Copyright (C) 2008-2009 bla <blapost@gmail.com>" where this copy
//       says 2008-2014 -- the only difference found in the header.
//
// !! ONE FUNCTION DIVERGES FROM UPSTREAM: lfsr_recovery64() !!
//       crapto1.c's lfsr_recovery64() in THIS copy is NOT the canonical
//       implementation. Upstream's version drives four precomputed constant
//       tables (S1/S2/T1/T2 plus C1/C2); this copy replaces that entirely
//       with a `uint32_t table[1 << 16]` bitmap plus an inline 16-round
//       verification loop, and leaves several of upstream's locals (hi, low,
//       win, tail) declared but unused. Whoever produced this copy rewrote
//       it. It is therefore UNVALIDATED against any real reference.
//       DO NOT CALL IT. Nothing in this firmware does: all three ported
//       attacks (nested, static-nested, darkside) use lfsr_recovery32()
//       only. It is kept solely so this file stays byte-identical to what
//       was vendored rather than silently edited. Two independent reasons
//       not to use it as-is: (1) correctness is unverified, and (2) that
//       256 KB `table[1 << 16]` is a STACK array -- it would overflow any
//       FreeRTOS task stack in this firmware instantly. Anyone who needs
//       64-bit recovery must re-derive it from the canonical upstream above
//       and re-cite it, not reach for this.
//
// MEMORY NOTE (real, and the reason this compiles unmodified on ESP32-P4).
//       lfsr_recovery32() malloc()s sizeof(uint32_t) << 21 twice (8 MB each)
//       plus sizeof(struct Crypto1State) << 18 (2 MB) -- ~18 MB peak. That
//       is upstream's own sizing, unchanged here. It lands in PSRAM
//       automatically on this target without touching this file: the
//       prebuilt framework config has CONFIG_SPIRAM_USE_MALLOC=y with
//       CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096 (verified in
//       framework-arduinoespressif32-libs/esp32p4/sdkconfig), so any single
//       allocation larger than 4 KB is served from external RAM. The caller
//       (nfc_mifare_crack.cpp) checks free PSRAM before invoking recovery
//       rather than trusting a 18 MB malloc to succeed -- see its
//       kRecoveryPsramBytes guard.
// ===========================================================================

/*  crapto1.h
    Copyright (C) 2008-2014 bla <blapost@gmail.com>
    License: GPLv2+
*/
#ifndef CRAPTO1_INCLUDED
#define CRAPTO1_INCLUDED
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

struct Crypto1State {uint32_t odd, even;};
struct Crypto1State* crypto1_create(uint64_t);
void crypto1_destroy(struct Crypto1State*);
void crypto1_get_lfsr(struct Crypto1State*, uint64_t*);
uint8_t crypto1_bit(struct Crypto1State*, uint8_t, int);
uint8_t crypto1_byte(struct Crypto1State*, uint8_t, int);
uint32_t crypto1_word(struct Crypto1State*, uint32_t, int);
uint32_t prng_successor(uint32_t x, uint32_t n);

struct Crypto1State* lfsr_recovery32(uint32_t ks2, uint32_t in);
struct Crypto1State* lfsr_recovery64(uint32_t ks2, uint32_t ks3);

uint8_t lfsr_rollback_bit(struct Crypto1State* s, uint32_t in, int fb);
uint8_t lfsr_rollback_byte(struct Crypto1State* s, uint32_t in, int fb);
uint32_t lfsr_rollback_word(struct Crypto1State* s, uint32_t in, int fb);
int nonce_distance(uint32_t from, uint32_t to);

#define LF_POLY_ODD (0x29CE5C)
#define LF_POLY_EVEN (0x870804)
#undef BIT
#define BIT(x, n) ((x) >> (n) & 1)
#define BEBIT(x, n) BIT(x, (n) ^ 24)

static inline int parity(uint32_t x)
{
  x ^= x >> 16;
  x ^= x >> 8;
  x ^= x >> 4;
  return BIT(0x6996, x & 0xf);
}

static inline int filter(uint32_t const x)
{
  uint32_t f;
  f  = 0xf22c0 >> (x       & 0xf) & 16;
  f |= 0x6c9c0 >> (x >>  4 & 0xf) &  8;
  f |= 0x3c8b0 >> (x >>  8 & 0xf) &  4;
  f |= 0x1e458 >> (x >> 12 & 0xf) &  2;
  f |= 0x0d938 >> (x >> 16 & 0xf) &  1;
  return BIT(0xEC57E80A, f);
}

#ifdef __cplusplus
}
#endif
#endif
