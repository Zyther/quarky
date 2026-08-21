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

/*  crypto1.c
    Copyright (C) 2008-2008 bla <blapost@gmail.com>
    License: GPLv2+
*/
#include "crapto1.h"
#include <stdlib.h>

#define SWAPENDIAN(x)\
  (x = (x >> 8 & 0xff00ff) | (x & 0xff00ff) << 8, x = x >> 16 | x << 16)

struct Crypto1State* crypto1_create(uint64_t key)
{
  struct Crypto1State *s = malloc(sizeof(*s));
  int i;
  for(i = 47; s && i > 0; i -= 2) {
    s->odd  = s->odd  << 1 | BIT(key, (i - 1) ^ 7);
    s->even = s->even << 1 | BIT(key, i ^ 7);
  }
  return s;
}

void crypto1_destroy(struct Crypto1State *state)
{
  free(state);
}

void crypto1_get_lfsr(struct Crypto1State *state, uint64_t *lfsr)
{
  int i;
  for(*lfsr = 0, i = 23; i >= 0; --i) {
    *lfsr = *lfsr << 1 | BIT(state->odd, i ^ 3);
    *lfsr = *lfsr << 1 | BIT(state->even, i ^ 3);
  }
}

uint8_t crypto1_bit(struct Crypto1State *s, uint8_t in, int is_encrypted)
{
  uint32_t feedin;
  uint8_t ret = filter(s->odd);
  feedin  = ret & !!is_encrypted;
  feedin ^= !!in;
  feedin ^= LF_POLY_ODD & s->odd;
  feedin ^= LF_POLY_EVEN & s->even;
  s->even = s->even << 1 | parity(feedin);
  s->odd ^= (s->odd ^= s->even, s->even ^= s->odd);
  return ret;
}

uint8_t crypto1_byte(struct Crypto1State *s, uint8_t in, int is_encrypted)
{
  uint8_t i, ret = 0;
  for (i = 0; i < 8; ++i)
    ret |= crypto1_bit(s, BIT(in, i), is_encrypted) << i;
  return ret;
}

uint32_t crypto1_word(struct Crypto1State *s, uint32_t in, int is_encrypted)
{
  uint32_t i, ret = 0;
  for (i = 0; i < 32; ++i)
    ret |= crypto1_bit(s, BEBIT(in, i), is_encrypted) << (i ^ 24);
  return ret;
}

uint32_t prng_successor(uint32_t x, uint32_t n)
{
  SWAPENDIAN(x);
  while(n--)
    x = x >> 1 | (x >> 16 ^ x >> 18 ^ x >> 19 ^ x >> 21) << 31;
  return SWAPENDIAN(x);
}
