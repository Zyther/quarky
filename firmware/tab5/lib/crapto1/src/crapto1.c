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

/*  crapto1.c
    Copyright (C) 2008-2014 bla <blapost@gmail.com>
    License: GPLv2+
*/
#include "crapto1.h"
#include <stdlib.h>

static void quicksort(uint32_t* const start, uint32_t* const stop)
{
  uint32_t *it = start + 1, *rit = stop, t;
  if(it > rit) return;
  while(it < rit)
    if(*it <= *start) ++it;
    else if(*rit > *start) --rit;
    else t = *it, *it = *rit, *rit = t;
  if(*rit >= *start) --rit;
  if(rit != start) t = *rit, *rit = *start, *start = t;
  quicksort(start, rit - 1);
  quicksort(rit + 1, stop);
}

static inline uint32_t* binsearch(uint32_t *start, uint32_t *stop)
{
  uint32_t mid, val = *stop & 0xff000000;
  while(start != stop)
    if(start[mid = (stop - start) >> 1] > val)
      stop = &start[mid];
    else
      start += mid + 1;
  return start;
}

static inline void
update_contribution(uint32_t *item, const uint32_t mask1, const uint32_t mask2)
{
  uint32_t p = *item >> 25;
  p = p << 1 | parity(*item & mask1);
  p = p << 1 | parity(*item & mask2);
  *item = p << 24 | (*item & 0xffffff);
}

static inline void
extend_table(uint32_t *tbl, uint32_t **end, int bit, int m1, int m2, uint32_t in)
{
  in <<= 24;
  for(*tbl <<= 1; tbl <= *end; *++tbl <<= 1)
    if(filter(*tbl) ^ filter(*tbl | 1)) {
      *tbl |= filter(*tbl) ^ bit;
      update_contribution(tbl, m1, m2);
      *tbl ^= in;
    } else if(filter(*tbl) == bit) {
      *++*end = tbl[1];
      tbl[1] = tbl[0] | 1;
      update_contribution(tbl, m1, m2);
      *tbl++ ^= in;
      update_contribution(tbl, m1, m2);
      *tbl ^= in;
    } else
      *tbl-- = *(*end)--;
}

static inline void extend_table_simple(uint32_t *tbl, uint32_t **end, int bit)
{
  for(*tbl <<= 1; tbl <= *end; *++tbl <<= 1)
    if(filter(*tbl) ^ filter(*tbl | 1))
      *tbl |= filter(*tbl) ^ bit;
    else if(filter(*tbl) == bit) {
      *++*end = *++tbl;
      *tbl = tbl[-1] | 1;
    } else
      *tbl-- = *(*end)--;
}

static struct Crypto1State*
recover(uint32_t *o_head, uint32_t *o_tail, uint32_t oks,
  uint32_t *e_head, uint32_t *e_tail, uint32_t eks, int rem,
  struct Crypto1State *sl, uint32_t in)
{
  uint32_t *o, *e, i;
  if(rem == -1) {
    for(e = e_head; e <= e_tail; ++e) {
      *e = *e << 1 ^ parity(*e & LF_POLY_EVEN) ^ !!(in & 4);
      for(o = o_head; o <= o_tail; ++o, ++sl) {
        sl->even = *o;
        sl->odd = *e ^ parity(*o & LF_POLY_ODD);
        sl[1].odd = sl[1].even = 0;
      }
    }
    return sl;
  }
  for(i = 0; i < 4 && rem--; i++) {
    oks >>= 1; eks >>= 1; in >>= 2;
    extend_table(o_head, &o_tail, oks & 1,
           LF_POLY_EVEN << 1 | 1, LF_POLY_ODD << 1, 0);
    if(o_head > o_tail) return sl;
    extend_table(e_head, &e_tail, eks & 1,
           LF_POLY_ODD, LF_POLY_EVEN << 1 | 1, in & 3);
    if(e_head > e_tail) return sl;
  }
  quicksort(o_head, o_tail);
  quicksort(e_head, e_tail);
  while(o_tail >= o_head && e_tail >= e_head)
    if(((*o_tail ^ *e_tail) >> 24) == 0) {
      o_tail = binsearch(o_head, o = o_tail);
      e_tail = binsearch(e_head, e = e_tail);
      sl = recover(o_tail--, o, oks,
             e_tail--, e, eks, rem, sl, in);
    }
    else if(*o_tail > *e_tail)
      o_tail = binsearch(o_head, o_tail) - 1;
    else
      e_tail = binsearch(e_head, e_tail) - 1;
  return sl;
}

struct Crypto1State* lfsr_recovery32(uint32_t ks2, uint32_t in)
{
  struct Crypto1State *statelist;
  uint32_t *odd_head = 0, *odd_tail = 0, oks = 0;
  uint32_t *even_head = 0, *even_tail = 0, eks = 0;
  int i;

  for(i = 31; i >= 0; i -= 2)
    oks = oks << 1 | BEBIT(ks2, i);
  for(i = 30; i >= 0; i -= 2)
    eks = eks << 1 | BEBIT(ks2, i);

  odd_head = odd_tail = (uint32_t*)malloc(sizeof(uint32_t) << 21);
  even_head = even_tail = (uint32_t*)malloc(sizeof(uint32_t) << 21);
  statelist = (struct Crypto1State*)malloc(sizeof(struct Crypto1State) << 18);
  if(!odd_tail-- || !even_tail-- || !statelist) {
    free(statelist);
    statelist = 0;
    goto out;
  }

  statelist->odd = statelist->even = 0;

  for(i = 1 << 20; i >= 0; --i) {
    if(filter(i) == (oks & 1))
      *++odd_tail = i;
    if(filter(i) == (eks & 1))
      *++even_tail = i;
  }

  for(i = 0; i < 4; i++) {
    extend_table_simple(odd_head,  &odd_tail, (oks >>= 1) & 1);
    extend_table_simple(even_head, &even_tail, (eks >>= 1) & 1);
  }

  in = (in >> 16 & 0xff) | (in << 16) | (in & 0xff00);
  recover(odd_head, odd_tail, oks,
    even_head, even_tail, eks, 11, statelist, in << 1);

out:
  free(odd_head);
  free(even_head);
  return statelist;
}

struct Crypto1State* lfsr_recovery64(uint32_t ks2, uint32_t ks3)
{
  struct Crypto1State *statelist, *sl;
  uint8_t oks[32], eks[32], hi[32];
  uint32_t low = 0, win = 0;
  uint32_t *tail, table[1 << 16];
  int i, j;

  sl = statelist = (struct Crypto1State*)malloc(sizeof(struct Crypto1State) << 4);
  if(!sl) return 0;
  sl->odd = sl->even = 0;

  for(i = 30; i >= 0; i -= 2) {
    oks[i >> 1] = BEBIT(ks2, i);
    eks[i >> 1] = BEBIT(ks2, i + 1);
  }
  for(i = 30; i >= 0; i -= 2) {
    oks[16 + (i >> 1)] = BEBIT(ks3, i);
    eks[16 + (i >> 1)] = BEBIT(ks3, i + 1);
  }

  for(i = 0; i < 1 << 16; i++)
    table[i] = 0;

  for(i = 0; i < 1 << 20; i++) {
    uint32_t st = i;
    uint8_t ok = 1;
    for(j = 0; ok && j < 16; j++)
      if(filter(st) != oks[j]) ok = 0;
      else st = st << 1 | (j < 15 ? oks[j + 1] : 0);
    if(ok) {
      table[i >> 4] |= 1 << (i & 0xf);
    }
  }

  for(i = 0; i < 1 << 20; i++) {
    uint32_t st = i;
    uint8_t ok = 1;
    for(j = 0; ok && j < 16; j++)
      if(filter(st) != eks[j]) ok = 0;
      else st = st << 1 | (j < 15 ? eks[j + 1] : 0);
    if(ok) {
      uint32_t o = i, e;
      for(j = 0; j < 1 << 16; j++) {
        if(!(table[j >> 4] & (1 << (j & 0xf)))) continue;
        e = j << 4;
        /* verify remaining bits */
        struct Crypto1State cs;
        cs.odd = o; cs.even = e;
        uint8_t valid = 1;
        for(int k = 16; valid && k < 32; k++) {
          if(filter(cs.odd) != oks[k]) valid = 0;
          uint32_t feedin = LF_POLY_ODD & cs.odd;
          feedin ^= LF_POLY_EVEN & cs.even;
          cs.even = cs.even << 1 | parity(feedin);
          uint32_t t = cs.odd; cs.odd = cs.even; cs.even = t;
        }
        if(valid) {
          sl->odd = o;
          sl->even = e;
          sl++;
          sl->odd = sl->even = 0;
        }
      }
    }
  }

  return statelist;
}

uint8_t lfsr_rollback_bit(struct Crypto1State *s, uint32_t in, int fb)
{
  int out;
  uint8_t ret;
  uint32_t t;
  s->odd &= 0xffffff;
  t = s->odd, s->odd = s->even, s->even = t;
  out = s->even & 1;
  out ^= LF_POLY_EVEN & (s->even >>= 1);
  out ^= LF_POLY_ODD & s->odd;
  out ^= !!in;
  out ^= (ret = filter(s->odd)) & !!fb;
  s->even |= parity(out) << 23;
  return ret;
}

uint8_t lfsr_rollback_byte(struct Crypto1State *s, uint32_t in, int fb)
{
  int i, ret = 0;
  for (i = 7; i >= 0; --i)
    ret |= lfsr_rollback_bit(s, BIT(in, i), fb) << i;
  return ret;
}

uint32_t lfsr_rollback_word(struct Crypto1State *s, uint32_t in, int fb)
{
  int i;
  uint32_t ret = 0;
  for (i = 31; i >= 0; --i)
    ret |= lfsr_rollback_bit(s, BEBIT(in, i), fb) << (i ^ 24);
  return ret;
}

static uint16_t *dist = 0;
int nonce_distance(uint32_t from, uint32_t to)
{
  uint16_t x, i;
  if(!dist) {
    dist = (uint16_t*)malloc(2 << 16);
    if(!dist) return -1;
    for (x = i = 1; i; ++i) {
      dist[(x & 0xff) << 8 | x >> 8] = i;
      x = x >> 1 | (x ^ x >> 2 ^ x >> 3 ^ x >> 5) << 15;
    }
  }
  return (65535 + dist[to >> 16] - dist[from >> 16]) % 65535;
}