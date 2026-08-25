#include "subghz_protocol_decode.h"
#include <cstring>

// ===========================================================================
// See subghz_protocol_decode.h for the full provenance note (which of these
// 8 decoders were extracted from rf433_protocol_decode.cpp verbatim vs.
// ported fresh for this task, and the real donor line citations for each).
//
// Six decoders (Holtek, Holtek HT12X, CAME, Nice FLO, Chamberlain, Ansonic,
// Linear) were EXTRACTED verbatim from firmware/tab5/src/features/rf433/
// rf433_protocol_decode.cpp (Phase 3 Task 7's own real, tested port) --
// function bodies, comments, and cited donor line ranges (from
// ~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.cpp) are unchanged
// from that file:
//
//   DDIFF macro                       SubGhzDecoders.cpp:4
//   sampleLevel()                     SubGhzDecoders.cpp:8-10
//   decode_came()   (CAME/Prastel/    SubGhzDecoders.cpp:14-73
//                    Airforce)
//   decode_nice_flo()                 SubGhzDecoders.cpp:130-181
//   decode_holtek()  (Holtek HT12)    SubGhzDecoders.cpp:187-237
//   decode_linear()                   SubGhzDecoders.cpp:241-289
//   decode_ansonic()                  SubGhzDecoders.cpp:292-323
//   chamb_to_bit() + decode_chamberlain()
//                                     SubGhzDecoders.cpp:1174-1228
//   decode_holtek_ht12x()  (Holtek    SubGhzDecoders.cpp:1102-1136
//                    HT12X, 12-bit)
//
// The seventh, decode_princeton(), is ported FRESH for this task from the
// same donor file, real function at SubGhzDecoders.cpp:78-126 (confirmed by
// direct read 2026-08-25, matching the plan's own "around line 78"
// citation). te_short=390, te_long=1170, te_delta=300, the 24-bit double-
// frame requirement (a match is only accepted once the SAME 24-bit value
// repeats back-to-back -- last_data == data, mirroring decode_holtek_ht12x's
// own double-frame pattern below), and m.te = te_short are all copied
// verbatim from that function, not re-derived.
//
// kDecoders[] table ordering       SubGhzDecoders.cpp:1813-1861
// SubGhzDecoders::decode() engine  SubGhzDecoders.cpp:1864-1882
//   (two-phase try loop, count < 8 => no match)
//
// The donor decoders are tried "most-specific first" (SubGhzDecoders.cpp:
// 1809-1812 comment) so a loose-tolerance decoder can't grab a frame another
// would parse correctly; this file preserves the SAME RELATIVE ORDER among
// the eight ported here: Holtek, Princeton, Holtek HT12X, CAME, Nice FLO,
// Chamberlain, Ansonic, then Linear last (Linear is explicitly commented
// "loose tolerance -- last" at SubGhzDecoders.cpp:1860).
//
// decode_holtek_ht12x was added to the extracted set in a round-2 review fix
// during Phase 3 Task 7, not that task's original port: the donor's own
// kDecoders[] table (SubGhzDecoders.cpp:1846-1849) carries an explicit
// comment that HT12X MUST be tried immediately before CAME, because the two
// share IDENTICAL 320/640us bit timing and are distinguished only by
// preamble length -- HT12X's sync window (ts=320, DDIFF(d, ts*28) < td*20 =>
// 4960-12960us) sits entirely inside CAME's (te_short=320, DDIFF(d,
// te_short*56) < te_delta*63 => 8450-27370us). Without HT12X ported and
// ordered ahead of CAME, a real HT12X remote's frame would silently decode
// AS "CAME" -- a confidently wrong answer, not a "no match" -- because
// CAME's wider preamble window also accepts HT12X's shorter one. Preserved
// here the same way the donor orders it: immediately before decode_came,
// and (per the donor's real kDecoders[] table order) immediately after
// decode_princeton.
// ===========================================================================

namespace SubghzProto {

namespace {

// abs difference of two unsigned durations. SubGhzDecoders.cpp:4.
#define DDIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

// Level of sample i for a given phase. The capture's starting level is
// unknown, so the engine tries phase 0 and 1; even index = HIGH within a
// phase. SubGhzDecoders.cpp:8-10.
inline bool sample_level(uint16_t i, uint8_t phase) {
    return (((uint16_t)(i + phase)) & 1u) == 0u;
}

// ── CAME / Prastel / Airforce ───────────────────────────────────────────
// Port of subghz_protocol_decoder_came_feed. SubGhzDecoders.cpp:14-73.
bool decode_came(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t te_short = 320, te_long = 640, te_delta = 150;
    enum { Reset, FoundStart, SaveDur, CheckDur };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0;
    uint8_t cnt = 0;

    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t duration = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(duration, te_short * 56) < te_delta * 63)
                    step = FoundStart;
                break;
            case FoundStart:
                if (!level) {
                    break;
                } else if (DDIFF(duration, te_short) < te_delta) {
                    step = SaveDur;
                    data = 0;
                    cnt = 0;
                } else {
                    step = Reset;
                }
                break;
            case SaveDur:
                if (!level) {
                    if (duration >= te_short * 4) {
                        step = FoundStart;
                        if (cnt == 12 || cnt == 18 || cnt == 25 || cnt == 42 || cnt == 24) {
                            m.name = "CAME";
                            if (cnt == 25 || cnt == 42) m.name = "Prastel";
                            else if (cnt == 18) m.name = "Airforce";
                            m.key = data;
                            m.bits = cnt;
                            m.te = (uint16_t)te_short;
                            return true;
                        }
                        break;
                    }
                    te_last = duration;
                    step = CheckDur;
                } else {
                    step = Reset;
                }
                break;
            case CheckDur:
                if (level) {
                    if (DDIFF(te_last, te_short) < te_delta && DDIFF(duration, te_long) < te_delta) {
                        data = data << 1 | 0;
                        cnt++;
                        step = SaveDur;
                    } else if (DDIFF(te_last, te_long) < te_delta && DDIFF(duration, te_short) < te_delta) {
                        data = data << 1 | 1;
                        cnt++;
                        step = SaveDur;
                    } else {
                        step = Reset;
                    }
                } else {
                    step = Reset;
                }
                break;
        }
    }
    return false;
}

// ── Princeton ────────────────────────────────────────────────────────────
// Ported fresh for this task (not extracted -- the one genuinely new
// protocol; see this file's header comment). Port of
// subghz_protocol_decoder_princeton_feed. Requires two identical frames
// before declaring a match (Flipper's last_data == decode_data guard).
// SubGhzDecoders.cpp:78-126 (confirmed by direct read 2026-08-25).
bool decode_princeton(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t te_short = 390, te_long = 1170, te_delta = 300;
    enum { Reset, SaveDur, CheckDur };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0, last_data = 0;
    uint8_t cnt = 0;

    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t duration = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(duration, te_short * 36) < te_delta * 36) {
                    step = SaveDur;
                    data = 0;
                    cnt = 0;
                }
                break;
            case SaveDur:
                if (level) {
                    te_last = duration;
                    step = CheckDur;
                }
                break;
            case CheckDur:
                if (!level) {
                    if (duration >= te_long * 2) {
                        step = SaveDur;
                        if (cnt == 24) {
                            if (last_data == data && last_data) {
                                m.name = "Princeton";
                                m.key = data;
                                m.bits = 24;
                                m.te = (uint16_t)te_short;
                                return true;
                            }
                            last_data = data;
                        }
                        data = 0;
                        cnt = 0;
                        break;
                    }
                    if (DDIFF(te_last, te_short) < te_delta && DDIFF(duration, te_long) < te_delta * 3) {
                        data = data << 1 | 0;
                        cnt++;
                        step = SaveDur;
                    } else if (DDIFF(te_last, te_long) < te_delta * 3 && DDIFF(duration, te_short) < te_delta) {
                        data = data << 1 | 1;
                        cnt++;
                        step = SaveDur;
                    } else {
                        step = Reset;
                    }
                } else {
                    step = Reset;
                }
                break;
        }
    }
    return false;
}

// ── Nice FLO ─────────────────────────────────────────────────────────────
// Port of subghz_protocol_decoder_nice_flo_feed. SubGhzDecoders.cpp:130-181.
bool decode_nice_flo(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t te_short = 700, te_long = 1400, te_delta = 200;
    enum { Reset, FoundStart, SaveDur, CheckDur };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0;
    uint8_t cnt = 0;

    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t duration = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(duration, te_short * 36) < te_delta * 36) step = FoundStart;
                break;
            case FoundStart:
                if (!level) break;
                else if (DDIFF(duration, te_short) < te_delta) {
                    step = SaveDur;
                    data = 0;
                    cnt = 0;
                } else step = Reset;
                break;
            case SaveDur:
                if (!level) {
                    if (duration >= te_short * 4) {
                        step = FoundStart;
                        if (cnt >= 12) {
                            m.name = "Nice FLO";
                            m.key = data;
                            m.bits = cnt;
                            m.te = (uint16_t)te_short;
                            return true;
                        }
                        break;
                    }
                    te_last = duration;
                    step = CheckDur;
                } else {
                    step = Reset;
                }
                break;
            case CheckDur:
                if (level) {
                    if (DDIFF(te_last, te_short) < te_delta && DDIFF(duration, te_long) < te_delta) {
                        data = data << 1 | 0;
                        cnt++;
                        step = SaveDur;
                    } else if (DDIFF(te_last, te_long) < te_delta && DDIFF(duration, te_short) < te_delta) {
                        data = data << 1 | 1;
                        cnt++;
                        step = SaveDur;
                    } else {
                        step = Reset;
                    }
                } else {
                    step = Reset;
                }
                break;
        }
    }
    return false;
}

// ── Holtek HT12 ──────────────────────────────────────────────────────────
// Port of subghz_protocol_decoder_holtek_feed. 40-bit frame, fixed 0x5
// header nibble. SubGhzDecoders.cpp:187-237.
bool decode_holtek(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t te_short = 430, te_long = 870, te_delta = 100;
    enum { Reset, FoundStart, SaveDur, CheckDur };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0;
    uint8_t cnt = 0;

    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t duration = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(duration, te_short * 36) < te_delta * 36) step = FoundStart;
                break;
            case FoundStart:
                if (level && DDIFF(duration, te_short) < te_delta) {
                    step = SaveDur;
                    data = 0;
                    cnt = 0;
                } else step = Reset;
                break;
            case SaveDur:
                if (!level) {
                    if (duration >= te_short * 10 + te_delta) {
                        if (cnt == 40 && (data & 0xF000000000ULL) == 0x5000000000ULL) {
                            m.name = "Holtek";
                            m.key = data;
                            m.bits = 40;
                            m.te = (uint16_t)te_short;
                            return true;
                        }
                        data = 0;
                        cnt = 0;
                        step = FoundStart;
                        break;
                    }
                    te_last = duration;
                    step = CheckDur;
                } else {
                    step = Reset;
                }
                break;
            case CheckDur:
                if (level) {
                    if (DDIFF(te_last, te_short) < te_delta && DDIFF(duration, te_long) < te_delta * 2) {
                        data = data << 1 | 0;
                        cnt++;
                        step = SaveDur;
                    } else if (DDIFF(te_last, te_long) < te_delta * 2 && DDIFF(duration, te_short) < te_delta) {
                        data = data << 1 | 1;
                        cnt++;
                        step = SaveDur;
                    } else {
                        step = Reset;
                    }
                } else {
                    step = Reset;
                }
                break;
        }
    }
    return false;
}

// ── Holtek HT12X ─────────────────────────────────────────────────────────
// Port of decode_holtek_ht12x. 12-bit, double-frame (a real code is only
// accepted once the SAME 12-bit value repeats back-to-back -- last_data ==
// data -- matching the donor exactly, not a simplification). Distinct from
// decode_holtek() above (that one is the 40-bit Holtek HT12 with a fixed
// 0x5 header nibble; this one is the 12-bit HT12X sibling with shared
// 320/640us bit timing -- see this file's header comment for why HT12X must
// be tried before decode_came() below). SubGhzDecoders.cpp:1102-1136.
bool decode_holtek_ht12x(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t ts = 320, tl = 640, td = 200;
    enum { Reset, Start, Save, Check };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0, last_data = 0;
    uint8_t cnt = 0;

    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t d = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(d, ts * 28) < td * 20) step = Start;
                break;
            case Start:
                if (level && DDIFF(d, ts) < td) {
                    step = Save;
                    data = 0;
                    cnt = 0;
                } else step = Reset;
                break;
            case Save:
                if (!level) {
                    if (d >= ts * 10 + td) {
                        if (cnt == 12) {
                            if (last_data == data && last_data) {
                                m.name = "Holtek_HT12X";
                                m.key = data;
                                m.bits = cnt;
                                m.te = (uint16_t)ts;
                                return true;
                            }
                            last_data = data;
                        }
                        data = 0;
                        cnt = 0;
                        step = Start;
                        break;
                    }
                    te_last = d;
                    step = Check;
                } else {
                    step = Reset;
                }
                break;
            case Check:
                if (level) {
                    if (DDIFF(te_last, tl) < td * 2 && DDIFF(d, ts) < td) {
                        data = data << 1 | 1;
                        cnt++;
                        step = Save;
                    } else if (DDIFF(te_last, ts) < td && DDIFF(d, tl) < td * 2) {
                        data = data << 1 | 0;
                        cnt++;
                        step = Save;
                    } else {
                        step = Reset;
                    }
                } else {
                    step = Reset;
                }
                break;
        }
    }
    return false;
}

// ── Linear ───────────────────────────────────────────────────────────────
// Port of subghz_protocol_decoder_linear_feed. 10-bit DIP code. Tried last
// (loose tolerance -- SubGhzDecoders.cpp:1860). SubGhzDecoders.cpp:241-289.
bool decode_linear(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t te_short = 500, te_long = 1500, te_delta = 350;
    enum { Reset, SaveDur, CheckDur };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0;
    uint8_t cnt = 0;

    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t duration = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(duration, te_short * 42) < te_delta * 15) {
                    data = 0;
                    cnt = 0;
                    step = SaveDur;
                }
                break;
            case SaveDur:
                if (level) {
                    te_last = duration;
                    step = CheckDur;
                } else step = Reset;
                break;
            case CheckDur:
                if (!level) {
                    if (duration >= te_short * 5) {
                        step = Reset;
                        if (DDIFF(duration, te_short * 42) > te_delta * 15) break;
                        if (DDIFF(te_last, te_short) < te_delta) {
                            data = data << 1 | 0;
                            cnt++;
                        } else if (DDIFF(te_last, te_long) < te_delta) {
                            data = data << 1 | 1;
                            cnt++;
                        }
                        if (cnt == 10) {
                            m.name = "Linear";
                            m.key = data;
                            m.bits = 10;
                            m.te = (uint16_t)te_short;
                            return true;
                        }
                        break;
                    }
                    if (DDIFF(te_last, te_short) < te_delta && DDIFF(duration, te_long) < te_delta) {
                        data = data << 1 | 0;
                        cnt++;
                        step = SaveDur;
                    } else if (DDIFF(te_last, te_long) < te_delta && DDIFF(duration, te_short) < te_delta) {
                        data = data << 1 | 1;
                        cnt++;
                        step = SaveDur;
                    } else {
                        step = Reset;
                    }
                } else {
                    step = Reset;
                }
                break;
        }
    }
    return false;
}

// ── Ansonic ──────────────────────────────────────────────────────────────
// SubGhzDecoders.cpp:292-323.
bool decode_ansonic(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t ts = 555, tl = 1111, td = 120;
    enum { Reset, Start, Save, Check };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0;
    uint8_t cnt = 0;
    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t d = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(d, ts * 35) < td * 35) step = Start;
                break;
            case Start:
                if (!level) break;
                else if (DDIFF(d, ts) < td) {
                    step = Save;
                    data = 0;
                    cnt = 0;
                } else step = Reset;
                break;
            case Save:
                if (!level) {
                    if (d >= ts * 4) {
                        step = Start;
                        if (cnt >= 12) {
                            m.name = "Ansonic";
                            m.key = data;
                            m.bits = cnt;
                            m.te = (uint16_t)ts;
                            return true;
                        }
                        break;
                    }
                    te_last = d;
                    step = Check;
                } else step = Reset;
                break;
            case Check:
                if (level) {
                    if (DDIFF(te_last, ts) < td && DDIFF(d, tl) < td) {
                        data = data << 1 | 1;
                        cnt++;
                        step = Save;
                    } else if (DDIFF(te_last, tl) < td && DDIFF(d, ts) < td) {
                        data = data << 1 | 0;
                        cnt++;
                        step = Save;
                    } else step = Reset;
                } else step = Reset;
                break;
        }
    }
    return false;
}

// ── Chamberlain Code ─────────────────────────────────────────────────────
// 4-bit symbol encoding (0b0111=bit0, 0b0011=bit1, 0b0001=stop); a captured
// frame is matched against the 7/8/9-DIP code masks and converted to bits.
// SubGhzDecoders.cpp:1174-1228.
bool chamb_to_bit(uint64_t *data, uint8_t size) {
    uint64_t t = *data, res = 0;
    for (uint8_t i = 0; i < size; i++) {
        uint64_t sym = t & 0xF;
        if (sym == 0b0111) {
            /* bit 0 */
        } else if (sym == 0b0011) {
            res |= (1ULL << i);
        } else return false;
        t >>= 4;
    }
    *data = res;
    return true;
}

bool decode_chamberlain(const unsigned int *dur, uint16_t n, uint8_t phase, Match &m) {
    const uint32_t ts = 1000, td = 200;
    enum { Reset, Start, Save, Check };
    uint32_t step = Reset, te_last = 0;
    uint64_t data = 0;
    uint8_t cnt = 0;
    for (uint16_t i = 0; i < n; i++) {
        bool level = sample_level(i, phase);
        uint32_t d = dur[i];
        switch (step) {
            case Reset:
                if (!level && DDIFF(d, ts * 39) < td * 20) step = Start;
                break;
            case Start:
                if (level && DDIFF(d, ts) < td) {
                    data = 0;
                    cnt = 0;
                    data = data << 4 | 0b0001; // stop marker
                    cnt++;
                    step = Save;
                } else step = Reset;
                break;
            case Save:
                if (!level) {
                    if (d > ts * 5) {
                        if (cnt >= 10 && cnt <= 11) {
                            uint64_t cd = data;
                            uint8_t cc = cnt;
                            bool ok = false;
                            if ((cd & 0xF000000FF0FULL) == 0x10000001101ULL) {
                                cc = 7;
                                cd &= ~0xF000000FF0FULL;
                                cd = (cd >> 12) | ((cd >> 4) & 0xF);
                                ok = true;
                            } else if ((cd & 0xF00000F00FULL) == 0x1000001001ULL) {
                                cc = 8;
                                cd &= ~0xF00000F00FULL;
                                cd = (cd >> 4) | ((uint64_t)0b0111 << 8);
                                ok = true;
                            } else if ((cd & 0xF000000000FULL) == 0x10000000001ULL) {
                                cc = 9;
                                cd &= ~0xF000000000FULL;
                                cd >>= 4;
                                ok = true;
                            }
                            if (ok && chamb_to_bit(&cd, cc)) {
                                m.name = "Cham_Code";
                                m.key = cd;
                                m.bits = cc;
                                m.te = (uint16_t)ts;
                                return true;
                            }
                        }
                        step = Reset;
                    } else {
                        te_last = d;
                        step = Check;
                    }
                } else step = Reset;
                break;
            case Check:
                if (level) {
                    if (DDIFF(te_last, ts * 3) < td && DDIFF(d, ts) < td) {
                        data = data << 4 | 0b0001;
                        cnt++;
                        step = Save;
                    } else if (DDIFF(te_last, ts * 2) < td && DDIFF(d, ts * 2) < td) {
                        data = data << 4 | 0b0011;
                        cnt++;
                        step = Save;
                    } else if (DDIFF(te_last, ts) < td && DDIFF(d, ts * 3) < td) {
                        data = data << 4 | 0b0111;
                        cnt++;
                        step = Save;
                    } else step = Reset;
                } else step = Reset;
                break;
        }
    }
    return false;
}

// ── Engine ───────────────────────────────────────────────────────────────
// SubGhzDecoders.cpp:1813-1861 orders decoders most-specific-first; this
// preserves the same relative order among the eight ported here (the real
// donor table's own relative order of these eight entries, verified by
// direct read 2026-08-25: holtek=1828, princeton=1842, holtek_ht12x=1846,
// came=1850, nice_flo=1851, chamberlain=1852, ansonic=1858, linear=1860).
typedef bool (*DecoderFn)(const unsigned int *, uint16_t, uint8_t, Match &);
const DecoderFn kDecoders[] = {
    decode_holtek,        // 40-bit, header mask -- most specific
    decode_princeton,     // 24-bit, double-frame
    decode_holtek_ht12x,  // 12-bit, double-frame -- MUST precede decode_came:
                           //   identical 320/640us bit timing, distinguished
                           //   only by preamble length (donor comment,
                           //   SubGhzDecoders.cpp:1846-1849; see this file's
                           //   header comment for the full reasoning).
    decode_came,           // 12/24-bit family
    decode_nice_flo,       // 12-bit
    decode_chamberlain,    // 7/8/9-DIP symbol code
    decode_ansonic,        // 12-bit
    decode_linear,         // 10-bit, loose tolerance -- last
};
constexpr uint8_t kNumDecoders = sizeof(kDecoders) / sizeof(kDecoders[0]);

} // namespace

bool decode(const unsigned int *dur, uint16_t count, Match *out) {
    if (out == nullptr || dur == nullptr) return false;
    // SubGhzDecoders::decode()'s own guard, count < 8: SubGhzDecoders.cpp:1866.
    if (count < 8) return false;

    Match m;
    for (uint8_t phase = 0; phase < 2; phase++) {
        for (uint8_t k = 0; k < kNumDecoders; k++) {
            if (kDecoders[k](dur, count, phase, m)) {
                *out = m;
                return true;
            }
        }
    }
    return false;
}

} // namespace SubghzProto
