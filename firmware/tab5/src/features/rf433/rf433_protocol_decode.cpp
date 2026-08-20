#include "rf433_protocol_decode.h"
#include "rf433_common.h"
#include <cstring>

// ===========================================================================
// Ported from ~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.cpp
// (see rf433_protocol_decode.h for the citation correction, the GPLv3
// provenance note, and why this ports six of the donor's 44 decoders rather
// than all of them). Exact line ranges below are from the donor file as read
// for this task; every timing constant (te_short/te_long/te_delta and the
// preamble-multiple/DDIFF tolerance checks) is copied verbatim, not
// reinvented.
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
//   kDecoders[] table ordering        SubGhzDecoders.cpp:1813-1861
//   SubGhzDecoders::decode() engine   SubGhzDecoders.cpp:1864-1882
//     (two-phase try loop, count < 8 => no match)
//
// The donor decoders are tried "most-specific first" (SubGhzDecoders.cpp:
// 1809-1812 comment) so a loose-tolerance decoder can't grab a frame another
// would parse correctly; this file preserves the SAME RELATIVE ORDER among
// the six ported here: Holtek, CAME, Nice FLO, Chamberlain, Ansonic, then
// Linear last (Linear is explicitly commented "loose tolerance -- last" at
// SubGhzDecoders.cpp:1860).
// ===========================================================================

namespace Rf433ProtocolDecode {

namespace {

// abs difference of two unsigned durations. SubGhzDecoders.cpp:4.
#define DDIFF(x, y) (((x) < (y)) ? ((y) - (x)) : ((x) - (y)))

// Level of sample i for a given phase. The capture's starting level is
// unknown, so the engine tries phase 0 and 1; even index = HIGH within a
// phase. SubGhzDecoders.cpp:8-10.
inline bool sample_level(uint16_t i, uint8_t phase) {
    return (((uint16_t)(i + phase)) & 1u) == 0u;
}

// Match produced by a single decoder -- mirrors SubGhzDecoders::Match
// (SubGhzDecoders.h:30-35), narrowed to the fields this port actually uses.
struct Match {
    const char *name = nullptr;
    uint64_t key = 0;
    uint8_t bits = 0;
};

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
// preserves the same relative order among the six ported here.
typedef bool (*DecoderFn)(const unsigned int *, uint16_t, uint8_t, Match &);
const DecoderFn kDecoders[] = {
    decode_holtek,      // 40-bit, header mask -- most specific
    decode_came,        // 12/24-bit family
    decode_nice_flo,    // 12-bit
    decode_chamberlain, // 7/8/9-DIP symbol code
    decode_ansonic,      // 12-bit
    decode_linear,       // 10-bit, loose tolerance -- last
};
constexpr uint8_t kNumDecoders = sizeof(kDecoders) / sizeof(kDecoders[0]);

// Max pulse durations derivable from one CapturedSignal: one fewer than the
// edge sample cap, since a duration is the gap BETWEEN two consecutive edges
// (see build_durations() below).
constexpr size_t kMaxDurations = Rf433Scan::kMaxEdgesPerSignal - 1;

// EdgeSample{timestamp_us, level} -> alternating HIGH/LOW pulse durations.
//
// SubGhzDecoders::decode()'s real caller, CC1101Util::pollReceive()
// (CC1101Util.cpp:220-227), never faces this conversion at all: its capture
// front end is the ESP32 RMT peripheral (RmtRf::readFrame(), RmtRf.h:43-45),
// which is fed by hardware and produces signed pulse DURATIONS directly --
// there is no per-edge timestamp array to diff in the donor's own pipeline.
// This project's capture front end (rf433_common.h) is deliberately
// different -- a GPIO CHANGE interrupt timestamping edges with micros()
// (see that header's top-of-file comment for why: polling loop() was tried
// first and was orders of magnitude too slow for real OOK pulse widths) --
// so this conversion step is genuinely new work, not something to find a
// donor precedent for.
//
// Each duration is the gap between two consecutive edges: for i = 1 ..
// edge_count-1, duration[i-1] = edges[i].timestamp_us - edges[i-1].
// timestamp_us. That duration is the pulse at the level edges[i-1].level
// held until edges[i] (EdgeSample.level is documented as "the level this
// sample transitioned TO", rf433_common.h:30 -- so the level that HOLDS
// between edges[i-1] and edges[i] is edges[i-1].level, matching the donor
// decoders' "alternating HIGH/LOW durations" semantics exactly).
//
// The FIRST edge sample is deliberately skipped, not synthesized: it records
// only the level the pin transitioned TO, with no earlier timestamp in this
// CapturedSignal to measure how long the signal dwelled at the level before
// it -- there is no real duration to compute for it, and inventing one would
// be exactly the kind of fabricated timing value this project's "real
// sources only" discipline forbids. This costs at most one leading pulse's
// worth of decode context, which every decoder above tolerates (each self-
// syncs on its own header/preamble rather than requiring the capture's very
// first edge to be meaningful).
size_t build_durations(const Rf433Scan::CapturedSignal &sig, unsigned int *out, size_t max_out) {
    if (sig.edge_count < 2) return 0;
    size_t n = sig.edge_count - 1;
    if (n > max_out) n = max_out;
    for (size_t i = 0; i < n; i++) {
        out[i] = static_cast<unsigned int>(sig.edges[i + 1].timestamp_us - sig.edges[i].timestamp_us);
    }
    return n;
}

} // namespace

bool decode(const Rf433Scan::CapturedSignal &sig, DecodedCode *out) {
    if (out == nullptr) return false;

    static unsigned int dur[kMaxDurations];
    size_t count = build_durations(sig, dur, kMaxDurations);
    // SubGhzDecoders::decode()'s own guard, count < 8: SubGhzDecoders.cpp:1866.
    if (count < 8) return false;

    Match m;
    for (uint8_t phase = 0; phase < 2; phase++) {
        for (uint8_t k = 0; k < kNumDecoders; k++) {
            if (kDecoders[k](dur, static_cast<uint16_t>(count), phase, m)) {
                std::memset(out->protocol_name, 0, sizeof(out->protocol_name));
                std::strncpy(out->protocol_name, m.name, sizeof(out->protocol_name) - 1);
                out->code = m.key;
                out->bit_length = m.bits;
                return true;
            }
        }
    }
    return false;
}

} // namespace Rf433ProtocolDecode
