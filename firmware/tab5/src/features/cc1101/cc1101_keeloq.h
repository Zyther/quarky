#pragma once

#include "cc1101_scan.h"
#include <cstdint>

// ===========================================================================
// KeeLoq decode + rolling-code "Replay +1" (Phase 10 Task 8).
//
// LEGALLY SENSITIVE (project owner, per this phase's spec / Phase 5 spec
// Section 1's own KeeLoq row): rolling-code replay is meaningfully different
// from passive scanning even under the project owner's own-equipment
// authorization. The UI consuming this module MUST show an explicit,
// unambiguous "actively attacking" state -- not a generic spinner -- while
// replay_plus_one() is in flight. See cc1101_keeloq.cpp's UI wiring
// (features screen, built alongside this module) for that indicator.
//
// Donor reference, all real local checkouts under ~/src/, read directly for
// this port (not re-derived):
//   - Raw-edge decode of a KeeLoq (RcSwitch "protocol 23") frame:
//     ~/src/unigeek-main/firmware/src/utils/rf/RCSwitchUtil.cpp's real
//     kProto[22] table entry (`{400, {0,10}, {2,1}, {1,2}, false}`, line 34)
//     and _matchProtocol()'s real KeeLoq special-case (te=400 fixed,
//     firstData=25 preamble edges skipped, 60% tolerance -- lines 112-134).
//   - KeeLoq block cipher (encrypt/decrypt, 528 rounds, NLF=0x3A5C742E),
//     unpack() (64-bit frame -> fix/encrypted/btn/serial), identify()
//     (try every manufacturer key), step() (rolling-code counter-increment +
//     re-encrypt for Replay +1), and the manufacturer-specific hop-word
//     bit-layout table: ~/src/unigeek-main/firmware/src/utils/rf/
//     KeeloqUtil.cpp, ported near-verbatim (cipher primitives byte-for-byte;
//     step()/identify() adapted from UniGeek's CC1101Util::Signal type to
//     this module's own KeeloqInfo).
//   - Manufacturer keystore file format (`mf_name;hex_key;learning_type`,
//     comments/blank lines skipped, silently empty if the file is absent):
//     ~/src/unigeek-main/firmware/src/utils/rf/KeeloqKeystore.cpp, same
//     format, same "no bundled/fabricated keys" behavior -- this project
//     ships none either; a user who has their own manufacturer key file
//     supplies it at /quarky/keeloq/mfcodes on the SD card.
// ===========================================================================

namespace Cc1101Keeloq {

struct KeeloqInfo {
    uint32_t fix = 0;
    uint32_t encrypted = 0;
    uint8_t btn = 0;
    uint32_t serial = 0;
    uint16_t cnt = 0;          // rolling counter, valid only if identified
    uint32_t hop = 0;          // decrypted hop plaintext, valid only if identified
    bool identified = false;
    char mf_name[24] = "";     // manufacturer name if identified() ("" otherwise)
    uint32_t freq_hz = 433920000u; // frequency the frame was captured on --
                                    // replay_plus_one() retunes to this
};

// Attempts to decode `sig` as a real KeeLoq (RcSwitch protocol 23) frame:
// locates a candidate 64-bit-data segment via the same separator-gap
// splitting RCSwitchUtil::decodeStream() uses, matches it against protocol
// 23's real timing, then unpacks fix/encrypted/btn/serial. If a manufacturer
// keystore is loaded (see keystore_count()), also attempts to identify the
// manufacturer and decrypt the rolling counter. Returns true iff a
// structurally valid 64-bit KeeLoq frame was found (identified() may still
// be false -- decode succeeding and identification succeeding are separate
// outcomes, matching UniGeek's own unpack()-always-safe /
// identify()-separate split).
bool decode(const Cc1101Scan::CapturedSignal &sig, KeeloqInfo *out);

// "Replay +1": given a successfully IDENTIFIED KeeloqInfo (info.identified
// must be true -- this function refuses otherwise), increments the rolling
// counter by 1, re-encrypts the hop word with the matched manufacturer key,
// re-encodes a real KeeLoq waveform (RCSwitchUtil's real keeloq preamble +
// 64-bit-frame encoding, ported verbatim) and transmits it from a dedicated
// FreeRTOS task (same shape as cc1101_replay.cpp -- see that file for the
// "why a background task" citation). Returns false immediately (nothing
// started) if info.identified is false, a transmit is already in flight, or
// CC1101 setup fails.
bool replay_plus_one(const KeeloqInfo &info);

// Number of manufacturer keys currently loaded from
// /quarky/keeloq/mfcodes (0 if the file doesn't exist / SD not mounted /
// empty -- decode() still runs and unpacks structural fields either way,
// just never sets identified=true).
size_t keystore_count();

// Async replay state, mirrors Cc1101Replay's shape -- polled by the UI to
// render the mandatory "actively attacking" indicator.
enum class ReplayState { kIdle, kTransmitting, kDone, kFailed };
ReplayState replay_state();
const char *replay_failure_reason();

// Called from main.cpp's loop(). No-ops unless a replay_plus_one() transmit
// is in flight or has just finished.
void poll();

// FeatureRegistry wiring, same shape as every other Tab5 feature module.
void register_module();
void open_screen();

} // namespace Cc1101Keeloq
