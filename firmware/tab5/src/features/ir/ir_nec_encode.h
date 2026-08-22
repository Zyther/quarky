#pragma once

#include "ir_file_format.h"
#include <cstddef>
#include <cstdint>

// ===========================================================================
// NEC / NECext protocol ENCODER (Phase 3 Task 18), address+command bytes ->
// real mark/space RMT-ready duration array. Pure, host-testable -- no
// Arduino/hardware dependency, matching ir_file_format.cpp's own precedent
// (features/ir/ir_file_format.h/.cpp, Task 21) and this codebase's other
// host-testable modules (rf433_protocol_decode.cpp, Task 7).
//
// WHY THIS EXISTS: the real, on-SD-card Flipper-IRDB copy loaded onto the
// Tab5 (`/quarky/ir/flipperdb/`) is overwhelmingly `type: parsed` NEC-family
// signals, not `type: raw` captures -- confirmed by reading two full real
// sample files this session (Window_cleaners/HOBOT/Hobot_2S.ir, HOBOT.ir),
// both 100% `type: parsed`, using protocol names `NECext` and `NEC`
// respectively, zero raw entries in either. IrFileFormat::decode() already
// parses `protocol`/`address`/`command` into IrSignal; what's been missing
// is turning THAT back into real transmit timing -- this module is exactly
// that missing half, consumed by ir_clone.cpp.
//
// REAL PROTOCOL DETAILS, cited from Flipper Devices' own real firmware
// source (flipperdevices/flipperzero-firmware, `dev` branch, fetched via
// raw.githubusercontent.com 2026-08-22 -- the same real-primary-source
// discipline this project has followed all session for crapto1/
// MFRC522_I2C/ST25R3916/etc.):
//
//   Timing (lib/infrared/encoder_decoder/nec/infrared_protocol_nec_i.h):
//     INFRARED_NEC_PREAMBLE_MARK  = 9000us
//     INFRARED_NEC_PREAMBLE_SPACE = 4500us
//     bit mark (both bit0 and bit1)     = 560us
//     bit1 space = 1690us, bit0 space = 560us
//
//   Bit order within the transmitted data word (lib/infrared/
//   encoder_decoder/common/infrared_common_encoder.c,
//   infrared_common_encode_pdwm()): `shift = bits_encoded % 8; // LSB
//   first` -- each byte of the assembled data word is transmitted LSB bit
//   first, byte 0 before byte 1 before byte 2 before byte 3 (byte index is
//   `bits_encoded / 8`, walked in increasing order). A 32-bit frame is
//   therefore 32 bit-cells (mark+space each) preceded by the preamble
//   mark+space and followed by ONE more terminating mark (the real
//   encoder's own `infrared_common_encode_bits()`: encoding is "done"
//   right after a bit's own mark half is emitted with a HIGH level and no
//   bits remain -- i.e. the last bit's SPACE half is still emitted first,
//   then one final closing mark ends it). Total real duration count for a
//   32-bit frame: 2 (preamble) + 32*2 (bit halves) + 1 (terminating mark)
//   = 67.
//
//   Frame assembly (lib/infrared/encoder_decoder/nec/
//   infrared_encoder_nec.c, infrared_encoder_nec_reset()):
//     NEC:    `*data1 = address | (~address << 8) | (command << 16) |
//              (~command << 24)` -- a single little-endian 32-bit word, so
//              in real transmitted BYTE order: address, ~address, command,
//              ~command. The complement bytes are DERIVED here from the
//              real single address/command byte, never read from the
//              stored 4-byte fields' 2nd-4th bytes (those are zero-padding
//              in the real .ir format for this protocol -- confirmed by
//              the real HOBOT.ir sample's own `address: 00 00 00 00` /
//              `command: 05 00 00 00`, only byte[0] of each meaningful).
//     NECext: `*data1 = (uint16_t)address | (command & 0xFFFF) << 16` --
//              no complement. In real transmitted byte order: the two real
//              stored address bytes verbatim (address[0] then address[1]),
//              then the two real stored command bytes verbatim
//              (command[0] then command[1]) -- confirmed against the real
//              Hobot_2S.ir sample (`address: C5 1F 00 00` / `command: 0D
//              F2 00 00`, both byte[0]/byte[1] pairs meaningful, byte
//              [2]/[3] zero padding). Note 0x0D/0xF2 happen to be bitwise
//              complements of each other in THIS specific remote's own
//              command value -- coincidental to this file, not a NECext
//              protocol rule, so this module never derives a NECext
//              command's 2nd byte from the 1st.
//
//   Carrier (lib/infrared/encoder_decoder/nec/infrared_protocol_nec_i.h's
//   own INFRARED_COMMON_CARRIER_FREQUENCY, and independently the same
//   well-established consumer-IR figure this project already cites
//   elsewhere without a chip-specific measurement -- ir_common.h's
//   kMaxDurationsPerTransmit comment, ir_learn.cpp's kAssumedCarrierHz,
//   ir_tvbgone.cpp's world_ir_codes.h data): 38000 Hz for the whole
//   NEC/NECext family.
//
// Duty cycle is deliberately NOT part of this module's output -- callers
// (ir_clone.cpp) pass IrCommon::transmit_raw() the same 1/3 default
// ir_tvbgone.cpp/ir_learn.cpp already use for the identical reason (this
// class of IR receiver is broadly duty-cycle-tolerant; the real duty cycle
// isn't recoverable from a decoded protocol description any more than it
// is from a raw capture).
//
// SCOPE: NEC and NECext only. NEC42/NEC42ext/Samsung32/RC5/RC5X/RC6/SIRC*/
// Kaseikyo/RCA/Pioneer -- all real protocol names the .ir format documents
// and the real corpus likely contains -- are explicitly refused (encode()
// returns false), matching this project's established "refuse rather than
// lie" convention (e.g. IrFileFormat::decode()'s own unknown-version
// refusal) rather than silently mis-encoding or guessing.
// ===========================================================================

namespace IrNecEncode {

// Real cited constants (see header comment above for citations).
constexpr uint32_t kCarrierHz = 38000;
constexpr uint16_t kPreambleMarkUs = 9000;
constexpr uint16_t kPreambleSpaceUs = 4500;
constexpr uint16_t kBitMarkUs = 560;
constexpr uint16_t kBit1SpaceUs = 1690;
constexpr uint16_t kBit0SpaceUs = 560;

// Real duration-array size for one 32-bit NEC/NECext frame: 2 (preamble
// mark+space) + 32*2 (32 bit-cells, mark+space each) + 1 (terminating
// mark) -- see header comment's citation trail. Callers should size their
// output buffer to at least this (IrCommon::kMaxDurationsPerTransmit=1024
// comfortably covers it).
constexpr size_t kDurationsPerFrame = 2 + 32 * 2 + 1; // = 67

// Encodes sig (must be SignalType::kParsed, protocol "NEC" or "NECext"
// exactly -- case-sensitive match against the real stored string, see
// ir_file_format.h's `protocol` field) into durations_out[0..*out_count),
// an alternating mark/space microsecond array starting with a mark (this
// project's established .ir-raw/IrCommon convention). *out_carrier_hz (if
// non-null) is always set to kCarrierHz on success.
//
// Returns false (durations_out/out_count/out_carrier_hz left untouched)
// if: sig.type != kParsed; sig.protocol doesn't match "NEC" or "NECext";
// or max_durations < kDurationsPerFrame (buffer too small to hold one real
// frame). This is a real refusal, not a best-effort partial encode.
bool encode(const IrFileFormat::IrSignal &sig, uint16_t *durations_out, size_t max_durations,
            size_t *out_count, uint32_t *out_carrier_hz);

} // namespace IrNecEncode
