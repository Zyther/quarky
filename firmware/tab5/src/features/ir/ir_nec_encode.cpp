#include "ir_nec_encode.h"
#include <cstring>

namespace IrNecEncode {

namespace {

// Case-sensitive exact match, matching ir_file_format.h's own stored
// `protocol` field convention (whatever the real .ir file's `protocol:`
// line says, verbatim).
bool protocol_is(const IrFileFormat::IrSignal &sig, const char *name) {
    return std::strcmp(sig.protocol, name) == 0;
}

} // namespace

bool encode(const IrFileFormat::IrSignal &sig, uint16_t *durations_out, size_t max_durations,
            size_t *out_count, uint32_t *out_carrier_hz) {
    if (durations_out == nullptr || out_count == nullptr) return false;
    if (sig.type != IrFileFormat::SignalType::kParsed) return false;
    if (max_durations < kDurationsPerFrame) return false;

    // Real 4-byte little-endian frame, byte order = transmission order --
    // see ir_nec_encode.h's citation trail for both branches below.
    uint8_t frame[4];
    if (protocol_is(sig, "NEC")) {
        // Real address/command are just byte[0] of the stored 4-byte
        // fields (the rest is zero-padding in the real .ir format for
        // this protocol) -- complement bytes are DERIVED here, never read
        // from the stored fields' own 2nd-4th bytes.
        uint8_t address = sig.address[0];
        uint8_t command = sig.command[0];
        frame[0] = address;
        frame[1] = static_cast<uint8_t>(~address);
        frame[2] = command;
        frame[3] = static_cast<uint8_t>(~command);
    } else if (protocol_is(sig, "NECext")) {
        // Real 16-bit address/command, transmitted as literally stored --
        // no complement derivation. address[0]/command[0] are the real
        // low bytes, address[1]/command[1] the real high bytes (both
        // zero-initialized by IrFileFormat::decode() if the file provided
        // fewer than 2 bytes for either field, so reading them
        // unconditionally here is safe).
        frame[0] = sig.address[0];
        frame[1] = sig.address[1];
        frame[2] = sig.command[0];
        frame[3] = sig.command[1];
    } else {
        // Unrecognized protocol name (NEC42/NEC42ext/Samsung32/RC5/RC6/
        // SIRC*/Kaseikyo/RCA/Pioneer/etc.) -- real refusal, matching this
        // project's established "refuse rather than lie" convention.
        return false;
    }

    size_t pos = 0;
    durations_out[pos++] = kPreambleMarkUs;
    durations_out[pos++] = kPreambleSpaceUs;

    // 32 bit-cells, byte 0 first, LSB-first within each byte -- matches
    // infrared_common_encode_pdwm()'s real `shift = bits_encoded % 8; //
    // LSB first` walk order exactly (see ir_nec_encode.h's citation).
    for (size_t bit_i = 0; bit_i < 32; bit_i++) {
        size_t byte_i = bit_i / 8;
        size_t shift = bit_i % 8;
        bool one = ((frame[byte_i] >> shift) & 0x01) != 0;
        durations_out[pos++] = kBitMarkUs;
        durations_out[pos++] = one ? kBit1SpaceUs : kBit0SpaceUs;
    }

    // Terminating mark closes the last bit's space -- see header comment's
    // citation (infrared_common_encode_bits()'s own "done" condition).
    durations_out[pos++] = kBitMarkUs;

    *out_count = pos;
    if (out_carrier_hz != nullptr) *out_carrier_hz = kCarrierHz;
    return true;
}

} // namespace IrNecEncode
