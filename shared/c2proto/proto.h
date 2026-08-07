#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

namespace c2proto {

constexpr uint32_t kMagic = 0x51524B43; // "QRKC"
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxPayload = 200; // stays under ESP-NOW's ~250B frame ceiling with headroom

enum class MsgType : uint8_t {
    CMD_START_FEATURE = 1,
    CMD_STOP_FEATURE  = 2,
    CMD_GET_STATUS    = 3,
    RESP_STATUS       = 4,
    RESP_TELEMETRY    = 5,
    RESP_BULK_READY   = 6,
};

#pragma pack(push, 1)
struct Frame {
    uint8_t version;
    MsgType type;
    uint16_t seq;
    uint16_t payload_len;
    uint8_t payload[kMaxPayload];
};
struct WireHeader {
    uint32_t magic;
    uint8_t version;
    MsgType type;
    uint16_t seq;
    uint16_t payload_len;
};
#pragma pack(pop)

int encode(const Frame &f, uint8_t *out, size_t out_cap);
bool decode(const uint8_t *in, size_t in_len, Frame &out);

} // namespace c2proto
