#pragma once
#include <cstdint>

// NVS-backed persistence for the 128-bit pre-shared key that authenticates
// both Tab5<->Cardputer-ADV C2 transports (WiFi, Task 11; BLE, Task 13).
// One PSK, generated once and stored in NVS, serves both -- there's no
// separate pairing ceremony per transport.
namespace PskStore {
bool load(uint8_t out[16]);
void save(const uint8_t psk[16]);
}
