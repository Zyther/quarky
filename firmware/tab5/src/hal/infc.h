#pragma once

// Detection-only NFC/RFID interface (Task 18). Full read/write/clone/replay
// logic for the HY2.0 NFC and RFID2 units is Phase 3 scope -- this interface
// only proves the HAL can talk to the unit at all.
class INFC {
public:
    virtual ~INFC() = default;
    virtual bool detect(const char *label) = 0; // logs `label` for which unit (NFC vs RFID2) this call is checking
};
