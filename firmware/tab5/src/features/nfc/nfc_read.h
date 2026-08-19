#pragma once

namespace NfcRead {

// Register the NFC unit (ST25R3916 @ 0x50) tag-read tile.
void register_module_nfc_unit();

// Register the RFID2 unit (WS1850S / MFRC522 @ 0x28) tag-read tile.
void register_module_rfid2_unit();

// Called from main.cpp's loop() to advance scan/poll state.
void poll();

} // namespace NfcRead

