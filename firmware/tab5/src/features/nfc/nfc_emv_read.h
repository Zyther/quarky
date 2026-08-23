#pragma once

// ===========================================================================
// EMV/APDU contactless payment-card reader (Phase 3 Task 13). NFC unit
// (ST25R3916) only -- ISO14443-4 is this project's own st25r3916_driver.{h,cpp}
// addition, and there is no equivalent path on the RFID2/WS1850S unit (that
// chip's own PICC-level library, MFRC522_I2C, is used elsewhere in this
// project -- nfc_read.cpp/nfc_mifare_crack.cpp/nfc_amiibo.cpp -- purely for
// its ISO14443-3 anticollision helpers, not for ISO14443-4/T=CL).
//
// READ-ONLY. This extracts whatever PAN/expiry/vendor data a real contactless
// EMV card returns unencrypted during Application Selection and card-data
// reading (SELECT PPSE -> SELECT AID -> GET PROCESSING OPTIONS -> READ
// RECORD). No payment/transaction logic of any kind is implemented or
// possible from what is read here -- see nfc_emv_read.cpp's own header
// comment for the full real-APDU-sequence citations (ported from Bruce's
// `~/src/firmware/src/modules/rfid/emv_reader.cpp`, adapted from PN532 calls
// to this project's own St25r3916::apdu_transceive()).
// ===========================================================================

namespace NfcEmvRead {

// Registers the "NFC: EMV Card Read" launcher tile (Category::NFC,
// Affinity::TAB5_NATIVE).
void register_module();

// Called from main.cpp's loop(). No-ops unless this screen is open.
void poll();

} // namespace NfcEmvRead
