#include "st25r3916_driver.h"
#include "../../hal/nfc_pn532.h"
#include "../../../boards/tab5/pins_config.h"
#include <Wire.h>
#include <Arduino.h>
#include <cstring>

// ===========================================================================
// SOURCES. Every register address, mode byte, command code and bit position
// below traces to one of these. Nothing here is recalled, inferred, or
// pattern-matched from a PN532/WS1850S/other-chip driver -- this project has
// already been burned three times by exactly that (ST7121 mistaken for
// ST7123, touch assumed to be a separate GT911, Unit RFID2 assumed PN532 when
// it is a WS1850S), and the whole reason this file exists as its own task is
// that no donor firmware in this program has any ST25R3916 code at all.
//
// [DS] PRIMARY SOURCE -- ST's own datasheet.
//      "ST25R3916/ST25R3917 -- High performance NFC universal device and
//      EMVCo reader", STMicroelectronics, doc ID DS12484 Rev 3.
//      https://www.st.com/resource/en/datasheet/st25r3916.pdf
//      (retrieved 2026-08-18; 156 pages; the copy read for this task was the
//      byte-identical mirror at
//      https://download.mikroe.com/documents/datasheets/ST25R3916%20Datasheet.pdf)
//      Sections used, by name so they stay findable if page numbers move:
//        - Sec 4.2.13  "Reader operation"          (p.42) -- Ready mode entry,
//                       osc_ok, then rx_en/tx_en before addressing a tag
//        - Table 11    "SPI operation modes"       (p.49) -- the two leading
//                       mode bits M1/M0, shared verbatim by the I2C framing
//        - Sec 4.3.4   "I2C interface"             (p.53-57) -- "The I2C
//                       address is 50h"; register write/read use "the same
//                       Register Write/Read mode byte as for SPI"
//        - Figure 20   "Writing a single register"   (p.54) -- the I2C
//                       register-write byte order used by write_register()
//        - Figure 25   "Sending a direct command"    (p.56) -- the one-byte
//                       I2C direct-command frame used by execute_command()
//        - Figure 26   "Read and Write mode for register space-B access"
//                       (p.56) -- the only figure whose legend spells the
//                       framing out in words: "S: Start, Sr: repeated Start,
//                       A: ACK, N: NAK, P: Stop", showing a register READ as
//                       S,addr+W,<mode byte>,Sr,addr+R,data...
//        - Table 13    "List of direct commands"   (p.58) -- C0/C1 Set
//                       default, C2/C3 Stop all activities, C8/C9 NFC field
//                       ON, FB Register space-B access
//        - Sec 4.4.1   "Set default"               (p.58) -- what C1 does:
//                       stop all activities, reset all registers to default,
//                       clear collision bits, and explicitly "No IRQ due to
//                       termination of direct command is produced"
//        - Table 21    "Operation control register" (p.72) -- address 02h,
//                       bit 7 en, bit 6 rx_en, bit 3 tx_en
//        - Table 98    "Auxiliary display register" (p.124) -- address 31h,
//                       read-only, bit 4 osc_ok = "Xtal oscillation is stable"
//        - Table 117   "IC identity register"      (p.134) -- address 3Fh,
//                       space A, read-only; ic_type<4:0> in bits 7..3 with
//                       "00101: ST25R3916/7"; ic_rev<2:0> in bits 2..0 with
//                       "010: rev 3.1"
//
// [REF] SECONDARY SOURCE -- ST's own reference driver, used to cross-check
//      every constant taken from [DS] and to copy the shape of the init /
//      chip-ID / oscillator-on sequences rather than invent one.
//      "STM32duino ST25R3916" v2.0.2, author=STMicroelectronics (the RFAL
//      ST25R3916 HAL, packaged for Arduino by ST's own stm32duino org).
//      https://github.com/stm32duino/ST25R3916
//      commit b7e708f1fe458cca4e0ec9d3b78402c99ffc4e71 (2026-01-27), read
//      2026-08-18. Files/lines used:
//        - src/st25r3916_com.cpp:48-56 -- ST25R3916_I2C_ADDR (0xA0>>1),
//          WRITE_MODE (0<<6), READ_MODE (1<<6), CMD_MODE (3<<6)
//        - src/st25r3916_com.h:101,184,225 -- REG_OP_CONTROL 0x02,
//          REG_AUX_DISPLAY 0x31, REG_IC_IDENTITY 0x3F
//        - src/st25r3916_com.h:264-268,918,1144-1158 -- op_control en/rx_en/
//          tx_en bit positions, aux_display osc_ok, ic_type/ic_rev masks and
//          the ic_type_st25r3916 (5U<<3) / ic_type_st25r3916B (6U<<3) values
//        - src/st25r3916.h:87-88,122 -- CMD_SET_DEFAULT 0xC1, CMD_STOP 0xC2,
//          CMD_SPACE_B_ACCESS 0xFB
//        - src/st25r3916.h:138 -- TOUT_OSC_STABLE 10 (ms), annotated
//          "DS: 700us"
//        - src/st25r3916.h:148,150 -- st25r3916TxRxOn()/TxRxOff() = set/clear
//          (rx_en | tx_en) in OP_CONTROL. This, not the C8 "NFC initial field
//          ON" direct command, is how RFAL turns the field on for a plain
//          reader; see the field_on() note below for why that matters here.
//        - src/st25r3916.cpp:101-119 -- Initialize(): Set Default, then
//          CheckChipID, and bail with ERR_HW_MISMATCH if the ID is wrong
//        - src/st25r3916.cpp:241-264 -- OscOn(): set en, wait for the
//          oscillator to stabilise, then require aux_display.osc_ok
//        - src/st25r3916.cpp:611-637 -- CheckChipID(): compares only the
//          MASKED ic_type field, and returns ic_rev separately as data
//
// [M5] The unit itself. docs.m5stack.com/en/unit/Unit_NFC (retrieved
//      2026-08-18): chip "ST25R3916-AQWT", "I2C @0x50 (100K / 400K)", and a
//      HY2.0-4P pinmap of exactly four wires -- black GND, red 5V, yellow
//      SDA, white SCL. This corroborates 0x50 independently of [DS], and it
//      establishes the constraint that shapes this whole driver: THERE IS NO
//      IRQ LINE. The ST25R3916 has a dedicated active-high IRQ output pin and
//      RFAL's flows are built around waiting on it ([REF] OscOn() waits for
//      ST25R3916_IRQ_MASK_OSC; ExecuteCommandAndGetResult() waits for
//      IRQ_MASK_DCT). None of that is available through a 4-pin connector, so
//      every wait in this file is a POLLED read of a status register instead.
//      Later NFC tasks must assume the same: no RFAL flow that blocks on an
//      interrupt can be used here unmodified.
//
// [EH] Corroboration only, cited for honesty about what was read, NOT used as
//      the source of any constant: wilson-elechouse/ST25R3916 (an ESP32 port
//      of [REF] that adds an Arduino-Wire I2C back end). Its
//      st25r3916_com.cpp read path is
//      beginTransmission / write(mode byte) / endTransmission(false) /
//      requestFrom -- i.e. the repeated-START framing [DS] Figure 26
//      documents, confirmed to be what a real Arduino I2C master does against
//      a real one of these chips. Every value it uses is identical to [REF]'s
//      because it is a fork of it.
//
// ---------------------------------------------------------------------------
// KNOWN-BAD COMMENT IN THE UPSTREAM REFERENCE, recorded so nobody "corrects"
// this file to match it: [REF]'s src/st25r3916_com.h:225 annotates
// REG_IC_IDENTITY as "Chip Id: 0 for old silicon, v2 silicon: 0x09". That is
// a leftover from the older ST25R3911 driver this library descends from. It
// contradicts both [DS] Table 117 AND the ic_type_st25r3916 (5U<<3 = 0x28)
// define sitting 900 lines below it in the same header, and [REF]'s own
// CheckChipID() ignores it. The value to expect is 0x28 in the type field,
// NOT 0x09.
//
// ===========================================================================
// ADDED 2026-08-19 (Phase 3 Task 4, fix round 1): ISO14443-A / NFC-A polled
// reader. SOURCES FOR THIS SECTION ONLY -- same discipline as above, every
// register address, bit position, command byte and analog-config value below
// traces to one of these; nothing is recalled or invented.
//
// WHY THIS EXISTS AT ALL. Task 4's first implementation reached for ST's own
// RFAL stack (RfalRfST25R3916Class) instead of extending this driver. An
// independent review found that path structurally impossible on this hardware
// and the controller ruled it out; the finding was re-verified here against
// the vendored library source before writing a line of the replacement:
//   * [EH] rfal_rfst25r3916.cpp:72-86 rfalInitialize() -- the I2C branch is
//     `if ((dev_i2c == NULL) || (int_pin < 0)) { return ERR_PARAM; }`. An
//     IRQ-less construction fails on the very first call. There is no
//     "worker-based polling" fallback; that claim was false.
//   * [EH] st25r3916_interrupt.cpp:120-141 st25r3916CheckForReceivedInterrupts()
//     -- `while (digitalRead(int_pin) == HIGH) { ...read IRQ regs... }`. The
//     GPIO is the ONLY trigger for ever reading the IRQ registers, so with no
//     wire every transceive in the library waits forever on a status word that
//     is never fetched. st25r3916_com.cpp's read/write/FIFO paths gate on the
//     same digitalRead() (lines 189, 264, 344, 416).
//   * [M5] (above) The Unit NFC's HY2.0-4P connector carries GND/5V/SDA/SCL.
//     There is no IRQ wire to give it.
// The substitute is the one the datasheet itself sanctions: reading the IRQ
// *status* registers over I2C is exactly what [DS] Sec 4.3.1 "Interrupt
// interface" (p.46) and Tables 62-65 (pp.97-100, addresses 1Ah-1Dh) describe
// as presenting the interrupt state. Reading also clears it -- Tables 63-65
// each carry an explicit "After register has been read, its content is set to
// 0" footnote; Table 62 (the Main interrupt register, 1Ah) has no such
// footnote of its own, but the same read-clears-it behavior for it is
// documented in Sec 4.3.1's prose and, separately, in Table 64's footnote.
// Either way, the library's own st25r3916ClearInterrupts() ([EH]
// st25r3916_interrupt.cpp:225-233) reads exactly those four registers with a
// plain multi-register read and no GPIO involvement. Polling them IS reading
// the interrupt state; the pin is only an optimisation that tells you when it
// would be worth reading.
//
// [DS] additional sections used. Same document and same copy as above
//      (DS12484 Rev 3, the mikroe mirror), re-downloaded and text-extracted on
//      2026-08-19 for this fix round so every table/page number below is one
//      that was actually read, not recalled:
//        - Table 11 "SPI operation modes" (p.49) -- the mode-byte table the
//          I2C interface reuses verbatim. FIFO load = 1000_0000b = 80h,
//          FIFO read = 1001_1111b = 9Fh, direct command = 11_C5..C0.
//        - Sec 4.3.4, Figure 23 "FIFO load" / Figure 24 "FIFO read" (p.55) --
//          the I2C framing: slave address, one mode byte (80h or 9Fh), then
//          the data bytes. No register address is involved.
//        - Sec 4.3.4 "I2C access to register space-B" (p.56) + Figure 26 --
//          "To access the register space-B, byte FBh has to be inserted
//          between the I2C slave address and the register read or write mode
//          byte. Access to register space-B remains active until an I2C Stop
//          Condition is received." Implemented by {read,write}_register_b().
//        - Table 13 "List of direct commands" (p.58) -- C2/C3 Stop all
//          activities, C4 Transmit with CRC, C5 Transmit without CRC,
//          C6 Transmit REQA ("ISO14443A mode only", requires en + tx_en),
//          D5 Reset RX gain, DB Clear FIFO. Note the "Interrupt after
//          termination" column reads No for every one of these, so nothing
//          here depends on the DCT interrupt.
//        - Table 22 "Mode definition register" (03h, p.73) + Table 23
//          "Initiator operation modes" -- om<3:0> in bits 6..3; 0001b =
//          ISO14443A. tr_am is bit 2 (0 = OOK, 1 = AM).
//        - Table 25 "Bit rate definition register" (04h, p.74) + Table 26
//          "Bit rate coding" -- tx_rate<1:0> bits 5..4, rx_rate<1:0> bits
//          1..0, 00b = fc/128 (~106 kb/s). So 106/106 is a plain 00h.
//        - Table 27 "ISO14443A and NFC 106kb/s settings register" (05h, p.75)
//          -- no_tx_par bit 7, no_rx_par bit 6, nfc_f0 bit 5, p_len<3:0> bits
//          4..1, antcl bit 0 ("Must be set to 1 for reception of ISO14443A bit
//          oriented anticollision frames in reader mode. Must be set to 0 for
//          all other frames and modes.").
//        - Table 48 "Mask receive timer register" (0Fh, p.89) -- mrt<7:0>,
//          step 64/fc (4.72 us) when mrt_step = 0.
//        - Tables 49/50 "No-response timer register 1/2" (10h/11h, p.90) --
//          nrt<15:0>, "Defines timeout after end of Tx. If this timeout
//          expires without detecting a response a No-Response interrupt is
//          sent." and, load-bearing here: "All 0: No-Response timer is not
//          started." -- so a zero NRT means no timeout at all, not an instant
//          one.
//        - Table 51 "Timer and EMV control register" (12h, p.91) -- gptc<2:0>
//          bits 7..5, mrt_step bit 3, nrt_emv bit 1, nrt_step bit 0.
//        - Tables 66/67 "FIFO status register 1/2" (1Eh/1Fh, p.101) --
//          fifo_b<7:0> in 1Eh, fifo_b<9:8> in 1Fh bits 7..6, fifo_ovr bit 4,
//          fifo_unf bit 5, fifo_lb<2:0> ("Number of bits in the last FIFO byte
//          if it was not complete") bits 3..1, np_lb bit 0.
//        - Table 68 "Collision display register" (20h, p.102) -- c_byte<3:0>
//          bits 7..4, c_bit<2:0> bits 3..1, c_pb bit 0.
//        - Tables 70/71 "Number of transmitted bytes register 1/2" (22h/23h,
//          p.104) -- ntx<12:5> in 22h, ntx<4:0> in 23h bits 7..3, nbtx<2:0> in
//          23h bits 2..0. Table 71's own Note 1 is the reason nbtx is zeroed
//          before REQA: "If anctl bit is set while card is in idle state and
//          nbtx is not 000, then i_par will be triggered during REQA and WUPA
//          direct command is issued."
//        - Tables 58/59 "Mask main / Mask timer and NFC interrupt register"
//          (16h/17h, p.95) -- every mask bit's Default column is 0 and its
//          Function reads "1: Mask IRQ due to ...", i.e. after Set Default
//          every interrupt is UNmasked. This driver therefore never has to
//          write a mask register to make a status bit appear.
//        - Sec 4.3.1 "Interrupt interface" (p.46), verbatim: "In case of
//          masking a certain interrupt source the IRQ line is not set high,
//          but the interrupt status bit is still set in IRQ status
//          registers. Reading the IRQ status registers presents and clears
//          also the masked interrupt bits." A stronger, mask-INDEPENDENT
//          reason the polled read-back design works: even if a mask register
//          were ever written away from its Tables 58/59 default, the status
//          bits this file polls would still be set and still be visible on
//          read. The design's foundation does not rest on nobody ever
//          touching a mask register.
//        - Tables 62/63/64/65 "Main / Timer and NFC / Error and wake-up /
//          Passive target interrupt register" (1Ah/1Bh/1Ch/1Dh, p.97-100),
//          all Type: R and all read-and-clear -- Tables 63-65's shared
//          footnote "After register has been read, its content is set to 0"
//          covers 1Bh/1Ch/1Dh directly; Table 62 (1Ah) carries no such
//          footnote of its own, but the same behavior for it is documented in
//          Sec 4.3.1's prose and, separately, in Table 64's footnote. Bit
//          positions used below:
//            1Ah: I_osc 7, I_wl 6, I_rxs 5, I_rxe 4, I_txe 3, I_col 2
//            1Bh: I_dct 7, I_nre 6, I_gpe 5
//            1Ch: I_crc 7, I_par 6, I_err2 5 (soft framing), I_err1 4 (hard
//                 framing -- and per Tables 67/68, when I_err1 is set the
//                 fifo_lb and collision-position fields are NOT valid)
//          These four registers, read back to back, ARE the interrupt state.
//          That is the whole basis of the polled substitute described above.
//
// [REF]/[EH] additional files/lines used (the pinned stm32duino driver cited
//      above and its ESP32 fork, which are the same code for everything below;
//      line numbers given for the [EH] copy actually read on this machine,
//      ~/src/wilson-elechouse/ST25R3916 @ 16eb6c7):
//        - st25r3916_com.cpp:52-57 -- FIFO_LOAD 0x80, FIFO_READ 0x9F mode bytes
//        - st25r3916_com.cpp:229-249 -- space-B read framing (send
//          CMD_SPACE_B_ACCESS, then (reg & ~0x40) | READ_MODE, then repeated
//          START and read); :311-331 the same for writes
//        - st25r3916_com.h:76 -- ST25R3916_SPACE_B 0x40 marker
//        - st25r3916_com.h:93-204 -- the register address map used below
//        - st25r3916_com.h:245,267-304,306-318,431-442,482,523,550-561,565-573,
//          581-587,649-664 -- every bit-position/mask constant mirrored below
//        - st25r3916_interrupt.h:68-89 -- IRQ bit values (RXS 0x20, RXE 0x10,
//          TXE 0x08, COL 0x04 in byte 0; NRE 0x4000 in byte 1; ERR1 0x100000,
//          ERR2 0x200000, PAR 0x400000, CRC 0x800000 in byte 2)
//        - st25r3916.cpp:364-368 st25r3916SetNumTxBits() -- writes the LOW
//          byte to NUM_TX_BYTES2 (23h) and the HIGH byte to NUM_TX_BYTES1 (22h)
//        - st25r3916.cpp:372-397 -- FIFO byte count / last-bit count decoding
//        - st25r3916.cpp:408-437 st25r3916SetNoResponseTime() -- NRT in 64/fc
//          steps, nrt_step bit selecting 64/fc vs 4096/fc
//        - rfal_rfst25r3916.cpp:261-284 rfalSetMode(RFAL_MODE_POLL_NFCA) --
//          "Enable ISO14443A mode": a plain write of om_iso14443a to 03h
//        - rfal_rfst25r3916.cpp:1145-1245 rfalPrepareTransceive() -- STOP +
//          RESET_RXGAIN, the ISO14443A_NFC parity/nfc_f0 programme, the
//          interrupt set to arm, and the FIFO-status reset
//        - rfal_rfst25r3916.cpp:1304-1378 rfalTransceiveTx() -- SetNumTxBits,
//          WriteFifo, then TRANSMIT_WITH_CRC / TRANSMIT_WITHOUT_CRC
//        - rfal_rfst25r3916.cpp:1984-2093 rfalISO14443ATransceiveShortFrame()
//          -- set AUX.no_crc_rx (ATQA carries no CRC), clear NUM_TX_BYTES2,
//          issue TRANSMIT_REQA (C6h), wait TXE then receive
//        - rfal_rfst25r3916.cpp:2097-2201 rfalISO14443ATransceiveAnticollision-
//          Frame() -- set ISO14443A_NFC.antcl and AUX.no_crc_rx, transmit
//          without CRC, and on collision read 20h for the byte/bit position
//        - rfal_rfst25r3916_analogConfigTbl.h:263-315 -- ST's own
//          analog-configuration table; the values applied by
//          apply_nfca_analog_config() below are copied from the CHIP_INIT,
//          CHIP_POLL_COMMON, NFC-A Rx common, NFC-A Tx 106 and NFC-A Rx 106
//          entries, verbatim. (The table's ANTICOL entry, :384-387, is NOT
//          applied -- see DELIBERATE SIMPLIFICATIONS #5 below; it is cited
//          here only for honesty about what was read, not as something this
//          driver does.)
//        - rfal_nfca.cpp:87-100 -- SEL_CMD per cascade level (93h/95h/97h),
//          "Digital 1.1 Table 15"
//        - rfal_nfca.cpp:209-366 rfalNfcaPollerSingleCollisionResolution() --
//          the cascade-level loop, the BCC check, the SEL_PAR = 70h select,
//          and the cascade-tag (88h) continuation rule this implements
//        - rfal_nfca.h:89 RFAL_NFCA_FDTMIN 1620 (1/fc); rfal_rf.h:230,239,251
//          GT 5 ms, FDT_LISTEN_NFCA_POLLER 1172, FDT_POLL_NFCA_POLLER 6780;
//          rfal_rfst25r3916.h:283,291,296 the MRT/FWT adjustment constants
//          used by the timer arithmetic below
//
// DELIBERATE SIMPLIFICATIONS, listed so a reader can tell "not implemented"
// from "missed":
//   1. Single tag only. RFAL's bit-level anticollision loop is not ported; a
//      COL interrupt aborts with kCollision. Every SDD_REQ this sends is the
//      full-byte NVB=20h form, so no partial-byte TX framing is needed either.
//   2. FDT(poll) is not enforced with the general-purpose timer. RFAL programs
//      GPT to guarantee >= 6780/fc (~500 us) between end-of-receive and the
//      next transmit; on this 100 kHz I2C bus a single register write already
//      costs ~300 us and each exchange costs several, so the requirement is
//      met by the transport being slower than the requirement. Stated rather
//      than silently assumed.
//   3. The analog configuration applies ST's poller-relevant entries only. The
//      CHIP_INIT entries that exist for listen/target mode (external-field
//      thresholds 2Ah/2Bh, passive-target fdel 08h, PT_MOD 29h, AUX_MOD load-
//      modulation bits) and the two SPI MISO pull-down entries (which [EH]'s
//      own Initialize() skips under I2C, st25r3916.cpp:103-106) are skipped.
//   4. No RC/regulator calibration. [EH] rfalCalibrate()/AdjustRegulators()
//      drive direct commands whose completion is signalled by the DCT
//      interrupt, i.e. exactly the thing this hardware cannot deliver.
//   5. CORR_CONF1.corr_s6 (space-B 0Ch, part of the register programme below)
//      is left at bring-up's 0x51 (corr_s6 SET) for every exchange, including
//      the anticollision one. ST's own analog-config table has a
//      POLL | TECH_NFCA | BITRATE_COMMON | ANTICOL entry
//      ([EH] rfal_rfst25r3916_analogConfigTbl.h:384-387) that clears corr_s6
//      -- ST's own comment: "Set collision detection level different from
//      data" -- and RFAL applies it immediately before every anticollision
//      frame (rfal_rfst25r3916.cpp:2115), restoring it afterward. This driver
//      does not port that adjustment, so with two tags present I_col may not
//      assert as reliably as it would with corr_s6 cleared for that one
//      exchange; a user presenting two tags at once is more likely to see a
//      BCC-mismatch kProtocolError than the intended kCollision "present one
//      tag at a time" message. Noted rather than silently missed. Correctness
//      does not depend on this: simplification #1 above already treats any
//      collision indication (I_col, or a BCC mismatch that looks like one) as
//      "not a single clean tag" and reports accordingly -- only the precision
//      of which error message the user sees is affected.
// ===========================================================================

namespace {

// --- I2C transport ---------------------------------------------------------
// [M5]/[DS] Sec 4.3.4 agree on 0x50; TAB5_NFC_I2C_ADDR in pins_config.h is the
// same value, already confirmed on this exact hardware by Phase 1's PORT.A
// bus census (it is the address that answered). Use the project constant so
// there is one definition, and static_assert that it still matches what the
// datasheet says this driver is written for -- if someone ever repoints
// TAB5_NFC_I2C_ADDR at the retired 0x24 PN532 candidate, this file should
// fail to build rather than silently talk PN532-shaped nonsense to nothing.
constexpr uint8_t kI2cAddr = TAB5_NFC_I2C_ADDR;
static_assert(kI2cAddr == 0x50,
              "ST25R3916's I2C address is 50h per DS12484 Rev 3 Sec 4.3.4; "
              "TAB5_NFC_I2C_ADDR no longer matches the chip this driver is for");

// [DS] Table 11 / Sec 4.3.4: the first two bits of the byte following the I2C
// slave address select the operation. The I2C interface reuses the SPI mode
// byte verbatim ("the same Register Write mode byte as for SPI").
//   00 A5..A0 -> register write        01 A5..A0 -> register read
//   11 C5..C0 -> direct command
// Cross-checked against [REF] st25r3916_com.cpp:51-53.
constexpr uint8_t kModeWrite = 0U << 6; // 0x00
constexpr uint8_t kModeRead  = 1U << 6; // 0x40
constexpr uint8_t kModeCmd   = 3U << 6; // 0xC0

// Highest register address expressible in the 6 address bits of a space-A
// mode byte. Anything above this would silently alias.
constexpr uint8_t kMaxSpaceARegister = 0x3FU;

// --- Registers ([DS] Tables 21, 98, 117; [REF] st25r3916_com.h) ------------
constexpr uint8_t kRegOpControl  = 0x02U; // RW  Operation control
constexpr uint8_t kRegAuxDisplay = 0x31U; // R   Auxiliary display
constexpr uint8_t kRegIcIdentity = 0x3FU; // R   IC identity

constexpr uint8_t kOpControlEn   = 1U << 7; // enable oscillator + regulators
constexpr uint8_t kOpControlRxEn = 1U << 6; // enable Rx operation
constexpr uint8_t kOpControlTxEn = 1U << 3; // enable Tx operation (RF field)

constexpr uint8_t kAuxDisplayOscOk = 1U << 4; // 1 = Xtal oscillation is stable

// --- Direct commands ([DS] Table 13; [REF] st25r3916.h:87-88) --------------
// These bytes are complete as listed in the datasheet -- 0xC1 is already
// 11_000001, i.e. the mode bits are baked into the tabulated code.
constexpr uint8_t kCmdSetDefault = 0xC1U; // power-up state ([DS] Sec 4.4.1)
// (0xC2 "Stop all activities" is deliberately NOT defined here yet: nothing in
// this task issues it, and an unused constant is one more thing to keep true.
// Set Default already performs a Stop internally per [DS] Sec 4.4.1.)

// [REF] st25r3916.h:138 -- "Max timeout for Oscillator to get stable
// DS: 700us", with the driver itself allowing 10 ms. We have no IRQ line
// ([M5]), so this is a polling budget rather than an interrupt wait.
constexpr uint32_t kOscStableTimeoutMs = 10U;

// Settle delay after Set Default. [DS] Sec 4.4.1 documents no completion IRQ
// for this command, and Table 13 marks it "Interrupt after termination: No",
// so there is nothing to wait ON -- but it does reset every register, so give
// the chip a moment before reading one back. 1 ms is far more than a register
// reset needs and costs nothing at init time.
constexpr uint32_t kSetDefaultSettleMs = 1U;

bool s_initialized = false;

// Read one space-A register.
//
// Framing ([DS] Sec 4.3.4 + Figure 26's spelled-out legend):
//   S, slave addr+W, <01 A5..A0>, Sr, slave addr+R, data, NAK, P
//
// ARDUINO-ESP32 SPECIFIC, verified in this framework's own source rather than
// assumed (packages/framework-arduinoespressif32/libraries/Wire/src/Wire.cpp):
//   * endTransmission(false) does NOT transmit anything and does NOT report
//     bus errors -- it only sets an internal `nonStop` flag and unconditionally
//     returns 0 (Wire.cpp:445-476). Checking its return value would be
//     checking a constant.
//   * It also deliberately KEEPS the bus lock held, handing it to the
//     following requestFrom(). Returning early between the two would leak that
//     lock and wedge Wire1 for every other caller. Hence: once the deferred
//     path is entered, requestFrom() must always run.
//   * requestFrom() with `nonStop` set issues the real combined
//     write-then-repeated-START-read via i2cWriteReadNonStop() and returns the
//     number of bytes actually received (Wire.cpp:517-534). THAT return value
//     is the only place a NACK from the chip becomes visible, so it is what
//     this function tests.
bool readRegisterRaw(uint8_t reg, uint8_t *val_out) {
    if (val_out == nullptr || reg > kMaxSpaceARegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    if (Wire1.write(static_cast<uint8_t>(reg | kModeRead)) != 1) {
        // Nothing has gone out on the wire yet; close the transaction the
        // normal way so the lock is released.
        Wire1.endTransmission(true);
        return false;
    }
    Wire1.endTransmission(false); // deferred; see note above -- return value
                                  // is a constant 0 on this core, not a status
    if (Wire1.requestFrom(kI2cAddr, static_cast<size_t>(1)) != 1) {
        return false;
    }
    if (!Wire1.available()) {
        return false;
    }
    *val_out = static_cast<uint8_t>(Wire1.read());
    return true;
}

// ===========================================================================
// Additions for the NFC-A polled reader. Citations are in the SOURCES block at
// the top of this file; each constant carries the table/line it came from.
// ===========================================================================

// [DS] Table 11 (p.49) -- FIFO mode bytes. Unlike a register access these
// carry no address; the byte IS the whole command.
constexpr uint8_t kFifoLoad = 0x80U; // 1000_0000b
constexpr uint8_t kFifoRead = 0x9FU; // 1001_1111b

// [DS] Table 13 (p.58) / [REF] st25r3916.h:89-119
constexpr uint8_t kCmdStop          = 0xC2U; // Stop all activities (clears FIFO)
constexpr uint8_t kCmdTxWithCrc     = 0xC4U;
constexpr uint8_t kCmdTxWithoutCrc  = 0xC5U;
// (C6h "Transmit REQA" is deliberately NOT defined: nfca_detect() uses WUPA
// for every wake, for the state-machine reason spelled out at its call site.
// Same unused-constant policy as the 0xC2 note further up this file.)
constexpr uint8_t kCmdTxWupa        = 0xC7U;
constexpr uint8_t kCmdResetRxGain   = 0xD5U;
constexpr uint8_t kCmdSpaceBAccess  = 0xFBU;

// --- Space-A registers ([DS] tables cited above; [REF] st25r3916_com.h:93-204)
constexpr uint8_t kRegIoConf2      = 0x01U;
constexpr uint8_t kRegMode         = 0x03U;
constexpr uint8_t kRegBitRate      = 0x04U;
constexpr uint8_t kRegIso14443aNfc = 0x05U;
constexpr uint8_t kRegAux          = 0x0AU;
constexpr uint8_t kRegRxConf1      = 0x0BU;
constexpr uint8_t kRegRxConf2      = 0x0CU;
constexpr uint8_t kRegRxConf3      = 0x0DU;
constexpr uint8_t kRegRxConf4      = 0x0EU;
constexpr uint8_t kRegMaskRxTimer  = 0x0FU;
constexpr uint8_t kRegNrt1         = 0x10U;
constexpr uint8_t kRegNrt2         = 0x11U;
constexpr uint8_t kRegTimerEmvCtrl = 0x12U;
constexpr uint8_t kRegIrqMain      = 0x1AU; // 1Ah..1Dh read back to back
constexpr uint8_t kIrqRegCount     = 4U;
constexpr uint8_t kRegFifoStatus1  = 0x1EU;
constexpr uint8_t kRegFifoStatus2  = 0x1FU;
// (The Collision display register at 20h is deliberately NOT defined: this
// driver reports a collision instead of resolving it, so it never reads the
// collision position. Same policy as the 0xC2 note above -- an unused constant
// is one more thing to keep true.)
constexpr uint8_t kRegNumTxBytes1  = 0x22U;
constexpr uint8_t kRegNumTxBytes2  = 0x23U;
constexpr uint8_t kRegAntTuneA     = 0x26U;
constexpr uint8_t kRegAntTuneB     = 0x27U;
constexpr uint8_t kRegTxDriver     = 0x28U;

// --- Space-B registers (address WITHIN space B; the FBh prefix is added by
// {read,write}RegisterBRaw, not baked into these values)
constexpr uint8_t kRegBCorrConf1   = 0x0CU;
constexpr uint8_t kRegBCorrConf2   = 0x0DU;
// (0x28, space-B AUX_MOD, is deliberately NOT defined: this driver never
// writes it -- see DELIBERATE SIMPLIFICATIONS #3 above. Same unused-constant
// policy as the 0xC2/0xC6/collision-register notes elsewhere in this file.)
constexpr uint8_t kRegBResAmMod    = 0x2AU;
constexpr uint8_t kRegBOvershoot1  = 0x30U;
constexpr uint8_t kRegBOvershoot2  = 0x31U;
constexpr uint8_t kRegBUndershoot1 = 0x32U;
constexpr uint8_t kRegBUndershoot2 = 0x33U;

// --- Bit positions ---------------------------------------------------------
constexpr uint8_t kIoConf2AatEn = 1U << 5; // [REF] st25r3916_com.h:234

// [DS] Table 22 (p.73) + Table 23: om<3:0> = bits 6..3, 0001b = ISO14443A.
constexpr uint8_t kModeOmMask       = 0x0FU << 3; // 0x78
constexpr uint8_t kModeOmIso14443a  = 0x01U << 3; // 0x08
constexpr uint8_t kModeTrAm         = 1U << 2;    // 0 = OOK, 1 = AM

// [DS] Table 25/26 (p.74): tx_rate bits 5..4, rx_rate bits 1..0, 00b = ~106.
constexpr uint8_t kBitRate106Both = 0x00U;

// [DS] Table 27 (p.75)
constexpr uint8_t kIso14443aNoTxPar = 1U << 7;
constexpr uint8_t kIso14443aNoRxPar = 1U << 6;
constexpr uint8_t kIso14443aNfcF0   = 1U << 5;
constexpr uint8_t kIso14443aAntcl   = 1U << 0;

// [DS] Table 36 (p.80). Note 1 of that table says receive-without-CRC is
// applied AUTOMATICALLY for the Transmit REQA/WUPA direct commands and while
// antcl is set -- so setting no_crc_rx for those two exchanges is belt-and-
// braces (and is exactly what [EH] rfal_rfst25r3916.cpp:2016/2124 does).
constexpr uint8_t kAuxNoCrcRx = 1U << 7;
constexpr uint8_t kAuxDisCorr = 1U << 2; // [DS] Table 37: 0 = correlator for ISO-A

constexpr uint8_t kRxConf2AgcEn = 1U << 3; // [REF] st25r3916_com.h:482

// [DS] Table 51 (p.91)
constexpr uint8_t kTimerEmvMrtStep = 1U << 3; // 0 = 64/fc
constexpr uint8_t kTimerEmvNrtStep = 1U << 0; // 0 = 64/fc

// [DS] Table 67 (p.101)
constexpr uint8_t kFifoStatus2ByteHiMask   = 3U << 6;
constexpr uint8_t kFifoStatus2ByteHiShift  = 6U;
constexpr uint8_t kFifoStatus2LastBitsMask = 7U << 1;
constexpr uint8_t kFifoStatus2LastBitsShift = 1U;

// [REF] st25r3916_com.h:649-664
constexpr uint8_t kTxDriverDResMask       = 0x0FU << 0;
constexpr uint8_t kTxDriverAmModMask      = 0x0FU << 4;
constexpr uint8_t kTxDriverAmMod12Percent = 0x07U << 4;

// --- Interrupt status bits, as a 32-bit word made of regs 1Ah..1Dh in that
// order ([DS] Tables 62-65; values identical to [REF] st25r3916_interrupt.h:
// 68-89, which is where the 32-bit packing convention comes from).
constexpr uint32_t kIrqRxe  = 0x00000010U; // 1Ah bit 4
// (1Ah bit 3, I_txe, is deliberately NOT defined: transceive() waits on a
// combined terminal mask that does not include end-of-transmission -- see the
// comment at its call site. Same unused-constant policy as the 0xC2/0xC6/
// collision-register notes elsewhere in this file.)
constexpr uint32_t kIrqCol  = 0x00000004U; // 1Ah bit 2
constexpr uint32_t kIrqNre  = 0x00004000U; // 1Bh bit 6
constexpr uint32_t kIrqErr1 = 0x00100000U; // 1Ch bit 4 (hard framing)
constexpr uint32_t kIrqErr2 = 0x00200000U; // 1Ch bit 5 (soft framing)
constexpr uint32_t kIrqPar  = 0x00400000U; // 1Ch bit 6
constexpr uint32_t kIrqCrc  = 0x00800000U; // 1Ch bit 7

// --- Timing ----------------------------------------------------------------
// FWT for every NFC-A frame this driver sends. [REF] rfal_nfca.h:89
// RFAL_NFCA_FDTMIN = 1620 (1/fc), plus [REF] rfal_rfst25r3916.h:291,296
// RFAL_FWT_ADJUSTMENT (64) + RFAL_FWT_A_ADJUSTMENT (512+64), converted to the
// NRT's 64/fc steps exactly as [REF] rfalISO14443ATransceiveShortFrame() does.
constexpr uint32_t kNfcaFwt1fc  = 1620U;
constexpr uint32_t kFwtAdjust1fc = 64U + 512U + 64U;
constexpr uint16_t kNrtSteps64fc =
    static_cast<uint16_t>((kNfcaFwt1fc + kFwtAdjust1fc) / 64U); // 35 -> ~165 us
static_assert(kNrtSteps64fc != 0,
              "an NRT of zero means 'timer not started' ([DS] Table 49) -- the "
              "no-response timeout would never fire and every miss would hang "
              "until the software deadline instead");

// RATS/ISO14443-4-activation NRT, real bug found & fixed via real-hardware
// testing (2026-08-23): transceive() used to hardcode EVERY exchange's
// hardware no-response timer to kNrtSteps64fc (~165us -- correct for a plain
// NFC-A anticollision reply) regardless of which command was being sent.
// RATS legitimately takes longer: a real card's ATS-generation logic (it may
// need to compute/format its answer, not just echo a fixed frame) needs the
// activation-specific timeout RFAL itself uses, confirmed directly against
// [REF] ~/src/wilson-elechouse/ST25R3916/NFC-RFAL/src/rfal_isoDep.cpp:202-210:
//   RFAL_ISODEP_T4T_DTIME_POLL_11 = 216960 (1/fc) -- "Delta Time for polling
//   during Activation (ATS): 16.4ms, Digital 1.1 13.8.1.1 & A.6 -- use 16ms
//   as testcase T4AT_BI_10_03 sends a frame exactly at the border" (216960 /
//   13560 = 16.0ms exactly -- a deliberately chosen literal, not a derived
//   approximation)
//   RFAL_ISODEP_T4T_FWT_ACTIVATION = 71680 + RFAL_ISODEP_T4T_DTIME_POLL_11
//   = 288640 (1/fc) -- "Activation frame waiting time FWT(act) = 71680/fc
//   (~5286us), Digital 1.1 13.8.1.1 & A.6" is the base FWT; RFAL adds the
//   16ms polling-delta margin ON TOP of it for the real activation timeout
//   it actually uses (rfal_isoDep.cpp:933, the real RATS transceive call) --
//   both terms are real and both are needed, not a double-count.
// Confirmed on real hardware: with the old fixed ~165us NRT, RATS got a
// genuine hardware-timer Xfer::kNoResponse on TWO different real payment
// cards, consistently -- not a per-card fluke. 288640/64 = 4510 exactly (a
// clean multiple, a good sign this is the intended real value): using this
// for RATS specifically, all other calls unaffected (still default to
// kNrtSteps64fc).
constexpr uint16_t kRatsNrtSteps64fc = 4510U; // 288640 (1/fc) / 64 = ~21.29ms

// FDT(listen) min for NFC-A: [REF] rfal_rf.h:239 RFAL_FDT_LISTEN_NFCA_POLLER
// = 1172 (1/fc), less [REF] rfal_rfst25r3916.h:283,325's
// RFAL_FDT_LISTEN_MRT_ADJUSTMENT (64) + RFAL_FDT_LISTEN_A_ADJUSTMENT (276-64),
// in the MRT's 64/fc steps.
constexpr uint8_t kMrtSteps64fc =
    static_cast<uint8_t>((1172U - (64U + (276U - 64U))) / 64U); // 14 -> ~66 us

// [REF] rfal_rf.h:230 RFAL_GT_NFCA = 5 ms guard time between switching the
// field on and the first command to a tag.
constexpr uint32_t kGuardTimeMs = 5U;

// Wall-clock ceiling for one nfca_detect() pass. Real cost is dominated by
// this bus (100 kHz -> ~0.3 ms per register write, ~0.8 ms for the 4-byte IRQ
// read), not by RF: the chip's own no-response timeout is ~165 us, so a miss
// is normally visible on the FIRST IRQ read after the transmit command. The
// budget exists to bound a wedged bus, not normal operation, and is set well
// under the project's ~50 ms poll() ceiling.
constexpr uint32_t kDetectBudgetMs = 25U;

// --- Transport helpers -----------------------------------------------------

// Multi-byte space-A register read. [DS] Sec 4.3.4 + Table 11's note that the
// register modes support address auto-incrementing; this is [EH]
// st25r3916ReadMultipleRegisters()'s I2C branch (st25r3916_com.cpp:229-249)
// with the digitalRead()-gated interrupt bookkeeping removed.
bool readRegistersRaw(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (buf == nullptr || len == 0 || reg > kMaxSpaceARegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    if (Wire1.write(static_cast<uint8_t>(reg | kModeRead)) != 1) {
        Wire1.endTransmission(true);
        return false;
    }
    Wire1.endTransmission(false); // deferred; see readRegisterRaw()'s note
    if (Wire1.requestFrom(kI2cAddr, static_cast<size_t>(len)) != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire1.available()) {
            return false;
        }
        buf[i] = static_cast<uint8_t>(Wire1.read());
    }
    return true;
}

// Space-B access. [DS] Sec 4.3.4 (p.56): "byte FBh has to be inserted between
// the I2C slave address and the register read or write mode byte. Access to
// register space-B remains active until an I2C Stop Condition is received."
// The mode byte itself is the ordinary space-A shape -- the space-B marker
// [REF] carries in bit 6 of its address constants is a software-only tag and
// must NOT reach the wire.
bool readRegisterBRaw(uint8_t reg, uint8_t *val_out) {
    if (val_out == nullptr || reg > kMaxSpaceARegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    const bool queued = (Wire1.write(kCmdSpaceBAccess) == 1) &&
                        (Wire1.write(static_cast<uint8_t>(reg | kModeRead)) == 1);
    if (!queued) {
        Wire1.endTransmission(true);
        return false;
    }
    Wire1.endTransmission(false);
    if (Wire1.requestFrom(kI2cAddr, static_cast<size_t>(1)) != 1) {
        return false;
    }
    if (!Wire1.available()) {
        return false;
    }
    *val_out = static_cast<uint8_t>(Wire1.read());
    return true;
}

bool writeRegisterBRaw(uint8_t reg, uint8_t val) {
    if (reg > kMaxSpaceARegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    const bool queued = (Wire1.write(kCmdSpaceBAccess) == 1) &&
                        (Wire1.write(static_cast<uint8_t>(reg | kModeWrite)) == 1) &&
                        (Wire1.write(val) == 1);
    const bool sent = (Wire1.endTransmission(true) == 0);
    return queued && sent;
}

// [DS] Table 11 + Figure 23 (p.55): slave address, the 80h FIFO-load mode
// byte, then the payload. No register address.
bool writeFifoRaw(const uint8_t *data, uint8_t len) {
    if (data == nullptr || len == 0) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    bool queued = (Wire1.write(kFifoLoad) == 1);
    for (uint8_t i = 0; i < len && queued; i++) {
        queued = (Wire1.write(data[i]) == 1);
    }
    const bool sent = (Wire1.endTransmission(true) == 0);
    return queued && sent;
}

// [DS] Table 11 + Figure 24 (p.55).
bool readFifoRaw(uint8_t *buf, uint8_t len) {
    if (buf == nullptr || len == 0) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    if (Wire1.write(kFifoRead) != 1) {
        Wire1.endTransmission(true);
        return false;
    }
    Wire1.endTransmission(false);
    if (Wire1.requestFrom(kI2cAddr, static_cast<size_t>(len)) != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire1.available()) {
            return false;
        }
        buf[i] = static_cast<uint8_t>(Wire1.read());
    }
    return true;
}

// Space-A register write / direct command without the bus-bring-up guard that
// the public St25r3916::write_register()/execute_command() re-run on every
// call. Everything below is only ever reached after init()/nfca_poller_begin()
// has already run that guard once, and an exchange issues a dozen of these.
bool writeRegisterRaw(uint8_t reg, uint8_t val) {
    if (reg > kMaxSpaceARegister) {
        return false;
    }
    Wire1.beginTransmission(kI2cAddr);
    const bool queued = (Wire1.write(static_cast<uint8_t>(reg | kModeWrite)) == 1) &&
                        (Wire1.write(val) == 1);
    const bool sent = (Wire1.endTransmission(true) == 0);
    return queued && sent;
}

bool executeCommandRaw(uint8_t cmd) {
    Wire1.beginTransmission(kI2cAddr);
    Wire1.write(static_cast<uint8_t>(cmd | kModeCmd));
    return Wire1.endTransmission(true) == 0;
}

// Read-modify-write of a masked field, the equivalent of [EH]
// st25r3916ChangeRegisterBits().
bool changeRegisterBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t cur = 0;
    if (!readRegisterRaw(reg, &cur)) {
        return false;
    }
    const uint8_t next = static_cast<uint8_t>((cur & ~mask) | (value & mask));
    if (next == cur) {
        return true;
    }
    return writeRegisterRaw(reg, next);
}

bool changeRegisterBitsB(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t cur = 0;
    if (!readRegisterBRaw(reg, &cur)) {
        return false;
    }
    const uint8_t next = static_cast<uint8_t>((cur & ~mask) | (value & mask));
    if (next == cur) {
        return true;
    }
    return writeRegisterBRaw(reg, next);
}

// --- Polled interrupt status ------------------------------------------------
// THE substitute for the IRQ pin this hardware does not have. Regs 1Ah..1Dh
// are read-and-clear ([DS] Tables 63-65's shared footnote, plus Table 62/1Ah
// via Sec 4.3.1 prose and Table 64's footnote), so one 4-byte burst read both
// samples and consumes the pending interrupt state -- which is
// precisely what [EH] st25r3916ClearInterrupts() (st25r3916_interrupt.cpp:
// 225-233) does. The only thing the IRQ pin ever contributed was permission to
// bother reading; on a bus this slow, reading unconditionally costs less than
// the RF exchange it is waiting on.
bool sampleIrqs(uint32_t *acc) {
    uint8_t regs[kIrqRegCount] = {0, 0, 0, 0};
    if (!readRegistersRaw(kRegIrqMain, regs, kIrqRegCount)) {
        return false;
    }
    *acc |= static_cast<uint32_t>(regs[0]) |
            (static_cast<uint32_t>(regs[1]) << 8) |
            (static_cast<uint32_t>(regs[2]) << 16) |
            (static_cast<uint32_t>(regs[3]) << 24);
    return true;
}

// Drops whatever is currently latched, so a following wait cannot be satisfied
// by a stale bit from the previous exchange.
bool clearIrqs() {
    uint32_t discard = 0;
    return sampleIrqs(&discard);
}

// Polls until any bit of `want` is latched, the absolute `deadline_ms` passes,
// or the bus fails. Returns the accumulated status; *io_ok goes false only on
// an I2C failure, which is a different problem from "the tag said nothing".
uint32_t waitIrqs(uint32_t want, uint32_t deadline_ms, bool *io_ok) {
    uint32_t acc = 0;
    *io_ok = true;
    for (;;) {
        if (!sampleIrqs(&acc)) {
            *io_ok = false;
            return acc;
        }
        if ((acc & want) != 0U) {
            return acc;
        }
        if (static_cast<int32_t>(millis() - deadline_ms) >= 0) {
            return acc;
        }
        yield(); // ~0.8 ms of I2C per iteration; keep the RTOS/WDT happy anyway
    }
}

// --- One ISO14443-A exchange ------------------------------------------------
enum class Xfer : uint8_t {
    kOk,          // a frame came back and is in rx[0..*rx_len)
    kNoResponse,  // NRT expired (or the software deadline did) with no RXE
    kCollision,   // more than one tag answered
    kProtocol,    // framing/parity/CRC error, or a response that cannot be stored
    kIo,          // the chip/bus stopped answering
};

// [REF] rfal_rf.h:99 RFAL_CRC_LEN = 2 -- the ISO14443 CRC_A width. This
// chip's FIFO still contains these 2 bytes after a crc_rx=true receive even
// though the CRC was already verified in hardware; see transceive()'s own
// content_len comment below for the real bug this constant's use fixes.
constexpr uint16_t kCrcLen = 2U;

// `short_cmd` is 0 for an ordinary FIFO-sourced frame, or the C6h/C7h direct
// command for REQA/WUPA. Those two are 7-bit short frames the chip generates
// itself ([DS] Table 13) and cannot be expressed as FIFO bytes, so `tx`/
// `tx_len` are ignored when one is given.
// `nrt_steps` (64/fc units) sizes the CHIP's own hardware no-response timer
// for this specific exchange -- defaults to kNrtSteps64fc (~165us, correct
// for a plain NFC-A anticollision reply). Pass kRatsNrtSteps64fc for RATS;
// see that constant's own declaration comment for the real bug this
// parameterization fixes (every call used to share one fixed, too-short
// value regardless of which command was actually being sent).
Xfer transceive(uint8_t short_cmd,
                const uint8_t *tx, uint8_t tx_len, bool crc_tx,
                bool antcl, bool crc_rx,
                uint8_t *rx, uint8_t rx_cap, uint8_t *rx_len,
                uint32_t deadline_ms, uint16_t nrt_steps = kNrtSteps64fc) {
    if (rx_len != nullptr) {
        *rx_len = 0;
    }

    // [EH] rfalPrepareTransceive() (rfal_rfst25r3916.cpp:1145-1160): reset the
    // receive logic and the Rx gain before every exchange. Stop also clears the
    // FIFO ([DS] Table 13).
    if (!executeCommandRaw(kCmdStop) || !executeCommandRaw(kCmdResetRxGain)) {
        return Xfer::kIo;
    }

    // [EH] rfalPrepareTransceive() (:1190-1209): hardware parity in both
    // directions and no NFCIP-1 transport framing; plus this exchange's antcl.
    if (!changeRegisterBits(kRegIso14443aNfc,
                            static_cast<uint8_t>(kIso14443aNoTxPar | kIso14443aNoRxPar |
                                                 kIso14443aNfcF0 | kIso14443aAntcl),
                            antcl ? kIso14443aAntcl : 0U)) {
        return Xfer::kIo;
    }
    if (!changeRegisterBits(kRegAux, kAuxNoCrcRx, crc_rx ? 0U : kAuxNoCrcRx)) {
        return Xfer::kIo;
    }
    // AGC on: [EH] only turns it off for anticollision when the COHERENT
    // receiver is selected (rfal_rfst25r3916.cpp:2148). This driver uses the
    // correlator ([DS] Table 37, dis_corr = 0), so AGC stays enabled.
    if (!changeRegisterBits(kRegRxConf2, kRxConf2AgcEn, kRxConf2AgcEn)) {
        return Xfer::kIo;
    }

    // Timers, in 64/fc steps for both ([DS] Table 51 bits mrt_step/nrt_step).
    // NRT is a 16-bit counter (kRegNrt1 = high byte, kRegNrt2 = low byte) --
    // real bug fixed here (2026-08-23): this used to hardcode kRegNrt1 to 0,
    // silently capping every exchange's no-response wait at 255 steps
    // (~1.2ms) regardless of `nrt_steps`. RATS needs kRatsNrtSteps64fc=4510,
    // which does not fit in the low byte alone -- see that constant's own
    // declaration comment for the real citation and hardware confirmation.
    if (!changeRegisterBits(kRegTimerEmvCtrl,
                            static_cast<uint8_t>(kTimerEmvMrtStep | kTimerEmvNrtStep), 0U) ||
        !writeRegisterRaw(kRegMaskRxTimer, kMrtSteps64fc) ||
        !writeRegisterRaw(kRegNrt1, static_cast<uint8_t>(nrt_steps >> 8)) ||
        !writeRegisterRaw(kRegNrt2, static_cast<uint8_t>(nrt_steps & 0xFFU))) {
        return Xfer::kIo;
    }

    if (!clearIrqs()) {
        return Xfer::kIo;
    }

    if (short_cmd != 0U) {
        // [DS] Table 71 Note 1 / [EH] rfal_rfst25r3916.cpp:2068-2071: nbtx must
        // be 000 before a REQA/WUPA direct command or the chip raises a parity
        // error ("If anctl bit is set while card is in idle state and nbtx is
        // not 000, then i_par will be triggered during REQA and WUPA").
        if (!writeRegisterRaw(kRegNumTxBytes2, 0U) || !executeCommandRaw(short_cmd)) {
            return Xfer::kIo;
        }
    } else {
        if (tx == nullptr || tx_len == 0) {
            return Xfer::kProtocol;
        }
        // [EH] st25r3916SetNumTxBits() (st25r3916.cpp:364-368): the register
        // pair holds a BIT count -- low byte to 23h, high byte to 22h. Every
        // frame this driver sends is whole bytes, so nbtx stays 000.
        const uint16_t tx_bits = static_cast<uint16_t>(tx_len) * 8U;
        if (!writeRegisterRaw(kRegNumTxBytes2, static_cast<uint8_t>(tx_bits & 0xFFU)) ||
            !writeRegisterRaw(kRegNumTxBytes1, static_cast<uint8_t>(tx_bits >> 8)) ||
            !writeFifoRaw(tx, tx_len) ||
            !executeCommandRaw(crc_tx ? kCmdTxWithCrc : kCmdTxWithoutCrc)) {
            return Xfer::kIo;
        }
    }

    // One combined wait, not the two ([EH] waits TXE then RXE) that made sense
    // when an edge-triggered pin delivered each event separately: the status
    // registers latch, so by the time the first 4-byte read completes (~0.8 ms
    // on this 100 kHz bus, against a ~165 us no-response timeout) the whole
    // outcome of the exchange is normally already sitting there.
    bool io_ok = true;
    const uint32_t terminal = kIrqRxe | kIrqNre | kIrqCol | kIrqErr1 | kIrqErr2;
    const uint32_t irqs = waitIrqs(terminal, deadline_ms, &io_ok);
    if (!io_ok) {
        return Xfer::kIo;
    }

    if ((irqs & kIrqCol) != 0U) {
        return Xfer::kCollision;
    }
    if ((irqs & kIrqRxe) == 0U) {
        // NRE, or the software deadline. Either way nothing answered.
        return Xfer::kNoResponse;
    }
    // Hard framing corrupts the data AND (per [DS] Tables 67/68) invalidates
    // the FIFO last-bit and collision fields, so there is nothing to salvage.
    if ((irqs & kIrqErr1) != 0U) {
        return Xfer::kProtocol;
    }
    if (crc_rx && ((irqs & (kIrqCrc | kIrqPar)) != 0U)) {
        return Xfer::kProtocol;
    }

    uint8_t st1 = 0;
    uint8_t st2 = 0;
    if (!readRegisterRaw(kRegFifoStatus1, &st1) ||
        !readRegisterRaw(kRegFifoStatus2, &st2)) {
        return Xfer::kIo;
    }
    const uint16_t n = static_cast<uint16_t>(
        (static_cast<uint16_t>((st2 & kFifoStatus2ByteHiMask) >> kFifoStatus2ByteHiShift) << 8) |
        st1);
    const uint8_t last_bits = static_cast<uint8_t>(
        (st2 & kFifoStatus2LastBitsMask) >> kFifoStatus2LastBitsShift);

    if (n == 0U || n > rx_cap) {
        return Xfer::kProtocol;
    }
    // Every response this driver expects is a whole number of bytes. A partial
    // last byte here means the tag answered something other than the frame we
    // asked for -- report it rather than silently rounding.
    if (last_bits != 0U) {
        return Xfer::kProtocol;
    }
    if (!readFifoRaw(rx, static_cast<uint8_t>(n))) {
        return Xfer::kIo;
    }
    // Real bug found & fixed via real-hardware testing (2026-08-23): when
    // crc_rx is true, this chip already verifies the CRC in hardware (a
    // mismatch already returned Xfer::kProtocol above) but still places the
    // 2 already-verified CRC bytes in the FIFO on top of the real content --
    // [EH]'s own rfalTransceiveRx() does exactly this unconditional
    // subtraction (rfal_rfst25r3916.cpp:1730-1738: "By default CRC will not
    // be placed into the rxBuffer... tmp -= RFAL_CRC_LEN", RFAL_CRC_LEN=2,
    // rfal_rf.h:99), which this driver had not been doing at this shared,
    // single choke point every crc_rx=true caller passes through. First
    // found via three real ATS responses from real payment cards, each
    // exactly 2 bytes longer than its own self-declared TL length field
    // (iso14443_4_activate() had a narrower, ATS-specific workaround for
    // this before this driver-wide fix landed); nfca_detect()'s own SEL_RES
    // handling already tolerated either length without needing the
    // subtraction (it only ever reads byte 0). Fixing it HERE means every
    // crc_rx=true caller (SEL_REQ, SLP_REQ, RATS, and Task 13's I-block/WTX
    // exchanges) gets the real, CRC-stripped content length automatically,
    // matching what every caller already assumed before this bug was found.
    uint16_t content_len = n;
    if (crc_rx && content_len > kCrcLen) {
        content_len = static_cast<uint16_t>(content_len - kCrcLen);
    }
    if (rx_len != nullptr) {
        *rx_len = static_cast<uint8_t>(content_len);
    }
    return Xfer::kOk;
}

// --- NFC-A protocol constants ([EH] rfal_nfca.cpp:87-100, "Digital 1.1
// Table 15" for the SEL_CMD codes; the NVB/CT values are ISO14443-3's own and
// are used identically there) ------------------------------------------------
constexpr uint8_t kSelCmdCl1 = 0x93U;
constexpr uint8_t kSelCmdCl2 = 0x95U;
constexpr uint8_t kSelCmdCl3 = 0x97U;
constexpr uint8_t kNvbAnticollision = 0x20U; // "2 bytes sent, 0 extra bits"
constexpr uint8_t kNvbSelect        = 0x70U; // "7 bytes sent" -> SEL_REQ
constexpr uint8_t kCascadeTag       = 0x88U; // CT: this level's UID is partial
constexpr uint8_t kSddResLen        = 5U;    // 4 UID bytes + BCC
constexpr uint8_t kSakLen           = 1U;    // + 2 CRC bytes kept in the FIFO

// SLP_REQ / HLTA, "Digital 1.1 6.9.1 & Table 20" per [EH] rfal_nfca.cpp:58-61.
// Sent with CRC; ISO14443-3 6.4.3 says the PICC acknowledges by staying
// silent, and [EH] rfalNfcaPollerSleep() (rfal_nfca.cpp:528-543) does not
// check for a response at all ("consider the HLTA command always acknowledged
// ... to improve interoperability").
constexpr uint8_t kSlpReq[2] = {0x50U, 0x00U};

uint8_t selCmdForLevel(uint8_t level) {
    switch (level) {
        case 0:  return kSelCmdCl1;
        case 1:  return kSelCmdCl2;
        default: return kSelCmdCl3;
    }
}

bool s_nfca_ready = false;

} // namespace

namespace St25r3916 {

bool read_register(uint8_t reg, uint8_t *val_out) {
    nfc_ensure_external_i2c_begun();
    return readRegisterRaw(reg, val_out);
}

bool write_register(uint8_t reg, uint8_t val) {
    nfc_ensure_external_i2c_begun();
    if (reg > kMaxSpaceARegister) {
        return false;
    }
    // [DS] Sec 4.3.4 / Figure 20 "Writing a single register": S, slave addr+W,
    // <00 A5..A0>, data, P.
    Wire1.beginTransmission(kI2cAddr);
    // Checked for the same reason readRegisterRaw() checks: consistency, and
    // because a short write would otherwise send a mode byte with no data and
    // report success. In practice 2 bytes never fail to enqueue into the
    // 128-byte TX buffer, so this is belt-and-braces rather than a live risk.
    const bool queued = (Wire1.write(static_cast<uint8_t>(reg | kModeWrite)) == 1) &&
                        (Wire1.write(val) == 1);
    const bool sent = (Wire1.endTransmission(true) == 0); // always run: releases
                                                          // the Wire1 lock
    return queued && sent;
}

bool execute_command(uint8_t cmd) {
    nfc_ensure_external_i2c_begun();
    // [DS] Sec 4.3.4 "Direct command mode" / Figure 25: S, slave addr+W,
    // <11 C5..C0>, P. The OR with kModeCmd is a belt-and-braces no-op for the
    // tabulated codes (which already carry the mode bits) and is exactly what
    // [REF] st25r3916ExecuteCommand() does.
    Wire1.beginTransmission(kI2cAddr);
    Wire1.write(static_cast<uint8_t>(cmd | kModeCmd));
    return Wire1.endTransmission(true) == 0;
}

bool read_chip_id(uint8_t *id_out) {
    if (id_out == nullptr) {
        return false;
    }
    // Deliberately does NOT compare against the expected value. Per this
    // task's brief the caller compares, so a mismatch is visible in the log
    // rather than collapsed into a bare false that cannot be told apart from
    // an I2C failure.
    return read_register(kRegIcIdentity, id_out);
}

bool init() {
    if (s_initialized) {
        return true;
    }

    // Reuses hal/nfc_pn532.cpp's bring-up: asserts EXT_5V_EN on the internal-
    // bus IO-expander (PORT.A is UNPOWERED at reset -- see that file's header)
    // and begins Wire1 at TAB5_EXTERNAL_I2C_FREQ_HZ. Idempotent.
    nfc_ensure_external_i2c_begun();

    // [REF] st25r3916Initialize() step 1 ([REF] st25r3916.cpp:117): put the
    // chip in its power-up state before touching anything else. [DS] Sec 4.4.1:
    // this performs Stop all activities, resets all registers to default, and
    // clears all collision bits.
    if (!execute_command(kCmdSetDefault)) {
        Serial.println("quarky-tab5: [st25r3916] Set Default (0xC1) NACKed -- "
                       "nothing is answering at I2C 0x50 on Wire1");
        return false;
    }
    delay(kSetDefaultSettleMs);

    // [REF] st25r3916Initialize() step 2 ([REF] st25r3916.cpp:123-125):
    // CheckChipID, and refuse to proceed on a mismatch (it returns
    // ERR_HW_MISMATCH). Same policy here.
    uint8_t id = 0;
    if (!read_chip_id(&id)) {
        Serial.println("quarky-tab5: [st25r3916] IC identity read (reg 0x3F) failed");
        return false;
    }

    const uint8_t type = id & kIcIdentityIcTypeMask;
    const uint8_t rev  = id & kIcIdentityIcRevMask;
    Serial.printf("quarky-tab5: [st25r3916] IC identity (reg 0x3F) = 0x%02X "
                  "(ic_type=0x%02X, ic_rev=%u)\n",
                  id, type, (unsigned)rev);

    if (type == kIcTypeSt25r3916) {
        Serial.printf("quarky-tab5: [st25r3916] ic_type 0x%02X == ST25R3916/7 "
                      "(DS12484 Rev 3 Table 117: 00101b) -- MATCH\n", type);
    } else if (type == kIcTypeSt25r3916B) {
        // Not a failure: the B variant is the same programming model for
        // everything this driver does. Called out because [REF]'s own
        // CheckChipID additionally requires ic_rev >= 1 on the B part, and
        // because later tasks that use RC calibration must branch on it.
        Serial.printf("quarky-tab5: [st25r3916] ic_type 0x%02X == ST25R3916B "
                      "(not the -AQWT the M5Stack docs list) -- accepted, but "
                      "note the B variant needs the RC-calibration step\n", type);
    } else {
        Serial.printf("quarky-tab5: [st25r3916] ic_type 0x%02X is NEITHER "
                      "ST25R3916 (0x%02X) nor ST25R3916B (0x%02X) -- refusing "
                      "to drive this part\n",
                      type, kIcTypeSt25r3916, kIcTypeSt25r3916B);
        return false;
    }

    s_initialized = true;
    return true;
}

bool field_on() {
    if (!init()) {
        return false;
    }

    // [DS] Sec 4.2.13 "Reader operation": "The Ready mode has to be entered by
    // setting the bit en of the Operation control register. In this mode the
    // oscillator is started and the regulators are enabled. When the
    // oscillator operation is stable an interrupt is sent and bit osc_ok
    // indicates it." We have no IRQ line ([M5]), so we poll osc_ok.
    //
    // [REF] OscOn() (st25r3916.cpp:241-264) does the same three things in the
    // same order -- check en, set en, then REQUIRE aux_display.osc_ok before
    // reporting success -- differing only in that it sleeps on the OSC
    // interrupt where this polls.
    uint8_t op = 0;
    if (!read_register(kRegOpControl, &op)) {
        return false;
    }
    if ((op & kOpControlEn) == 0) {
        if (!write_register(kRegOpControl, static_cast<uint8_t>(op | kOpControlEn))) {
            return false;
        }
    }

    bool osc_ok = false;
    // Track whether the polling loop ever managed to READ the register at all.
    // Without this, "the oscillator never stabilised" and "I2C was dead for the
    // whole 10 ms" produce the identical message, and they call for opposite
    // investigations (a crystal/analog problem vs. a bus problem -- and on this
    // board a torn-down Wire1 is a live possibility, see hal/rf433_gpio.cpp's
    // GPIO53 note).
    bool aux_read_ok = false;
    uint8_t aux = 0;
    const uint32_t deadline = millis() + kOscStableTimeoutMs;
    do {
        if (read_register(kRegAuxDisplay, &aux)) {
            aux_read_ok = true;
            if ((aux & kAuxDisplayOscOk) != 0) {
                osc_ok = true;
                break;
            }
        }
    } while ((int32_t)(millis() - deadline) < 0);

    if (!osc_ok) {
        // [REF] OscOn() returns ERR_SYSTEM in exactly this case. Do not press
        // on and enable the transmitter against an unstable carrier.
        if (aux_read_ok) {
            Serial.printf("quarky-tab5: [st25r3916] oscillator did not report "
                          "osc_ok within %u ms -- aux_display (0x31) last read "
                          "0x%02X, bit 4 clear. The chip is talking; the "
                          "crystal is not stabilising. Field NOT enabled\n",
                          (unsigned)kOscStableTimeoutMs, aux);
        } else {
            Serial.printf("quarky-tab5: [st25r3916] could not read aux_display "
                          "(0x31) even once in %u ms -- this is an I2C failure, "
                          "NOT an oscillator problem. Field NOT enabled\n",
                          (unsigned)kOscStableTimeoutMs);
        }
        return false;
    }

    // [DS] Sec 4.2.13: "Before sending any command to a transponder the
    // transmitter and receiver have to be enabled by setting the bits rx_en
    // and tx_en." tx_en (Table 21 bit 3) is the bit that actually turns the
    // RF field on. This is [REF]'s st25r3916TxRxOn() (st25r3916.h:148).
    //
    // WHY NOT the C8 "NFC initial field ON" direct command ([DS] Table 13):
    // that one performs Initial RF Collision Avoidance first and signals
    // completion via an interrupt -- which this 4-wire unit cannot deliver.
    // [REF] likewise uses the OP_CONTROL bits, not C8, for plain reader
    // field-on; C8 belongs to the NFCIP-1 peer-to-peer flows.
    if (!read_register(kRegOpControl, &op)) {
        return false;
    }
    return write_register(
        kRegOpControl,
        static_cast<uint8_t>(op | kOpControlRxEn | kOpControlTxEn));
}

void field_off() {
    if (!s_initialized) {
        return;
    }
    // [REF] st25r3916TxRxOff() (st25r3916.h:150) / Deinitialize()
    // (st25r3916.cpp:~330): clear rx_en and tx_en, and deliberately LEAVE the
    // oscillator (en) running -- "Disable Tx and Rx, Keep OSC On". Restarting
    // the crystal costs ~700 us ([REF] st25r3916.h:138) every time the field
    // is toggled, which a scan loop does constantly.
    uint8_t op = 0;
    if (!read_register(kRegOpControl, &op)) {
        return;
    }
    write_register(kRegOpControl,
                   static_cast<uint8_t>(op & ~(kOpControlRxEn | kOpControlTxEn)));
}

// ===========================================================================
// ISO14443-A / NFC-A polled reader (added by Phase 3 Task 4's fix round)
// ===========================================================================

bool read_register_b(uint8_t reg, uint8_t *val_out) {
    nfc_ensure_external_i2c_begun();
    return readRegisterBRaw(reg, val_out);
}

bool write_register_b(uint8_t reg, uint8_t val) {
    nfc_ensure_external_i2c_begun();
    return writeRegisterBRaw(reg, val);
}

namespace {

// The NFC-A 106 kb/s poller register programme.
//
// PROVENANCE: every value here is copied from ST's own analog-configuration
// table, [EH] rfal_rfst25r3916_analogConfigTbl.h, entries
//   RFAL_ANALOG_CONFIG_TECH_CHIP | CHIP_INIT                       (:263-281)
//   RFAL_ANALOG_CONFIG_TECH_CHIP | CHIP_POLL_COMMON                (:283-295)
//   POLL | TECH_NFCA | BITRATE_COMMON | RX                         (:297-300)
//   POLL | TECH_NFCA | BITRATE_106    | TX                         (:302-309)
//   POLL | TECH_NFCA | BITRATE_106    | RX                         (:311-319)
// plus the mode/bit-rate writes from [EH] rfalSetMode() (rfal_rfst25r3916.cpp:
// 275-284, "Enable ISO14443A mode") and rfalSetBitRate() (:502-506).
//
// FOLDED: RFAL applies those five entries in sequence, so some values are
// written and then immediately overwritten. This applies the NET result and
// says so rather than replaying a write it knows is dead:
//   * CHIP_POLL_COMMON sets MODE.tr_am = AM and OVERSHOOT/UNDERSHOOT = 00h;
//     the NFC-A 106 TX entry then sets tr_am = OOK and OVERSHOOT/UNDERSHOOT =
//     40h/03h. Only the latter is applied here.
//   * CHIP_POLL_COMMON's AUX_MOD (am_mode|res_am) = 00h is the register's own
//     power-up value, which init()'s Set Default has already restored, and it
//     selects between AM variants that OOK does not use. Skipped.
// OMITTED, deliberately (see the "DELIBERATE SIMPLIFICATIONS" note at the top
// of this file): CHIP_INIT's listen/target-mode entries -- external-field
// activation/deactivation thresholds (2Ah/2Bh), passive-target fdel (08h),
// PT_MOD (29h), AUX_MOD load-modulation bits, EMD suppression (space-B 05h) --
// and its two SPI MISO pull-down entries, which [EH]'s own Initialize() also
// skips under I2C (st25r3916.cpp:103-106). None of them is in the poller's
// transmit or receive path.
bool apply_nfca_config() {
    struct RegWrite { bool space_b; uint8_t reg; uint8_t mask; uint8_t value; };
    // mask 0xFF means "write the whole register".
    static const RegWrite kProgramme[] = {
        // --- CHIP_INIT, poller-relevant entries -----------------------------
        {false, kRegIoConf2,  kIoConf2AatEn,      kIoConf2AatEn},      // enable AAT
        {false, kRegTxDriver, kTxDriverDResMask,  0x00U},              // RFO resistance, active Tx
        {true,  kRegBResAmMod, 0xFFU,             0x80U},              // minimum non-overlap
        // --- Mode + bit rate ([DS] Table 22/23, Table 25/26) ----------------
        // MUST come after the oscillator is stable: [DS] Table 22 note 1 --
        // "Register can be written only in case crystal clock is present and
        // stable (oscok = 1)". nfca_poller_begin() calls field_on() first for
        // exactly this reason.
        {false, kRegMode,     kModeOmMask,        kModeOmIso14443a},
        {false, kRegBitRate,  0xFFU,              kBitRate106Both},
        // --- CHIP_POLL_COMMON (net) -----------------------------------------
        {false, kRegTxDriver, kTxDriverAmModMask, kTxDriverAmMod12Percent},
        {false, kRegAntTuneA, 0xFFU,              0x80U},
        {false, kRegAntTuneB, 0xFFU,              0x40U},
        // --- NFC-A Rx common: correlator receiver ([DS] Table 37) ----------
        {false, kRegAux,      kAuxDisCorr,        0x00U},
        // --- NFC-A Tx 106: OOK + ST's overshoot/undershoot protection -------
        {false, kRegMode,     kModeTrAm,          0x00U},
        {true,  kRegBOvershoot1,  0xFFU,          0x40U},
        {true,  kRegBOvershoot2,  0xFFU,          0x03U},
        {true,  kRegBUndershoot1, 0xFFU,          0x40U},
        {true,  kRegBUndershoot2, 0xFFU,          0x03U},
        // --- NFC-A Rx 106 ---------------------------------------------------
        {false, kRegRxConf1,  0xFFU,              0x08U},
        {false, kRegRxConf2,  0xFFU,              0x2DU},
        {false, kRegRxConf3,  0xFFU,              0x00U},
        {false, kRegRxConf4,  0xFFU,              0x00U},
        {true,  kRegBCorrConf1, 0xFFU,            0x51U},
        {true,  kRegBCorrConf2, 0xFFU,            0x00U},
    };

    for (const RegWrite &w : kProgramme) {
        const bool ok = (w.mask == 0xFFU)
                            ? (w.space_b ? writeRegisterBRaw(w.reg, w.value)
                                         : writeRegisterRaw(w.reg, w.value))
                            : (w.space_b ? changeRegisterBitsB(w.reg, w.mask, w.value)
                                         : changeRegisterBits(w.reg, w.mask, w.value));
        if (!ok) {
            Serial.printf("quarky-tab5: [st25r3916] NFC-A config write failed at "
                          "%s register 0x%02X\n",
                          w.space_b ? "space-B" : "space-A", w.reg);
            return false;
        }
    }
    return true;
}

// Copies one cascade level's contribution to the NFCID1. Returns false if it
// would not fit -- which cannot happen for a well-formed 4/7/10-byte UID, but
// a malfunctioning tag is exactly the case this code exists to survive.
bool append_uid(Iso14443aTag *tag, const uint8_t *src, uint8_t n) {
    if (static_cast<uint16_t>(tag->uid_len) + n > sizeof(tag->uid)) {
        return false;
    }
    for (uint8_t i = 0; i < n; i++) {
        tag->uid[tag->uid_len++] = src[i];
    }
    return true;
}

} // namespace

bool nfca_poller_begin() {
    if (s_nfca_ready) {
        return true;
    }
    if (!init()) {
        return false;
    }
    // field_on() first, not last: it is what starts and stabilises the
    // oscillator, and the Mode definition register cannot be written until
    // osc_ok is set ([DS] Table 22 note 1). The ~12 ms during which the
    // carrier is up before apply_nfca_config() lands is harmless -- and
    // more benign than this comment used to claim: [DS] Table 22's Default
    // column shows the chip's power-up modulation setting IS ISO14443A/OOK
    // already, not the NFCIP-1 setting once assumed here, so this window
    // isn't even running a different modulation than the one being
    // configured. Nothing is being addressed yet either way, and the NFC-A
    // guard time below starts after the configuration lands.
    if (!field_on()) {
        return false;
    }
    if (!apply_nfca_config()) {
        // field_on() has already energised the RF field. nfca_poller_end()
        // would normally be what turns it back off, but it early-returns on
        // !s_nfca_ready -- which is still false here -- and nothing else in
        // this file or in nfc_read.cpp's teardown() calls it on THIS specific
        // partial-bring-up-failure path (nfc_read.cpp only calls
        // nfca_poller_end() when s_unit_ready is true, and a failed begin()
        // leaves that false too). Without this, a single I2C hiccup anywhere
        // in the ~20-entry analog-config programme would leave the carrier on
        // with nothing anywhere that will ever switch it off.
        field_off();
        return false;
    }
    // [EH] rfal_rf.h:230 RFAL_GT_NFCA: 5 ms guard time from field-on before
    // the first command may be sent to a tag (Digital 2.0 6.10.4.1).
    delay(kGuardTimeMs);
    s_nfca_ready = true;
    Serial.println("quarky-tab5: [st25r3916] NFC-A poller ready "
                   "(ISO14443A, 106 kb/s, polled IRQ status -- no IRQ pin)");
    return true;
}

void nfca_poller_end() {
    if (!s_nfca_ready) {
        return;
    }
    s_nfca_ready = false;
    // Stop all activities first ([DS] Table 13, C2h): leaves no timer running
    // and clears the FIFO, so a later begin() starts from a known state.
    executeCommandRaw(kCmdStop);
    field_off();
}

NfcaResult nfca_detect(Iso14443aTag *out, bool keep_active) {
    if (out == nullptr) {
        return NfcaResult::kProtocolError;
    }
    if (!s_nfca_ready) {
        return NfcaResult::kHardwareError;
    }

    // NOT const: refreshed before every transceive() call below (WUPA, then
    // each cascade level's SDD_REQ and SEL_REQ). kDetectBudgetMs's own
    // comment says it "bound[s] a wedged bus, not normal operation" for ONE
    // exchange -- computing it once here and reusing it unrefreshed across
    // an entire multi-cascade-level resolution silently turns a per-call
    // budget into a shrinking, shared one. Real bug found via real-hardware
    // testing (2026-08-23): a 7-byte-UID contactless payment card -- the
    // first tag this driver was ever tested against that needs a second
    // cascade level -- got a genuine Xfer::kNoResponse (confirmed via
    // temporary diagnostic instrumentation: zero bytes back, not a corrupted
    // response) on cascade level 1's SDD_REQ. Every prior real-hardware test
    // of this driver used a 4-byte-UID tag (single cascade level, 3 total
    // exchanges -- WUPA+SDD+SEL -- comfortably inside 25ms), so this
    // shared-deadline bug was never exercised before. A 7-byte UID needs 5
    // exchanges (WUPA + 2x SDD_REQ + 2x SEL_REQ), and by real measurement
    // (this function's own per-call cost-model comment above
    // kDetectBudgetMs: ~0.3ms/register write, ~0.8ms/IRQ read, roughly a
    // dozen I2C operations per transceive() call) the original 25ms shared
    // budget was already exhausted, or nearly so, by the time cascade level
    // 1's SDD_REQ was attempted -- so waitIrqs() saw its own software
    // deadline already past and reported kNoResponse immediately, regardless
    // of whether the tag itself would have answered in its real ~165us NRT
    // window ([DS]-cited kNrtSteps64fc comment above).
    uint32_t deadline = millis() + kDetectBudgetMs;

    *out = Iso14443aTag{};

    // --- 1. ALL_REQ (WUPA) -> SENS_RES (ATQA), 2 bytes, no CRC -------------
    //
    // WUPA (C7h), not REQA (C6h), and this is a deliberate choice with a real
    // consequence, so it is spelled out rather than left to be rediscovered:
    // ISO14443-3's state machine puts a tag into ACTIVE once it has been
    // SELECTed, and an ACTIVE or HALTed tag does NOT answer REQA. A reader
    // that used REQA and then left the tag where it was could read it exactly
    // once per physical presentation -- press Scan again without lifting the
    // card and it would report "no tag", which is a bug that looks like flaky
    // hardware. WUPA (ALL_REQ) is answered from both IDLE and HALT, and
    // nfca_detect() parks every tag it reads in HALT (step 3 below). That
    // wake/sleep pairing is [EH]'s own: rfalNfcaPollerFullCollisionResolution()
    // sends WUPA specifically because a Sleep was sent before
    // (rfal_nfca.cpp:385-389, "Activity 1.1 9.3.4.1"), and
    // rfalNfcaPollerTechnologyDetection() sends SLP_REQ after the initial REQA
    // (rfal_nfca.cpp:199-203, "Activity 1.1 9.2.3.6").
    uint8_t atqa[4] = {0, 0, 0, 0};
    uint8_t atqa_len = 0;
    switch (transceive(kCmdTxWupa, nullptr, 0, /*crc_tx=*/false,
                       /*antcl=*/false, /*crc_rx=*/false,
                       atqa, sizeof(atqa), &atqa_len, deadline)) {
        case Xfer::kOk:         break;
        case Xfer::kNoResponse: return NfcaResult::kNoTag;
        case Xfer::kCollision:  return NfcaResult::kCollision;
        case Xfer::kIo:         return NfcaResult::kHardwareError;
        default:
            Serial.println("quarky-tab5: [st25r3916] DIAG nfca_detect: WUPA "
                           "xfer returned an unexpected outcome");
            return NfcaResult::kProtocolError;
    }
    if (atqa_len < 2) {
        Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: ATQA short, "
                      "len=%u\n", (unsigned)atqa_len);
        return NfcaResult::kProtocolError;
    }
    out->atqa[0] = atqa[0];
    out->atqa[1] = atqa[1];

    // --- 2. Per cascade level: SDD_REQ then SEL_REQ -------------------------
    // [EH] rfal_nfca.cpp:232-366. The bit-level collision loop is not ported
    // (see the file header's simplification #1), so NVB is always 20h -- "2
    // bytes of SEL_CMD+NVB sent, no partial byte" -- and any collision aborts.
    for (uint8_t level = 0; level < 3; level++) {
        const uint8_t sel_cmd = selCmdForLevel(level);

        // Refreshed every iteration -- see this deadline variable's own
        // declaration comment above for why a single budget computed once
        // before this loop started a real bug on 7-byte-UID tags.
        deadline = millis() + kDetectBudgetMs;

        uint8_t sdd_req[2] = {sel_cmd, kNvbAnticollision};
        uint8_t sdd_res[8] = {0};
        uint8_t sdd_len = 0;
        const Xfer sdd_xfer = transceive(/*short_cmd=*/0U, sdd_req, sizeof(sdd_req), /*crc_tx=*/false,
                           /*antcl=*/true, /*crc_rx=*/false,
                           sdd_res, sizeof(sdd_res), &sdd_len, deadline);
        switch (sdd_xfer) {
            case Xfer::kOk:         break;
            case Xfer::kCollision:  return NfcaResult::kCollision;
            case Xfer::kIo:         return NfcaResult::kHardwareError;
            case Xfer::kNoResponse: // a tag answered WUPA and then went silent
            default:
                Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: "
                              "SDD_REQ (cascade level %u) xfer=%u (0=kOk,1=kNoResponse,"
                              "2=kCollision,3=kProtocol,4=kIo) sdd_len=%u\n",
                              (unsigned)level, (unsigned)sdd_xfer, (unsigned)sdd_len);
                return NfcaResult::kProtocolError;
        }
        if (sdd_len != kSddResLen) {
            Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: SDD_RES "
                          "wrong length at level %u: got %u, want %u\n",
                          (unsigned)level, (unsigned)sdd_len, (unsigned)kSddResLen);
            return NfcaResult::kProtocolError;
        }
        // BCC check, [EH] rfal_nfca.cpp:332 / rfalNfcaCalculateBcc().
        const uint8_t bcc = static_cast<uint8_t>(sdd_res[0] ^ sdd_res[1] ^
                                                 sdd_res[2] ^ sdd_res[3]);
        if (bcc != sdd_res[4]) {
            Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: BCC "
                          "mismatch at level %u: uid=%02X%02X%02X%02X bcc=%02X "
                          "computed=%02X\n", (unsigned)level, sdd_res[0],
                          sdd_res[1], sdd_res[2], sdd_res[3], sdd_res[4], bcc);
            return NfcaResult::kProtocolError;
        }

        // Refreshed again -- a separate transceive() call from SDD_REQ above,
        // same reasoning as this loop's own top-of-iteration refresh.
        deadline = millis() + kDetectBudgetMs;

        // SEL_REQ: SEL_CMD, NVB=70h, the 4 UID bytes, BCC -- with CRC.
        const uint8_t sel_req[7] = {sel_cmd,    kNvbSelect, sdd_res[0], sdd_res[1],
                                    sdd_res[2], sdd_res[3], sdd_res[4]};
        uint8_t sel_res[8] = {0};
        uint8_t sel_len = 0;
        switch (transceive(/*short_cmd=*/0U, sel_req, sizeof(sel_req), /*crc_tx=*/true,
                           /*antcl=*/false, /*crc_rx=*/true,
                           sel_res, sizeof(sel_res), &sel_len, deadline)) {
            case Xfer::kOk:         break;
            case Xfer::kCollision:  return NfcaResult::kCollision;
            case Xfer::kIo:         return NfcaResult::kHardwareError;
            case Xfer::kNoResponse:
            default:
                Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: "
                              "SEL_REQ (cascade level %u) got no/bad response\n",
                              (unsigned)level);
                return NfcaResult::kProtocolError;
        }
        // SEL_RES is one SAK byte. The chip VERIFIES the CRC in hardware (a
        // mismatch would have raised I_crc and been rejected above) but still
        // places the two CRC bytes in the FIFO -- which is why [EH]'s
        // rfalTransceiveRx() subtracts RFAL_CRC_LEN in software
        // (rfal_rfst25r3916.cpp:1735-1741). Expect 3; accept 1 rather than
        // fail if this silicon strips them, and log the surprise either way.
        if (sel_len < kSakLen) {
            Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: SEL_RES "
                          "too short at level %u: got %u bytes\n",
                          (unsigned)level, (unsigned)sel_len);
            return NfcaResult::kProtocolError;
        }
        if (sel_len != (kSakLen + 2U) && sel_len != kSakLen) {
            Serial.printf("quarky-tab5: [st25r3916] SEL_RES was %u bytes "
                          "(expected 1 SAK + 2 CRC) -- taking byte 0 as SAK\n",
                          (unsigned)sel_len);
        }
        out->sak = sel_res[0];

        // [EH] rfal_nfca.cpp:355-363: a cascade tag (88h) in the first UID
        // byte means this level carried only 3 real UID bytes and another
        // cascade level follows.
        if (sdd_res[0] == kCascadeTag) {
            if (!append_uid(out, &sdd_res[1], 3)) {
                Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: "
                              "append_uid overflow at level %u (cascade)\n",
                              (unsigned)level);
                return NfcaResult::kProtocolError;
            }
            continue;
        }
        if (!append_uid(out, &sdd_res[0], 4)) {
            Serial.printf("quarky-tab5: [st25r3916] DIAG nfca_detect: "
                          "append_uid overflow at level %u (final)\n",
                          (unsigned)level);
            return NfcaResult::kProtocolError;
        }

        // --- 3. SLP_REQ (HLTA): park the tag in HALT --------------------
        // Without this the tag stays ACTIVE and would ignore the WUPA that
        // starts the next detection pass, so a card left sitting on the
        // antenna could be read exactly once. The tag acknowledges by NOT
        // responding (ISO14443-3 6.4.3), so the transceive outcome is
        // deliberately discarded -- same as [EH] rfalNfcaPollerSleep().
        //
        // SKIPPED when keep_active is true (added by Phase 3 Task 13): a
        // HALTed tag does not answer RATS, only WUPA, so a caller that wants
        // to chain an ISO14443-4 activation onto this exact tag needs it left
        // ACTIVE. Every existing caller gets the default (false) and this
        // block runs exactly as before -- see keep_active's own doc comment
        // in the header for the full backward-compatibility argument.
        if (!keep_active) {
            uint8_t slp_rx[4] = {0};
            uint8_t slp_len = 0;
            (void)transceive(/*short_cmd=*/0U, kSlpReq, sizeof(kSlpReq), /*crc_tx=*/true,
                             /*antcl=*/false, /*crc_rx=*/true,
                             slp_rx, sizeof(slp_rx), &slp_len, millis() + 2U);
        }

        return NfcaResult::kFound;
    }

    // Three cascade levels all reported "more to come" -- ISO14443-3 has no
    // fourth level, so the tag is not following the standard.
    Serial.println("quarky-tab5: [st25r3916] DIAG nfca_detect: exhausted all "
                   "3 cascade levels, still incomplete");
    return NfcaResult::kProtocolError;
}

// ===========================================================================
// NFC Forum Type 2 Tag (MIFARE Ultralight / NTAG21x family) page read
// (added 2026-08-24 by Phase 3 Task 24's content-emulation extension)
//
// WHY THIS EXISTS. Listen Mode emulation that answers only anticollision/
// SELECT was proven useless against real readers on 2026-08-24 (see the
// Listen Mode section's own note further down this file). To answer a
// reader's READ commands with real data, the emulator first needs real data
// -- and NfcCommon::TagInfo carried nothing but UID/SAK/ATQA. This is the
// capture half: the same real T2T protocol features/nfc/nfc_amiibo.cpp has
// been reading since 2026-08-21, but through the NFC unit (this chip) rather
// than the separate RFID2/WS1850S unit that module drives, so that a tag
// scanned and saved on the NFC-unit screen carries its own page content into
// the tag library.
//
// SOURCES FOR THIS SECTION ONLY, same discipline as every other section here:
//
// [REF-T2T] ~/src/wilson-elechouse/ST25R3916/NFC-RFAL/src/rfal_t2t.cpp/.h --
//      ST's own NFC Forum Type 2 Tag reader layer, part of the same vendored
//      RFAL tree already cited above for rfal_nfca.cpp/rfal_isoDep.cpp, read
//      directly for this task:
//        - rfal_t2t.cpp:73-77 `rfalT2Tcmds`, annotated "NFC-A T2T command set
//          T2T 1.0 5.1": RFAL_T2T_CMD_READ = 0x30, RFAL_T2T_CMD_WRITE = 0xA2,
//          RFAL_T2T_CMD_SECTOR_SELECT = 0xC2. This is the citation for the
//          0x30 command byte below -- NOT recall, and NOT MFRC522's
//          identically-valued PICC_CMD_MF_READ (which is the same real byte
//          for the same real reason, and is what nfc_amiibo.cpp's donor loop
//          goes through).
//        - rfal_t2t.cpp:80-84 `rfalT2TReadReq`, "T2T 1.0 5.2 and table 11":
//          the whole command is exactly two bytes, `code` then `blNo`. CRC is
//          appended by the hardware (rfalT2TPollerRead() passes
//          RFAL_TXRX_FLAGS_DEFAULT, i.e. CRC on, at :127).
//        - rfal_t2t.h:63-64 RFAL_T2T_BLOCK_LEN = 4 and RFAL_T2T_READ_DATA_LEN
//          = 4 * RFAL_T2T_BLOCK_LEN = 16 -- one READ returns four 4-byte
//          pages, which is why this loop advances 4 pages at a time.
//        - rfal_t2t.cpp:53 RFAL_FDT_POLL_READ_MAX = rfalConvMsTo1fc(5), "as
//          defined in TS T2T 1.0 table 18" -- the real 5 ms per-READ timeout,
//          converted below into this chip's own 64/fc NRT steps exactly the
//          way kRatsNrtSteps64fc already converts ISO-DEP's.
//        - rfal_t2t.cpp:55-57 RFAL_T2T_ACK_NACK_LEN = 1 ("Len of NACK in
//          bytes (4 bits)"), RFAL_T2T_ACK = 0x0A, RFAL_T2T_ACK_MASK = 0x0F,
//          used at :130 as "T2T 1.0 5.2.1.7 The Reader/Writer SHALL treat a
//          NACK in response to a READ Command as a Protocol Error". So a real
//          NAK is a single 4-BIT frame whose value is not 0x0A -- it is not a
//          whole byte, and it carries no CRC.
//
// [MI2] The vendored MFRC522_I2C library (lib/MFRC522_I2C/), already this
//      project's cited source for the RFID2 unit's PICC layer:
//        - MFRC522_I2C.h:308 `MF_ACK = 0xA` -- "The MIFARE Classic uses a 4
//          bit ACK/NAK. Any other value than 0xA is NAK." (the same real
//          encoding RFAL's ACK/ACK_MASK pair above describes, stated as a
//          rule rather than a constant).
//        - MFRC522_I2C.cpp:452,466-467 -- the real NAK DETECTION this
//          project already ships: read RxLastBits, then
//          `if (*backLen == 1 && _validBits == 4) return STATUS_MIFARE_NACK;`
//        - MFRC522_I2C.cpp PICC_GetType(): `case 0x00: return
//          PICC_TYPE_MIFARE_UL;` -- SAK 0x00 is the real Ultralight/NTAG21x
//          family marker nfc_read.cpp now dispatches on.
//
// [AM] features/nfc/nfc_amiibo.cpp -- this project's OWN already-reviewed,
//      already-real port of the donor read loop (Bruce's
//      ~/src/firmware/src/modules/rfid/RFID2.cpp), reused verbatim in
//      structure here rather than re-derived:
//        - the loop bound: RFID2.cpp:428 `for (byte page = 0; page <= 252;
//          page += 4)` -> nfc_amiibo.cpp's kDonorLastPageStart = 252.
//        - the terminator: RFID2.cpp:431-432, a NACK from MIFARE_Read is the
//          real, standard "read past end of memory" end-of-loop signal, not a
//          failure.
//        - the trim: RFID2.cpp:264-265 `dataPages = (readStatus == SUCCESS &&
//          dataPages > 0) ? dataPages - 1 : dataPages;` -- a real T2T tag's
//          READ near the end of memory returns the last valid page repeated
//          to fill its 16-byte answer rather than NAKing immediately, so the
//          final successfully-read group over-counts by exactly one page.
//          nfc_amiibo.cpp ports that empirical correction; so does this, on
//          the same two paths it does (tag-terminated and donor-bound-
//          exhausted) and NOT on its own buffer-capacity path.
//
// ONE HONEST IMPRECISION, disclosed rather than papered over: this driver
// cannot cleanly tell a real 4-bit NAK apart from a genuine framing/CRC error
// at the end of memory. transceive() is the single shared exchange primitive
// (deliberately -- it is the real-hardware-verified one), and it rejects both
// a partial last byte and a hardware CRC flag as Xfer::kProtocol before ever
// reading the FIFO; a 4-bit NAK carries no CRC and is exactly such a frame.
// Widening transceive() to surface RxLastBits would mean modifying the code
// path every verified reader feature (anticollision, SELECT, RATS, every EMV
// APDU) runs through, for a distinction that changes nothing about the
// result: the loop below treats ANY non-clean answer after at least one
// successful page group as end-of-memory, which is precisely what the donor
// loop's own `status != STATUS_OK` branch does with the NACK case as its
// expected flavour. The raw Xfer code is logged so a real anomaly is still
// visible.
// ===========================================================================

namespace {

// [REF-T2T] rfal_t2t.cpp:74, "T2T 1.0 5.1".
constexpr uint8_t kT2tCmdRead = 0x30U;
// [REF-T2T] rfal_t2t.h:63 RFAL_T2T_BLOCK_LEN.
constexpr uint8_t kT2tPageLen = 4U;
static_assert(kT2tPageLen == kListenPageLen,
              "the capture side and the Listen Mode responder must agree on "
              "the T2T page size -- both are RFAL_T2T_BLOCK_LEN");
// [REF-T2T] rfal_t2t.h:63-64.
constexpr uint8_t kT2tPagesPerRead = 4U;
constexpr uint8_t kT2tReadDataLen  = 16U; // 4 * RFAL_T2T_BLOCK_LEN
// [AM]/RFID2.cpp:428 -- the donor's own last page-group start.
constexpr int kT2tDonorLastPageStart = 252;
// [REF-T2T] rfal_t2t.cpp:53 RFAL_FDT_POLL_READ_MAX = rfalConvMsTo1fc(5).
// rfal_rf.h:96 RFAL_1MS_IN_1FC = 13560, so 5 ms = 67800 (1/fc); the NRT
// counts 64/fc steps ([DS] Table 51 nrt_step = 0), 67800 / 64 = 1059.
constexpr uint16_t kT2tReadNrtSteps64fc = 1059U;
// Room for the largest real answer (16 data bytes + the 2 CRC bytes this chip
// leaves in the FIFO) with margin.
constexpr uint8_t kT2tRxBufLen = 24U;

// Real bug found & fixed via real-hardware testing (2026-08-24): a real
// NTAG215 (which has 135 real pages, per this project's own
// nfc_amiibo.cpp citation of that exact figure from RFID2.cpp) capture
// stopped at page 44 with `Xfer::kProtocol` -- 44 is not a real NTAG215
// memory-boundary value at all, so this was very likely a genuine transient
// RF/CRC error mid-sequence, not the tag's real end of memory. This
// section's own "ONE HONEST IMPRECISION" note above already discloses that a
// real NAK and a genuine protocol error are indistinguishable at this
// shared transceive() call, but the fix for THAT ambiguity is not "trust the
// very first non-clean answer," it is retrying before concluding either.
// A real end-of-memory NAK
// is a deterministic property of the tag and will reproduce identically on
// immediate retry; a transient RF glitch over one of the 34 back-to-back
// exchanges a full 135-page NTAG215 needs is exactly the kind of rare-per-
// exchange, likely-somewhere-in-a-long-sequence error retrying is for. 2
// retries (3 total attempts per page group) is a real, bounded, small
// multiplier on this loop's own worst-case cost -- nowhere near the ~5s
// task-watchdog window this project has already been bitten by twice
// (hal/ir_unit.h's and hal/storage_sd.cpp's own header comments).
constexpr uint8_t kT2tReadMaxRetries = 2U;

} // namespace

void nfca_halt() {
    if (!s_nfca_ready) {
        return;
    }
    // Byte-for-byte the same SLP_REQ exchange nfca_detect()'s own step 3
    // performs, including deliberately discarding the outcome -- see that
    // call site for the ISO14443-3 6.4.3 / [EH] rfalNfcaPollerSleep()
    // citation on why silence IS the acknowledgement.
    uint8_t slp_rx[4] = {0};
    uint8_t slp_len = 0;
    (void)transceive(/*short_cmd=*/0U, kSlpReq, sizeof(kSlpReq), /*crc_tx=*/true,
                     /*antcl=*/false, /*crc_rx=*/true,
                     slp_rx, sizeof(slp_rx), &slp_len, millis() + 2U);
}

bool t2t_read_pages(uint8_t *out, size_t cap_bytes, uint8_t *pages_out) {
    if (pages_out != nullptr) {
        *pages_out = 0;
    }
    if (out == nullptr || cap_bytes < kT2tPageLen || !s_nfca_ready) {
        nfca_halt();
        return false;
    }

    size_t cap_pages = cap_bytes / kT2tPageLen;
    if (cap_pages > 255U) {
        cap_pages = 255U; // *pages_out is a uint8_t
    }

    size_t pages = 0;
    bool trim = false; // donor's -1 correction applies (see [AM] above)

    for (int start = 0; start <= kT2tDonorLastPageStart;
         start += kT2tPagesPerRead) {
        if (pages >= cap_pages) {
            // Buffer full. Deliberately NOT trimmed -- this is this driver's
            // own capacity limit, not the tag's end of memory, so the last
            // group read is genuinely valid data. Mirrors nfc_amiibo.cpp's
            // own kMaxPages safety-cap branch, which likewise skips the trim.
            break;
        }

        // [REF-T2T] rfal_t2t.cpp:80-84 + :123-124: the whole command is
        // {code, blNo}, sent with CRC (RFAL_TXRX_FLAGS_DEFAULT at :127).
        // Plain ISO14443-3 data exchange -- NOT wrapped in an ISO14443-4
        // I-block, so this deliberately uses the same low-level transceive()
        // nfca_detect() itself uses and not apdu_transceive().
        const uint8_t req[2] = {kT2tCmdRead, static_cast<uint8_t>(start)};
        uint8_t rx[kT2tRxBufLen] = {0};
        uint8_t rx_len = 0;
        Xfer x = Xfer::kProtocol;
        // Bounded retry (kT2tReadMaxRetries, see that constant's own
        // real-hardware-bug comment above) before concluding end-of-memory --
        // a fresh per-exchange deadline every attempt, for exactly the reason
        // nfca_detect()'s own `deadline` variable documents at length:
        // kDetectBudgetMs bounds ONE exchange, and reusing a single computed
        // budget across a long sequence of them (retries included) silently
        // turns it into a shrinking shared one.
        for (uint8_t attempt = 0; attempt <= kT2tReadMaxRetries; attempt++) {
            const uint32_t deadline = millis() + kDetectBudgetMs;
            x = transceive(/*short_cmd=*/0U, req, sizeof(req), /*crc_tx=*/true,
                           /*antcl=*/false, /*crc_rx=*/true,
                           rx, sizeof(rx), &rx_len, deadline,
                           kT2tReadNrtSteps64fc);
            if (x == Xfer::kOk && rx_len == kT2tReadDataLen) {
                break;
            }
            if (attempt < kT2tReadMaxRetries) {
                Serial.printf("quarky-tab5: [st25r3916] t2t_read_pages: page %d "
                              "attempt %u failed (xfer=%u len=%u) -- retrying\n",
                              start, (unsigned)(attempt + 1), (unsigned)x,
                              (unsigned)rx_len);
            }
        }

        if (x != Xfer::kOk || rx_len != kT2tReadDataLen) {
            // End of memory (the real NAK case, see this section's "ONE
            // HONEST IMPRECISION" note), or a genuine failure that survived
            // kT2tReadMaxRetries retries. Either way the tag has no more to
            // give.
            Serial.printf("quarky-tab5: [st25r3916] t2t_read_pages: stopped at "
                          "page %d after %u retries (xfer=%u 0=kOk,1=kNoResponse,"
                          "2=kCollision,3=kProtocol,4=kIo; len=%u) after %u pages\n",
                          start, (unsigned)kT2tReadMaxRetries, (unsigned)x,
                          (unsigned)rx_len, (unsigned)pages);
            trim = (pages > 0);
            break;
        }

        const size_t room = cap_pages - pages;
        const size_t take = (room < kT2tPagesPerRead) ? room : kT2tPagesPerRead;
        std::memcpy(out + (pages * kT2tPageLen), rx, take * kT2tPageLen);
        pages += take;

        if (start >= kT2tDonorLastPageStart) {
            // [AM]/RFID2.cpp:428's own loop bound reached without the tag ever
            // saying stop -- nfc_amiibo.cpp treats that as end-of-memory too,
            // trim included.
            trim = (pages > 0);
            break;
        }
    }

    // [AM]/RFID2.cpp:264-265's real post-read correction.
    if (trim && pages > 0) {
        pages--;
    }

    if (pages_out != nullptr) {
        *pages_out = static_cast<uint8_t>(pages);
    }

    // Restore the exact post-condition a plain nfca_detect() leaves behind --
    // see nfca_halt()'s own header comment for why skipping this would make
    // the very next scan report "no tag".
    nfca_halt();

    Serial.printf("quarky-tab5: [st25r3916] t2t_read_pages: captured %u pages "
                  "(%u bytes)\n", (unsigned)pages, (unsigned)(pages * kT2tPageLen));
    return pages > 0;
}

// ===========================================================================
// ISO14443-4 (T=CL) activation and single-APDU exchange (Phase 3 Task 13,
// EMV/APDU reader)
//
// SOURCES FOR THIS SECTION ONLY. Everything below -- the RATS command byte,
// the RATS PARAM's FSDI/DID packing, the ATS TL/T0/TB layout, the FWI->FWT
// formula, and the I-block PCB encoding/block-number toggle -- traces to ST's
// own RFAL ISO-DEP layer, read directly rather than recalled, matching this
// project's established discipline for every other protocol section in this
// file:
//   ~/src/wilson-elechouse/ST25R3916/NFC-RFAL/src/rfal_isoDep.cpp / .h
//   (the same vendored RFAL tree already cited above for rfal_nfca.cpp/
//   rfal_rfst25r3916.cpp; this is its ISO-DEP/T=CL half, not previously read
//   by this project before this task).
//     - rfal_isoDep.cpp:139  RFAL_ISODEP_CMD_RATS = 0xE0     "Digital 1.1 13.6.1"
//     - rfal_isoDep.cpp:181-183 RATS PARAM packing: FSDI in bits 7-4
//       (RFAL_ISODEP_RATS_PARAM_FSDI_MASK/SHIFT), DID (CID) in bits 3-0
//       (RFAL_ISODEP_RATS_PARAM_DID_MASK)
//     - rfal_isoDep.cpp:929-931 rfalIsoDepRATS(): the literal composition
//       `ratsReq.CMD = RFAL_ISODEP_CMD_RATS; ratsReq.PARAM = (FSDI<<4)|DID`
//     - rfal_isoDep.h:69 RFAL_ISODEP_NO_DID = 0x00 -- "DID value indicating
//       the ISO-DEP layer not to use DID [CID]". This driver, like every
//       single-tag-only reader in this file, always requests DID=0/NO_DID:
//       there is only ever one tag active (nfca_detect()'s own documented
//       single-tag scope), so no CID field is ever needed to disambiguate,
//       and rfal_isoDep.cpp:336/354 confirm RFAL itself only adds the PCB
//       DID bit to outgoing frames when its own `did != RFAL_ISODEP_NO_DID`.
//     - rfal_isoDep.cpp:141-146 ATS layout: RFAL_ISODEP_ATS_MIN_LEN=1 (a bare
//       TL byte, meaning "no optional fields, all defaults"),
//       RFAL_ISODEP_ATS_T0_FSCI_MASK=0x0F (card's own FSC, low nibble of T0)
//     - rfal_isoDep.h:135-137 T0 optional-field presence bits: TA=0x10,
//       TB=0x20, TC=0x40
//     - rfal_isoDep.cpp:936-938 rfalIsoDepRATS()'s own ATS validity check:
//       "Check for valid ATS length Digital 1.1 13.6.2.1 & 13.6.2.3" --
//       `(rcvLen < RFAL_ISODEP_ATS_MIN_LEN) || (rcvLen > RFAL_ISODEP_ATS_MAX_LEN)
//       || (ats->TL != rcvLen)` is an error. This driver applies the same
//       TL-equals-received-length check as its ATS validity gate.
//     - rfal_isoDep.cpp:1094-1096 FWI extraction from TB, when present:
//       `FWI = (TB >> RFAL_ISODEP_ATS_TB_FWI_SHIFT) & RFAL_ISODEP_ATS_FWI_MASK`
//       (shift=4, mask=0x0F); rfal_isoDep.h:74 RFAL_ISODEP_FWI_DEFAULT=4 when
//       TB is absent.
//     - rfal_isoDep.cpp:805-807 rfalIsoDepFWI2FWT(): "FWT = (256 x 16/fC) x
//       2^FWI => 2^(FWI+12)" -- this driver computes the same power-of-two in
//       fc-cycles and divides by fc=13.56 MHz for a millisecond timeout,
//       exactly as commented at fwiToFwtMs()'s definition below.
//     - rfal_isoDep.cpp:815-838 rfalIsoDepFSxI2FSx(): the ONE FSxI->FSx (max
//       frame size) table RFAL uses for BOTH directions -- the reader's own
//       FSDI->FSD and the card's FSCI->FSC are the same function with the same
//       table, called with a different integer. Its real values come from
//       rfal_isoDep.h:152-165 (rfalIsoDepFSxI enum, FSXI_16=0 ... FSXI_4096=12)
//       and rfal_isoDep.h:168-182 (rfalIsoDepFSx enum, FSX_16=16, FSX_24=24,
//       FSX_32=32, FSX_40=40, FSX_48=48, FSX_64=64, FSX_96=96, FSX_128=128,
//       FSX_256=256, then 512/1024/2048/4096 from ISO14443-3 Amd2 2012).
//       rfal_isoDep.cpp:821 additionally CLAMPS the incoming integer before the
//       switch -- `MIN(FSxI, RFAL_ISODEP_FSDI_MAX_NFC)` with
//       RFAL_ISODEP_FSDI_MAX_NFC = 8 (:178, "Digital 2.0 14.6.1.9 & B7 & B8")
//       in the default/NFC compliance mode -- so a card declaring FSCI 9..15
//       (the ISO14443-3-Amd2 / RFU range) is treated as FSCI=8/FSC=256, not as
//       an error. fsciToFsc() below is that exact table plus that exact clamp;
//       see its own comment for why the clamp is also what this driver's
//       255-byte transmit path can actually honour.
//       * TRANSMIT direction (this driver's own declared FSD): this driver
//         declares FSDI=7 in RATS (kRatsParam below), not the NFC-Forum-max
//         FSDI=8/FSD=256, because this file's own transceive()'s rx_cap
//         parameter is a uint8_t (max 255), so FSD=256 could never actually be
//         honoured if a compliant card ever sent a full-size frame -- FSD=128
//         is the largest declared value that cannot itself cause a real
//         overflow, and is generously larger than every real APDU response
//         this project's EMV feature module sends or expects to receive
//         (SELECT/GPO/READ RECORD responses; see nfc_emv_read.cpp).
//       * RECEIVE direction (the CARD's own declared FSC): rfal_isoDep.cpp:1074
//         seeds `isoDepDev->info.FSxI = RFAL_ISODEP_FSXI_32` with the literal
//         comment "FSC default value is 32 bytes ISO14443-A 5.2.3", and :1081-
//         1083 overwrites it from the real ATS -- `isoDepDev->info.FSxI =
//         (ATS.T0 & RFAL_ISODEP_ATS_T0_FSCI_MASK)` (mask 0x0F, rfal_isoDep.h:138)
//         -- but ONLY inside the `if (ATS.TL > RFAL_ISODEP_ATS_MIN_LEN)` guard
//         at :1080, i.e. only when a T0 byte is actually present. This driver's
//         iso14443_4_activate() applies exactly that: default 32, overwritten
//         from T0's low nibble when ats_len >= 2.
//     - rfal_isoDep.cpp:844-853 rfalIsoDepGetMaxInfLen(): the real per-I-block
//       INF budget, `gIsoDep.fsx - gIsoDep.hdrLen - ISODEP_CRC_LEN`, where
//       hdrLen is RFAL_ISODEP_PCB_LEN=1 for a no-DID/no-NAD session (:466,
//       :511) and ISODEP_CRC_LEN=RFAL_CRC_LEN=2 (:56). i.e. FSC counts the PCB
//       AND the two CRC bytes, so usable INF per frame is FSC-3. The same
//       arithmetic appears as an outright transmit reject at :362,
//       `if (txBufLen > (gIsoDep.fsx - ISODEP_CRC_LEN)) return ERR_NOTSUPP;`
//       where txBufLen already includes the PCB. This is the exact bound
//       apdu_transceive()'s new PCD->PICC fragmenter uses.
//     - rfal_isoDep.cpp:66-89 I-block PCB encoding: `ISODEP_PCB_IBLOCK=0x00`,
//       `ISODEP_PCB_B2_BIT=0x02` (a MUST-be-1 bit on every I-block per
//       ISO14443-4), `isoDep_PCBIBlock(bn) = IBLOCK | B2_BIT | (bn & 0x01)`
//       -- i.e. 0x02 for block number 0, 0x03 for block number 1. Block
//       number toggles every successful I-block round-trip
//       (isoDep_ToggleBN(), rfal_isoDep.cpp:269) and starts at 0 after RATS
//       (rfal_isoDep.cpp:459 gIsoDep.blockNumber = 0, in rfalIsoDepInitialize()).
//     - rfal_isoDep.cpp:102-117 S-block PCB encoding: `ISODEP_PCB_SBLOCK=0xC0`,
//       `ISODEP_PCB_WTX=0x30` (the S-block subtype bits), `ISODEP_PCBSBLOCK =
//       SBLOCK | B2_BIT`, `ISODEP_PCB_SWTX = ISODEP_PCBSBLOCK | WTX` = 0xF2.
//     - rfal_isoDep.cpp:415-421 the WTX ACK this driver's own handling below
//       is modelled on: the S(WTX) reply echoes the SAME PCB (0xF2) and the
//       SAME one-byte "power" INF field the card's own S(WTX) REQUEST carried
//       -- `ctrlMsgBuf[...] = param` where `param` is exactly the byte the
//       request supplied, not a value this driver computes or negotiates.
//
//     - rfal_isoDep.cpp:678-705 the PICC-chaining receive branch this
//       driver's own chaining support (added 2026-08-23) is ported from:
//       `isoDep_PCBisChaining(rxPCB)` (:229, chaining bit 0x10 per :86) ->
//       block-number match check (:682) -> `isoDep_ToggleBN` (:684) -> "Rule
//       2 - Send ACK" via `isoDepHandleControlMsg(ISODEP_R_ACK, ...)` (:689),
//       whose PCB is `isoDep_PCBRACK(gIsoDep.blockNumber)` (:389) = `0x00 |
//       ISODEP_PCB_RBLOCK(0x80) | ISODEP_PCB_B6_BIT(0x20) |
//       ISODEP_PCB_B2_BIT(0x02) | bn` (:253-254, with ISODEP_PCB_ACK=0x00 at
//       :107) -- i.e. 0xA2|bn, built from the ALREADY-TOGGLED block number.
//       R-blocks carry no INF (:95 ISODEP_RBLOCK_INF_LEN = 0).
//
//     - PCD->PICC (reader-to-card) I-block chaining, added 2026-08-24. Two
//       real RFAL layers, ported together because this driver has no state
//       machine to split them across:
//       * rfal_isoDep.cpp:1446-1470 rfalIsoDepApdu2IBLockParam(): the
//         FRAGMENTER. `if ((apduParam.txBufLen - txPos) > rfalIsoDepGetMaxInfLen())
//         { isTxChaining = true; txBufLen = rfalIsoDepGetMaxInfLen(); } else
//         { isTxChaining = false; txBufLen = (apduParam.txBufLen - txPos); }`
//         -- i.e. every fragment but the last is exactly one full max-INF
//         block, the last carries the remainder, and the chaining FLAG is
//         simply "is there more after this one". :1505-1519
//         rfalIsoDepGetApduTransceiveStatus() is the loop around it: on each
//         completed block, `gIsoDep.APDUTxPos += gIsoDep.txBufLen` then
//         re-derive the next fragment and transmit again.
//       * rfal_isoDep.cpp:342-344 isoDepTx(): the chaining BIT itself --
//         `if ((gIsoDep.isTxChaining) && (isoDep_PCBisIBlock(computedPcb)))
//         { computedPcb |= ISODEP_PCB_CHAINING_BIT; }`. Same 0x10 bit (:86)
//         the PICC uses in the other direction; :251
//         isoDep_PCBIBlockChaining(bn) is the composed form.
//       * rfal_isoDep.cpp:651-664 the R(ACK) HANDSHAKE the card answers each
//         non-final fragment with, inside rfalIsoDepGetTransceiveStatus()'s
//         R-block branch: `if (isoDep_PCBisRACK(rxPCB))` -> `if (isoDep_GetBN(rxPCB)
//         == gIsoDep.blockNumber)` (:652 -- the card echoes the SAME block
//         number the fragment carried, NOT a toggled one) -> "Rule B - ACK
//         with expected bn -> Increment block number" `gIsoDep.blockNumber =
//         isoDep_PCBNextBN(...)` (:654) -> "R-ACK only allowed when PCD
//         chaining" `if (!gIsoDep.isTxChaining) return ERR_PROTO;` (:656-658)
//         -> "Rule 7 - Chaining transaction done, continue chaining"
//         (:661-663). This is the mirror image of the PICC->PCD case above,
//         and the toggle ORDERING is deliberately the other way round: for a
//         PICC-chained I-block this driver toggles BEFORE building its own
//         R(ACK); for a PCD-chained fragment it validates the card's R(ACK)
//         against the UN-toggled number first and toggles only after.
//       * rfal_isoDep.cpp:226-227 / :73-77 / :236 the R(ACK) VALIDITY test
//         applied to that answer: `isoDep_PCBisRBlock(pcb)` is
//         `(pcb & (0xC0|0x20|0x04|0x02)) == (0x80|0x20|0x02)` (xBLOCK_MASK |
//         RB_VALID_MASK vs RBLOCK | RB_VALID_VAL -- B6 and B2 must be 1, B3
//         must be 0), and `isoDep_PCBisACK(pcb)` is
//         `(pcb & ISODEP_PCB_Rx_MASK(0x10)) == ISODEP_PCB_ACK(0x00)`.
//       NOT ported from this path: RFAL's ":659-666 Rule 6 - R-ACK with wrong
//       block number retransmit" (it re-enters ISODEP_ST_PCD_TX to resend the
//       same fragment, up to maxRetriesI). This driver has no retransmission
//       in either direction (see the R-block scope bullet below); a wrong-bn
//       R(ACK) is reported as a failure instead, consistently with how a
//       wrong-bn I-block is already treated on the receive side.
//
// SCOPE, stated honestly, same policy as nfca_detect()'s own header comment:
//   * PICC->PCD I-block chaining IS handled, bounded in rounds
//     (kMaxChainingRounds), in reassembled size (kMaxReassembledLen) and in
//     wall clock (kMaxApduCallMs, shared with the S(WTX) path rather than
//     added to it). This was NOT true before 2026-08-23: real-hardware
//     testing against a real Mastercard-style card produced a READ RECORD
//     response with PCB 0x12 -- an I-block with the chaining bit set,
//     because its record does not fit the FSD=128 this driver declares --
//     which the previous code silently accepted as if it were complete
//     (the truncated TLV was then correctly refused by nfc_emv_read.cpp's
//     own BER-TLV length-overflow guard, which is how the bug surfaced).
//   * PCD->PICC I-block chaining IS handled too, as of 2026-08-24. It was
//     NOT before, and the comment that used to sit here ("every C-APDU this
//     project's EMV module sends is under 32 bytes and fits one frame by
//     construction") stopped being true the moment the PDOL-based GET
//     PROCESSING OPTIONS path landed the day before: a real Visa card's own
//     PDOL was 27 bytes, producing a 62-byte I-block frame, and real PDOLs
//     can legitimately be larger. Worse, the old code did not even reject
//     such a frame -- it happily transmitted it in ONE I-block regardless of
//     the card's own declared FSC, and a real Visa card and a real older
//     Mastercard both answered with nothing at all (Xfer::kNoResponse, a
//     genuine NRT timeout, reproduced three times in a row) -- which is what
//     a compliant card does with a frame that violates the frame size it
//     declared in its ATS. Hedged honestly: those two cards' actual FSCI
//     values were never observed, BECAUSE the code discarded that nibble;
//     what is certain is that the frame was sent without any size check at
//     all. The root cause was that the card's FSC was never read:
//     iso14443_4_activate() parsed T0 only for the TA/TB presence bits and
//     discarded the FSCI nibble that carries it. Both halves are fixed
//     together -- fsciToFsc() + s_card_fsc capture the real declared size,
//     and apdu_transceive() fragments anything larger across chained
//     I-blocks, consuming the card's R(ACK) between fragments. Bounded in
//     fragment count (kMaxTxChainFragments) and sharing -- not extending --
//     the same kMaxApduCallMs envelope the WTX and PICC-chaining paths
//     already use.
//   * No R-block RECEIVE handling, and no retransmission. This driver never
//     resends an I-block after a NAK; a NAK-shaped or unrecognised PCB in the
//     response is treated as failure, and RFAL's own "Rule 5 - PICC chaining
//     invalid I-Block -> R-ACK" recovery (rfal_isoDep.cpp:702-703) is
//     deliberately not ported either, since it exists to request a
//     retransmission this driver has no path to consume. Real EMV read-only
//     exchanges over a short-range link essentially never need this on the PCD
//     side in practice; disclosed rather than silently assumed to be unneeded.
//     (The one R-block this driver does SEND is the R(ACK) that drives PICC
//     chaining forward, above -- that is a transmit-only use of the R-block
//     encoding, not the receive/retransmit state machine described here.)
//   * S(WTX) (waiting-time extension) IS handled, bounded to kMaxWtxRounds
//     rounds per APDU -- see apdu_transceive()'s own comment for why (a real
//     card is reasonably likely to ask for one during GET PROCESSING OPTIONS,
//     which can involve on-card cryptographic computation).
//   * No S(DESELECT) is ever sent. A polite ISO14443-4 session end is an
//     S-block DESELECT/response handshake (rfal_isoDep.cpp's ISODEP_PCB_SDSL);
//     this driver instead always ends a session via nfca_poller_end()'s
//     field_off(), which de-powers the tag outright -- a real, if less polite,
//     way to end any contactless session, and the one nfc_read.cpp's own
//     teardown() already uses for the plain UID-read path. A card left
//     mid-session when the field cuts is not left in any worse state than
//     simply being pulled out of range, which every contactless reader must
//     already tolerate.
// ===========================================================================

namespace {

constexpr uint8_t kCmdRats = 0xE0U; // [REF] rfal_isoDep.cpp:139
// FSDI=7 (FSD=128, see the SOURCES note above for why not 8/256), DID=0
// (RFAL_ISODEP_NO_DID -- no CID, single-tag reader).
constexpr uint8_t kRatsFsdi  = 7U;
constexpr uint8_t kRatsParam = static_cast<uint8_t>(kRatsFsdi << 4);

constexpr uint8_t kAtsMinLen = 1U; // [REF] RFAL_ISODEP_ATS_MIN_LEN
constexpr uint8_t kAtsMaxLen = 32U; // generous local cap; real ATS historical
                                    // bytes are rarely long and this driver
                                    // does not need to parse them anyway
constexpr uint8_t kAtsT0TaPresent = 0x10U; // [REF] rfal_isoDep.h:135
constexpr uint8_t kAtsT0TbPresent = 0x20U; // [REF] rfal_isoDep.h:136
constexpr uint8_t kFwiDefault = 4U; // [REF] RFAL_ISODEP_FWI_DEFAULT

// --- The CARD's own maximum receivable frame size (FSC) --------------------
// [REF] rfal_isoDep.h:138 RFAL_ISODEP_ATS_T0_FSCI_MASK = 0x0F -- the low
// nibble of the ATS's T0 byte is FSCI, the card's own declared max frame size
// index. This project's SOURCES block above cited this exact field from day
// one but never extracted it; not doing so is the real root cause of the
// 2026-08-24 "62-byte GPO gets no answer at all" bug (see the scope bullet).
constexpr uint8_t kAtsT0FsciMask = 0x0FU;
// [REF] rfal_isoDep.cpp:1074 -- "FSC default value is 32 bytes ISO14443-A
// 5.2.3", used when the ATS has no T0 byte at all (TL == 1).
constexpr uint16_t kCardFscDefault = 32U;
// [REF] rfal_isoDep.cpp:178 RFAL_ISODEP_FSDI_MAX_NFC = 8, applied at :821 as
// `MIN(FSxI, RFAL_ISODEP_FSDI_MAX_NFC)` BEFORE the FSxI->FSx switch in the
// default (non-EMVCo) compliance mode. So FSCI 9..15 -- the ISO14443-3 Amd2
// 512/1024/2048/4096 range plus the genuinely-RFU codes 13..15 -- collapse to
// FSCI=8/FSC=256 rather than being an error. This driver keeps that behavior
// AND independently benefits from it: 256 is the largest FSC it could honour
// anyway, since transceive()'s own tx_len parameter is a uint8_t and this
// function's frame buffer is 255 bytes -- an FSC=256 frame is at most 256-2
// (CRC) = 254 transmitted bytes, which fits exactly; anything larger could
// not be transmitted at all.
constexpr uint8_t kFsciMaxNfc = 8U;
constexpr uint16_t kCardFscMax = 256U;

// [REF] rfal_isoDep.cpp:815-838 rfalIsoDepFSxI2FSx() -- the same real table
// RFAL uses for the reader's FSDI->FSD and the card's FSCI->FSC alike, with
// its values from rfal_isoDep.h:152-165 (FSXI enum) and :168-182 (FSX enum).
// Only the 0..8 rows are spelled out here because :821's clamp above means
// nothing above 8 can ever reach the switch.
uint16_t fsciToFsc(uint8_t fsci) {
    const uint8_t fsi = (fsci > kFsciMaxNfc) ? kFsciMaxNfc : fsci;
    switch (fsi) {
        case 0U: return 16U;   // [REF] RFAL_ISODEP_FSXI_16   -> FSX_16
        case 1U: return 24U;   // [REF] RFAL_ISODEP_FSXI_24   -> FSX_24
        case 2U: return 32U;   // [REF] RFAL_ISODEP_FSXI_32   -> FSX_32
        case 3U: return 40U;   // [REF] RFAL_ISODEP_FSXI_40   -> FSX_40
        case 4U: return 48U;   // [REF] RFAL_ISODEP_FSXI_48   -> FSX_48
        case 5U: return 64U;   // [REF] RFAL_ISODEP_FSXI_64   -> FSX_64
        case 6U: return 96U;   // [REF] RFAL_ISODEP_FSXI_96   -> FSX_96
        case 7U: return 128U;  // [REF] RFAL_ISODEP_FSXI_128  -> FSX_128
        default: return kCardFscMax; // fsi == 8, [REF] FSXI_256 -> FSX_256
    }
}

// [REF] rfal_isoDep.cpp:844-853 rfalIsoDepGetMaxInfLen(): usable INF per
// I-block is `fsx - hdrLen - ISODEP_CRC_LEN`, with hdrLen = RFAL_ISODEP_PCB_LEN
// = 1 for this driver's no-DID/no-NAD session (:466, :511) and CRC_LEN = 2
// (:56). i.e. FSC counts the PCB byte AND the two CRC bytes the chip appends
// in hardware, so only FSC-3 bytes of C-APDU fit in one frame. The smallest
// legal FSC (16) therefore still leaves 13 usable bytes, so this never
// underflows.
constexpr uint16_t kIBlockOverhead = 1U /*PCB*/ + 2U /*CRC*/;

// A SECOND, non-protocol ceiling on one transmitted frame, found while adding
// PCD->PICC chaining (2026-08-24) and real for this build specifically:
// writeFifoRaw() above pushes the whole frame in ONE Wire1 transaction, and
// Arduino-ESP32's TwoWire::write() refuses a byte once its buffer is full --
// `if (txLength >= bufferSize) { return 0; }`
// (~/.platformio/packages/framework-arduinoespressif32/libraries/Wire/src/
// Wire.cpp:558-560), with bufferSize defaulting to I2C_BUFFER_LENGTH = 128
// (Wire.h:48-49; nothing in this firmware calls Wire1.setBufferSize()). One of
// those 128 bytes is the FIFO-load command byte writeFifoRaw() sends first, so
// at most 127 frame bytes (PCB + INF) can be queued -- 126 bytes of INF.
// Without this cap the new fragmenter would happily build a 254-byte fragment
// for a card that declares FSC=256 and writeFifoRaw() would fail the frame
// outright (Xfer::kIo), which is a worse outcome than simply chaining one
// extra time. Applied as MIN(FSC-3, 126); note it never binds for FSC <= 128,
// i.e. for every card whose ATS this project has actually seen.
constexpr size_t kMaxTxInfPerI2cFrame = 126U;

constexpr uint8_t kPcbTypeMask   = 0xC0U;
constexpr uint8_t kPcbIBlockType = 0x00U;
constexpr uint8_t kPcbSBlockType = 0xC0U;
constexpr uint8_t kPcbSTypeMask  = 0x30U;
constexpr uint8_t kPcbWtxType    = 0x30U;
constexpr uint8_t kPcbB2Bit      = 0x02U; // MUST be 1 on every I/S-block
constexpr uint8_t kPcbB6Bit      = 0x20U; // MUST be 0 on a valid I-block

// --- PICC->PCD I-block chaining (added 2026-08-23, real-hardware driven) ---
// [REF] rfal_isoDep.cpp:86  ISODEP_PCB_CHAINING_BIT = 0x10U -- "Bit mask for
// the chaining bit of an ISO DEP I-Block in PCB". A received I-block with
// this bit SET is a non-final fragment: the card has more of this R-APDU to
// send and is waiting for an R(ACK) before sending it.
constexpr uint8_t kPcbChainingBit = 0x10U;
// [REF] rfal_isoDep.cpp:68  ISODEP_PCB_RBLOCK = 0x80U, :85 ISODEP_PCB_B6_BIT
// = 0x20U, :83 ISODEP_PCB_B2_BIT = 0x02U, :107 ISODEP_PCB_ACK = 0x00U, and
// the two macros that compose them:
//   :253 isoDep_PCBRBlock(bn) = 0x00 | RBLOCK | B6_BIT | B2_BIT | (bn & 0x01)
//   :254 isoDep_PCBRACK(bn)   = isoDep_PCBRBlock(bn) | ISODEP_PCB_ACK
// i.e. R(ACK) = 0xA2 | bn -- 0xA2 for block number 0, 0xA3 for 1. The ACK
// subtype is the ZERO value of the 0x10 R-block type bit ([REF] :106-108,
// ISODEP_PCB_Rx_MASK=0x10, ACK=0x00, NAK=0x10), so nothing is OR'd in for it.
constexpr uint8_t kPcbRBlockAckBase = 0xA2U;

// --- PCD->PICC I-block chaining (added 2026-08-24, real-hardware driven) ---
// Validity test for an R(ACK) RECEIVED from the card between transmit
// fragments. [REF] rfal_isoDep.cpp:227 isoDep_PCBisRBlock(pcb) is
// `(pcb & (ISODEP_PCB_xBLOCK_MASK|ISODEP_PCB_RB_VALID_MASK)) ==
//  (ISODEP_PCB_RBLOCK|ISODEP_PCB_RB_VALID_VAL)`, which with :66-77's values
// (xBLOCK_MASK=0xC0, RB_VALID_MASK=B6|B3|B2=0x26, RBLOCK=0x80,
// RB_VALID_VAL=B6|B2=0x22) is exactly `(pcb & 0xE6) == 0xA2`; and :236
// isoDep_PCBisACK(pcb) is `(pcb & ISODEP_PCB_Rx_MASK) == ISODEP_PCB_ACK`
// with Rx_MASK=0x10 (:106) and ACK=0x00 (:107). Note 0x10 is numerically the
// same bit as kPcbChainingBit above but a different field -- chaining on an
// I-block, ACK/NAK on an R-block -- so it gets its own name rather than
// reusing that one.
constexpr uint8_t kPcbRBlockValidMask = 0xE6U;
constexpr uint8_t kPcbRBlockValidVal  = 0xA2U;
constexpr uint8_t kPcbRTypeMask       = 0x10U; // 0 = ACK, 0x10 = NAK
constexpr uint8_t kPcbRAck             = 0x00U;

// Bound on how many I-block fragments ONE outgoing C-APDU may be split into.
// Derived, not picked: this function already refuses any tx_len above 254
// (transceive()'s tx_len parameter is a uint8_t and one PCB byte precedes the
// APDU), and the smallest FSC any ISO14443-4 card may declare is 16, leaving
// 16-3 = 13 usable INF bytes per frame -- so ceil(254/13) = 20 is the largest
// fragment count the driver's OWN existing bounds can ever produce. Setting
// the cap there means it never refuses a command the tx_len bound already
// admitted (a lower, arbitrary-looking number would), while still being a
// hard, finite loop bound. The real wall-clock protection is the shared
// call_deadline below, which is checked before every additional fragment
// exactly as it already is before every WTX ack and every receive-chaining
// R(ACK) -- so 20 fragments cannot cost 20 full per-exchange timeouts. Real
// traffic is nowhere near this: the largest command nfc_emv_read.cpp can
// build is a 128-byte PDOL GPO (kMaxPdolDataLen=120), which is 5 fragments
// against a typical FSC=32 card and 1 against an FSC=128+ one.
constexpr uint8_t kMaxTxChainFragments = 20U;

// Bound 1 of 2 on chaining reassembly: how many R(ACK)/fragment rounds this
// driver will run for ONE apdu_transceive() call. 8 is chosen against the
// real frame size this driver negotiates: RATS declares FSDI=7/FSD=128
// (kRatsParam above), and real cards answer with an FSC of at least 32-64
// bytes, so 8 fragments carry 256-1000+ bytes of INF -- far more than any
// real EMV READ RECORD response (see kMaxReassembledLen below). A card that
// still has not finished after 8 fragments is either malfunctioning or
// sending something this read-only reader has no use for; failing there is
// preferable to an unbounded loop, which is exactly the class of bug this
// project has already been bitten by twice (hal/ir_unit.h's and
// hal/storage_sd.cpp's own header comments on real watchdog-timeout crashes).
constexpr uint8_t kMaxChainingRounds = 8U;

// Bound 2 of 2: the largest reassembled R-APDU this driver will hand back,
// independent of how few rounds it took. Real EMV READ RECORD responses --
// the only responses observed to chain on real hardware, because a single
// record can carry Track 2 equivalent data, cardholder name, CVM list, and
// a 128/176/248-byte issuer public-key certificate at once -- top out around
// 250-300 bytes; 512 leaves generous headroom over that without letting a
// hostile or broken card drive this driver's memory use up. apdu_transceive()
// additionally refuses to write past the CALLER's own rx_cap, so this is the
// ceiling on what a caller may usefully ask for, not a hidden buffer.
constexpr size_t kMaxReassembledLen = 512U;

// Per-exchange wall-clock cap. Computed from the card's own declared FWI
// (via fwiToFwtMs() below) but hard-clamped here to keep a single
// apdu_transceive() call -- and, more importantly, the handful of them a full
// EMV read performs back to back -- comfortably inside the ~5 s ESP32 task
// watchdog window this project has already been bitten by twice (see
// hal/ir_unit.h's and hal/storage_sd.cpp's own header comments). FWI's legal
// range extends to a ~4.95 s FWT (FWI=14); honouring that literally for
// several exchanges in a row inside one synchronous read (see
// nfc_emv_read.cpp) could alone exceed the watchdog budget even though every
// individual wait was protocol-legal. Real EMV SELECT/GPO/READ RECORD
// exchanges this driver sends need nowhere near this: typical real-card FWI
// for these commands is small (low-to-mid single digits), and this cap only
// ever bites a pathologically slow or malfunctioning card, which is exactly
// when failing fast is preferable to risking a watchdog panic.
constexpr uint32_t kMaxSingleExchangeMs = 500U;

// A card requesting more time than it was just granted (another S(WTX) right
// after this driver's ack) is retried up to this many times before this
// driver gives up on the whole APDU. Raised 2->5 (2026-08-24), real-hardware-
// driven: a real physical Visa card's PDOL-based GPO asked for 3 consecutive
// WTX extensions (confirmed via this function's own DIAG logging: three
// "F2 01" S(WTX) responses in a row) before it was ready to answer -- with
// the old cap of 2, this driver gave up right as the card was still working,
// on the exact same exchange the kMinFwtMs floor above had already fixed the
// OTHER half of (the card getting no chance to even ASK for more time). 5
// leaves real margin over the 3 actually observed. Bounds worst-case time
// for ONE apdu_transceive() call to roughly (kMaxWtxRounds + 1) *
// kMaxSingleExchangeMs even against a card that does nothing but ask for
// extensions.
constexpr uint8_t kMaxWtxRounds = 5U;

// Whole-call wall-clock envelope for ONE apdu_transceive(), shared by the
// S(WTX) rounds and the PICC->PCD chaining rounds rather than being added to
// them. This is the theoretical worst case ((1 + kMaxWtxRounds) *
// kMaxSingleExchangeMs = 3000 ms as of the kMaxWtxRounds raise above) for a
// card that is BOTH high-FWI (needing the full 500ms per exchange) AND needs
// every one of the 5 WTX rounds -- no real card seen on this hardware
// combines both (every one so far is FWI=7, whose 100ms-floored per-exchange
// cost makes 5 real rounds cost roughly 500ms total, not 3000ms). 3000ms
// alone already exceeds nfc_emv_read.cpp's own 2.5s kOverallReadBudgetMs
// "aspirational" ceiling for a single step in that genuinely pathological
// case -- disclosed honestly rather than hidden -- but stays safely under
// the ~5s ESP32 task-watchdog hard limit that actually matters for crash
// prevention (hal/ir_unit.h's and hal/storage_sd.cpp's own header comments
// record the two real crashes that discipline comes from), with real margin
// (3000ms vs 5000ms) even if a future card manages to hit that worst case.
// Checked before every additional exchange (WTX ack or chaining R(ACK)), the
// same way nfc_emv_read.cpp's apdu_step() checks its own overall budget
// before every top-level APDU.
constexpr uint32_t kMaxApduCallMs = (1U + kMaxWtxRounds) * kMaxSingleExchangeMs;

// Practical floor under the raw FWT calculation below, real-hardware-driven
// (2026-08-24), NOT spec-derived -- disclosed honestly as empirical rather
// than cited, unlike the rest of this file. A real physical Visa card
// (FWI=7, whose raw FWT computes to 39ms) gave a genuine, reproducible
// (3x in a row, fresh activation each time) zero-byte hardware timeout on
// GET PROCESSING OPTIONS specifically -- not SELECT PPSE, not SELECT AID,
// both of which this exact card answered promptly moments earlier -- and
// not because the command was too large for its declared FSC (FSC=256,
// comfortably larger than the 61-byte command; PCD->PICC chaining correctly
// determined no fragmentation was needed and still saw no response at all,
// not even a WTX request). GPO is the one EMV command that can involve real
// internal application processing before a card answers, which is exactly
// why S(WTX) handling exists at all here -- but a card cannot ask for more
// time if the reader's own hardware no-response timer fires before the card
// manages to get even a WTX byte onto the RF link. The raw ISO14443-4 FWT
// formula is a real, spec-derived MINIMUM a compliant card must be given,
// not a promise that ordinary consumer cards need no more than that in
// practice for their slower commands -- kMinFwtMs is a pragmatic margin
// against exactly that gap, still tiny next to kMaxSingleExchangeMs's own
// 500ms safety clamp and nfc_emv_read.cpp's 2.5s overall read budget.
constexpr uint32_t kMinFwtMs = 100U;

// [REF] rfal_isoDep.cpp:805-807: "FWT = (256 x 16/fC) x 2^FWI => 2^(FWI+12)",
// fc = 13.56 MHz. Returns milliseconds, rounded up by one, floored at
// kMinFwtMs (see its own comment), then clamped to kMaxSingleExchangeMs --
// see that constant's own comment for why the clamp exists and why it is
// safe for the read-only EMV command set this drives.
uint32_t fwiToFwtMs(uint8_t fwi) {
    // 14 = [REF]'s own ISODEP_FWI_MAX. An out-of-range value in a real ATS
    // would itself be non-compliant; fall back to the default rather than
    // shift by an unbounded amount.
    const uint8_t clamped_fwi = (fwi > 14U) ? kFwiDefault : fwi;
    const uint64_t fc_cycles = static_cast<uint64_t>(1U) << (clamped_fwi + 12U);
    uint32_t ms = static_cast<uint32_t>((fc_cycles / 13560U) + 1U); // fc in kHz
    if (ms < kMinFwtMs) {
        ms = kMinFwtMs;
    }
    if (ms > kMaxSingleExchangeMs) {
        ms = kMaxSingleExchangeMs;
    }
    return ms;
}

// Converts an already-clamped ms timeout (kMaxSingleExchangeMs's own comment
// explains the clamp) into 64/fc steps for the CHIP's own hardware NRT
// register -- real bug found & fixed via real-hardware testing (2026-08-23):
// apdu_transceive()'s own transceive() calls never passed an nrt_steps
// argument at all, silently defaulting to kNrtSteps64fc (~165us, correct
// only for anticollision replies) regardless of this FWI-derived timeout
// already being computed and stored in s_apdu_timeout_ms -- the exact same
// class of bug already found and fixed for RATS's own NRT (see
// kRatsNrtSteps64fc's declaration comment). Clamped to 0xFFFFU (the NRT
// register's own real max in 64/fc mode, [REF]
// ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/st25r3916.cpp:49
// ST25R3916_NRT_MAX): this driver only ever configures 64/fc mode (never the
// alternate 4096/fc mode RFAL's own st25r3916SetNoResponseTime() falls back
// to for larger values), so a pathologically high FWI could in principle
// need more than the ~309ms this mode can represent -- in that rare case the
// hardware NRT clamps below the still-independent software `deadline_ms`,
// which only makes a bad exchange fail slightly sooner, never later or
// incorrectly.
uint16_t msToNrtSteps64fc(uint32_t ms) {
    const uint64_t steps = (static_cast<uint64_t>(ms) * 13560U) / 64U;
    return steps > 0xFFFFU ? 0xFFFFU : static_cast<uint16_t>(steps);
}

uint8_t s_pcb_block_number = 0U;
// The CARD's own declared max receivable frame size, from its ATS T0's FSCI
// nibble via fsciToFsc(). Already clamped to [16, kCardFscMax] by that
// function's own table + :821 clamp, so apdu_transceive() can subtract
// kIBlockOverhead from it without underflow and the result always fits its
// 255-byte frame buffer (a separate, smaller per-transaction I2C ceiling also
// applies -- see kMaxTxInfPerI2cFrame). Reset to the ISO14443-A 5.2.3 default
// (32) at the start of every activation so a previous card's larger FSC can
// never be applied to a new one.
uint16_t s_card_fsc = kCardFscDefault;
uint32_t s_apdu_timeout_ms = 0U;
uint16_t s_apdu_nrt_steps = 0U; // set alongside s_apdu_timeout_ms, both
                                // derived from the same FWI value

// Last ATS received by iso14443_4_activate(), verbatim and CRC-stripped
// (TL byte first, exactly as it came off the wire). Retained so a feature
// module can export the card's real ISO14443-4 interface bytes -- Phase 3
// Task 13's Flipper ".nfc" exporter writes T0/TA(1)/TB(1)/TC(1)/T1...Tk
// straight out of this, and there is nowhere else in this driver those bytes
// survive (activate() itself keeps only T0's FSCI nibble and TB's FWI nibble,
// as s_card_fsc and s_apdu_timeout_ms, and discards the rest).
// Cleared at the start of every activation so a stale ATS from a previous
// card can never be exported against a new one.
uint8_t s_ats[kAtsMaxLen] = {0};
uint8_t s_ats_len = 0U;

} // namespace

bool iso14443_4_activate() {
    if (!s_nfca_ready) {
        return false;
    }
    s_pcb_block_number = 0U;
    s_ats_len = 0U; // never export a previous card's ATS -- see s_ats above
    s_card_fsc = kCardFscDefault; // refined below from the real ATS's T0/FSCI
    s_apdu_timeout_ms = fwiToFwtMs(kFwiDefault); // refined below once the
                                                 // real ATS is read, if it
                                                 // says otherwise
    s_apdu_nrt_steps = msToNrtSteps64fc(s_apdu_timeout_ms);

    const uint8_t rats_req[2] = {kCmdRats, kRatsParam};
    uint8_t ats[kAtsMaxLen] = {0};
    uint8_t ats_len = 0;
    // Activation gets its own generous-but-bounded deadline, separate from
    // kMaxSingleExchangeMs: RATS is a one-time cost per tag, not repeated per
    // APDU, so there is no "several of these in a row" risk to guard against
    // the way there is for apdu_transceive()'s steady-state calls. This 100ms
    // SOFTWARE deadline was never the problem -- see kRatsNrtSteps64fc's own
    // declaration comment: the chip's own HARDWARE no-response timer was
    // firing at ~165us, long before this software deadline could matter.
    const uint32_t deadline = millis() + 100U;
    const Xfer xr = transceive(/*short_cmd=*/0U, rats_req, sizeof(rats_req),
                               /*crc_tx=*/true, /*antcl=*/false, /*crc_rx=*/true,
                               ats, sizeof(ats), &ats_len, deadline,
                               kRatsNrtSteps64fc);
    if (xr != Xfer::kOk) {
        Serial.printf("quarky-tab5: [st25r3916] RATS (0xE0) failed, xfer=%u\n",
                      static_cast<unsigned>(xr));
        return false;
    }
    // [REF] rfal_isoDep.cpp:936-938's own ATS validity gate: TL must equal
    // the actual received length, and be at least the 1-byte minimum. `ats`
    // here is already CRC-stripped by transceive() itself -- real bug found
    // via real-hardware testing (2026-08-23), see that function's own
    // content_len comment for the full story: this chip keeps 2
    // already-verified CRC bytes in the FIFO on every crc_rx=true receive,
    // confirmed on three different real ATS responses (each exactly TL+2
    // bytes long before that fix) and fixed once, centrally, rather than
    // patched separately at every call site including this one.
    if (ats_len < kAtsMinLen || ats[0] != ats_len) {
        Serial.printf("quarky-tab5: [st25r3916] ATS malformed: len=%u TL=0x%02X\n",
                      (unsigned)ats_len, ats_len > 0 ? ats[0] : 0);
        return false;
    }

    // Retain the whole validated ATS for iso14443_4_get_ats() -- see s_ats's
    // own declaration comment. Done AFTER the TL/length validity gate above
    // so only a well-formed ATS is ever exported.
    memcpy(s_ats, ats, ats_len);
    s_ats_len = ats_len;

    uint8_t fwi = kFwiDefault;
    uint8_t fsci = 0U;
    bool has_t0 = false;
    if (ats_len >= 2U) {
        const uint8_t t0 = ats[1];
        // [REF] rfal_isoDep.cpp:1080-1083: FSCI is read from T0's low nibble,
        // and ONLY when a T0 is actually present (RFAL guards the whole
        // optional-field block on `ATS.TL > RFAL_ISODEP_ATS_MIN_LEN`, which is
        // exactly this `ats_len >= 2` -- TL counts itself). Without a T0 the
        // ISO14443-A 5.2.3 default of 32 set above stands.
        //
        // This is the field the 2026-08-24 no-response bug hinged on. Stated
        // honestly: the failing Visa card's actual FSCI has never been read
        // back -- the pre-fix code discarded this nibble, which is precisely
        // why nobody could see it, and the log line at the end of this
        // function is what will finally show it. What IS certain is that the
        // code sent a 62-byte single-frame GPO without ever comparing it to
        // any declared limit, and that a frame exceeding the card's declared
        // FSC being silently dropped (no answer at all, exactly the observed
        // Xfer::kNoResponse) is the standard-compliant behavior for a card in
        // that situation. Any FSCI of 4 or below (FSC <= 48) makes that the
        // literal explanation.
        fsci = static_cast<uint8_t>(t0 & kAtsT0FsciMask);
        has_t0 = true;
        s_card_fsc = fsciToFsc(fsci);
        size_t idx = 2U;
        if ((t0 & kAtsT0TaPresent) != 0U) {
            idx++; // TA present, skip it -- not needed for this task
        }
        if ((t0 & kAtsT0TbPresent) != 0U) {
            if (idx < ats_len) {
                // [REF] rfal_isoDep.cpp:1096: FWI = (TB >> 4) & 0x0F
                fwi = static_cast<uint8_t>((ats[idx] >> 4) & 0x0FU);
            }
            idx++;
        }
        // TC (idx, if T0's 0x40 bit is set) is not consulted -- this driver
        // never requests DID/NAD, so TC's advanced-features bits are
        // irrelevant here.
    }
    s_apdu_timeout_ms = fwiToFwtMs(fwi);
    s_apdu_nrt_steps = msToNrtSteps64fc(s_apdu_timeout_ms);

    Serial.printf("quarky-tab5: [st25r3916] ISO14443-4 activated: ATS len=%u "
                  "FWI=%u -> per-exchange timeout %u ms; FSCI=%s%u -> card FSC=%u "
                  "(FSC alone allows %u C-APDU bytes per I-block)\n",
                  (unsigned)ats_len, (unsigned)fwi, (unsigned)s_apdu_timeout_ms,
                  has_t0 ? "" : "absent/default ", (unsigned)fsci,
                  (unsigned)s_card_fsc,
                  (unsigned)(s_card_fsc - kIBlockOverhead));
    return true;
}

uint16_t iso14443_4_get_card_fsc() {
    return s_card_fsc;
}

size_t iso14443_4_get_ats(uint8_t *out, size_t cap) {
    if (out == nullptr || cap == 0U || s_ats_len == 0U) {
        return 0;
    }
    const size_t n = (static_cast<size_t>(s_ats_len) < cap) ? s_ats_len : cap;
    memcpy(out, s_ats, n);
    return n;
}

bool apdu_transceive(const uint8_t *tx, size_t tx_len,
                     uint8_t *rx, size_t rx_cap, size_t *rx_len) {
    if (rx_len != nullptr) {
        *rx_len = 0;
    }
    if (!s_nfca_ready || tx == nullptr || tx_len == 0 || rx == nullptr) {
        return false;
    }
    // One PCB byte precedes the APDU; transceive()'s own tx_len parameter is
    // a uint8_t, so this driver's usable APDU size tops out at 254 bytes.
    // That is now a real ceiling on a real feature rather than a theoretical
    // one: since the PDOL-based GPO path landed (2026-08-23) a C-APDU can be
    // ~130 bytes (nfc_emv_read.cpp's kMaxPdolDataLen=120 plus header/Lc/Le),
    // which is why PCD->PICC chaining below now exists. rx_cap is NOT capped
    // at 255 (it was, before PICC->PCD chaining existed, because one frame was
    // one response): a reassembled R-APDU can legitimately be longer than a
    // single frame. It is capped at kMaxReassembledLen instead -- see that
    // constant's own comment.
    if (tx_len > 254U || rx_cap > kMaxReassembledLen) {
        return false;
    }

    // Whole-call envelope shared by the WTX rounds and BOTH chaining
    // directions below; see kMaxApduCallMs for why this is equal to (not
    // added to) the worst case the WTX path alone already had. The new
    // PCD->PICC transmit-chaining path deliberately does NOT get a budget of
    // its own -- it checks this same deadline before every additional
    // fragment, exactly as the receive path already does before every R(ACK).
    const uint32_t call_deadline = millis() + kMaxApduCallMs;

    uint8_t frame[255];
    uint8_t resp[255];
    uint8_t resp_len = 0;
    Xfer xr = Xfer::kNoResponse;

    // Extended DIAG instrumentation (2026-08-23's real-hardware debugging aid,
    // still in active use) -- now called after EVERY exchange, including the
    // new transmit-chaining ones, not only the first.
    auto diag_dump_resp = [&]() {
        if (resp_len > 0) {
            char hexbuf[3 * 32 + 1] = {0};
            const uint8_t dump_len = resp_len < 32U ? resp_len : 32U;
            for (uint8_t i = 0; i < dump_len; i++) {
                snprintf(&hexbuf[i * 3], 4, "%02X ", resp[i]);
            }
            Serial.printf("quarky-tab5: [st25r3916] DIAG apdu_transceive: resp bytes: %s\n",
                          hexbuf);
        }
    };

    // S(WTX) handling, extracted verbatim from the receive loop it used to be
    // inlined in so BOTH directions can use it. A card may ask for a
    // waiting-time extension at ANY point in an exchange -- [REF]
    // rfal_isoDep.cpp:618-624 handles S(WTX) generically, ahead of the R-block
    // and I-block branches alike -- so it can arrive while this driver is
    // waiting for a transmit-chaining R(ACK) just as it can before a response
    // I-block. Returns false when the card's answer is unusable or the WTX/
    // whole-call bounds are exhausted; true when `resp` holds a non-S(WTX)
    // block for the caller to interpret. Bounds are unchanged from before the
    // extraction: kMaxWtxRounds per wait, plus the shared call_deadline.
    auto consume_wtx = [&]() -> bool {
        for (uint8_t wtx_round = 0; ; wtx_round++) {
            if (xr != Xfer::kOk || resp_len == 0) {
                Serial.println("quarky-tab5: [st25r3916] DIAG apdu_transceive: giving up -- "
                               "xfer not kOk or zero-length response");
                return false;
            }
            const uint8_t p = resp[0];
            const bool is_s_wtx = ((p & kPcbTypeMask) == kPcbSBlockType) &&
                                  ((p & kPcbSTypeMask) == kPcbWtxType);
            if (!is_s_wtx) {
                return true;
            }
            // [REF] rfal_isoDep.cpp:415-421: the WTX ack echoes the SAME PCB
            // and the SAME one-byte power-multiplier INF field the request
            // carried.
            if (resp_len < 2U || wtx_round >= kMaxWtxRounds ||
                static_cast<int32_t>(millis() - call_deadline) >= 0) {
                Serial.println("quarky-tab5: [st25r3916] S(WTX) request malformed, "
                               "too many rounds, or call budget exhausted -- "
                               "giving up on this APDU");
                return false;
            }
            const uint8_t ack[2] = {p, resp[1]};
            xr = transceive(/*short_cmd=*/0U, ack, sizeof(ack), /*crc_tx=*/true,
                            /*antcl=*/false, /*crc_rx=*/true,
                            resp, sizeof(resp), &resp_len,
                            millis() + s_apdu_timeout_ms, s_apdu_nrt_steps);
            diag_dump_resp();
        }
    };

    // --- PCD->PICC transmit, fragmented across chained I-blocks if the card's
    // own declared FSC demands it (added 2026-08-24; see this section's
    // SOURCES block for the RFAL citations). [REF] rfal_isoDep.cpp:844-853
    // rfalIsoDepGetMaxInfLen(): FSC counts the PCB byte and the 2 CRC bytes,
    // so a single I-block can carry only FSC-3 bytes of C-APDU. Every real
    // command sent before 2026-08-23 was small enough that this never
    // mattered; a PDOL-based GPO is not.
    const size_t fsc_inf = static_cast<size_t>(s_card_fsc) - kIBlockOverhead;
    // ...and, independently, whatever one Wire1 transaction can actually carry
    // -- see kMaxTxInfPerI2cFrame. Both are hard limits, so the smaller wins.
    const size_t max_inf = (fsc_inf < kMaxTxInfPerI2cFrame) ? fsc_inf
                                                            : kMaxTxInfPerI2cFrame;
    const size_t total_frags = (tx_len + max_inf - 1U) / max_inf;
    if (total_frags > kMaxTxChainFragments) {
        Serial.printf("quarky-tab5: [st25r3916] apdu_transceive: C-APDU of %u bytes "
                      "would need %u I-block fragments at card FSC=%u -- over the "
                      "%u-fragment bound, refusing\n",
                      (unsigned)tx_len, (unsigned)total_frags, (unsigned)s_card_fsc,
                      (unsigned)kMaxTxChainFragments);
        return false;
    }
    if (total_frags > 1U) {
        Serial.printf("quarky-tab5: [st25r3916] DIAG apdu_transceive: C-APDU is %u bytes "
                      "but card FSC=%u (and this bus) allow only %u per I-block -- "
                      "PCD->PICC chaining across %u fragments\n",
                      (unsigned)tx_len, (unsigned)s_card_fsc, (unsigned)max_inf,
                      (unsigned)total_frags);
    }

    size_t tx_pos = 0;
    for (uint8_t frag = 0; frag < static_cast<uint8_t>(total_frags); frag++) {
        const bool is_last = (static_cast<size_t>(frag) + 1U == total_frags);
        const size_t inf_n = is_last ? (tx_len - tx_pos) : max_inf;

        // [REF] rfal_isoDep.cpp:342-344 isoDepTx(): the chaining bit is OR'd
        // into the I-block PCB for every fragment that is not the last, and
        // only for I-blocks. :251 isoDep_PCBIBlockChaining(bn) is this exact
        // composition. The block number is NOT toggled per fragment here --
        // it is toggled only once the card's R(ACK) for that fragment has been
        // validated, below ([REF] :652-654's ordering).
        frame[0] = static_cast<uint8_t>(kPcbB2Bit | (s_pcb_block_number & 0x01U) |
                                        (is_last ? 0U : kPcbChainingBit));
        memcpy(&frame[1], &tx[tx_pos], inf_n);
        const uint8_t frame_len = static_cast<uint8_t>(inf_n + 1U);

        // nrt_steps=s_apdu_nrt_steps: real bug found & fixed via real-hardware
        // testing (2026-08-23) -- this call used to omit the nrt_steps argument
        // entirely, silently defaulting to the anticollision-tuned ~165us NRT
        // regardless of s_apdu_timeout_ms already being computed for exactly
        // this purpose. See s_apdu_nrt_steps's own declaration comment.
        xr = transceive(/*short_cmd=*/0U, frame, frame_len, /*crc_tx=*/true,
                        /*antcl=*/false, /*crc_rx=*/true,
                        resp, sizeof(resp), &resp_len,
                        millis() + s_apdu_timeout_ms, s_apdu_nrt_steps);
        Serial.printf("quarky-tab5: [st25r3916] DIAG apdu_transceive: sent frame_len=%u "
                      "pcb=0x%02X frag=%u/%u inf=%u tx[0..4]=%02X %02X %02X %02X "
                      "-- xfer=%u resp_len=%u\n",
                      (unsigned)frame_len, frame[0], (unsigned)(frag + 1),
                      (unsigned)total_frags, (unsigned)inf_n,
                      inf_n > 0 ? tx[tx_pos] : 0,
                      inf_n > 1 ? tx[tx_pos + 1] : 0,
                      inf_n > 2 ? tx[tx_pos + 2] : 0,
                      inf_n > 3 ? tx[tx_pos + 3] : 0,
                      (unsigned)xr, (unsigned)resp_len);
        diag_dump_resp();

        if (is_last) {
            // The card's answer to the FINAL fragment is the R-APDU itself --
            // fall through to the receive/reassembly loop below with `xr`,
            // `resp` and `resp_len` exactly as the pre-chaining code left
            // them, so the (real-hardware-verified) PICC->PCD path is
            // unchanged and composes with this one: a single call may now
            // both send a chained command AND receive a chained response.
            break;
        }
        tx_pos += inf_n;

        // A non-final fragment is answered with an R(ACK), not with data
        // ([REF] rfal_isoDep.cpp:651-663 "Rule 7 - Chaining transaction done,
        // continue chaining"). S(WTX) may legally arrive first.
        if (!consume_wtx()) {
            return false;
        }
        const uint8_t ack_pcb = resp[0];
        // [REF] rfal_isoDep.cpp:227 isoDep_PCBisRBlock + :236 isoDep_PCBisACK,
        // expanded into kPcbRBlockValidMask/Val and kPcbRTypeMask/kPcbRAck --
        // see their declarations. A NAK, an I-block, or any other block here
        // is a protocol failure this driver does not try to recover from.
        const bool is_r_ack = ((ack_pcb & kPcbRBlockValidMask) == kPcbRBlockValidVal) &&
                              ((ack_pcb & kPcbRTypeMask) == kPcbRAck);
        // [REF] rfal_isoDep.cpp:652: the R(ACK) echoes the block number the
        // fragment carried -- compared BEFORE the toggle, unlike the
        // PICC->PCD case which toggles first and then builds its own R(ACK).
        const bool ack_bn_matches = (ack_pcb & 0x01U) == (s_pcb_block_number & 0x01U);
        if (!is_r_ack || !ack_bn_matches) {
            Serial.printf("quarky-tab5: [st25r3916] apdu_transceive: PCD chaining -- "
                          "fragment %u/%u was not acknowledged with a valid R(ACK) "
                          "(pcb=0x%02X is_r_ack=%u bn_matches=%u expected_bn=%u)\n",
                          (unsigned)(frag + 1), (unsigned)total_frags, ack_pcb,
                          (unsigned)is_r_ack, (unsigned)ack_bn_matches,
                          (unsigned)(s_pcb_block_number & 0x01U));
            return false;
        }
        // [REF] rfal_isoDep.cpp:654 "Rule B - ACK with expected bn -> Increment
        // block number".
        s_pcb_block_number ^= 1U;
        Serial.printf("quarky-tab5: [st25r3916] DIAG apdu_transceive: PCD chaining -- "
                      "fragment %u/%u acknowledged (R(ACK)=0x%02X), next bn=%u\n",
                      (unsigned)(frag + 1), (unsigned)total_frags, ack_pcb,
                      (unsigned)s_pcb_block_number);

        if (static_cast<int32_t>(millis() - call_deadline) >= 0) {
            Serial.printf("quarky-tab5: [st25r3916] apdu_transceive: PCD chaining "
                          "exhausted the call budget after fragment %u/%u -- giving up\n",
                          (unsigned)(frag + 1), (unsigned)total_frags);
            return false;
        }
    }

    // Reassembly cursor. Every accepted fragment's INF field (the frame minus
    // its own PCB byte) is appended here; for the overwhelmingly common
    // single-frame case the loop below runs exactly once and this behaves
    // identically to the pre-chaining code.
    size_t assembled = 0;

    for (uint8_t chain_round = 0; ; chain_round++) {
        // --- S(WTX) sub-handling, unchanged in behavior and bounds, now
        // living in the consume_wtx() helper above so the transmit-chaining
        // path can share it. A card may request a waiting-time extension
        // before ANY fragment, not just the first, so this stays inside the
        // chaining loop.
        if (!consume_wtx()) {
            return false;
        }

        const uint8_t pcb = resp[0];
        // [REF] ISODEP_PCB_IB_VALID_MASK/VAL: bits (0x20|0x02) must read
        // (0|0x02) on a real I-block -- the B2 must-be-1 bit set AND the B6
        // must-be-0 bit clear, on top of the type bits identifying an I-block
        // in the first place.
        const bool is_i_block = ((pcb & kPcbTypeMask) == kPcbIBlockType) &&
                                ((pcb & kPcbB2Bit) != 0U) &&
                                ((pcb & kPcbB6Bit) == 0U);
        // [REF] rfal_isoDep.cpp:709-712 (final block) and :682 (chained
        // block, which applies the identical `isoDep_GetBN(rxPCB) ==
        // gIsoDep.blockNumber` test): the PICC's I-block echoes the SAME
        // block-number bit the PCD's last frame carried -- RFAL itself treats
        // a mismatch as a mandatory reject (Digital 1.1 15.2.6.4 / EMVCo 2.6
        // 10.3.5.4), not an optional check. Missing this would let a stale or
        // duplicate I-block (e.g. a retransmission after RF noise) be
        // silently accepted as the answer to whatever was just sent.
        const bool bn_matches = (pcb & 0x01U) == (s_pcb_block_number & 0x01U);
        if (!is_i_block || !bn_matches) {
            Serial.printf("quarky-tab5: [st25r3916] DIAG apdu_transceive: rejected -- "
                          "pcb=0x%02X is_i_block=%u bn_matches=%u expected_bn=%u\n",
                          pcb, (unsigned)is_i_block, (unsigned)bn_matches,
                          (unsigned)(s_pcb_block_number & 0x01U));
            // An R-block, a malformed/unsupported S-block, or an I-block with
            // the wrong block number -- not handled (see this section's
            // SOURCES note on scope). Reported as failure rather than guessed
            // at. NOTE this deliberately does NOT implement RFAL's own "Rule
            // 5 - PICC chaining invalid I-Block -> R-ACK" recovery
            // (rfal_isoDep.cpp:702-703): that is a retransmission-request
            // path, and this driver has no retransmission behavior in either
            // direction (see the R-block scope bullet above).
            return false;
        }

        const size_t inf_len = static_cast<size_t>(resp_len) - 1U;
        if (inf_len > rx_cap - assembled) {
            Serial.printf("quarky-tab5: [st25r3916] apdu_transceive: reassembled "
                          "response would exceed caller capacity (%u + %u > %u)\n",
                          (unsigned)assembled, (unsigned)inf_len, (unsigned)rx_cap);
            return false;
        }
        memcpy(&rx[assembled], &resp[1], inf_len);
        assembled += inf_len;

        // [REF] rfal_isoDep.cpp:269 isoDep_ToggleBN(): the block number
        // toggles after every accepted I-block -- both the final one (:712)
        // and each chained one (:684). Done here, once, for both cases.
        s_pcb_block_number ^= 1U;

        // [REF] rfal_isoDep.cpp:86 ISODEP_PCB_CHAINING_BIT / :229
        // isoDep_PCBisChaining(). Bit clear -> this was the last fragment.
        if ((pcb & kPcbChainingBit) == 0U) {
            break;
        }

        if (chain_round >= kMaxChainingRounds ||
            static_cast<int32_t>(millis() - call_deadline) >= 0) {
            Serial.printf("quarky-tab5: [st25r3916] apdu_transceive: PICC chaining "
                          "exceeded its bound (round=%u, %u bytes so far) -- giving up\n",
                          (unsigned)chain_round, (unsigned)assembled);
            return false;
        }

        // [REF] rfal_isoDep.cpp:678-700, the PICC-chaining branch of
        // rfalIsoDepDataExchangePCD()'s receive handling, ported literally:
        // on a chained I-block with the expected block number, RFAL toggles
        // its block number FIRST (:684 isoDep_ToggleBN) and only then emits
        // "Rule 2 - Send ACK" (:689 isoDepHandleControlMsg(ISODEP_R_ACK)),
        // whose PCB is isoDep_PCBRACK(gIsoDep.blockNumber) (:389) -- i.e. the
        // ALREADY-TOGGLED number. The toggle above therefore happens before
        // this R(ACK) is built, matching RFAL's ordering exactly; getting it
        // backwards would ACK with the previous block number and stall the
        // chain. The R-block carries no INF field ([REF] :95
        // ISODEP_RBLOCK_INF_LEN = 0), so the frame is the single PCB byte.
        const uint8_t rack[1] = {
            static_cast<uint8_t>(kPcbRBlockAckBase | (s_pcb_block_number & 0x01U))};
        Serial.printf("quarky-tab5: [st25r3916] DIAG apdu_transceive: PICC chaining "
                      "(pcb=0x%02X), sending R(ACK)=0x%02X for fragment %u\n",
                      pcb, rack[0], (unsigned)(chain_round + 1));
        xr = transceive(/*short_cmd=*/0U, rack, sizeof(rack), /*crc_tx=*/true,
                        /*antcl=*/false, /*crc_rx=*/true,
                        resp, sizeof(resp), &resp_len,
                        millis() + s_apdu_timeout_ms, s_apdu_nrt_steps);
        diag_dump_resp();
    }

    if (rx_len != nullptr) {
        *rx_len = assembled;
    }
    return true;
}

// ===========================================================================
// NFC-A Listen Mode / tag emulation (Phase 3 Task 24)
//
// SOURCES FOR THIS SECTION ONLY. Every register/bit/command below traces to
// the same two documents already cited at the top of this file (DS12484
// Rev 3 + the vendored RFAL reference driver), PLUS the vendored tree's
// Listen Mode implementation specifically, not previously read by this
// project before this task:
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/rfal_rfst25r3916.h
//     - rfalLm struct (:79-91), rfalLmConfPA (NFC-RFAL/src/rfal_rf.h:485-491):
//       the real, silicon-level "answer with an arbitrary UID/ATQA/SAK"
//       configuration surface this task needed to confirm exists before any
//       code was written -- it does.
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/rfal_rfst25r3916.cpp
//     - rfalListenStart() (:2407-2502): the PT-memory build (NFCID triple +
//       SENS_RES + 3x SEL_RES, with the INCOMPLETE bit set on non-final
//       cascade levels for 7-byte UIDs -- ":2456-2458"), the AUX nfc_id
//       length select (:2439/2443), the PASSIVE_TARGET autoResp bits
//       (:2431-2433, then :2462 clearing d_106_ac_a specifically), the MODE
//       register's target/listen-NFCA encoding (:2429/2464-2467), and the
//       Guard Time register write (:2488-2492).
//     - rfalRunListenModeWorker() (:2737-2935) and rfalListenSetState()
//       (:2616-2734): confirms the chip's own "Passive Target Anticollision"
//       (PTA) hardware state machine autonomously walks
//       Idle->ReadyL1->ReadyL2->Active (i.e. REQA/WUPA, anticollision, and
//       SELECT) on its own once armed -- firmware's whole job is polling
//       PASSIVE_TARGET_STATUS (0x21) to see which state it reached, exactly
//       the same "polled status register instead of an IRQ pin" substitution
//       nfca_detect() above already makes for the reader path. BUT (see the
//       2026-08-24 real-hardware bug note below) arming alone does NOT start
//       that walk: rfalListenStart()'s own last statement is
//       "return rfalListenSetState(RFAL_LM_STATE_POWER_OFF)" (:2501), and it
//       is that state-entry sequence -- not the arming writes -- that puts the
//       PTA engine into its operational cycle.
//     - rfalListenSetState()'s RFAL_LM_STATE_POWER_OFF case (:2634-2671):
//       set OP_CONTROL.rx_en (:2635), CMD_STOP (:2636), then for NFC-A clear
//       PASSIVE_TARGET.d_106_ac_a and CMD_GOTO_SENSE (:2638-2641), clear
//       ISO14443A_NFC.nfc_f0 (:2643), re-arm the interrupt set (:2644-2654),
//       write MODE's targ/om/nfc_ar bits from the mdReg value built back in
//       rfalListenStart() (:2658-2660), and finally branch on
//       rfalIsExtFieldOn() (:2662): field already present -> re-enter the same
//       switch with RFAL_LM_STATE_IDLE (a do/while(reSetState) loop, :2630/
//       2663-2664); no field -> clear tx_en|rx_en|en (:2666-2669), i.e. power
//       the receiver back down while waiting.
//     - rfalListenSetState()'s RFAL_LM_STATE_IDLE case (:2673-2695): if
//       OP_CONTROL.en is not already set, set en|rx_en in ONE write (:2675)
//       and wait for the real OSC interrupt with a 10 ms timeout
//       (:2677-2682); else just consume the pending OSC interrupt (:2684).
//       Then, only if the PREVIOUS Lm state was ACTIVE_A, clear d_106_ac_a and
//       re-issue CMD_GOTO_SENSE (:2687-2690). ALWAYS finish with CMD_CLEAR_FIFO
//       then CMD_UNMASK_RECEIVE_DATA (:2692-2693).
//     - rfalRunListenModeWorker()'s own POWER_OFF case (:2747-2757): the
//       worker sits in POWER_OFF until the EON ("external field on")
//       interrupt, and only then calls rfalListenSetState(IDLE) -- i.e. the
//       POWER_OFF -> IDLE transition is software-driven every single time, not
//       something the PTA hardware does by itself. This driver polls the same
//       field-presence condition from AUX_DISPLAY.efd_o instead of taking the
//       EON interrupt (see rfalIsExtFieldOn() below), for the same
//       "no IRQ pin on this unit" reason as everywhere else in this file.
//     - rfalIsExtFieldOn() (:2401-2404) -> [REF] st25r3916.h:154's
//       st25r3916IsExtFieldOn() macro: literally
//       "st25r3916CheckReg(ST25R3916_REG_AUX_DISPLAY,
//        ST25R3916_REG_AUX_DISPLAY_efd_o, ST25R3916_REG_AUX_DISPLAY_efd_o)",
//       i.e. AUX_DISPLAY (0x31) bit 6 ([REF] st25r3916_com.h:877). That single
//       bit is the whole external-reader-present test.
//     - rfalInitialize() (:110-112): "Enable External Field Detector as:
//       Automatics" -- st25r3916ChangeRegisterBits(OP_CONTROL, en_fd_mask,
//       en_fd_auto_efd). The EFD is OFF after Set Default, so efd_o (and the
//       EON/EOF interrupts) are dead until this is written. RFAL does it once
//       at init; this driver does it in listen_start() and undoes it in
//       listen_stop() -- see those functions' own comments.
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/st25r3916_com.h
//     - :107 REG_PASSIVE_TARGET=0x08 (RW), :143 REG_PASSIVE_TARGET_STATUS=
//       0x21 (R); :392/393/395 the three "d_" (disable) bits d_ac_ap2p (bit
//       3), d_212_424_1r (bit 2), d_106_ac_a (bit 0); :597-611 the
//       pta_state<3:0> value table (power_off=0 [":597",
//       pta_st_power_off -- this project's own first reading of this table
//       skipped it, which cost a day; see the 2026-08-24 note below], idle=1,
//       ready_l1=2, ready_l2=3, active=5, halt=9, ready_l1_x=0xA,
//       ready_l2_x=0xB, active_x=0xD); :246-253 OP_CONTROL's en_fd<1:0>
//       External Field Detector mode bits (bits 1:0; 00b = off, 11b =
//       automatic); :877 AUX_DISPLAY's efd_o (bit 6); :255-285 the
//       MODE register's targ (bit7), om3 (bit6), om0 (bit3) and nfc_ar<1:0>
//       bits; :433-438 AUX register nfc_id<1:0> length-select bits
//       (bits 5:4); :454-478 the RX_CONF1 lp/hz and RX_CONF2 amd_sel bits used by the
//       Listen-On analog config below; :540-552 TIMER_EMV_CONTROL's gptc
//       and mrt_step bits.
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/st25r3916.h
//     - :96/97 CMD_GOTO_SENSE=0xCD "Passive target logic to Sense/Idle
//       state", CMD_GOTO_SLEEP=0xCE "...to Sleep/Halt state" -- both plain
//       one-byte direct commands, same I2C framing execute_command() above
//       already uses; :99/107 CMD_UNMASK_RECEIVE_DATA=0xD1,
//       CMD_CLEAR_FIFO=0xDB; :154 st25r3916IsExtFieldOn() (see above).
//       (Field presence is read from AUX_DISPLAY.efd_o exactly as RFAL's own
//       macro does. The EOF bit of the IRQ status word sampleIrqs() already
//       reads -- st25r3916_interrupt.h:80, "external field off interrupt" --
//       is kept as a second, edge-triggered field-loss signal alongside it,
//       because efd_o is a level and a reader that dips its field between two
//       polling ticks would otherwise go unnoticed.)
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/st25r3916_com.cpp
//     - st25r3916WritePTMem() (:474-542): confirms a genuine I2C (not
//       SPI-only) code path exists for loading PT memory -- one mode byte
//       (PT_A_CONFIG_LOAD, :56, 0xA0) followed by a plain byte-at-a-time I2C
//       burst write, the same shape write_register()/execute_command()
//       already use, just a different mode byte and a multi-byte payload.
//       :81 PTM_A_LEN=15 (10-byte NFCID triple + 2-byte SENS_RES + 3x 1-byte
//       SEL_RES).
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/rfal_rfst25r3916_analogConfigTbl.h
//     - :482-493 "Default Analog Configuration for Chip-Specific Listen On"
//       (RFAL_ANALOG_CONFIG_TECH_CHIP|CHIP_LISTEN_ON): the exact 9 register
//       writes apply_listen_config() below applies, ported the same
//       "fold RFAL's analog-config table into a literal register list" way
//       apply_nfca_config() (this file's reader-path analog config) already
//       does for CHIP_INIT/CHIP_POLL_COMMON -- this is the Listen-mode
//       counterpart of that exact table, not a new technique.
//   ~/src/wilson-elechouse/ST25R3916/NFC-RFAL/src/rfal_rf.h
//     - :106 RFAL_LM_MASK_NFCA; :111 RFAL_LM_SENS_RES_LEN=2; :118
//       RFAL_NFCID1_TRIPLE_LEN=10; :272 RFAL_LM_NFCID_INCOMPLETE=0x04 (SEL_RES
//       bit meaning "another cascade level follows"); :457/458
//       RFAL_LM_NFCID_LEN_04/07; :92/95/96 RFAL_1FC_IN_512FC=512,
//       RFAL_US_IN_MS=1000, RFAL_1MS_IN_1FC=13560 (fc=13.56 MHz) -- used
//       below to compute the literal Guard Time register value the same way
//       kNrtSteps64fc/kMrtSteps64fc above compute theirs from RFAL's own
//       1/fc-unit constants; :281 RFAL_LM_GT = rfalConvUsTo1fc(100) = (100 *
//       13560) / 1000 = 1356 (1/fc units), then rfalConv1fcTo512fc(1356) =
//       1356 / 512 = 2 (512/fc units, integer division per RFAL's own macro)
//       is the literal byte this section writes to MASK_RX_TIMER.
//
// CONFIRMED FEASIBLE BEFORE WRITING ANY CODE (this task's Step 1 gate,
// matching Task 2/15/23's precedent): the chip's PTA engine answers
// REQA/WUPA + the full anticollision/SELECT sequence autonomously, firmware
// CAN program an arbitrary 4- or 7-byte UID (not just a factory-fixed one),
// and every access needed (register writes, PT-memory writes, status-register
// polling) is available over I2C without an IRQ pin -- so this is
// implemented, not reported blocked.
//
// SCOPE, stated honestly, same policy as every other section in this file:
//   * Single UID length per session (4 or 7 bytes) -- no 10-byte/triple-
//     cascade support (RFAL's own rfalLmConfPA has none either).
//   * No SLEEP_A / re-select-after-HALT handling (RFAL_LM_STATE_SLEEP_A and
//     rfalListenSleepStart() are not ported) -- a reader that HALTs our
//     emulated tag and later tries to WUPA it again gets whatever the PTA
//     hardware does on its own; this driver does not actively re-arm for
//     that specific case beyond the general EOF-triggered re-arm below.
//   * No response to anything past SELECT (RATS, READ, etc.) -- the plan's
//     own out-of-scope stretch goal (full memory-content emulation). This
//     driver only reports that a reader reached the "active"/"active*" PTA
//     state, i.e. that our UID/SAK were accepted.
//   * The chip's own antenna/RF analog tuning (RFAL's separate LISTEN AP2P
//     entries, and any board-specific antenna-matching values beyond the two
//     literal ANT_TUNE_A/B bytes the cited table specifies) is not
//     independently re-verified against a real reader in this task -- that
//     is exactly what Step 4's PAUSE FOR HARDWARE is for.
//
// REAL BUG FOUND & FIXED VIA REAL-HARDWARE TESTING (2026-08-24): Listen Mode
// was armed correctly and still answered nothing. listen_start() logged
// "Listen Mode armed" with a verified-correct PT memory image (a real saved
// EMV card: uid_len=4, cfg.uid=08:92:2F:97, cfg.atqa=0400, cfg.sak=20,
// pt_mem=08 92 2F 97 00 00 00 00 00 00 04 00 20 20 20), yet with TWO different
// real external readers (an NFC Tools phone app and a Chameleon Ultra) held
// against the antenna, listen_poll() reported pts=0x00 / irqs=0x00000080 (that
// 0x80 is just I_osc from the oscillator start) and then pts=0x00 /
// irqs=0x00000000 forever -- both readers said "no tag found".
//
// pts=0x00 is pta_state=0 = pta_st_power_off ([REF] st25r3916_com.h:597): the
// PTA hardware state machine never left its power-off state, which is exactly
// what the RFAL sources above say happens if firmware stops after the arming
// writes. Two concrete defects, both of them sequencing, none of them register
// VALUES (the arming values were and are correct):
//   1. The External Field Detector was never enabled. Nothing in this driver
//      ever wrote OP_CONTROL's en_fd<1:0>, so it sat at its post-Set-Default
//      00b = off. With the EFD off, efd_o never asserts and the EON/EOF
//      interrupts never fire -- the chip has no "a reader's field is here"
//      signal at all, and the POWER_OFF -> IDLE transition (which is
//      software-driven in RFAL, and now here) can never be triggered. RFAL
//      enables it once at init ([REF] rfalInitialize():110-112); this port had
//      simply never ported that line, because the reader path -- which drives
//      its own field and needs no detector -- never missed it.
//   2. The IDLE-entry commands ran in the wrong order and only once. The old
//      tail issued CMD_GOTO_SENSE / CMD_CLEAR_FIFO / CMD_UNMASK_RECEIVE_DATA
//      BEFORE OP_CONTROL.en was set and before osc_ok was polled, i.e. while
//      the receiver was still down. [REF] rfalListenSetState()'s
//      RFAL_LM_STATE_IDLE case (:2673-2695) does the opposite and for a
//      reason: en|rx_en first, oscillator stable second, CLEAR_FIFO +
//      UNMASK_RECEIVE_DATA last -- and it re-runs that whole sequence on every
//      POWER_OFF -> IDLE transition, not once at arming time.
// THE FIX: port RFAL's POWER_OFF -> (conditionally) IDLE state entry as
// listenEnterPowerOff()/listenEnterIdle() below, call it from listen_start()'s
// tail the way rfalListenStart():2501 does, and drive the same transition from
// listen_poll() on every field-presence edge the way
// rfalRunListenModeWorker():2747-2757 does.
//
// ===========================================================================
// SECOND REAL-HARDWARE FINDING, SAME DAY (2026-08-24), AND THE CONTENT-
// EMULATION EXTENSION IT DROVE.
//
// With the fix above in place, both real readers -- an iPhone running NFC
// Tools and a Chameleon Ultra -- genuinely completed the whole anticollision/
// SELECT sequence against the emulated tag: PASSIVE_TARGET_STATUS's pta_state
// field walked 1(idle) -> 2/3(ready_l1/l2) -> 5(active), repeatedly and
// reproducibly, against BOTH a saved EMV card (SAK 0x20) and a saved real
// NTAG (SAK 0x00, UID 04:73:45:E4:3A:02:89, ATQA 0x0044). The protocol worked.
//
// And NEITHER reader ever showed a stable "tag found" to its user: both kept
// re-polling, which is exactly the 1 -> 2 -> 3 -> 5 -> 0 -> 1 ... cycling
// visible in the DIAG log. Testing the SAK 0x00 tag (not ISO14443-4 capable,
// so RATS cannot be the explanation) produced the identical cycling, which
// isolates the real cause: this driver answered NOTHING after SELECT. Every
// real reader, whatever the tag's SAK, reads something back before it declares
// success to its own user; getting silence, it concludes the tag has gone and
// re-polls. That was the baseline's own disclosed scope boundary ("No response
// to anything past SELECT ... the plan's own out-of-scope stretch goal"), and
// it makes emulation useless as shipped.
//
// THE EXTENSION: answer the real NFC Forum Type 2 Tag READ command from a
// captured page image (captured by t2t_read_pages() above, stored in
// NfcCommon::TagInfo, passed in through ListenConfig::pages).
//
// ADDITIONAL SOURCES FOR THIS EXTENSION ONLY:
//   ~/src/wilson-elechouse/ST25R3916/ST25R3916_ELECHOUSE/src/rfal_rfst25r3916.cpp
//     - rfalRunListenModeWorker()'s RFAL_LM_STATE_ACTIVE_A / _ACTIVE_Ax case
//       (:2850-2891) -- THE mechanism for receiving data in the active state,
//       and the model for listenServiceFrame() below. On RXE (:2853/2856): OR
//       in the PAR/CRC/ERR2/ERR1 status (:2857-2860), read
//       st25r3916GetNumFIFOBytes() (:2861), and if any of CRC/ERR1/PAR is set
//       OR the length is <= RFAL_CRC_LEN, discard: zero the length, CLEAR_FIFO
//       + UNMASK_RECEIVE_DATA, and drop back to IDLE (or, from ACTIVE_Ax, to
//       SLEEP_A) (:2863-2881). Otherwise subtract RFAL_CRC_LEN (:2884 -- the
//       same 2-byte CRC-in-FIFO subtraction this file's own kCrcLen already
//       centralises for the reader path), read the FIFO (:2885), and hand the
//       raw bytes to the application via dataFlag/rxLen (:2886-2887). RFAL
//       does NOT interpret the command or build the response -- that is the
//       application's job in its own layering, and is what is implemented
//       here for the first time in this project (every other feature in this
//       codebase is a READER interpreting a TAG's answer; this is the
//       reverse).
//       On EOF instead of RXE (:2854-2855) it returns to POWER_OFF, which is
//       what listen_poll()'s existing field-loss handling already does.
//     - rfalListenSetState()'s RFAL_LM_STATE_ACTIVE_A / _ACTIVE_Ax case
//       (:2705-2712) -- the state ENTRY that must happen once the PTA reaches
//       active: `st25r3916SetRegisterBits(REG_PASSIVE_TARGET, d_106_ac_a)`,
//       i.e. DISABLE the chip's autonomous 106 kb/s anticollision now that
//       selection is over, so every subsequent frame is delivered to firmware
//       as ordinary RXE data instead of being consumed by the PTA hardware;
//       then clear the stale PAR/CRC/ERR2/ERR1 status and enable RXE. Ported
//       as listenEnterActive() below (the interrupt-enable half is a no-op
//       here for the same reason listenEnterPowerOff() already documents:
//       this driver never writes the four IRQ MASK registers at all, leaving
//       them at Set Default's all-enabled state).
//     - rfalPrepareTransceive() (:1145-1160) -- the ONE load-bearing
//       difference between the Listen Mode transmit path and the reader
//       path's own transceive(): "In Passive Listen Mode do not use STOP as
//       it stops FDT timer" -- CMD_CLEAR_FIFO replaces the CMD_STOP +
//       CMD_RESET_RXGAIN pair. This was checked against the vendored source
//       specifically rather than assumed identical, and it is not: reusing
//       transceive() verbatim for a PICC response would stop the very timer
//       that paces the answer. Everything after that IS the same: the
//       no_tx_par/no_rx_par/nfc_f0 programme (:1190-1209) and the AGC enable
//       (:1211-1216).
//     - rfalTransceiveTx()'s RFAL_TXRX_STATE_TX_TRANSMIT case (:1341-1350,
//       :1370-1375) -- st25r3916SetNumTxBits(bit count), st25r3916WriteFifo(),
//       then CMD_TRANSMIT_WITH_CRC / CMD_TRANSMIT_WITHOUT_CRC. Identical to
//       the reader path, including the BIT (not byte) count register pair,
//       which is what lets a 4-bit NAK be transmitted at all. :1359-1366 adds
//       one Listen-Mode-only guard -- refuse to transmit if the external
//       field has gone -- which is explicitly skipped while the Lm state is
//       ACTIVE_A/ACTIVE_Ax, i.e. in exactly the state this driver transmits
//       from, so there is nothing to port.
//     - rfalListenSleepStart()'s RFAL_LM_STATE_SLEEP_A branch (:2516-2529) --
//       the real HALT handling, ported as listenEnterSleep(): clear
//       PASSIVE_TARGET.d_106_ac_a (re-arming the PTA's autonomous
//       anticollision so a later WUPA can wake us), CMD_GOTO_SLEEP (:2518,
//       [REF] st25r3916.h:97 "Passive target logic to Sleep/Halt state"),
//       re-write MODE's targ/om/nfc_ar bits (:2519-2521), clear
//       ISO14443A_NFC.nfc_f0 (:2528) and CMD_UNMASK_RECEIVE_DATA (:2529).
//   ~/src/wilson-elechouse/ST25R3916/NFC-RFAL/src/rfal_t2t.cpp/.h
//     - the T2T command set, READ response length, and the 4-bit ACK/NAK
//       encoding. Fully cited in the "NFC Forum Type 2 Tag page read" SOURCES
//       block earlier in this file; the responder below answers exactly the
//       commands that block describes reading.
//
// DISCLOSED GAP -- GET_VERSION IS NOT IMPLEMENTED, DELIBERATELY. Real
// NTAG21x and Ultralight EV1 tags support a GET_VERSION command that returns
// an 8-byte product identification, and real readers do send it. There is no
// citable source for either its command byte or its response layout anywhere
// in this project's vendored sources -- not in RFAL's own T2T layer
// (rfal_t2t.cpp:73-77 lists exactly three commands: READ, WRITE, SECTOR
// SELECT), not in the MFRC522_I2C library, not in the Bruce/Poseidon/UniGeek
// donors (grepped, 2026-08-24). Inventing the byte and fabricating a
// plausible-looking 8-byte answer is precisely what this file's SOURCES
// discipline exists to prevent, so it is NOT done: an unrecognised command
// gets a NAK, which is also genuinely what a real plain MIFARE Ultralight
// (MF0ICU1, SAK 0x00 -- the exact family and the exact SAK this emulates)
// does with GET_VERSION. A reader that gets a NAK there falls back to plain
// READs, which is the path that actually carries the tag's content. If a real
// reader is later observed to need it, the honest fix is to capture the real
// tag's own GET_VERSION response during t2t_read_pages() and replay it, not
// to synthesise one.
// ===========================================================================

namespace {

// --- Registers ---------------------------------------------------------
constexpr uint8_t kRegPassiveTarget       = 0x08U; // RW Passive target definition
constexpr uint8_t kRegPassiveTargetStatus = 0x21U; // R  Passive target state status

// --- Direct commands -----------------------------------------------------
constexpr uint8_t kCmdGotoSense          = 0xCDU; // PTA logic -> Sense/Idle
constexpr uint8_t kCmdGotoSleep          = 0xCEU; // PTA logic -> Sleep/Halt
constexpr uint8_t kCmdUnmaskReceiveData  = 0xD1U;
constexpr uint8_t kCmdClearFifo          = 0xDBU;

// --- PASSIVE_TARGET (0x08) bits: each is a DISABLE bit; clearing one
// re-enables the corresponding autonomous hardware behaviour.
constexpr uint8_t kPtDisableAp2p    = 1U << 3; // d_ac_ap2p
constexpr uint8_t kPtDisable212424  = 1U << 2; // d_212_424_1r
constexpr uint8_t kPtDisable106Ac   = 1U << 0; // d_106_ac_a -- 0 = chip auto-
                                               // answers 106k anticollision
constexpr uint8_t kPtAllDisabled =
    static_cast<uint8_t>(kPtDisableAp2p | kPtDisable212424 | kPtDisable106Ac);
// This driver only ever arms NFC-A at 106 kb/s, so 212/424 and AP2P stay
// permanently disabled; only d_106_ac_a is cleared when armed.
constexpr uint8_t kPtArmed =
    static_cast<uint8_t>(kPtDisableAp2p | kPtDisable212424);

// --- PASSIVE_TARGET_STATUS (0x21) pta_state<3:0> values.
// (0x0 = pta_st_power_off, 0x1 = idle, 0x2/0x3 = ready_l1/l2, 0x9 = halt,
// 0xA/0xB = ready_l1_x/l2_x are deliberately NOT defined as constants: this
// driver only branches on "active-or-not". They ARE printed raw by the DIAG
// line in listen_poll(), and 0x0 in particular is what identified the
// 2026-08-24 bug documented above -- same unused-constant policy as the
// 0xC2/0xC6/collision-register notes elsewhere in this file.)
constexpr uint8_t kPtaStateMask   = 0x0FU;
constexpr uint8_t kPtaStActive    = 0x05U;
constexpr uint8_t kPtaStActiveX   = 0x0DU;

// --- OP_CONTROL (0x02) External Field Detector mode, en_fd<1:0> = bits 1:0.
// [REF] st25r3916_com.h:246-253. 00b leaves the detector off (the state after
// Set Default, and what this driver ran in until 2026-08-24); 11b is the
// "automatic" mode RFAL enables at init ([REF] rfalInitialize():112) and the
// only mode in which AUX_DISPLAY.efd_o below means anything.
constexpr uint8_t kOpControlEnFdMask    = 3U << 0;
constexpr uint8_t kOpControlEnFdAutoEfd = 3U << 0;
constexpr uint8_t kOpControlEnFdOff     = 0U << 0;

// --- AUX_DISPLAY (0x31) efd_o (bit 6): 1 = an external reader's field is
// present right now. [REF] st25r3916_com.h:877, read exactly as [REF]
// st25r3916.h:154's st25r3916IsExtFieldOn() macro does.
constexpr uint8_t kAuxDisplayEfdO = 1U << 6;

// --- MODE (0x03) target-mode bits, distinct from the reader path's own
// kModeOmIso14443a poller value above -- targ selects target(1)/initiator(0);
// om3|om0 together select the "listen NFC-A" operating sub-mode (a different
// om<3:0> encoding than the reader path's om<3:0>=0001b).
constexpr uint8_t kModeTarg       = 1U << 7;
constexpr uint8_t kModeOm3        = 1U << 6;
constexpr uint8_t kModeOm0        = 1U << 3;
constexpr uint8_t kModeNfcArMask  = 3U << 0;
constexpr uint8_t kModeNfcArOff   = 0U << 0;

// --- AUX (0x0A) NFCID length select (bits 5:4).
constexpr uint8_t kAuxNfcIdMask    = 3U << 4;
constexpr uint8_t kAuxNfcId4Bytes  = 0U << 4;
constexpr uint8_t kAuxNfcId7Bytes  = 1U << 4;

// --- Listen-On analog config (RX_CONF1/RX_CONF2 bits not already defined
// for the reader path).
constexpr uint8_t kRxConf1LpMask       = 7U << 3;
constexpr uint8_t kRxConf1Lp1200khz    = 0U << 3;
constexpr uint8_t kRxConf1HzMask       = 0x0FU << 0;
constexpr uint8_t kRxConf1Hz12_200khz  = 1U << 0;
constexpr uint8_t kRxConf2AmdSelMask   = 1U << 6;
constexpr uint8_t kRxConf2AmdSelMixer  = 1U << 6;

// --- TIMER_EMV_CONTROL (0x12) bits not already defined for the reader path.
constexpr uint8_t kTimerEmvGptcMask       = 7U << 5;
constexpr uint8_t kTimerEmvGptcNoTrigger  = 0U << 5;
constexpr uint8_t kTimerEmvMrtStep512     = 1U << 3; // vs. the reader path's
                                                     // own kTimerEmvMrtStep=64

// --- IRQ status bits (same 32-bit-word-from-regs-1Ah..1Dh packing this file
// already uses for kIrqRxe etc.).
constexpr uint32_t kIrqEof = 0x00000800U; // 1Bh bit 3: external field off

// --- PT memory (PT_A_CONFIG_LOAD, 0xA0) ------------------------------------
constexpr uint8_t kPtAConfigLoad  = 0xA0U;
constexpr uint8_t kPtMemALen      = 15U;  // NFCID triple(10) + SENS_RES(2) + 3x SEL_RES
constexpr uint8_t kNfcidTripleLen = 10U;
constexpr uint8_t kSelResIncomplete = 0x04U; // RFAL_LM_NFCID_INCOMPLETE

// I2C burst write of the whole PT_A memory block: one mode byte, then
// kPtMemALen data bytes, matching [REF] st25r3916WritePTMem()'s i2c_enabled
// branch (st25r3916_com.cpp:518-538) with the digitalRead()-gated interrupt
// bookkeeping removed, same policy readRegistersRaw()/writeFifoRaw() already
// apply to their own [REF] counterparts.
bool writePtMemA(const uint8_t (&buf)[kPtMemALen]) {
    Wire1.beginTransmission(kI2cAddr);
    bool queued = (Wire1.write(kPtAConfigLoad) == 1);
    for (uint8_t i = 0; i < kPtMemALen && queued; i++) {
        queued = (Wire1.write(buf[i]) == 1);
    }
    return queued && (Wire1.endTransmission(true) == 0);
}

// The Listen-On analog config programme, folded from
// rfal_rfst25r3916_analogConfigTbl.h's "Chip-Specific Listen On" entry (see
// this section's SOURCES block) -- same RegWrite/loop shape
// apply_nfca_config() above already uses for its own analog config table.
bool apply_listen_config() {
    struct RegWrite { bool space_b; uint8_t reg; uint8_t mask; uint8_t value; };
    static const RegWrite kProgramme[] = {
        {false, kRegAntTuneA,     0xFFU,             0x00U},
        {false, kRegAntTuneB,     0xFFU,             0xE0U},
        {false, kRegRxConf1,      kRxConf1LpMask,    kRxConf1Lp1200khz},
        {false, kRegRxConf1,      kRxConf1HzMask,    kRxConf1Hz12_200khz},
        {false, kRegRxConf2,      kRxConf2AmdSelMask, kRxConf2AmdSelMixer},
        {true,  kRegBOvershoot1,  0xFFU,             0x00U},
        {true,  kRegBOvershoot2,  0xFFU,             0x00U},
        {true,  kRegBUndershoot1, 0xFFU,             0x00U},
        {true,  kRegBUndershoot2, 0xFFU,             0x00U},
    };
    for (const RegWrite &w : kProgramme) {
        const bool ok = (w.mask == 0xFFU)
                            ? (w.space_b ? writeRegisterBRaw(w.reg, w.value)
                                         : writeRegisterRaw(w.reg, w.value))
                            : (w.space_b ? changeRegisterBitsB(w.reg, w.mask, w.value)
                                         : changeRegisterBits(w.reg, w.mask, w.value));
        if (!ok) {
            Serial.printf("quarky-tab5: [st25r3916] Listen config write failed at "
                          "%s register 0x%02X\n",
                          w.space_b ? "space-B" : "space-A", w.reg);
            return false;
        }
    }
    return true;
}

bool s_listen_armed = false;
ListenState s_listen_state = ListenState::kNotArmed;
// Mirrors RFAL's own gRFAL.Lm.state split between RFAL_LM_STATE_POWER_OFF
// (false -- no reader field, the PTA engine is parked) and everything from
// RFAL_LM_STATE_IDLE onwards (true). The public ListenState deliberately does
// NOT expose this: to the UI both are "armed, waiting".
bool s_listen_field_on = false;

// --- Content-emulation state (2026-08-24 extension) ------------------------
// The emulated tag's real captured page image, copied out of ListenConfig by
// listen_start() so the caller's buffer needs no lifetime past that call.
// 231 pages x 4 bytes = 924 bytes of BSS, sized and justified in
// nfc_common.h's own kMaxT2tPages comment (NTAG216's real capacity).
uint8_t s_lm_pages[kListenMaxPages][kListenPageLen];
uint8_t s_lm_page_count = 0;
// Mirrors RFAL's gRFAL.Lm.state == RFAL_LM_STATE_ACTIVE_A/_Ax: true once the
// ACTIVE state ENTRY (listenEnterActive()) has run for the current selection,
// so it runs exactly once per selection rather than on every tick.
bool s_lm_active_entered = false;
uint32_t s_lm_read_count = 0;

// Largest received frame this responder will pull out of the FIFO. Every
// command it answers is 2 bytes + CRC (T2T READ, HLTA) and the longest real
// T2T command of any kind is WRITE's 6 bytes + CRC, so 32 is generous; a
// larger frame is not a command this read-only Type 2 emulation can answer
// and is discarded (with a CLEAR_FIFO) rather than truncated.
constexpr uint8_t kLmMaxRxLen = 32U;

// --- Frame-servicing budget, and why it is not a per-tick register read ----
// The reader's own T2T READ timeout is 5 ms ([REF-T2T] rfal_t2t.cpp:53
// RFAL_FDT_POLL_READ_MAX = rfalConvMsTo1fc(5), "TS T2T 1.0 table 18"). One
// listen_poll() tick costs ~1.2 ms of I2C before it even looks at the FIFO,
// and one LVGL frame on this 1280x720 display costs far more than 5 ms -- so
// answering at most one frame per loop() iteration would miss EVERY command a
// real reader sends. Once the PTA reports its active state, this driver
// therefore stays inside a bounded servicing loop, re-sampling the IRQ status
// registers directly (~0.45 ms per look) so the answer latency is one I2C
// round trip rather than one UI frame.
//
// Bounded twice over, and only ever entered while a reader has actually
// SELECTed us:
//   * kLmServiceIdleMs -- leave as soon as the reader has been quiet this
//     long. Real readers send their commands back to back (well under 1 ms
//     apart), so 12 ms is generous think-time without being a stall.
//   * kLmServiceBudgetMs -- a hard ceiling per tick regardless. 250 ms is
//     sized against the real work: a full NTAG215 dump is 135 pages = 34 READ
//     commands at ~4 ms each = ~140 ms, so an entire real read completes
//     inside one tick. Nothing is lost if it does not -- RXE latches in the
//     status register and the frame stays in the FIFO -- the answer just
//     arrives a UI frame later. 250 ms leaves a 20x margin against the ~5 s
//     task-watchdog window this project has already been bitten by twice
//     (hal/ir_unit.h's and hal/storage_sd.cpp's own header comments).
constexpr uint32_t kLmServiceBudgetMs = 250U;
constexpr uint32_t kLmServiceIdleMs   = 12U;

// [REF-T2T] rfal_t2t.cpp:56 RFAL_T2T_ACK = 0x0A and :57 RFAL_T2T_ACK_MASK =
// 0x0F, plus [MI2] MFRC522_I2C.h:308 stating the rule directly: "The MIFARE
// Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK." So a NAK is
// a single 4-BIT frame (RFAL_T2T_ACK_NACK_LEN = 1 byte / 4 bits, :55) whose
// low nibble is anything but 0x0A, carrying no CRC. 0x00 is used here; the
// specific non-ACK nibble is not itself cited anywhere in the vendored
// sources, and per the rule above it does not need to be.
constexpr uint8_t kT2tNakNibble  = 0x00U;
constexpr uint16_t kT2tNakBits   = 4U;
static_assert((kT2tNakNibble & 0x0FU) != 0x0AU,
              "a NAK must not carry the ACK value 0x0A ([REF-T2T] "
              "RFAL_T2T_ACK / [MI2] MFRC522_I2C.h:308)");

// The real RFAL_LM_STATE_IDLE state entry, ported. [REF]
// rfalListenSetState()'s RFAL_LM_STATE_IDLE case (rfal_rfst25r3916.cpp:
// 2673-2695).
//
// This is the sequence whose absence was the 2026-08-24 bug (see this
// section's bug note above): the three PTA commands are only meaningful once
// the chip is enabled and its oscillator is stable, and they must be re-issued
// on every entry into IDLE -- not once at arming time.
//
// NOT ported: [REF] :2687-2690's "if the PREVIOUS Lm state was ACTIVE_A, clear
// d_106_ac_a and re-issue CMD_GOTO_SENSE" sub-case. This port only ever enters
// IDLE from its own POWER_OFF equivalent (listenEnterPowerOff() below, and
// listen_poll()'s field-appeared edge, which is the same transition), and
// listenEnterPowerOff() performs exactly that clear+GOTO_SENSE pair itself a
// few writes earlier -- so an "was the previous state active" parameter here
// would be dead in every call this driver can make. The ACTIVE -> re-arm path
// that sub-case exists for is covered by listen_poll()'s field-loss handling,
// which routes back through listenEnterPowerOff().
bool listenEnterIdle() {
    uint8_t op = 0;
    if (!readRegisterRaw(kRegOpControl, &op)) {
        return false;
    }
    // In this port en is normally already set -- listen_start() turns the
    // oscillator on before it arms anything, and listenEnterPowerOff()
    // deliberately does not turn it back off (see its own tail comment) -- so
    // the branch below is the exception, not the rule. It is kept because
    // [REF] has it and because it is the only thing that would make a future
    // "really power down between readers" change safe.
    if ((op & kOpControlEn) == 0U) {
        // ONE write setting en and rx_en together, exactly as [REF] :2675
        // does -- not two read-modify-writes.
        if (!writeRegisterRaw(
                kRegOpControl,
                static_cast<uint8_t>(op | kOpControlEn | kOpControlRxEn))) {
            return false;
        }
        // [REF] :2677-2682 waits on the real OSC interrupt with a 10 ms
        // timeout. We have no IRQ line ([M5]), so this polls osc_ok for the
        // same bounded 10 ms -- byte-for-byte the pattern field_on() already
        // uses for the reader path's own oscillator start, and the only
        // blocking wait anywhere in the Listen Mode path.
        bool osc_ok = false;
        const uint32_t deadline = millis() + kOscStableTimeoutMs;
        do {
            uint8_t aux = 0;
            if (readRegisterRaw(kRegAuxDisplay, &aux) &&
                ((aux & kAuxDisplayOscOk) != 0U)) {
                osc_ok = true;
                break;
            }
        } while (static_cast<int32_t>(millis() - deadline) < 0);
        if (!osc_ok) {
            return false;
        }
    }
    // ([REF] :2684's else branch consumes the pending OSC interrupt so a later
    // wait cannot be satisfied by it. There is no per-source clear to make
    // here: sampleIrqs() reads all four status registers read-and-clear, so
    // the next listen_poll() tick consumes it as a matter of course -- and
    // nothing in this path ever waits on OSC again.)

    // [REF] :2692-2693 -- ALWAYS, and only now that en/rx_en are on and the
    // oscillator is stable.
    return executeCommandRaw(kCmdClearFifo) &&
           executeCommandRaw(kCmdUnmaskReceiveData);
}

// The real RFAL_LM_STATE_POWER_OFF state entry, ported. [REF]
// rfalListenSetState()'s RFAL_LM_STATE_POWER_OFF case (:2634-2671), including
// its conditional tail-call into IDLE (:2662-2664's reSetState loop).
//
// Called from listen_start()'s tail -- which is what rfalListenStart():2501
// does with its own "return rfalListenSetState(RFAL_LM_STATE_POWER_OFF)" --
// and again from listen_poll() whenever the reader's field goes away.
bool listenEnterPowerOff() {
    s_listen_field_on = false;
    // The selection is over, so the ACTIVE state entry must run again next
    // time a reader gets that far. (This function CLEARS d_106_ac_a a few
    // writes below -- the exact bit listenEnterActive() sets -- so leaving the
    // flag true would skip re-setting it and the responder would never see a
    // frame after the next selection.)
    s_lm_active_entered = false;

    if (!changeRegisterBits(kRegOpControl, kOpControlRxEn, kOpControlRxEn) || // [REF] :2635
        !executeCommandRaw(kCmdStop)) {                                      // [REF] :2636
        return false;
    }

    // NFC-A: re-enable the chip's 106 kb/s auto-anticollision and send the PTA
    // engine back to its Sense/Idle entry point. [REF] :2638-2641.
    if (!changeRegisterBits(kRegPassiveTarget, kPtDisable106Ac, 0U) ||
        !executeCommandRaw(kCmdGotoSense)) {
        return false;
    }

    // [REF] :2643 -- normal (non-FeliCa) framing.
    if (!changeRegisterBits(kRegIso14443aNfc, kIso14443aNfcF0, 0U)) {
        return false;
    }

    // [REF] :2644-2654 disables every interrupt and then clears-and-enables a
    // specific set (NFCT, RXS, CRC, ERR1, OSC, ERR2, PAR, EON, EOF + the
    // mode-specific WU_A/WU_A_X/RXE_PTA). Only the "clear" half is ported, and
    // deliberately so: this driver has no IRQ pin and never writes the four
    // IRQ MASK registers at all. [REF] st25r3916Initialize() (st25r3916.cpp:
    // 119-120) states "After reset all interrupts are enabled, so disable them
    // at first" -- this driver's init() issues Set Default and then leaves the
    // mask registers at exactly that all-enabled reset value, so every source
    // [REF] enables here is already enabled, and porting the "disable all"
    // half could only hide status bits from sampleIrqs(). Clearing matters
    // though: a stale EOF latched by the reader path (or by a previous Listen
    // session) would otherwise read as this session's own field loss on the
    // very first listen_poll() tick.
    if (!clearIrqs()) {
        return false;
    }

    // MODE (0x03): target mode, listen-NFCA sub-mode, no auto-response
    // chaining -- [REF] :2658-2660 writes the mdReg value built back in
    // rfalListenStart():2429/2464-2467, and writes it HERE (after the analog
    // config, on every POWER_OFF entry), not as part of the arming block.
    // Uses the full kModeOmMask (not just the om3|om0 bits this call happens
    // to set), matching every other MODE-register write in this file -- the
    // two om encodings this driver ever writes (reader's 0001b, listen's
    // 1001b) both happen to have om1/om2 clear, so a narrower mask would be
    // harmless today, but only the full-mask form is guaranteed not to leave a
    // stray om1/om2 bit behind if that ever changes.
    if (!changeRegisterBits(
            kRegMode,
            static_cast<uint8_t>(kModeTarg | kModeOmMask | kModeNfcArMask),
            static_cast<uint8_t>(kModeTarg | kModeOm3 | kModeOm0 | kModeNfcArOff))) {
        return false;
    }

    // [REF] :2662 -- is a reader's field ALREADY present? If so, RFAL loops
    // straight back into the same switch as RFAL_LM_STATE_IDLE; that loop is
    // this direct call.
    uint8_t aux = 0;
    if (!readRegisterRaw(kRegAuxDisplay, &aux)) {
        return false;
    }
    if ((aux & kAuxDisplayEfdO) != 0U) {
        s_listen_field_on = true;
        return listenEnterIdle();
    }

    // DELIBERATE DEVIATION from [REF] :2666-2669, which clears tx_en, rx_en
    // AND en here -- powering the receiver and the whole analog front end back
    // down while no reader is near. RFAL can afford that because it is woken
    // by the EON interrupt on a real IRQ line; this driver has no IRQ line
    // ([M5]) and instead re-reads AUX_DISPLAY.efd_o on every listen_poll()
    // tick -- and efd_o is produced by the External Field Detector, which is
    // part of what clearing en would switch off. So the "armed but no field
    // yet" state this driver leaves the chip in is: en=1, rx_en=1, tx_en=0
    // (a Listen Mode target never drives its own carrier), EFD=automatic,
    // MODE=target/listen-NFCA, PTA parked in pta_st_power_off. The only cost
    // versus RFAL is idle current, which is irrelevant on a mains/battery
    // tablet that is already running a 1280x720 display.
    return true;
}

// ---------------------------------------------------------------------------
// The PICC-side data-exchange responder (2026-08-24 content-emulation
// extension). See this section's SOURCES block for every citation below.
// ---------------------------------------------------------------------------

// The real RFAL_LM_STATE_ACTIVE_A / _ACTIVE_Ax state entry, ported. [REF]
// rfalListenSetState() (rfal_rfst25r3916.cpp:2705-2712).
//
// The load-bearing write is SETTING d_106_ac_a -- the same bit
// listenEnterPowerOff() CLEARS to arm the PTA's autonomous anticollision.
// Once a reader has finished selecting us, that hardware engine must stop
// intercepting frames, so that everything the reader sends next is delivered
// to firmware as an ordinary RXE receive for this file to interpret. Without
// it the PTA would keep treating incoming frames as anticollision traffic and
// the responder below would never see a single command.
//
// NOT ported: [REF] :2707-2711's st25r3916GetInterrupt(PAR|CRC|ERR2|ERR1) and
// st25r3916EnableInterrupts(RXE). The first is redundant here -- listen_poll()
// has just consumed all four status registers via sampleIrqs() (they are
// read-and-clear), which is strictly more than RFAL clears. The second is a
// no-op for the same reason listenEnterPowerOff() already documents at
// length: this driver never writes the four IRQ MASK registers at all, so
// every source is already enabled at Set Default's own reset value.
bool listenEnterActive() {
    return changeRegisterBits(kRegPassiveTarget, kPtDisable106Ac, kPtDisable106Ac);
}

// The real RFAL_LM_STATE_SLEEP_A entry, ported. [REF] rfalListenSleepStart()
// (:2516-2529). This is what a real HLTA/SLP_REQ from the reader means: the
// emulated tag stops answering data commands and parks in HALT, from which
// ONLY a WUPA wakes it again -- a genuinely distinct protocol state from
// "the field went away".
//
// HOW IT COMPOSES WITH THE EXISTING FIELD-LOSS PATH, stated explicitly since
// the two are easy to confuse:
//   * HALT (here) leaves the chip powered, the field still present, and
//     re-ARMS the PTA's autonomous anticollision (d_106_ac_a cleared again)
//     so the hardware itself can answer the reader's next WUPA and walk
//     ready_l1_x/ready_l2_x -> active_x on its own. listen_poll() then sees
//     pta_state back at 0xD and re-runs listenEnterActive(), so a reader that
//     HALTs and re-selects is served again with no software re-arming.
//   * Field loss (listenEnterPowerOff(), unchanged) is the harder reset: chip
//     activity stopped, PTA sent back to Sense/Idle, MODE re-written.
//   * The user-visible ListenState is deliberately NOT downgraded by a HALT.
//     A reader HALTing us right after reading is a NORMAL, successful end to
//     a transaction -- it is what nfca_detect() itself does to every tag it
//     reads -- so reporting "no longer selected" there would replace a true
//     success message with a misleading one. The latch is released only by
//     real field loss, exactly as the baseline already did for kSelected.
//
// NOT ported from [REF]: its trailing rfalIsExtFieldOn() check and
// rfalListenStop()-on-link-loss (:2541-2544). listen_poll()'s own
// efd_o/EOF handling already covers field loss on the very next tick, via a
// path that recovers (POWER_OFF re-entry) instead of tearing the session down.
bool listenEnterSleep() {
    if (!changeRegisterBits(kRegPassiveTarget, kPtDisable106Ac, 0U) || // [REF] :2517
        !executeCommandRaw(kCmdGotoSleep)) {                          // [REF] :2518
        return false;
    }
    // [REF] :2519-2521 -- the same MODE value listenEnterPowerOff() writes.
    if (!changeRegisterBits(
            kRegMode,
            static_cast<uint8_t>(kModeTarg | kModeOmMask | kModeNfcArMask),
            static_cast<uint8_t>(kModeTarg | kModeOm3 | kModeOm0 | kModeNfcArOff))) {
        return false;
    }
    // [REF] :2528-2529
    if (!changeRegisterBits(kRegIso14443aNfc, kIso14443aNfcF0, 0U)) {
        return false;
    }
    return executeCommandRaw(kCmdUnmaskReceiveData);
}

// The Listen Mode transmit half. Deliberately NOT transceive(): that function
// opens with CMD_STOP + CMD_RESET_RXGAIN, and [REF] rfalPrepareTransceive()
// (:1150-1160) branches on exactly this -- "In Passive Listen Mode do not use
// STOP as it stops FDT timer", issuing CMD_CLEAR_FIFO instead. This was
// checked against the vendored source rather than assumed identical to the
// reader path, and it is not identical. Everything after that IS the same
// sequence transceive()'s own TX half already uses: the parity/framing
// programme ([REF] :1190-1209), AGC on ([REF] :1211-1216), the BIT-count
// register pair, the FIFO load, then TRANSMIT_WITH_CRC / WITHOUT_CRC ([REF]
// rfalTransceiveTx() :1341-1350, :1370-1375).
//
// `bits` is a BIT count, not a byte count, because a real T2T NAK is 4 bits
// ([REF-T2T] RFAL_T2T_ACK_NACK_LEN's own "1 byte (4 bits)" annotation) and
// this chip's NUM_TX_BYTES1/2 pair is natively a bit counter ([EH]
// st25r3916SetNumTxBits(), st25r3916.cpp:364-368) -- the same registers
// transceive() already writes, just not always a multiple of 8.
//
// Deliberately does NOT wait for TXE afterwards. RFAL waits because its state
// machine has somewhere to go next; here a wait would have to consume the IRQ
// status registers, and those are read-and-CLEAR -- swallowing the RXE of the
// reader's NEXT command, which arrives within a millisecond or two. The
// servicing loop re-samples them on its own a moment later, which both
// observes the transmit completing and catches that next command.
bool listenTransmit(const uint8_t *data, uint16_t bits, bool with_crc) {
    const uint16_t bytes = static_cast<uint16_t>((bits + 7U) / 8U);
    if (data == nullptr || bytes == 0U || bytes > 255U) {
        return false;
    }
    if (!executeCommandRaw(kCmdClearFifo)) { // [REF] :1159, NOT CMD_STOP
        return false;
    }
    // [REF] :1190-1209: hardware parity in both directions, no NFCIP-1
    // framing. antcl is included in the mask (and cleared) because [DS]
    // Table 27 says it "Must be set to 0 for all other frames and modes",
    // and the reader path's transceive() is what may have last set it.
    if (!changeRegisterBits(kRegIso14443aNfc,
                            static_cast<uint8_t>(kIso14443aNoTxPar | kIso14443aNoRxPar |
                                                 kIso14443aNfcF0 | kIso14443aAntcl),
                            0U)) {
        return false;
    }
    if (!changeRegisterBits(kRegRxConf2, kRxConf2AgcEn, kRxConf2AgcEn)) { // [REF] :1214
        return false;
    }
    // [EH] st25r3916SetNumTxBits(): low byte to NUM_TX_BYTES2 (23h), high byte
    // to NUM_TX_BYTES1 (22h). Identical to transceive()'s own call site.
    if (!writeRegisterRaw(kRegNumTxBytes2, static_cast<uint8_t>(bits & 0xFFU)) ||
        !writeRegisterRaw(kRegNumTxBytes1, static_cast<uint8_t>(bits >> 8))) {
        return false;
    }
    if (!writeFifoRaw(data, static_cast<uint8_t>(bytes))) {
        return false;
    }
    return executeCommandRaw(with_crc ? kCmdTxWithCrc : kCmdTxWithoutCrc);
}

bool listenSendNak() {
    const uint8_t nak = kT2tNakNibble;
    // No CRC: a 4-bit ACK/NAK frame carries none ([REF-T2T]
    // rfalT2TPollerRead()'s own ERR_INCOMPLETE_BYTE + 1-byte/4-bit detection
    // at :130 is what a NAK looks like to a reader).
    return listenTransmit(&nak, kT2tNakBits, /*with_crc=*/false);
}

// Builds the real 16-byte answer to a T2T READ of `block`. Returns false when
// the request is genuinely out of range and must be NAKed instead.
//
// THE END-OF-MEMORY WRAP is not an approximation -- it reproduces the exact
// real behaviour this project already had to compensate for on the READING
// side: [AM] nfc_amiibo.cpp's own donor-cited trim (RFID2.cpp:264-265) exists
// because "a Type 2 tag's MIFARE_Read near the end of memory returns the last
// valid page's data repeated to fill the 16-byte response rather than NAKing
// immediately". An emulated tag that instead NAKed a partially-in-range READ
// would behave differently from the real tag its page image was captured
// from, and would break a reader walking the tag four pages at a time.
bool buildT2tReadResponse(uint8_t block, uint8_t (&resp)[kT2tReadDataLen]) {
    if (s_lm_page_count == 0U || block >= s_lm_page_count) {
        return false;
    }
    for (uint8_t i = 0; i < kT2tPagesPerRead; i++) {
        uint16_t page = static_cast<uint16_t>(block) + i;
        if (page >= s_lm_page_count) {
            page = static_cast<uint16_t>(s_lm_page_count - 1U);
        }
        std::memcpy(&resp[i * kT2tPageLen], s_lm_pages[page], kT2tPageLen);
    }
    return true;
}

enum class LmFrame : uint8_t {
    kNone,      // nothing usable in the FIFO (discarded)
    kServiced,  // a command was interpreted and answered
    kHalted,    // the reader sent HLTA; the PTA is now parked in Sleep/Halt
    kIoError,   // I2C failure
};

// Reads ONE received frame out of the FIFO and answers it. Modelled directly
// on [REF] rfalRunListenModeWorker()'s RFAL_LM_STATE_ACTIVE_A/_Ax RXE branch
// (:2856-2887) for everything up to "here are the raw command bytes"; the
// interpretation and response-building after that are the application's job
// in RFAL's own layering, and are this project's own.
LmFrame listenServiceFrame(uint32_t irqs) {
    // [REF] :2861 st25r3916GetNumFIFOBytes(), i.e. the same FIFO status
    // register pair transceive() already decodes.
    uint8_t st1 = 0;
    uint8_t st2 = 0;
    if (!readRegisterRaw(kRegFifoStatus1, &st1) ||
        !readRegisterRaw(kRegFifoStatus2, &st2)) {
        return LmFrame::kIoError;
    }
    const uint16_t n = static_cast<uint16_t>(
        (static_cast<uint16_t>((st2 & kFifoStatus2ByteHiMask) >> kFifoStatus2ByteHiShift) << 8) |
        st1);

    // [REF] :2863-2881: CRC/ERR1/PAR, or a frame no longer than the CRC
    // itself, is not a command -- discard it with CLEAR_FIFO +
    // UNMASK_RECEIVE_DATA and stay where we are. (RFAL additionally drops
    // back to IDLE/SLEEP here; this driver does not, deliberately: it tracks
    // the PTA's own state register rather than a software mirror of it, so
    // the next listen_poll() tick reads the truth from the hardware instead
    // of guessing. A genuinely lost selection shows up there as pta_state
    // leaving active.)
    const uint32_t rx_errors = kIrqCrc | kIrqErr1 | kIrqPar;
    if ((irqs & rx_errors) != 0U || n <= kCrcLen || n > kLmMaxRxLen) {
        executeCommandRaw(kCmdClearFifo);
        executeCommandRaw(kCmdUnmaskReceiveData);
        return LmFrame::kNone;
    }

    uint8_t buf[kLmMaxRxLen] = {0};
    if (!readFifoRaw(buf, static_cast<uint8_t>(n))) {
        return LmFrame::kIoError;
    }
    // [REF] :2884 `*gRFAL.Lm.rxLen -= RFAL_CRC_LEN` -- the chip verified the
    // CRC in hardware (a mismatch was rejected above) but still leaves the two
    // bytes in the FIFO, exactly as transceive()'s own crc_rx path already
    // documents at length for the reader direction.
    const uint16_t len = static_cast<uint16_t>(n - kCrcLen);

    // --- T2T READ (0x30 + block number) ------------------------------------
    // [REF-T2T] rfal_t2t.cpp:74 RFAL_T2T_CMD_READ, :80-84 rfalT2TReadReq
    // ("T2T 1.0 5.2 and table 11"): the command is exactly {code, blNo}.
    if (len == 2U && buf[0] == kT2tCmdRead) {
        uint8_t resp[kT2tReadDataLen] = {0};
        if (!buildT2tReadResponse(buf[1], resp)) {
            // Out of range (or no page image captured at all) -- the real
            // answer is a NAK. [REF-T2T] :130 cites "T2T 1.0 5.2.1.7 The
            // Reader/Writer SHALL treat a NACK in response to a READ Command
            // as a Protocol Error", i.e. this is the defined way for a Type 2
            // tag to refuse a READ.
            (void)listenSendNak();
            return LmFrame::kServiced;
        }
        // 16 bytes with CRC appended in hardware -- [REF-T2T] rfal_t2t.h:64
        // RFAL_T2T_READ_DATA_LEN, and rfalT2TPollerRead():127 passing
        // RFAL_TXRX_FLAGS_DEFAULT (CRC on) for the reader's own side of the
        // same exchange.
        if (!listenTransmit(resp, static_cast<uint16_t>(kT2tReadDataLen) * 8U,
                            /*with_crc=*/true)) {
            return LmFrame::kIoError;
        }
        s_lm_read_count++;
        if (s_lm_read_count == 1U) {
            Serial.printf("quarky-tab5: [st25r3916] Listen Mode: answered a real "
                          "T2T READ (page %u) with captured content -- a reader "
                          "is genuinely reading this emulated tag\n",
                          (unsigned)buf[1]);
        }
        return LmFrame::kServiced;
    }

    // --- HLTA / SLP_REQ (0x50 0x00) ----------------------------------------
    // kSlpReq is this file's own already-cited constant ([EH] rfal_nfca.cpp:
    // 58-61, "Digital 1.1 6.9.1 & Table 20") -- the very bytes nfca_detect()
    // SENDS as a reader. Answering it correctly is the mirror image.
    if (len == sizeof(kSlpReq) && buf[0] == kSlpReq[0] && buf[1] == kSlpReq[1]) {
        // A real tag acknowledges HLTA by staying SILENT (ISO14443-3 6.4.3,
        // the same rule nfca_detect() relies on from the other side), so
        // nothing is transmitted here -- only the state change.
        Serial.println("quarky-tab5: [st25r3916] Listen Mode: reader sent HLTA "
                       "-- parking the emulated tag in HALT (only a WUPA wakes "
                       "it now)");
        if (!listenEnterSleep()) {
            return LmFrame::kIoError;
        }
        return LmFrame::kHalted;
    }

    // --- Anything else -----------------------------------------------------
    // T2T WRITE (0xA2) and SECTOR SELECT (0xC2) ([REF-T2T] rfal_t2t.cpp:75-76)
    // are deliberately unimplemented -- this is read-only emulation, matching
    // the read-only discipline this project's EMV reader already follows.
    // GET_VERSION is unimplemented for the reason spelled out in full in this
    // section's "DISCLOSED GAP" note (no citable source for its command byte
    // or response layout in any vendored source, and NAKing it is what a real
    // plain MIFARE Ultralight does anyway). RATS (0xE0) likewise: ISO14443-4
    // card emulation is out of scope.
    //
    // All of them get a NAK, and the emulated tag deliberately stays ACTIVE
    // afterwards. A strict MIFARE Ultralight returns to IDLE after NAKing an
    // unsupported command; staying ACTIVE is a deliberate interoperability
    // deviation, disclosed here, so that a reader which probes with an
    // unsupported command first (GET_VERSION being the common real case) can
    // still go straight on to the plain READs that carry the tag's content
    // instead of having to re-run anticollision.
    Serial.printf("quarky-tab5: [st25r3916] Listen Mode: NAKing unsupported "
                  "command 0x%02X (len=%u)\n", buf[0], (unsigned)len);
    (void)listenSendNak();
    return LmFrame::kServiced;
}

// The bounded ACTIVE-state servicing loop. See kLmServiceBudgetMs's own
// comment for why this is a loop rather than one frame per listen_poll()
// tick. Returns false only on a real I2C failure.
bool listenServiceActive(uint32_t irqs) {
    if (!s_lm_active_entered) {
        if (!listenEnterActive()) {
            return false;
        }
        s_lm_active_entered = true;
    }

    const uint32_t budget_end = millis() + kLmServiceBudgetMs;
    uint32_t quiet_since = millis();
    uint32_t acc = irqs;

    for (;;) {
        if ((acc & kIrqEof) != 0U) {
            // The reader's field went away mid-exchange. Leave immediately and
            // let listen_poll()'s existing, unchanged field-loss path run on
            // the next tick -- it is the one that re-enters POWER_OFF.
            return true;
        }
        if ((acc & kIrqRxe) != 0U) {
            const LmFrame r = listenServiceFrame(acc);
            if (r == LmFrame::kIoError) {
                return false;
            }
            if (r == LmFrame::kHalted) {
                // The PTA is parked in Sleep/Halt and its autonomous
                // anticollision is re-armed; the ACTIVE entry must run again
                // if the reader wakes us with a WUPA.
                s_lm_active_entered = false;
                return true;
            }
            quiet_since = millis();
        }

        if (static_cast<int32_t>(millis() - budget_end) >= 0) {
            return true;
        }
        if (static_cast<int32_t>(millis() - (quiet_since + kLmServiceIdleMs)) >= 0) {
            return true;
        }

        yield(); // ~0.45 ms of I2C per iteration; keep the RTOS/WDT happy
                 // anyway, exactly as waitIrqs() already does.
        acc = 0;
        if (!sampleIrqs(&acc)) {
            return false;
        }
    }
}

} // namespace

bool listen_start(const ListenConfig &cfg) {
    if (cfg.uid_len != 4U && cfg.uid_len != 7U) {
        return false; // PT memory has no 10-byte/triple-cascade encoding
    }
    if (!init()) {
        return false;
    }

    // Listen Mode and the reader path share REG_MODE/REG_PASSIVE_TARGET and
    // cannot run concurrently -- end whichever was open first, same
    // "tear down before arming" discipline nfca_poller_begin() itself follows
    // by calling field_on()/init() unconditionally at its own start.
    listen_stop();
    if (s_nfca_ready) {
        nfca_poller_end();
    }

    // Oscillator and regulators on, FIRST -- before any of the arming writes
    // below. [REF] does this once at init time (rfalInitialize() ->
    // st25r3916Initialize() -> st25r3916OscOn(), st25r3916.cpp:109-110), so by
    // the time its rfalListenStart() runs, en has always been set for a long
    // while and the PT memory it then writes is being written to a chip whose
    // regulators are up. This driver's init() deliberately does NOT start the
    // oscillator (field_on() does, for the reader path, and Listen Mode never
    // calls field_on()), so the same step has to happen here -- and it has to
    // happen before the PT-memory write, not after it, so that the arming
    // writes land on a chip in the same powered state RFAL's do. It is also a
    // precondition for reading a meaningful External Field Detector bit at
    // all. Same osc_ok polling substitution field_on() already documents.
    //
    // Note what is NOT set: tx_en. A Listen Mode target answers by
    // load-modulating the READER's carrier (the PTA engine does this itself),
    // never by generating its own field, and [REF] rfalListenSetState()'s
    // IDLE case (:2675) likewise sets only en|rx_en.
    uint8_t op = 0;
    if (!readRegisterRaw(kRegOpControl, &op)) {
        return false;
    }
    if ((op & kOpControlEn) == 0U) {
        if (!writeRegisterRaw(kRegOpControl, static_cast<uint8_t>(op | kOpControlEn))) {
            return false;
        }
        bool osc_ok = false;
        const uint32_t deadline = millis() + kOscStableTimeoutMs;
        do {
            uint8_t aux = 0;
            if (readRegisterRaw(kRegAuxDisplay, &aux) && ((aux & kAuxDisplayOscOk) != 0U)) {
                osc_ok = true;
                break;
            }
        } while (static_cast<int32_t>(millis() - deadline) < 0);
        if (!osc_ok) {
            Serial.println("quarky-tab5: [st25r3916] Listen Mode arm failed: "
                           "oscillator never reported osc_ok");
            return false;
        }
    }

    // External Field Detector -> automatic. [REF] rfalInitialize():110-112
    // ("Enable External Field Detector as: Automatics"). Without this the
    // detector stays at its post-Set-Default off state and AUX_DISPLAY.efd_o
    // -- the one bit that tells this driver a reader is present -- is dead,
    // which was half of the 2026-08-24 bug documented in this section's header.
    // listen_stop() puts it back to off, so the reader path keeps running in
    // exactly the configuration it was verified in.
    if (!changeRegisterBits(kRegOpControl, kOpControlEnFdMask, kOpControlEnFdAutoEfd)) {
        return false;
    }

    // AUX (0x0A): NFCID length select. [REF] rfalListenStart():2439/2443.
    //
    // no_crc_rx is cleared in the same write (2026-08-24, with the
    // content-emulation extension). It is not part of RFAL's own
    // rfalListenStart() because RFAL never sets it outside the two reader-side
    // short-frame/anticollision exchanges that need it -- but THIS driver's
    // transceive() sets and clears it per call, so whichever reader-path
    // exchange ran last decides what Listen Mode inherits. It must be 0 here:
    // the responder below relies on the chip verifying the received CRC in
    // hardware and reporting a mismatch via I_crc, which is exactly what
    // [REF] rfalRunListenModeWorker()'s ACTIVE_A branch (:2857-2870) checks
    // before trusting a frame. Harmless for the anticollision/SELECT sequence
    // that was verified against two real readers today (the PTA hardware
    // handles those frames itself and [DS] Table 36 Note 1 applies
    // receive-without-CRC automatically for them regardless).
    if (!changeRegisterBits(kRegAux,
                            static_cast<uint8_t>(kAuxNfcIdMask | kAuxNoCrcRx),
                            (cfg.uid_len == 4U) ? kAuxNfcId4Bytes : kAuxNfcId7Bytes)) {
        return false;
    }

    // Copy the real captured page image (if any) into the driver's own
    // storage. Clamped rather than rejected: a caller with a larger image
    // still gets the pages that fit, which is strictly better than refusing to
    // emulate at all, and kListenMaxPages already covers NTAG216, the largest
    // tag in the family this responds for.
    s_lm_page_count = 0;
    s_lm_read_count = 0;
    s_lm_active_entered = false;
    std::memset(s_lm_pages, 0, sizeof(s_lm_pages));
    if (cfg.pages != nullptr && cfg.page_count > 0U) {
        s_lm_page_count = (cfg.page_count > kListenMaxPages) ? kListenMaxPages
                                                             : cfg.page_count;
        std::memcpy(s_lm_pages, cfg.pages,
                    static_cast<size_t>(s_lm_page_count) * kListenPageLen);
    }

    // Build and write the 15-byte PT_A memory block. [REF] rfalListenStart():
    // 2435-2460 -- NFCID triple, then SENS_RES, then 3x SEL_RES with the
    // INCOMPLETE bit (0x04) set on byte 0 only when uid_len==7 (signalling
    // "another cascade level follows" during CL1 anticollision; the final
    // SELECT response, bytes 1/2, never carries it).
    uint8_t pt_mem[kPtMemALen] = {0};
    for (uint8_t i = 0; i < kNfcidTripleLen; i++) {
        pt_mem[i] = (i < cfg.uid_len) ? cfg.uid[i] : 0U;
    }
    pt_mem[10] = cfg.atqa[0];
    pt_mem[11] = cfg.atqa[1];
    pt_mem[12] = (cfg.uid_len == 4U) ? static_cast<uint8_t>(cfg.sak & ~kSelResIncomplete)
                                     : static_cast<uint8_t>(cfg.sak | kSelResIncomplete);
    pt_mem[13] = static_cast<uint8_t>(cfg.sak & ~kSelResIncomplete);
    pt_mem[14] = static_cast<uint8_t>(cfg.sak & ~kSelResIncomplete);
    // DIAG (2026-08-24, real-hardware Listen Mode debugging): the exact 15
    // bytes about to be armed, before the write that puts them on the chip.
    Serial.printf("quarky-tab5: [st25r3916] DIAG listen_start: uid_len=%u "
                  "cfg.uid=%02X:%02X:%02X:%02X:%02X:%02X:%02X cfg.atqa=%02X%02X "
                  "cfg.sak=%02X pt_mem=%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X\n",
                  (unsigned)cfg.uid_len, cfg.uid[0], cfg.uid[1], cfg.uid[2],
                  cfg.uid[3], cfg.uid[4], cfg.uid[5], cfg.uid[6], cfg.atqa[0],
                  cfg.atqa[1], cfg.sak, pt_mem[0], pt_mem[1], pt_mem[2],
                  pt_mem[3], pt_mem[4], pt_mem[5], pt_mem[6], pt_mem[7],
                  pt_mem[8], pt_mem[9], pt_mem[10], pt_mem[11], pt_mem[12],
                  pt_mem[13], pt_mem[14]);
    if (!writePtMemA(pt_mem)) {
        return false;
    }

    // PASSIVE_TARGET (0x08): arm 106 kb/s autonomous anticollision, leave
    // 212/424 and AP2P disabled (this driver is NFC-A 106 kb/s only, matching
    // the reader path's own single-bitrate scope). [REF] rfalListenStart():
    // 2431-2433/2462-2467/2482-2486.
    if (!writeRegisterRaw(kRegPassiveTarget, kPtArmed)) {
        return false;
    }

    // (The MODE register's target/listen-NFCA bits are NOT written here.
    // [REF] rfalListenStart() only BUILDS that value at :2429/2464-2467 and
    // leaves the actual register write to rfalListenSetState()'s
    // RFAL_LM_STATE_POWER_OFF case (:2658-2660), i.e. after the analog config
    // below and re-applied on every POWER_OFF entry. listenEnterPowerOff()
    // does it, at the same point in the sequence.)

    // ISO14443A_NFC (0x05): normal parity handling, not FeliCa framing.
    // [REF] rfalListenStart():2494-2497.
    if (!changeRegisterBits(kRegIso14443aNfc,
                            static_cast<uint8_t>(kIso14443aNoTxPar | kIso14443aNoRxPar | kIso14443aNfcF0),
                            0U)) {
        return false;
    }

    // TIMER_EMV_CONTROL / MASK_RX_TIMER: Guard Time, per this section's
    // SOURCES block arithmetic (RFAL_LM_GT=1356 1/fc -> 2 in 512/fc steps).
    // [REF] rfalListenStart():2488-2492.
    if (!changeRegisterBits(kRegTimerEmvCtrl, kTimerEmvGptcMask, kTimerEmvGptcNoTrigger) ||
        !changeRegisterBits(kRegTimerEmvCtrl, kTimerEmvMrtStep512, kTimerEmvMrtStep512) ||
        !writeRegisterRaw(kRegMaskRxTimer, 2U)) {
        return false;
    }

    if (!apply_listen_config()) {
        return false;
    }

    // The real state entry, and the reason this function no longer stops at
    // "the registers are armed": [REF] rfalListenStart()'s own last statement
    // is "return rfalListenSetState(RFAL_LM_STATE_POWER_OFF)" (:2501).
    // listenEnterPowerOff() also writes MODE and, if a reader's field is
    // already present, runs straight on into the IDLE entry -- exactly as
    // [REF] :2662-2664's reSetState loop does.
    if (!listenEnterPowerOff()) {
        return false;
    }

    s_listen_armed = true;
    s_listen_state = ListenState::kIdle;
    Serial.printf("quarky-tab5: [st25r3916] Listen Mode armed "
                  "(NFC-A, 106 kb/s, hardware PTA auto-anticollision); "
                  "%u pages of real captured content to answer T2T READs with%s; "
                  "external field %s\n",
                  (unsigned)s_lm_page_count,
                  (s_lm_page_count == 0U)
                      ? " (UID/SAK/ATQA-only emulation -- every data command "
                        "will be NAKed; re-scan and re-save this tag on the NFC "
                        "unit to capture its pages)"
                      : "",
                  s_listen_field_on ? "already present -- entered IDLE"
                                    : "not present yet -- parked in POWER_OFF, "
                                      "listen_poll() will enter IDLE when it appears");
    return true;
}

ListenState listen_poll() {
    if (!s_listen_armed) {
        return ListenState::kNotArmed;
    }

    uint32_t irqs = 0;
    if (!sampleIrqs(&irqs)) {
        s_listen_state = ListenState::kHardwareError;
        return s_listen_state;
    }
    uint8_t pts = 0;
    if (!readRegisterRaw(kRegPassiveTargetStatus, &pts)) {
        s_listen_state = ListenState::kHardwareError;
        return s_listen_state;
    }
    // The third and last register read of a tick: AUX_DISPLAY, for efd_o --
    // [REF] rfalIsExtFieldOn()/st25r3916IsExtFieldOn() (st25r3916.h:154). This
    // is what tells us a reader has arrived, since this unit has no IRQ pin to
    // deliver [REF]'s EON interrupt.
    uint8_t aux = 0;
    if (!readRegisterRaw(kRegAuxDisplay, &aux)) {
        s_listen_state = ListenState::kHardwareError;
        return s_listen_state;
    }
    const bool field_on = ((aux & kAuxDisplayEfdO) != 0U);

    // DIAG (2026-08-24, real-hardware Listen Mode debugging): log the raw
    // PASSIVE_TARGET_STATUS byte, the IRQ word and the external-field-detector
    // bit only when one of them changes, so a real external reader's attempt is
    // fully visible without flooding the log on every poll() tick while idle.
    // pta_state<3:0> values: power_off=0, idle=1, ready_l1=2, ready_l2=3,
    // active=5, halt=9, ready_l1_x=0xA, ready_l2_x=0xB, active_x=0xD (this
    // file's own kPtaState* constants and [REF]-cited table above). A healthy
    // reader presentation should now walk 0 -> 1 -> 2/3 -> 5, where before the
    // 2026-08-24 fix it sat at 0 forever.
    static uint8_t s_last_pts = 0xFFU;
    static uint32_t s_last_irqs = 0xFFFFFFFFU;
    static int8_t s_last_field = -1;
    if (pts != s_last_pts || irqs != s_last_irqs ||
        s_last_field != static_cast<int8_t>(field_on ? 1 : 0)) {
        Serial.printf("quarky-tab5: [st25r3916] DIAG listen_poll: pts=0x%02X "
                      "(pta_state=%u) irqs=0x%08lX aux=0x%02X (efd_o=%u) lm=%s\n",
                      pts, (unsigned)(pts & kPtaStateMask), (unsigned long)irqs,
                      aux, (unsigned)(field_on ? 1U : 0U),
                      s_listen_field_on ? "IDLE+" : "POWER_OFF");
        s_last_pts = pts;
        s_last_irqs = irqs;
        s_last_field = static_cast<int8_t>(field_on ? 1 : 0);
    }

    if (!s_listen_field_on) {
        // POWER_OFF equivalent: nothing to do until a reader's field shows up,
        // and then the whole IDLE entry has to run. [REF]
        // rfalRunListenModeWorker()'s RFAL_LM_STATE_POWER_OFF case
        // (:2747-2757) does exactly this, waking on the EON interrupt where
        // this polls efd_o. Skipping this transition -- assuming the arming
        // writes alone were enough -- was the other half of the 2026-08-24 bug.
        if (!field_on) {
            return s_listen_state; // still kIdle: "armed, waiting"
        }
        s_listen_field_on = true;
        if (!listenEnterIdle()) {
            s_listen_state = ListenState::kHardwareError;
            return s_listen_state;
        }
        s_listen_state = ListenState::kIdle;
        return s_listen_state;
    }

    if (!field_on || (irqs & kIrqEof) != 0U) {
        // The reader's field went away -- back to POWER_OFF, which re-arms the
        // PTA engine (clear d_106_ac_a + GOTO_SENSE) and, if the field is in
        // fact already back, re-enters IDLE by itself. [REF]
        // rfalRunListenModeWorker() drops to RFAL_LM_STATE_POWER_OFF on EOF
        // from every one of its own states (:2781-2782, :2826-2827,
        // :2857-2858). Both signals are checked: efd_o is a level (correct
        // after the fact, even if the EOF edge was consumed by an earlier
        // sampleIrqs() call), while EOF is the edge (correct even if the
        // reader dipped its field entirely between two polling ticks).
        if (!listenEnterPowerOff()) {
            s_listen_state = ListenState::kHardwareError;
            return s_listen_state;
        }
        s_listen_state = ListenState::kIdle;
        return s_listen_state;
    }

    const uint8_t state = static_cast<uint8_t>(pts & kPtaStateMask);
    if (state == kPtaStActive || state == kPtaStActiveX) {
        // Latched: once a reader has SELECTed us, keep reporting at least
        // kSelected (rather than flipping back to kIdle on, e.g., a HALT the
        // reader sends right after reading) until the field genuinely
        // disappears (handled above). That is real user-visible information
        // worth keeping on screen.
        if (s_listen_state != ListenState::kDataRead) {
            s_listen_state = ListenState::kSelected;
        }

        // The 2026-08-24 content-emulation extension: this is where the
        // baseline used to simply stop. A reader that has SELECTed us is
        // about to send real commands, and answering them is the whole
        // difference between "our UID was accepted" and "a reader actually
        // read this tag". Bounded -- see kLmServiceBudgetMs.
        if (!listenServiceActive(irqs)) {
            s_listen_state = ListenState::kHardwareError;
            return s_listen_state;
        }
        if (s_lm_read_count > 0U) {
            // Latched for the same reason kSelected is, and released by the
            // same event (real field loss, handled above).
            s_listen_state = ListenState::kDataRead;
        }
    } else {
        // Not selected right now. If the PTA left the active state without
        // going through our own HALT handler (the reader walked away
        // mid-transaction, or re-ran anticollision), the ACTIVE state entry
        // must run again next time it gets there.
        s_lm_active_entered = false;
        if (s_listen_state != ListenState::kSelected &&
            s_listen_state != ListenState::kDataRead) {
            s_listen_state = ListenState::kIdle;
        }
    }
    return s_listen_state;
}

ListenState listen_get_state() {
    return s_listen_state;
}

uint32_t listen_get_read_count() {
    return s_lm_read_count;
}

void listen_stop() {
    if (!s_listen_armed) {
        return;
    }
    s_listen_armed = false;
    s_listen_state = ListenState::kNotArmed;
    s_listen_field_on = false;
    s_lm_active_entered = false;
    s_lm_page_count = 0;
    // s_lm_read_count is deliberately NOT zeroed here: listen_start() zeroes
    // it, and leaving it alone lets a caller read the final count back after
    // tearing the session down.

    executeCommandRaw(kCmdStop); // [DS] Table 13 C2h: stop all activities

    // Restore REG_MODE's targ bit to 0 (initiator) and clear the om/ar bits
    // Listen Mode set -- nfca_poller_begin()'s own apply_nfca_config() only
    // ORs in its om bits via kModeOmMask and never touches targ, so leaving
    // targ=1 here would silently leave the NEXT reader-path session running
    // in target mode instead of initiator mode.
    changeRegisterBits(kRegMode,
                       static_cast<uint8_t>(kModeTarg | kModeOmMask | kModeNfcArMask), 0U);
    // Restore PASSIVE_TARGET to fully disabled, matching [REF]
    // rfalListenStop() (:2571-2574).
    writeRegisterRaw(kRegPassiveTarget, kPtAllDisabled);

    // Same "leave the oscillator running, only clear rx_en" policy field_off()
    // already documents -- tx_en was never set for Listen Mode in the first
    // place, so only rx_en needs clearing here.
    //
    // The External Field Detector listen_start() switched to automatic goes
    // back to off in the same write. [REF] never does this (rfalInitialize()
    // enables auto EFD once and rfalListenStop() leaves it alone, so RFAL runs
    // its READER path with the detector on too, which is proof the reader path
    // tolerates it). This driver restores it anyway: the NFC-A reader path
    // above -- nfca_detect()/iso14443_4_activate()/apdu_transceive() -- is
    // real-hardware-verified against real payment cards in exactly the
    // EFD-off configuration, and a tag-emulation session must not quietly
    // change the register state the next reader session starts from.
    uint8_t op = 0;
    if (readRegisterRaw(kRegOpControl, &op)) {
        writeRegisterRaw(kRegOpControl,
                         static_cast<uint8_t>((op & ~kOpControlRxEn &
                                               ~kOpControlEnFdMask) |
                                              kOpControlEnFdOff));
    }
}

} // namespace St25r3916
