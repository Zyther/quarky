#include "st25r3916_driver.h"
#include "../../hal/nfc_pn532.h"
#include "../../../boards/tab5/pins_config.h"
#include <Wire.h>
#include <Arduino.h>

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
// The substitute is the one the datasheet itself sanctions: the IRQ *status*
// registers are read-and-clear registers on the I2C bus ([DS] Sec 4.5
// "Interrupt handling" / Tables 42-45, addresses 1Ah-1Dh), and the library's
// own st25r3916ClearInterrupts() ([EH] st25r3916_interrupt.cpp:225-233) reads
// exactly those four registers with a plain multi-register read and no GPIO
// involvement. Polling them IS reading the interrupt state; the pin is only an
// optimisation that tells you when it would be worth reading.
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
//        - Tables 62/63/64/65 "Main / Timer and NFC / Error and wake-up /
//          Passive target interrupt register" (1Ah/1Bh/1Ch/1Dh, p.97-100),
//          all Type: R with the footnote "After register has been read, its
//          content is set to 0". Bit positions used below:
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
//        - rfal_rfst25r3916_analogConfigTbl.h:263-315,384-387 -- ST's own
//          analog-configuration table; the values applied by
//          apply_nfca_analog_config() below are copied from the CHIP_INIT,
//          CHIP_POLL_COMMON, NFC-A Rx common, NFC-A Tx 106 and NFC-A Rx 106
//          entries, verbatim
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
constexpr uint8_t kRegBAuxMod      = 0x28U;
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
constexpr uint32_t kIrqTxe  = 0x00000008U; // 1Ah bit 3
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
constexpr uint8_t  kNrtSteps64fc =
    static_cast<uint8_t>((kNfcaFwt1fc + kFwtAdjust1fc) / 64U); // 35 -> ~165 us
static_assert(kNrtSteps64fc != 0,
              "an NRT of zero means 'timer not started' ([DS] Table 49) -- the "
              "no-response timeout would never fire and every miss would hang "
              "until the software deadline instead");

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
// are read-and-clear ([DS] Tables 62-65's shared footnote), so one 4-byte
// burst read both samples and consumes the pending interrupt state -- which is
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

// `short_cmd` is 0 for an ordinary FIFO-sourced frame, or the C6h/C7h direct
// command for REQA/WUPA. Those two are 7-bit short frames the chip generates
// itself ([DS] Table 13) and cannot be expressed as FIFO bytes, so `tx`/
// `tx_len` are ignored when one is given.
Xfer transceive(uint8_t short_cmd,
                const uint8_t *tx, uint8_t tx_len, bool crc_tx,
                bool antcl, bool crc_rx,
                uint8_t *rx, uint8_t rx_cap, uint8_t *rx_len,
                uint32_t deadline_ms) {
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
    if (!changeRegisterBits(kRegTimerEmvCtrl,
                            static_cast<uint8_t>(kTimerEmvMrtStep | kTimerEmvNrtStep), 0U) ||
        !writeRegisterRaw(kRegMaskRxTimer, kMrtSteps64fc) ||
        !writeRegisterRaw(kRegNrt1, 0U) ||
        !writeRegisterRaw(kRegNrt2, kNrtSteps64fc)) {
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
    if (rx_len != nullptr) {
        *rx_len = static_cast<uint8_t>(n);
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
    // carrier is up with the power-up (NFCIP-1) modulation settings is
    // harmless -- nothing is being addressed yet, and the NFC-A guard time
    // below starts after the configuration lands.
    if (!field_on()) {
        return false;
    }
    if (!apply_nfca_config()) {
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

NfcaResult nfca_detect(Iso14443aTag *out) {
    if (out == nullptr) {
        return NfcaResult::kProtocolError;
    }
    if (!s_nfca_ready) {
        return NfcaResult::kHardwareError;
    }

    const uint32_t deadline = millis() + kDetectBudgetMs;

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
        default:                return NfcaResult::kProtocolError;
    }
    if (atqa_len < 2) {
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

        uint8_t sdd_req[2] = {sel_cmd, kNvbAnticollision};
        uint8_t sdd_res[8] = {0};
        uint8_t sdd_len = 0;
        switch (transceive(/*short_cmd=*/0U, sdd_req, sizeof(sdd_req), /*crc_tx=*/false,
                           /*antcl=*/true, /*crc_rx=*/false,
                           sdd_res, sizeof(sdd_res), &sdd_len, deadline)) {
            case Xfer::kOk:         break;
            case Xfer::kCollision:  return NfcaResult::kCollision;
            case Xfer::kIo:         return NfcaResult::kHardwareError;
            case Xfer::kNoResponse: // a tag answered WUPA and then went silent
            default:                return NfcaResult::kProtocolError;
        }
        if (sdd_len != kSddResLen) {
            return NfcaResult::kProtocolError;
        }
        // BCC check, [EH] rfal_nfca.cpp:332 / rfalNfcaCalculateBcc().
        const uint8_t bcc = static_cast<uint8_t>(sdd_res[0] ^ sdd_res[1] ^
                                                 sdd_res[2] ^ sdd_res[3]);
        if (bcc != sdd_res[4]) {
            return NfcaResult::kProtocolError;
        }

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
            default:                return NfcaResult::kProtocolError;
        }
        // SEL_RES is one SAK byte. The chip VERIFIES the CRC in hardware (a
        // mismatch would have raised I_crc and been rejected above) but still
        // places the two CRC bytes in the FIFO -- which is why [EH]'s
        // rfalTransceiveRx() subtracts RFAL_CRC_LEN in software
        // (rfal_rfst25r3916.cpp:1735-1741). Expect 3; accept 1 rather than
        // fail if this silicon strips them, and log the surprise either way.
        if (sel_len < kSakLen) {
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
                return NfcaResult::kProtocolError;
            }
            continue;
        }
        if (!append_uid(out, &sdd_res[0], 4)) {
            return NfcaResult::kProtocolError;
        }

        // --- 3. SLP_REQ (HLTA): park the tag in HALT --------------------
        // Without this the tag stays ACTIVE and would ignore the WUPA that
        // starts the next detection pass, so a card left sitting on the
        // antenna could be read exactly once. The tag acknowledges by NOT
        // responding (ISO14443-3 6.4.3), so the transceive outcome is
        // deliberately discarded -- same as [EH] rfalNfcaPollerSleep().
        uint8_t slp_rx[4] = {0};
        uint8_t slp_len = 0;
        (void)transceive(/*short_cmd=*/0U, kSlpReq, sizeof(kSlpReq), /*crc_tx=*/true,
                         /*antcl=*/false, /*crc_rx=*/true,
                         slp_rx, sizeof(slp_rx), &slp_len, millis() + 2U);

        return NfcaResult::kFound;
    }

    // Three cascade levels all reported "more to come" -- ISO14443-3 has no
    // fourth level, so the tag is not following the standard.
    return NfcaResult::kProtocolError;
}

} // namespace St25r3916
