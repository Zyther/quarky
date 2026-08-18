#include <Arduino.h>
#include <lvgl.h>
#include "hal/display_tab5.h"
#include "hal/touch_tab5.h"
#include "hal/hosted_link.h"
#include "hal/radio_esp_hosted.h"
#include "hal/storage_sd.h"
#include "hal/c2link_wifi.h"
#include "hal/c2link_ble.h"
#include "hal/nfc_pn532.h"
#include "hal/rf433_gpio.h"
#include "ui/lvgl_port.h"
#include "ui/shell.h"
#include "ui/screen_stack.h"
#include "ui/devices_panel.h"
#include "ui/pairing_screen.h"
#include "features/ping_feature.h"
#include "features/wifi/wifi_scan.h"
#include "features/wifi/wifi_spectrum.h"
#include "features/wifi/wifi_pmkid.h"
#include "features/wifi/wifi_connect.h"
#include "features/wifi/wifi_evil_portal.h"
#include "features/ble/ble_scan.h"
#include "features/ble/ble_spam.h"
#include "features/ble/ble_finder.h"
#include "features/ble/ble_sniffer.h"
#include "features/ble/ble_clone.h" // Task 9 (2nd Phase 2 plan): scan random-address
                                     // peripherals, set own BLE identity to a
                                     // captured target's MAC, advertise under its name
#include "features/ble/ble_karma.h" // Task 10 (2nd Phase 2 plan): passive scan +
                                     // rotate own BLE identity/name every 2s while
                                     // nearby traffic is present ("air activity"
                                     // gimmick, not true per-target karma)
#include "features/ble/ble_sourapple.h" // Task 11 (2nd Phase 2 plan): rotating
                                         // multi-vendor BLE advertisement flood
                                         // (Apple ProximityPair/Nearby Action/
                                         // AirTag, Samsung Buds/Watch, MS Swift
                                         // Pair, Google Fast Pair), CVE-2023-42941
#include "features/ble/ble_central_spike.h" // Task 1 (2nd Phase 2 plan): spike
                                             // only -- no register_module(), no
                                             // launcher tile; serial-trigger 'c'
#include "features/ble/ble_hid_spike.h" // Task 2 (2nd Phase 2 plan): the HID
                                         // GATT service/advertising/keystroke
                                         // transport Task 15's BLE Bad-KB tile
                                         // is built on; still no
                                         // register_module()/launcher tile of
                                         // its own -- serial-triggers 'h'/'j'
                                         // remain as a headless-verification
                                         // convenience
#include "features/ble/ble_findmy.h" // Task 12 (2nd Phase 2 plan): Apple
                                      // offline-finding advertisement emulator
                                      // -- fake Find My tag, rotates identity
                                      // every 60s
#include "features/ble/ble_gatt_explorer.h" // Task 13 (2nd Phase 2 plan): scan,
                                             // connect (via Task 1's
                                             // BleCentral), enumerate GATT
                                             // services/characteristics
#include "features/ble/ble_flood.h" // Task 14 (2nd Phase 2 plan): rapid
                                     // connection-request flood (via Task 1's
                                     // BleCentral) against the first
                                     // BLE-scanned device
#include "features/ble/ble_bad_kb.h" // Task 15 (2nd Phase 2 plan): BLE Bad-KB
                                      // -- Ducky-script text-entry launcher
                                      // tile built on Task 2's ble_hid_spike
                                      // transport (no GATT service of its
                                      // own; see ble_bad_kb.h)
#include "features/ble/ble_fastpair_exploit.h" // Task 16 (2nd Phase 2 plan):
                                                // Google Fast Pair (0xFE2C)
                                                // KBP-characteristic
                                                // overflow/state-confusion
                                                // write probe, via Task 1's
                                                // BleCentral, against the first
                                                // BLE-scanned device
#include "features/ble/ble_hfp_exploit.h" // Task 17 (2nd Phase 2 plan):
                                           // Handsfree (0x111E) / Handsfree AG
                                           // (0x111F) GATT service-leak
                                           // foothold check, via Task 1's
                                           // BleCentral, against the first
                                           // BLE-scanned device. Named
                                           // "exploit" after the donor; it is
                                           // honestly just a leak check -- see
                                           // ble_hfp_exploit.h
#include "features/ble/ble_whisperpair.h" // Task 18 (2nd Phase 2 plan):
                                           // WhisperPair CVE-2025-36911
                                           // detector -- connect, find Fast
                                           // Pair 0xFE2C + its KBP
                                           // characteristic, subscribe via
                                           // real CCCD discovery, write a real
                                           // ECDH/AES-derived probe outside
                                           // pairing mode and see whether the
                                           // accessory replies. Via Task 1's
                                           // BleCentral, against a target the
                                           // user picks in BleTargetPicker
#include "features/ble/ble_target_picker.h" // 2026-08-17 UX fix: the shared
                                             // "scan -> tap a target -> hand
                                             // off" picker screen the four
                                             // connect-based BLE features
                                             // (Flood / Fast Pair / HFP /
                                             // WhisperPair) now start with,
                                             // replacing their old dependence
                                             // on whatever device the BLE Scan
                                             // screen happened to leave in
                                             // slot 0. No launcher tile of its
                                             // own -- it is a component, not a
                                             // feature -- but it does own a
                                             // screen and a scan, so it needs
                                             // a poll() like any other
#include "features/rf433/rf433_common.h" // Task 1 (Phase 3 plan): interrupt-
                                          // driven 433MHz edge-timing capture.
                                          // Spike only for now -- no
                                          // register_module(), no launcher
                                          // tile; serial-trigger 'r' is its
                                          // only entry point (same shape as
                                          // 'c'/'h' above). Becomes the shared
                                          // receive front-end for the later
                                          // RF433 scan/decode/replay tasks.
#include "features/nfc/st25r3916_driver.h" // Task 2 (Phase 3 plan): real
                                            // register-level driver for the
                                            // PORT.A NFC unit's ST25R3916.
                                            // Spike only for now -- no
                                            // register_module(), no launcher
                                            // tile; serial-trigger 'n' is its
                                            // only entry point (same shape as
                                            // 'c'/'h'/'r' above). Becomes the
                                            // foundation for the later NFC
                                            // tag-protocol tasks.
#include "hal/psk_store.h"
#include "../boards/tab5/pins_config.h"
#include <feature_registry.h>
#include <crypto.h>

DisplayTab5 display;
TouchTab5 touch;
RadioEspHosted radio;
StorageSD storage;
C2LinkWifi c2link_wifi;
C2LinkBle c2link_ble;
FeatureRegistry g_registry; // populated by PingFeature::register_module() (Task 20)

// Task 20: dispatched on receipt of a RESP_TELEMETRY frame from Cardputer-ADV
// over either transport -- see both c2link_*.set_receive_handler(...) calls
// in setup(), below.
void on_c2_receive(const c2proto::Frame &frame) {
    if (frame.type == c2proto::MsgType::RESP_TELEMETRY) {
        char msg[c2proto::kMaxPayload + 1] = {0};
        memcpy(msg, frame.payload, frame.payload_len);
        Serial.printf("quarky-tab5: telemetry received: %s\n", msg);
    }
}

// Task 18: HY2.0 peripheral detection (NFC, RFID2, RF433R/T). See
// hal/nfc_pn532.cpp and boards/tab5/pins_config.h for the real-hardware
// address/pin research behind these constructor arguments.
//
// Task 18's census found NOTHING at all on GPIO 53/54 with a unit plugged in,
// which it correctly suspected was a power problem rather than a wrong
// address. The HY2.0 port-power hotfix (hotfix-hy20-port-report.md) confirmed
// exactly that: PORT.A's 5V rail is gated by EXT_5V_EN on IO-expander 0x43
// P2, which nothing was asserting. With the gate asserted the census returns
// 0x50, so the NFC unit's address is now CONFIRMED on hardware and the 0x24
// PN532 guess is retired.
NfcPN532 nfc_unit(TAB5_NFC_I2C_ADDR);     // 0x50, confirmed (ST25R3916 Unit NFC)
NfcPN532 rfid2_unit(TAB5_RFID2_I2C_ADDR); // 0x28, per docs (WS1850S); no such
                                          // unit was plugged in during the
                                          // hotfix's hardware run, so this
                                          // address remains doc-only.
Rf433Gpio rf433; // Intentionally has NO init() call in setup() as of 2026-08-18
                 // -- GPIO53 is shared with the external I2C SDA line and
                 // claiming it at boot tore down Wire1. See the long comment at
                 // the removal site in setup(). The instance is kept because it
                 // is this board's IRF433 implementation and Task 6 (RF433
                 // transmit/replay) is its real first caller.

#ifdef QUARKY_SERIAL_DEBUG
// Task 1 (Phase 3 plan): drain and print whatever the RF433 edge-capture ISR
// collected. Shared by the 'r' toggle's stop half and the auto-stop safety
// below so both paths print identically.
//
// The drain buffer is `static`, not a local: capture_capacity() is 512
// EdgeSamples (~4KB), and the Arduino main task's stack is not somewhere to
// put that.
//
// What to look for in the output (this is the whole point of the spike):
// a real OOK/ASK fixed-code remote produces deltas clustered into a SMALL
// NUMBER OF DISTINCT BUCKETS -- classically two, a "short" of a few hundred
// microseconds and a "long" at a small integer multiple of it -- and repeats
// the same burst 4-8x per button press. Evenly-spaced chatter with no
// repeating structure is noise on a floating pin, i.e. GPIO53 is NOT the
// receive line.
static void rf433_dump_capture() {
    static Rf433Common::EdgeSample s_dump[512];
    size_t cap = Rf433Common::capture_capacity();
    if (cap > 512) cap = 512;
    size_t n = Rf433Common::capture_read(s_dump, cap);
    Serial.printf("quarky-tab5: [rf433] captured %u edges%s\n", (unsigned)n,
                  n >= cap ? " (buffer FULL -- older edges were overwritten)" : "");
    for (size_t i = 0; i < n; i++) {
        uint32_t delta = (i == 0) ? 0 : (s_dump[i].timestamp_us - s_dump[i - 1].timestamp_us);
        Serial.printf("quarky-tab5: [rf433] %4u  t=%10u us  d=%8u us  ->%d\n",
                      (unsigned)i, (unsigned)s_dump[i].timestamp_us,
                      (unsigned)delta, s_dump[i].level ? 1 : 0);
    }
    Serial.println("quarky-tab5: [rf433] end of capture dump");
}

// Wall-clock start of the current capture, used only by the auto-stop safety
// in loop(). 0 when no capture is running.
static uint32_t s_rf433_capture_started_ms = 0;
// How many captures have already run and finished this boot. Used only to
// make a RE-ARM visually distinct from a first start in the log -- see the
// 'r' handler.
static uint32_t s_rf433_captures_completed = 0;
static constexpr uint32_t kRf433CaptureMaxMs = 20000; // matches the ~10-20s
                                                      // capture window this
                                                      // spike's acceptance
                                                      // test calls for
#endif

// Task 9's WiFi STA smoke test still ships with placeholder credentials.
// Attempting them costs a guaranteed-to-fail 15s connect timeout on every
// single boot, so the test is skipped until someone substitutes a real
// SSID/password here. Keeping the guard (rather than deleting the call) means
// the code path stays exercised the moment real credentials are filled in.
static const char *kTestSsid = "YOUR_TEST_SSID";
static const char *kTestPassword = "YOUR_TEST_PASSWORD";
static bool test_credentials_configured() {
    return strcmp(kTestSsid, "YOUR_TEST_SSID") != 0;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // ---- UI first, radios second -------------------------------------------
    // Deliberate ordering, changed as part of the C6 SDIO hotfix. Radio
    // bring-up talks to a separate ESP32-C6 chip over SDIO and is the single
    // most failure-prone thing in setup(): when its pins were wrong the board
    // crash-looped into the brownout detector and the display never appeared,
    // making every other subsystem un-debuggable. Bringing display/touch/LVGL
    // up first guarantees the screen is alive before anything can go wrong on
    // the radio, so a radio failure degrades to "device works, no comms"
    // instead of "device is a brick". See hosted_link.h and
    // hotfix-c6-sdio-pins-report.md.
    display.init();
    touch.init();
    lvgl_port_init(display, touch);

    PingFeature::register_module(); // makes the "Ping Satellite" tile appear in
                                     // Shell::build's launcher grid (Task 7),
                                     // so this must run before Shell::build below
    WifiScanFeature::register_module(); // Task 3: first WiFi-category tile
                                         // (Category::WIFI), same reason --
                                         // must run before Shell::build below
    WifiSpectrumFeature::register_module(); // Task 5: live per-channel RSSI
                                             // bar chart, same reason --
                                             // must run before Shell::build below
    WifiConnectFeature::register_module(); // Task 3 (2nd Phase 2 plan): thin
                                            // UI wrapper over Phase 1's
                                            // proven connect_wifi(), same
                                            // reason -- must run before
                                            // Shell::build below
    WifiEvilPortalFeature::register_module(); // Task 4 (2nd Phase 2 plan):
                                               // softAP + captive HTTP form,
                                               // same reason -- must run
                                               // before Shell::build below
    // Task 6's WifiPmkidFeature::register_module() is deliberately NOT
    // called. Final whole-branch review finding I3 (2026-08-13): real
    // hardware confirmed promiscuous mode is a hard esp-hosted limitation
    // (see wifi_pmkid.cpp's file-level comment) -- a registered tile would
    // fail on every single tap, and (before this same fix also reordered
    // wifi_pmkid.cpp's start()) was writing a fresh, permanently-empty pcap
    // file to SD before even checking promiscuous-mode availability, so
    // every tap also littered /quarky/captures/wifi/ with junk. The plan's
    // decision to keep the CODE merged (UI, ring buffer, pcap writer,
    // IStorage capture-file methods) as reusable reference for a future
    // native-radio port stands -- it just isn't reachable from the
    // launcher. Re-add this call if/when that port happens.
    BleScanFeature::register_module(); // Task 7: first BLE-category tile
                                        // (Category::BLE), same reason --
                                        // must run before Shell::build below
    BleSpamFeature::register_module(); // Task 8: fake AirPods Continuity
                                        // advertisement, same reason --
                                        // must run before Shell::build below
    BleFinderFeature::register_module(); // Task 6 (2nd Phase 2 plan): AirTag/
                                          // SmartTag/Tile detect + geiger-mode
                                          // finder, same reason -- must run
                                          // before Shell::build below
    BleSnifferFeature::register_module(); // Task 7 (2nd Phase 2 plan): passive
                                           // BLE scan -> CSV export, same
                                           // reason -- must run before
                                           // Shell::build below
    BleCloneFeature::register_module(); // Task 9 (2nd Phase 2 plan): capture +
                                         // replay a random-address target's BLE
                                         // identity, same reason -- must run
                                         // before Shell::build below
    BleKarmaFeature::register_module(); // Task 10 (2nd Phase 2 plan): rotate own
                                         // identity/name while nearby BLE traffic
                                         // is present, same reason -- must run
                                         // before Shell::build below
    BleSourAppleFeature::register_module(); // Task 11 (2nd Phase 2 plan): rotating
                                             // multi-vendor BLE advertisement flood,
                                             // same reason -- must run before
                                             // Shell::build below
    BleFindMyFeature::register_module(); // Task 12 (2nd Phase 2 plan): Apple
                                          // offline-finding advertisement
                                          // emulator, same reason -- must run
                                          // before Shell::build below
    BleGattExplorerFeature::register_module(); // Task 13 (2nd Phase 2 plan):
                                                // GATT service/characteristic
                                                // explorer, same reason --
                                                // must run before Shell::build
                                                // below
    BleFloodFeature::register_module(); // Task 14 (2nd Phase 2 plan): BLE
                                         // connection-request flood, same
                                         // reason -- must run before
                                         // Shell::build below
    BleBadKbFeature::register_module(); // Task 15 (2nd Phase 2 plan): BLE
                                         // Bad-KB Ducky-script typer, same
                                         // reason -- must run before
                                         // Shell::build below. Drives Task 2's
                                         // ble_hid_spike.cpp transport rather
                                         // than owning any GATT service of
                                         // its own -- see ble_bad_kb.h
    BleFastPairExploitFeature::register_module(); // Task 16 (2nd Phase 2 plan):
                                                   // Fast Pair KBP write probe,
                                                   // same reason -- must run
                                                   // before Shell::build below
    BleHfpExploitFeature::register_module(); // Task 17 (2nd Phase 2 plan):
                                              // HFP (0x111E/0x111F) GATT
                                              // service-leak check, same
                                              // reason -- must run before
                                              // Shell::build below
    BleWhisperPairFeature::register_module(); // Task 18 (2nd Phase 2 plan):
                                               // WhisperPair CVE-2025-36911
                                               // detector, same reason -- must
                                               // run before Shell::build below

    lv_obj_t *root = Shell::build(g_registry);
    ScreenStack::push(root);
    Serial.println("quarky-tab5: lvgl ready");

    // Task 10: SD mount + write test. The SD card is on SDMMC slot 0 with
    // silicon-fixed IOMUX pins, entirely disjoint from the C6's slot-1 link
    // (see hal/storage_sd.cpp), so it is exercised independently of, and
    // before, any radio bring-up.
    Serial.println("quarky-tab5: mounting sd card...");
    bool sd_ok = storage.mount() && storage.write_test_file();
    Serial.printf("quarky-tab5: sd mount+write: %s\n", sd_ok ? "OK" : "FAILED");

    // Task 18: HY2.0 peripheral detection (NFC, RFID2, RF433R/T). Detection
    // only -- no read/write/clone/replay logic yet (Phase 3 scope). The NFC
    // and RFID2 units are both I2C, on the EXTERNAL bus (HY2.0 PORT.A, GPIO
    // 53/54) -- a different bus from the internal one display/touch/SD use.
    // The census runs first and unconditionally so a wrong constructor
    // address shows up as an actionable fact in the log, not just a bare
    // "not found".
    nfc_scan_external_i2c_bus();
    nfc_unit.detect("NFC");
    rfid2_unit.detect("RFID2");

    // rf433.init() is DELIBERATELY NOT CALLED HERE. Do not "helpfully" put it
    // back -- doing so reintroduces a real, observed bug.
    //
    // THE INCIDENT (2026-08-18, Phase 3 Task 2 Step 3, first attempt on real
    // hardware). The project owner swapped the NFC unit onto PORT.A. The
    // census three lines above correctly found `0x50 ST25R3916` -- the unit was
    // present and answering. Moments later the 'n' trigger's
    // St25r3916::init() failed 2/2 with
    // `esp32-hal-i2c-ng.c: bus is not initialized`, on the exact bus and
    // address that had just answered.
    //
    // ROOT CAUSE: TAB5_EXTERNAL_I2C_SDA_GPIO, TAB5_RF433T_PIN and
    // TAB5_RF433R_PIN are ALL GPIO53 -- one physical pin, because PORT.A is
    // one physical HY2.0 socket that holds exactly one unit at a time
    // (NFC *or* RFID2 *or* RF433R/T). Rf433Gpio::init() unconditionally does
    // pinMode(GPIO53, ...), and pinMode() does far more than re-route a pin:
    // esp32-hal-gpio.c:161 calls perimanSetPinBus(pin, ESP32_BUS_TYPE_GPIO, ...),
    // which at esp32-hal-periman.c:174-183 invokes the PREVIOUS owner's deinit
    // callback -- here i2cDetachBus -> i2cDeinit() -- tearing down the whole
    // Wire1 I2C peripheral, not just the pin. Hence "bus is not initialized"
    // rather than a NACK.
    //
    // THE FIX: RF433 claims GPIO53 on demand, only when an RF433 feature is
    // actually running -- the same arbitration this project already uses for
    // other exclusive shared hardware (e.g. BLE's single advertising slot,
    // claimed on open and released on close rather than held from boot).
    // Claiming both pin functions at boot regardless of which unit is really
    // plugged in was never right; it only looked harmless because Task 1's
    // receive test uses raw digitalRead()/attachInterrupt() and never touches
    // the I2C peripheral.
    //
    // Nothing needs wiring up in its place today: Rf433Common::capture_start()
    // already does its own pinMode(TAB5_RF433R_PIN, INPUT) every time the 'r'
    // trigger fires (rf433_common.cpp:123), so RF433 receive is unaffected by
    // this removal. Rf433Gpio::init() itself is unchanged and still correct --
    // it simply has no caller until Task 6 (RF433 transmit/replay) is built,
    // at which point that feature calls it when its screen starts
    // transmitting.

    // ---- ESP32-C6 radio co-processor ---------------------------------------
    // One bounded attempt at the shared esp-hosted SDIO link, with the result
    // latched. Every radio subsystem below re-checks it and no-ops if it is
    // down, so a broken/absent/mismatched C6 costs exactly one logged failure
    // rather than an unbounded retry storm.
    if (!hosted_link::begin()) {
        Serial.println("quarky-tab5: radios DISABLED for this boot "
                       "(WiFi + BLE + C2 transports unavailable); "
                       "UI and SD remain functional");
    } else {
        if (test_credentials_configured()) {
            Serial.println("quarky-tab5: connecting wifi via esp-hosted...");
            bool ok = radio.connect_wifi(kTestSsid, kTestPassword);
            Serial.printf("quarky-tab5: wifi connect %s, ip=%u\n",
                          ok ? "OK" : "FAILED", radio.local_ip());
            Serial.printf("quarky-tab5: wifi still connected after sd write: %s\n",
                          radio.is_connected() ? "YES" : "NO");
        } else {
            Serial.println("quarky-tab5: wifi STA connect test skipped "
                           "(placeholder credentials in main.cpp)");
        }

        // Task 11 (amended 2026-08-07, was ESP-NOW -- see task-11-report.md for
        // why that was replaced): C2LinkWifi. Starts the Tab5's self-contained
        // WiFi AP + TCP server; joining as a station and opening a connection
        // is Cardputer-ADV's side (Task 15).
        //
        // Task 20: this used to pass a local all-zero placeholder array here,
        // completely disconnected from Task 12's pairing_screen.cpp (which
        // generates/persists a PSK to NVS via PskStore but never fed it back
        // into these init() calls) -- so a Cardputer-ADV hardcoded with the
        // real, pairing-screen-displayed PSK would fail HMAC verification
        // against Tab5's still-all-zero key and have every frame silently
        // dropped ("bad auth") on both transports. Fixed here by loading (or,
        // on a factory-fresh device with nobody having opened "Pair Satellite"
        // yet, generating+persisting) the same NVS-backed PSK pairing_screen.cpp
        // uses, so both call sites -- boot-time C2 link init and the pairing
        // screen's later display -- always agree on one real provisioned key.
        uint8_t provisioned_psk[16];
        if (!PskStore::load(provisioned_psk)) {
            c2proto::generate_psk(provisioned_psk);
            PskStore::save(provisioned_psk);
            Serial.println("quarky-tab5: generated and persisted new PSK (boot-time)");
        } else {
            Serial.println("quarky-tab5: loaded existing PSK from NVS (boot-time)");
        }

        bool c2_wifi_ok = c2link_wifi.init(provisioned_psk, "Quarky-Tab5-Test", "quarkytest123", 7777);
        Serial.printf("quarky-tab5: c2link_wifi init %s\n", c2_wifi_ok ? "OK" : "FAILED");

        // Task 13: C2LinkBle -- the second C2 transport, used when the WiFi
        // radio is busy with an active feature. Coexists with C2LinkWifi above
        // (both transports are brought up here; feature code picks which one
        // to actually use at runtime, per the foundation spec). Same
        // provisioned PSK as the WiFi transport, per the fix above. A known,
        // already-flagged, deferred concern from Task 11 is that
        // WiFi.mode(WIFI_AP) (above) and BLE together touch the same C6
        // co-processor radio at runtime.
        // Queue the HID keyboard service into the one shared GATT server
        // before c2link_ble.init() starts the NimBLE host task. It MUST
        // happen here, not from any runtime trigger -- NimBLE drains the
        // queued service-def list exactly once, at host startup, and a
        // second hand-rolled ble_gatts_start() later is a use-after-free of
        // the live ATT database (task-2-review.md finding C1;
        // ble_hid_spike.h's header comment has the full mechanism).
        //
        // Unconditional as of Task 15 (2nd Phase 2 plan) -- previously gated
        // behind QUARKY_SERIAL_DEBUG, which was correct while this was Task
        // 2's spike (no launcher tile, no way for a default build to even
        // trigger it). Task 15 promoted it into BleBadKbFeature, a real
        // launcher tile any user can open in every build, so the HID service
        // this feature drives must actually be present in the ATT database
        // unconditionally too -- gating registration behind a debug-only
        // build flag while the tile itself ships in every build would leave
        // BleBadKbFeature's "Send" button permanently non-functional in a
        // normal release build.
        c2link_ble_add_gatt_hook(BleHidSpike::register_service);

        bool c2_ble_ok = c2link_ble.init(provisioned_psk, "Quarky-Tab5");
        Serial.printf("quarky-tab5: c2link_ble init %s\n", c2_ble_ok ? "OK" : "FAILED");

        // Task 20: both transports report RESP_TELEMETRY (and any future
        // RESP_*) frames to the same handler -- it's transport-agnostic.
        c2link_wifi.set_receive_handler(on_c2_receive);
        c2link_ble.set_receive_handler(on_c2_receive);
    }

    // Subscribe the Arduino loop task to the ESP-IDF task watchdog, now that
    // setup() -- which includes documented multi-second blocking stages the
    // watchdog is never fed during (the 15s WiFi STA connect timeout above,
    // C6 SDIO bring-up, SD mount, the HY2.0 I2C census) -- is done. Task
    // review (2026-08-12) on the original placement (top of setup()) found
    // exactly this: arming a 5s panic timer over a path the code itself
    // documents as able to take up to 15s would turn a slow-but-legitimate
    // boot into a reboot loop the moment real WiFi credentials are filled
    // in above, defeating the very "device works, no comms" degradation this
    // file's own radio-bring-up ordering comment (see the top of setup())
    // is designed to guarantee. Moved here, and armed only once loop()'s own
    // per-iteration budget (~50ms, Global Constraint) is what's being
    // watched -- see enableLoopWDT()'s own comment at its definition for the
    // full story of what it protects against.
    enableLoopWDT();

    Serial.println("quarky-tab5: setup complete");
}

void loop() {
    lvgl_port_tick();
    c2link_wifi.poll(); // no-ops when the AP never came up
    c2link_ble.poll();  // drains BLE frames received on the NimBLE host task
    WifiSpectrumFeature::poll(); // no-ops unless the WiFi Spectrum screen is open
    WifiConnectFeature::poll();  // no-ops unless a connect is in flight; drains the
                                  // background connect_task()'s result -- real-hardware
                                  // fix (2026-08-14), see wifi_connect.cpp for why
                                  // connect_wifi() cannot run on this task directly
    WifiPmkidFeature::poll();    // no-ops unless the PMKID capture screen is open;
                                  // drains the promiscuous-mode ring buffer to SD
    BleScanFeature::poll();      // no-ops unless the BLE Scan screen is open;
                                  // pushes gap_scan_event_cb's discoveries to the list
    BleSpamFeature::poll();      // no-ops unless the BLE Spam screen is open;
                                  // rotates the fake AirPods advertisement every 200ms.
                                  // NOTE: while active this STOPS c2link_ble's C2
                                  // advertisement (legacy BLE adv is single-instance,
                                  // system-wide) -- disclosed tradeoff, see ble_spam.cpp
    BleCentralSpike::poll();     // no-ops unless the central-connect spike is in
                                  // flight; only enforces the spike's own timeout so
                                  // a stalled connection can't be left dangling
    BleFinderFeature::poll();    // no-ops unless the BLE Tracker Finder screen is
                                  // open AND locked ('f' trigger below); drives
                                  // update_geiger_ui()'s proximity-tier readout
    BleSnifferFeature::poll();   // no-ops unless the BLE Sniffer screen is open;
                                  // drains gap_scan_event_cb's ring buffer to SD
                                  // (buffered, not written synchronously on the
                                  // NimBLE host task -- see ble_sniffer.cpp's
                                  // file-level comment for why)
    BleCloneFeature::poll();     // no-ops unless the BLE Clone screen is open;
                                  // refreshes the pick-list every 500ms while
                                  // scanning. NOTE: tapping a target calls
                                  // ble_hs_id_set_rnd(), a persistent, host-wide
                                  // identity change -- see ble_clone.cpp's
                                  // clone_target() comment for the cross-feature
                                  // side effect this leaves for ble_hid_spike.cpp
    BleKarmaFeature::poll();     // no-ops unless the BLE Karma screen is open;
                                  // rotates identity/name every 2s while nearby
                                  // BLE traffic is present. NOTE: also calls
                                  // ble_hs_id_set_rnd() repeatedly, each time
                                  // with a fresh random address -- see
                                  // ble_karma.cpp's rotate_identity() comment
                                  // for how this differs from ble_clone.cpp's
                                  // one-time version of the same persistent,
                                  // host-wide identity side effect
    BleSourAppleFeature::poll(); // no-ops unless the Sour Apple screen is open;
                                  // sends one rotating vendor template every
                                  // 200ms. NOTE: also calls ble_hs_id_set_rnd()
                                  // every send, same persistent host-wide
                                  // identity side effect as ble_clone.cpp/
                                  // ble_karma.cpp above
    BleFindMyFeature::poll();    // no-ops unless the Find My Emulator screen
                                  // is open; rotates the fake tag's identity
                                  // and Apple offline-finding advertisement
                                  // every 60s. NOTE: also calls
                                  // ble_hs_id_set_rnd() every rotation, same
                                  // persistent host-wide identity side effect
                                  // as ble_clone.cpp/ble_karma.cpp/
                                  // ble_sourapple.cpp above
    BleGattExplorerFeature::poll(); // no-ops unless the GATT Explorer screen
                                     // is open; drives both the pre-connect
                                     // target-picker list refresh (while
                                     // scanning) and the post-connect
                                     // discovery-log refresh (independent of
                                     // scan state) from buffered NimBLE
                                     // host-task callback output
    BleFloodFeature::poll();     // no-ops unless the BLE Flood screen is
                                  // open; drives the connect/cancel tick
                                  // every 250ms and the attempt counter label
    BleBadKbFeature::poll();     // no-ops unless the BLE Bad-KB screen is
                                  // open; updates the Paired/Disconnected
                                  // status label from BleHidSpike::is_connected(),
                                  // and -- while a script is in flight --
                                  // advances the Ducky-script parser by
                                  // exactly one keystroke-or-bookkeeping
                                  // action per tick (see ble_bad_kb.cpp's
                                  // file comment for why this must not be a
                                  // single blocking call: the same watchdog-
                                  // starvation bug class wifi_connect.cpp
                                  // hit for real)
    BleFastPairExploitFeature::poll(); // no-ops unless the Fast Pair Exploit
                                        // screen is open; renders the status
                                        // line that connect/discovery/write
                                        // callbacks buffered on the NimBLE
                                        // host task -- those callbacks must
                                        // never call lv_* themselves (same
                                        // LV_OS_NONE constraint as
                                        // ble_gatt_explorer.cpp)
    BleHfpExploitFeature::poll();      // no-ops unless the HFP Exploit screen
                                        // is open; renders the status line that
                                        // the connect and the two service
                                        // discoveries buffered on the NimBLE
                                        // host task -- those callbacks must
                                        // never call lv_* themselves (same
                                        // LV_OS_NONE constraint as
                                        // ble_fastpair_exploit.cpp)
    BleWhisperPairFeature::poll();     // no-ops unless the WhisperPair screen
                                        // is open. Does three main-task-only
                                        // jobs the NimBLE host task must not:
                                        // renders the buffered status line,
                                        // enables the Send Probe button once
                                        // the KBP notify subscription is
                                        // confirmed, and expires the
                                        // post-probe notify wait window (which
                                        // is what emits the "target stayed
                                        // silent" verdict)
    BleTargetPicker::poll();           // no-ops unless the shared BLE target
                                        // picker screen is open and still
                                        // scanning; rebuilds the tappable
                                        // device list on the main/LVGL task
                                        // from the buffer its NimBLE host-task
                                        // scan callback fills (same
                                        // host-task-must-not-call-lv_* split
                                        // as every other BLE screen here)

#ifdef QUARKY_SERIAL_DEBUG
    // --- Serial-driven headless-verification aid (intentionally kept, gated) ---
    // Drives the same actions a real touch tap on "Pair Satellite"/"Ping
    // Satellite" would, from Serial input. This is deliberately retained (Task
    // 20): the automated hardware-verification harness has no physical touch
    // access to the device, so this is the only way to exercise send_ping()
    // (and re-run pairing) headlessly for regression checks. It calls the exact
    // same code paths the UI tiles do -- no behavioral divergence from a real
    // tap. 'k' opens the pairing screen (generates/logs the PSK); 'p' calls
    // PingFeature::send_ping().
    //
    // Gated behind QUARKY_SERIAL_DEBUG (off by default -- not in platformio.ini
    // build_flags) per task review: an unconditionally-compiled single-byte
    // trigger on the same UART used for real console/debug traffic risks
    // firing on stray line noise or incidental text in production firmware.
    // Pass `-DQUARKY_SERIAL_DEBUG` for hardware-verification runs via the
    // environment:
    //
    //   PLATFORMIO_BUILD_FLAGS="-DQUARKY_SERIAL_DEBUG" pio run -t upload
    //
    // (This comment used to suggest a `pio run --build-flags=...` option.
    // There is no such option -- `pio run` rejects it; corrected 2026-08-12
    // after hitting it during the WiFi-scan hang investigation.)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'k') {
            Serial.println("quarky-tab5: [debug] opening pairing screen via serial trigger");
            ScreenStack::push(build_pairing_screen());
        } else if (c == 'p') {
            Serial.println("quarky-tab5: [debug] send_ping() via serial trigger");
            PingFeature::send_ping();
        } else if (c == 'b') {
            // Stands in for the menu-bar Back button. Without it a headless
            // run can only ever push screens, which is not how the UI is
            // actually used and which piles up LVGL objects that a real user
            // would have freed on the way back out.
            Serial.println("quarky-tab5: [debug] ScreenStack::pop() via serial trigger");
            ScreenStack::pop();
        } else if (c == 'w') {
            // Same entry point the "WiFi Scan" launcher tile calls. Added for
            // the 2026-08-12 hang investigation: the failure is only reachable
            // by a physical tap otherwise, which the headless
            // hardware-verification harness cannot produce.
            Serial.println("quarky-tab5: [debug] WifiScanFeature::start() via serial trigger");
            WifiScanFeature::start();
        } else if (c == 's') {
            // Same entry point the "WiFi Spectrum" launcher tile calls.
            // Added for the Task 5 real-hardware verification pass (same
            // reasoning as 'w' above): only reachable by a physical tap
            // otherwise.
            Serial.println("quarky-tab5: [debug] WifiSpectrumFeature::start() via serial trigger");
            WifiSpectrumFeature::start();
        } else if (c == 'm') {
            // Same entry point the "WiFi PMKID Capture" launcher tile calls.
            // Added for Task 6's real-hardware verification pass (same
            // reasoning as 'w'/'s' above): only reachable by a physical tap
            // otherwise, and this is the first exercise of promiscuous mode
            // over esp-hosted in this project -- worth being able to trigger
            // headlessly.
            Serial.println("quarky-tab5: [debug] WifiPmkidFeature::start() via serial trigger");
            WifiPmkidFeature::start();
        } else if (c == 'g') {
            // Same entry point the "BLE Scan" launcher tile calls. Added for
            // Task 7's real-hardware verification pass (same reasoning as
            // 'w'/'s'/'m' above): only reachable by a physical tap otherwise,
            // and this is the first exercise of ble_gap_disc() (BLE central/
            // observer role) in this project -- worth being able to trigger
            // headlessly to check for the concurrent scan-while-advertising
            // question this task's brief flags.
            Serial.println("quarky-tab5: [debug] BleScanFeature::start() via serial trigger");
            BleScanFeature::start();
        } else if (c == 'a') {
            // Same entry point the "BLE Spam (AirPods)" launcher tile calls.
            // Added for Task 8's real-hardware verification pass (same
            // reasoning as 'w'/'s'/'m'/'g' above): only reachable by a
            // physical tap otherwise, and this is the first exercise of a
            // second, rotating ble_gap_adv_start() advertisement in this
            // project -- worth being able to trigger headlessly to check
            // that it correctly stops c2link_ble's C2 advertisement (see
            // ble_spam.cpp's disclosed single-advertisement-instance note).
            Serial.println("quarky-tab5: [debug] BleSpamFeature::start() via serial trigger");
            BleSpamFeature::start();
        } else if (c == 'c') {
            // Task 1 of the second Phase 2 plan: the BLE central/
            // client-connect SPIKE. Unlike every other trigger above this one
            // has NO launcher tile at all -- it is a one-shot experiment, not
            // a feature, so serial is its only entry point by design (not
            // just for headless convenience).
            //
            // Connects to the FIRST device the most recent BLE Scan found, so
            // run BLE Scan ('g') first and let it populate. See
            // ble_central_spike.h for exactly what this tests and why a
            // negative result matters.
            Serial.println("quarky-tab5: [debug] BleCentralSpike::run() via serial trigger");
            const uint8_t *addr = BleScanFeature::first_device_addr();
            if (addr) {
                BleCentralSpike::run(addr, BleScanFeature::first_device_addr_type());
            } else {
                Serial.println("quarky-tab5: [debug] no scanned device available -- run BLE Scan ('g') first");
            }
        } else if (c == 'h') {
            // Task 2 of the second Phase 2 plan: the BLE HID / Bad-KB SPIKE.
            // Like 'c' above this has NO launcher tile -- it is a one-shot
            // experiment, not a feature.
            //
            // NOTE, disclosed: this STOPS c2link_ble's C2 advertisement --
            // legacy BLE advertising is single-instance system-wide, same
            // constraint ble_spam.cpp documents. Reboot to get the C2
            // advertisement back. The GATT server itself is NOT disturbed:
            // the HID service was registered at boot by the hook installed in
            // setup() above, not here.
            Serial.println("quarky-tab5: [debug] BleHidSpike::start() via serial trigger");
            BleHidSpike::start();
        } else if (c == 'j') {
            // Second half of the Task 2 spike: send one 'a' keystroke to the
            // paired host. The brief suggested 'k' for this, but 'k' is the
            // pairing-screen trigger (and 'a'/'c'/'g'/'s' -- the other obvious
            // mnemonics for "a", "hid", "keystroke", "send" -- are all taken
            // too), so 'j' it is.
            Serial.println("quarky-tab5: [debug] BleHidSpike::send_test_keystroke() via serial trigger");
            BleHidSpike::send_test_keystroke();
        } else if (c == 'f') {
            // Task 6 (2nd Phase 2 plan): BLE Tracker Finder's geiger-mode
            // lock. As brief-written this feature's target-lock UI is
            // deferred (tap-a-row-to-lock is a documented future addition in
            // ble_finder.cpp) -- without SOME trigger to set s_locked = true,
            // update_geiger_ui() (called every poll() tick) would be
            // permanently unreachable dead code with no way to verify it on
            // real hardware. 'f' (mnemonic: finder-lock) locks onto whichever
            // tracker BLE Tracker Finder's scan most recently added to its
            // list -- open the screen ('BLE Tracker Finder' tile, or drive it
            // manually the same way 'g' drives BLE Scan) and let at least one
            // real tracker show up first, same "run the scan trigger before
            // the dependent trigger" pattern 'c' uses with 'g'.
            Serial.println("quarky-tab5: [debug] BleFinderFeature::lock_last_seen() via serial trigger");
            BleFinderFeature::lock_last_seen();
        } else if (c == 'r') {
            // Task 1 (Phase 3 plan): the RF433 receive-pin SPIKE. Like 'c' and
            // 'h' above it has NO launcher tile -- it is a one-shot experiment
            // (does TAB5_RF433R_PIN=GPIO53 actually carry the RF433R unit's
            // data line?), so serial is its only entry point by design.
            //
            // 'r' is a toggle, per this task's brief: first press starts the
            // interrupt-driven edge capture, second press stops it and dumps
            // every captured edge's inter-edge delta and post-edge level. The
            // letter is free -- k/p/b/w/s/m/g/a/c/h/j/f are all taken above.
            //
            // Procedure: press 'r', press a real 433MHz remote several times
            // over the next 10-20 seconds, press 'r' again (or just wait for
            // the auto-stop below), then read the dump.
            if (Rf433Common::is_capturing()) {
                Serial.println("quarky-tab5: [debug] Rf433Common::capture_stop() via serial trigger");
                Rf433Common::capture_stop();
                s_rf433_capture_started_ms = 0;
                s_rf433_captures_completed++;
                rf433_dump_capture();
            } else {
                // Say loudly when this 'r' is a RE-ARM rather than a first
                // start. Real operator error, 2026-08-18: after the 20s
                // auto-stop had already fired, a second 'r' was sent expecting
                // it to be the "stop" half of the toggle. It was not -- the
                // capture had already stopped, so the toggle read as "start",
                // and the ISR was left armed indefinitely when the session
                // ended. Same keystroke, opposite meaning, and the only
                // difference in the log was one easily-missed line. The module
                // is now hard-bounded against that (Rf433Common's ISR
                // self-disarms), but the operator should still be told.
                if (s_rf433_captures_completed > 0) {
                    Serial.printf("quarky-tab5: [debug] *** 'r' is RE-ARMING capture #%u -- "
                                  "this is a START, not a stop (the previous capture already "
                                  "ended). Send 'r' again to stop it. ***\n",
                                  (unsigned)(s_rf433_captures_completed + 1));
                }
                Serial.println("quarky-tab5: [debug] Rf433Common::capture_start() via serial trigger");
                Rf433Common::capture_start();
                s_rf433_capture_started_ms = millis();
            }
        } else if (c == 'n') {
            // Task 2 (Phase 3 plan): the ST25R3916 chip-ID readback SPIKE.
            // Like 'c'/'h'/'r' above it has NO launcher tile -- it is a
            // one-shot experiment (does the PORT.A NFC unit really answer
            // ST25R3916 register framing, and report ST's documented IC
            // identity?), so serial is its only entry point by design.
            //
            // 'n' (mnemonic: NFC) is free -- k/p/b/w/s/m/g/a/c/h/j/f/r are the
            // thirteen already taken above.
            //
            // *** HARDWARE PRECONDITION ***: the NFC unit -- NOT the RFID2
            // unit and NOT the RF433R receiver Task 1 used -- must be the
            // thing plugged into the Tab5's single HY2.0 PORT.A socket. There
            // is only one such socket. If RF433R is still connected this will
            // report an I2C NACK, which means "wrong unit plugged in", not
            // "the driver is wrong".
            //
            // ACCEPTANCE CRITERION (this task's brief, and it is a specific
            // cited value, not "something non-zero came back"): the IC
            // identity register at 0x3F must have its ic_type field -- bits
            // 7..3, masked with 0xF8 -- equal to 0x28, i.e. 00101b, which
            // DS12484 Rev 3 Table 117 documents as "ST25R3916/7". The low
            // three bits are ic_rev and are silicon-revision dependent (that
            // table's default column shows 010b = "rev 3.1", making 0x2A the
            // whole-byte value to expect on that revision) -- so the revision
            // is LOGGED but is deliberately NOT part of pass/fail.
            Serial.println("quarky-tab5: [debug] St25r3916 chip-ID readback via serial trigger");
            uint8_t id = 0;
            bool init_ok = St25r3916::init();
            bool read_ok = St25r3916::read_chip_id(&id);
            if (!read_ok) {
                Serial.println("quarky-tab5: [st25r3916] *** FAIL: could not read "
                               "IC identity (reg 0x3F) at all -- I2C 0x50 on Wire1 "
                               "did not answer. Is the NFC unit (not RFID2, not "
                               "RF433R) on PORT.A? ***");
            } else {
                uint8_t type = id & St25r3916::kIcIdentityIcTypeMask;
                uint8_t rev = id & St25r3916::kIcIdentityIcRevMask;
                Serial.printf("quarky-tab5: [st25r3916] EXPECTED ic_type = 0x%02X "
                              "(DS12484 Rev 3 Table 117, 00101b = ST25R3916/7); "
                              "whole-byte value on the datasheet's documented "
                              "rev 3.1 silicon would be 0x2A\n",
                              St25r3916::kIcTypeSt25r3916);
                Serial.printf("quarky-tab5: [st25r3916] ACTUAL   reg 0x3F = 0x%02X "
                              "-> ic_type = 0x%02X, ic_rev = %u\n",
                              id, type, (unsigned)rev);
                if (type == St25r3916::kIcTypeSt25r3916) {
                    Serial.println("quarky-tab5: [st25r3916] *** PASS: ic_type matches "
                                   "ST25R3916/7 ***");
                } else if (type == St25r3916::kIcTypeSt25r3916B) {
                    Serial.println("quarky-tab5: [st25r3916] *** PASS (variant): ic_type "
                                   "is ST25R3916B, not the -AQWT M5Stack documents. "
                                   "Same programming model for this driver, but later "
                                   "tasks must add the B part's RC-calibration step. ***");
                } else {
                    Serial.println("quarky-tab5: [st25r3916] *** FAIL: ic_type matches "
                                   "neither ST25R3916 nor ST25R3916B. Something answered "
                                   "at 0x50, but it is not this chip -- do NOT paper over "
                                   "this by relaxing the check. ***");
                }
            }
            Serial.printf("quarky-tab5: [st25r3916] init() returned %s\n",
                          init_ok ? "true" : "false");
        }
    }

    // Task 1 (Phase 3 plan) safety net, NOT in the brief but deliberate: bound
    // how long one capture can stay armed, so a caller who never sends the
    // second 'r' does not leave the ISR running forever. This is the
    // main-task half of the guard only -- it depends on loop() still running,
    // so it is NOT protection against genuine loop() starvation. The real
    // bound against that lives in Rf433Common's ISR, which counts edges and
    // masks its own interrupt source at a hard ceiling with no main-task
    // participation at all. Both exist because they fail differently.
    //
    // Also fires on overrun so that an ISR self-disarm gets a full main-task
    // teardown (capture_stop()) and a dump, instead of sitting in the
    // half-state "capturing, but the interrupt is masked".
    bool rf433_timed_out = s_rf433_capture_started_ms != 0 &&
                           millis() - s_rf433_capture_started_ms > kRf433CaptureMaxMs;
    bool rf433_overran = Rf433Common::is_capturing() && Rf433Common::overrun();
    if (rf433_timed_out || rf433_overran) {
        if (rf433_overran) {
            Serial.printf("quarky-tab5: [rf433] *** ISR SELF-DISARMED at the %u-edge "
                          "ceiling -- the interrupt was firing far longer or far faster "
                          "than a legitimate capture. Completing teardown. ***\n",
                          (unsigned)Rf433Common::edges_this_capture());
        } else {
            Serial.printf("quarky-tab5: [rf433] auto-stopping capture after %u ms\n",
                          (unsigned)kRf433CaptureMaxMs);
        }
        Rf433Common::capture_stop();
        s_rf433_capture_started_ms = 0;
        s_rf433_captures_completed++;
        rf433_dump_capture();
    }
    // --- end debug aid ---
#endif

    // Task 19: derive the shell status bar's link label from how recently
    // each C2 transport last received a frame. Neither transport has a peer
    // yet (Cardputer-ADV, Tasks 14-17, doesn't exist as of this task) so this
    // is expected to read "disconnected" for now -- verified end-to-end once
    // Task 20's ping feature generates real traffic on both sides. Polled at
    // a fixed interval rather than every loop() iteration since the label
    // only needs to be roughly live, not per-frame-accurate.
    static uint32_t s_last_devices_poll_ms = 0;
    uint32_t now = millis();
    if (now - s_last_devices_poll_ms > 500) {
        s_last_devices_poll_ms = now;
        // A last-recv value of 0 means "never received a frame" (both
        // statics' un-set default). Without this guard, `now - 0` is just
        // `now`, which is < 5000ms for the first few seconds after boot --
        // that reads as freshly "connected" before a single real frame ever
        // arrived. Excluding the 0 sentinel keeps the label honestly
        // "disconnected" until an actual frame shows up.
        uint32_t wifi_last = c2link_wifi_last_recv_ms();
        uint32_t ble_last = c2link_ble_last_recv_ms();
        uint32_t wifi_age = now - wifi_last;
        uint32_t ble_age = now - ble_last;
        bool wifi_connected = wifi_last != 0 && wifi_age < 5000;
        bool ble_connected = ble_last != 0 && ble_age < 5000;
        int32_t freshest_age = wifi_connected ? (int32_t)wifi_age : (int32_t)ble_age;
        DevicesPanel::update(wifi_connected, ble_connected, freshest_age);
    }

    delay(5);
}
