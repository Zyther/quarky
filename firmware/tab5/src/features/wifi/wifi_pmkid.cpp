#include "wifi_pmkid.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../hal/storage_sd.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino.h> // Serial, millis(), IRAM_ATTR, portMUX_TYPE/portENTER_CRITICAL
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp (Phase 1 Task 10)

// -----------------------------------------------------------------------------
// NON-FUNCTIONAL ON TAB5-NATIVE -- CONFIRMED HARD LIMITATION, NOT A BUG IN
// THIS FILE. Real hardware (2026-08-12): both esp_wifi_set_promiscuous_rx_cb()
// and esp_wifi_set_promiscuous(true) return 262 (0x106, ESP_ERR_NOT_SUPPORTED)
// immediately, every time.
//
// This file used to claim (based on `nm` alone showing the RPC proxy symbols
// present) that "the RPC surface is not simply missing/stubbed for this
// target." That inference was wrong -- nm can't distinguish a real RPC proxy
// from a stub with the same symbol name. A follow-up investigation
// disassembled the actual linked code and found esp_wifi_remote_set_
// promiscuous()/_rx_cb() are Espressif's weak DEFAULT stubs
// (esp_wifi_remote_weak.c.obj): exactly `li a0,262; ret` -- two instructions,
// no RPC call, no SDIO round-trip to the C6 at all. esp_wifi_remote_80211_tx()
// (Task 2's raw-TX-injection spike, also ESP_ERR_NOT_SUPPORTED) is a
// byte-identical stub -- same root cause, not a coincidence.
//
// This rules out every "maybe fixable" theory checked: not a co-processor
// firmware version-skew issue (the check happens entirely on the P4 host
// before any SDIO traffic, so the C6's firmware version is irrelevant by
// construction), not an AP_STA-vs-STA mode question (same reason), and not a
// missing Kconfig flag (none exists for this). Upstream esp-hosted has
// promiscuous-mode support written but commented out server-side as of this
// check, its wire protocol reserves message IDs for it without defining
// payloads, and the maintainers have an open issue stating there are no
// near-term plans to implement it. No donor firmware (Bruce/Poseidon/
// UniGeek) solved this either -- Poseidon's handshake capture runs on a
// node with a native (non-proxied) WiFi radio, not through esp-hosted.
//
// Full investigation, sources, and the one real (high-risk, out of scope)
// escape hatch found: task-6-promiscuous-investigation-report.md in this
// plan's SDD workspace. Per project decision, this feature is deferred
// rather than reworked here: it needs to run on hardware with a native
// WiFi radio (a future Cardputer-ADV satellite feature, or the planned
// ESP32-C5 sidecar), not the Tab5's esp-hosted-proxied one. The code below
// (UI, ring buffer, pcap writer, IStorage capture-file methods) is left
// merged as-is -- it's correct and directly reusable once ported to a
// native-radio target, it just cannot capture anything on Tab5-native today.
// -----------------------------------------------------------------------------

namespace WifiPmkidFeature {

// Minimal pcap file writer -- global header once, then one packet record per
// captured 802.11 frame, LINKTYPE_IEEE802_11 (105). Standard format
// hashcat/Wireshark/hcxpcapngtool all read directly; no custom framing.
struct PcapGlobalHeader {
    uint32_t magic = 0xa1b2c3d4;
    uint16_t version_major = 2, version_minor = 4;
    int32_t thiszone = 0;
    uint32_t sigfigs = 0;
    uint32_t snaplen;
    uint32_t network = 105; // LINKTYPE_IEEE802_11
} __attribute__((packed));

struct PcapPacketHeader {
    uint32_t ts_sec, ts_usec, incl_len, orig_len;
} __attribute__((packed));

// Per-frame capture cap. 802.11 management frames and the EAPOL key frames
// this feature exists to isolate are both well under this (a few hundred
// bytes); a full 2304-byte data frame is truncated at this length rather
// than grown to fit -- exactly how any pcap snaplen works. incl_len (what's
// stored) vs. orig_len (what was actually on the air, from rx_ctrl.sig_len)
// records that honestly per packet rather than silently dropping the frame.
constexpr size_t kSnapLen = 512;

// -----------------------------------------------------------------------------
// ISR-to-main-task handoff, matching hal/c2link_ble.cpp's established
// pattern (its rx_access_cb -> s_rx_queue -> poll() shape): promiscuous_rx_cb
// below runs on the WiFi driver's own task context (IRAM_ATTR'd for speed,
// like any promiscuous callback), NOT the main/LVGL task, and SD_MMC access
// is not safe to call from there -- not ISR-safe, and blocking/allocating in
// a driver callback risks stalling the radio. c2link_ble.cpp solves the
// analogous problem (its rx_access_cb runs on the NimBLE host task, a
// different task from poll()'s main task) with a small fixed-size buffer
// written under a portMUX critical section by the producer and drained
// under the same critical section by the consumer, with the actual blocking
// work done OUTSIDE the critical section. This reuses that exact shape.
//
// The one structural difference: c2link_ble.cpp's queue holds fixed-size
// Frame structs (a classic SPSC index ring), while pcap records here are
// variable-length (PcapPacketHeader + up to kSnapLen bytes of frame data),
// so this is a byte-oriented circular buffer instead. That turns out to be
// simpler, not harder, for the "cap how much poll() drains per call" Global
// Constraint: because draining is just copying raw bytes verbatim into the
// pcap file, it never needs to respect record boundaries. poll() can drain
// an arbitrary byte count and stop mid-record -- the remainder just sits in
// the ring for the next poll() call, and the file ends up byte-for-byte
// identical to what an uncapped drain would have produced.
constexpr size_t kRingBufferSize = 8192; // 8KB, per this task's brief
constexpr size_t kMaxDrainPerPollBytes = 4096; // bounds poll()'s SD write time

static uint8_t s_ring_buf[kRingBufferSize];
static volatile size_t s_ring_head = 0; // producer writes here (WiFi driver task)
static volatile size_t s_ring_tail = 0; // consumer reads here (main task, via poll())
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_dropped_count = 0; // frames dropped because the ring was full

// Caller must hold s_ring_mux.
static size_t ring_used_locked() {
    return (s_ring_head - s_ring_tail + kRingBufferSize) % kRingBufferSize;
}

// Appends len bytes to the ring buffer. Bounds-checked, drop-and-count if it
// doesn't fit -- never blocks, never allocates. Called only from
// promiscuous_rx_cb (the single producer).
static void ring_push(const uint8_t *data, size_t len) {
    portENTER_CRITICAL(&s_ring_mux);
    // -1 leaves a one-byte gap so a full ring (head caught up to tail) can
    // never look identical to an empty one (head == tail).
    size_t free_bytes = kRingBufferSize - 1 - ring_used_locked();
    if (len > free_bytes) {
        s_dropped_count++;
        portEXIT_CRITICAL(&s_ring_mux);
        return;
    }
    size_t first_chunk = kRingBufferSize - s_ring_head;
    if (first_chunk > len) first_chunk = len;
    memcpy(&s_ring_buf[s_ring_head], data, first_chunk);
    if (first_chunk < len) {
        memcpy(&s_ring_buf[0], data + first_chunk, len - first_chunk);
    }
    s_ring_head = (s_ring_head + len) % kRingBufferSize;
    portEXIT_CRITICAL(&s_ring_mux);
}

// s_active and s_packet_count cross the same driver-task/main-task boundary
// s_ring_head/s_ring_tail/s_dropped_count above do (s_active is read inside
// promiscuous_rx_cb and written from the main task; s_packet_count is
// incremented in the callback and read in poll()), so both are volatile for
// the same reason -- final whole-branch review finding M5 (2026-08-13)
// caught these as inconsistent with s_dropped_count three lines above.
// s_path/s_header_written are touched only from the main task (start()/
// poll()/the LV_EVENT_DELETE handler), never from promiscuous_rx_cb, so
// they correctly stay plain.
static volatile bool s_active = false;
static char s_path[64];
static volatile uint32_t s_packet_count = 0;
static bool s_header_written = false;

static void IRAM_ATTR promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!s_active) return;
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
    auto *pkt = (wifi_promiscuous_pkt_t *)buf;

    // Filter for EAPOL (PMKID/handshake) frames: DATA type with an 802.1X
    // EtherType (0x888E) inside -- a real implementation should parse the
    // LLC/SNAP header to confirm. This first cut appends every MGMT+DATA
    // frame it sees (beacons, probes, EAPOL, everything) to keep this task's
    // scope bounded, with the real EAPOL filter left as a follow-up once the
    // capture pipeline itself is proven on real hardware (Step 5 of the
    // brief). This is a scope note, not a placeholder: capturing everything
    // and filtering later with hcxpcapngtool/hashcat is a valid, real first
    // implementation -- the brief's own framing.
    uint16_t incl_len = pkt->rx_ctrl.sig_len;
    if (incl_len > kSnapLen) incl_len = kSnapLen;

    PcapPacketHeader phdr;
    phdr.ts_sec = millis() / 1000;
    phdr.ts_usec = (millis() % 1000) * 1000;
    phdr.incl_len = incl_len;
    phdr.orig_len = pkt->rx_ctrl.sig_len;

    // Built as one contiguous entry (header immediately followed by frame
    // bytes) so ring_push only needs a single critical section per packet --
    // pushing the header and frame separately would let poll() observe a
    // header with no frame bytes behind it yet if it drained between the two
    // pushes.
    uint8_t entry[sizeof(PcapPacketHeader) + kSnapLen];
    memcpy(entry, &phdr, sizeof(phdr));
    memcpy(entry + sizeof(phdr), pkt->payload, incl_len);
    ring_push(entry, sizeof(phdr) + incl_len);

    s_packet_count++;
}

static lv_obj_t *s_status_label = nullptr;

static lv_obj_t *build_screen() {
    // Menu-bar Back button + flex content area -- see ui/screen_scaffold.cpp
    // for why every sub-screen must build through this rather than
    // hand-positioning its own Back button (this task's brief predates that
    // amendment; the amendment note at the top of task-6-brief.md is what
    // this follows instead of the brief's own inline lv_obj_create()
    // sample).
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("WiFi PMKID Capture", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Starting capture...");

    // The scaffold's own Back button (screen_scaffold.cpp) just calls
    // ScreenStack::pop() -- it has no hook for feature-specific cleanup, and
    // pop() can also be reached indirectly (any future "close all"/nav
    // action). So capture teardown (stopping promiscuous mode, clearing
    // screen-owned state) is done from the label's own LV_EVENT_DELETE,
    // which fires no matter which path destroys this screen -- the same
    // pattern wifi_scan.cpp's list and wifi_spectrum.cpp's chart both use,
    // for the same "must survive being popped via any path" reasoning.
    lv_obj_add_event_cb(s_status_label, [](lv_event_t *e) {
        s_active = false;
        esp_wifi_set_promiscuous(false);
        s_status_label = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_pmkid", "WiFi PMKID Capture", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    // Same AP+STA coexistence handling wifi_common.cpp/wifi_spectrum.cpp
    // use: c2link_wifi's SoftAP (the WiFi C2 transport) must stay up, so
    // flip to AP_STA rather than a bare STA mode that would silently drop
    // it. See the file-level comment above for why promiscuous mode's
    // actual behavior under AP_STA -- as opposed to a plain scan's, already
    // proven fine on real hardware -- is NOT confirmed the same way: this is
    // the best-known-good mode choice carried over from Tasks 3/5, not a
    // proven-safe one for promiscuous mode specifically, pending the
    // real-hardware pass.
    if (WiFi.getMode() == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);
    } else if (WiFi.getMode() == WIFI_OFF) {
        WiFi.mode(WIFI_STA);
    }

    ScreenStack::push(build_screen());

    s_packet_count = 0;
    s_dropped_count = 0;
    s_ring_head = 0;
    s_ring_tail = 0;
    s_header_written = false;

    // Final whole-branch review finding I3 (2026-08-13): this used to
    // create and write the pcap file BEFORE checking whether promiscuous
    // mode is even available -- on hardware where it isn't (confirmed:
    // it's a hard esp-hosted limitation, see the file-level comment above),
    // every single call created a permanently-empty capture file with
    // nothing ever cleaning it up. Reordered so the capability check comes
    // first; no file is created at all unless there's something to write
    // into it. This tile is no longer registered in the launcher
    // (main.cpp), but the ordering is fixed here too since it would be
    // just as wrong on the native-radio target this code is kept as
    // reference for.
    esp_err_t cb_err = esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    esp_err_t en_err = esp_wifi_set_promiscuous(true);
    Serial.printf("quarky-tab5: wifi_pmkid promiscuous rx_cb_err=%d set_err=%d\n",
                  (int)cb_err, (int)en_err);

    if (cb_err != ESP_OK || en_err != ESP_OK) {
        s_active = false;
        if (en_err == ESP_OK) esp_wifi_set_promiscuous(false); // don't leave it half-enabled
        if (s_status_label) lv_label_set_text(s_status_label, "Promiscuous mode failed to start");
        return;
    }

    snprintf(s_path, sizeof(s_path), "/quarky/captures/wifi/capture_%lu.pcap", millis());
    PcapGlobalHeader ghdr;
    ghdr.snaplen = (uint32_t)kSnapLen;
    if (!storage.write_capture_file(s_path, (const uint8_t *)&ghdr, sizeof(ghdr))) {
        Serial.println("quarky-tab5: wifi_pmkid capture file header write FAILED");
        esp_wifi_set_promiscuous(false); // don't capture into a file that doesn't exist
        s_active = false;
        if (s_status_label) lv_label_set_text(s_status_label, "SD write failed, capture not started");
        return;
    }
    s_header_written = true;
    s_active = true;
}

void poll() {
    if (!s_status_label) return; // capture screen not open

    // Drain the ring buffer into the pcap file, capped per call so this
    // never risks the ~50ms loop() budget (Global Constraint) even if the
    // ring is full. See the ring-buffer comment above for why draining an
    // arbitrary byte count -- not necessarily a whole number of pcap
    // records -- is safe: it's a raw byte stream, appended verbatim.
    static uint8_t s_drain_buf[kMaxDrainPerPollBytes];
    size_t to_drain;
    portENTER_CRITICAL(&s_ring_mux);
    size_t used = ring_used_locked();
    to_drain = used < kMaxDrainPerPollBytes ? used : kMaxDrainPerPollBytes;
    if (to_drain > 0) {
        size_t first_chunk = kRingBufferSize - s_ring_tail;
        if (first_chunk > to_drain) first_chunk = to_drain;
        memcpy(s_drain_buf, &s_ring_buf[s_ring_tail], first_chunk);
        if (first_chunk < to_drain) {
            memcpy(s_drain_buf + first_chunk, &s_ring_buf[0], to_drain - first_chunk);
        }
        s_ring_tail = (s_ring_tail + to_drain) % kRingBufferSize;
    }
    portEXIT_CRITICAL(&s_ring_mux);

    if (to_drain > 0 && s_header_written) {
        if (!storage.append_capture_file(s_path, s_drain_buf, to_drain)) {
            Serial.println("quarky-tab5: wifi_pmkid append_capture_file FAILED");
        }
    }

    // Task review finding (2026-08-12): this used to run unconditionally,
    // which clobbered start()'s own failure-message labels ("SD write
    // failed...", "Promiscuous mode failed to start") one loop iteration
    // after they were set -- s_active stays false in both failure paths, so
    // the screen would silently settle on "Capturing... 0 packets (0
    // dropped)" forever, reading as a healthy just-started capture instead
    // of the failure it actually is. Gating on s_active makes a failure
    // message set by start() persist until the screen closes or a new
    // start() supersedes it.
    if (!s_active) return;

    char buf[64];
    snprintf(buf, sizeof(buf), "Capturing... %lu packets (%lu dropped)",
             (unsigned long)s_packet_count, (unsigned long)s_dropped_count);
    lv_label_set_text(s_status_label, buf);
}

} // namespace WifiPmkidFeature
