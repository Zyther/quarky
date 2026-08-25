#include "cc1101_replay.h"
#include "cc1101_hw.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

// See cc1101_replay.h and rf433_replay.cpp's own header comment for the full
// "why a dedicated core-pinned task, not poll()" citation this follows
// verbatim (same real crash class: WifiConnectFeature's task_wdt abort,
// Task 5/6 RF433's own kMaxEdgesPerPoll/priority-pinning fixes).

namespace Cc1101Replay {
namespace {

struct TransmitArgs {
    Cc1101Scan::EdgeSample edges[Cc1101Scan::kMaxEdgesPerSignal];
    size_t edge_count;
};

// Same defensive ceiling and reasoning as rf433_replay.cpp's
// kMaxSingleDelayUs -- bounds a single delayMicroseconds() call in this
// non-watchdog-subscribed task against a pathological/hand-edited signal.
constexpr uint32_t kMaxSingleDelayUs = 1000000; // 1s

// Same priority/core-pinning reasoning as rf433_replay.cpp's
// kTransmitTaskPriority/kTransmitTaskCore (see that file's header comment
// for the full citation trail: loopTask runs LVGL + poll() at priority 1 on
// ARDUINO_RUNNING_CORE, so a same-priority no-affinity task can be preempted
// mid-pulse by an LVGL tick on the same core).
constexpr UBaseType_t kTransmitTaskPriority = 5;
constexpr BaseType_t kLoopTaskCore = ARDUINO_RUNNING_CORE;
constexpr BaseType_t kTransmitTaskCore = (kLoopTaskCore == 0) ? 1 : 0;

volatile bool s_task_running = false;
volatile bool s_task_done = false;

State s_state = State::kIdle;
char s_failure_reason[80] = "";
bool s_last_truncated = false;

void set_failure(const char *reason) {
    std::strncpy(s_failure_reason, reason, sizeof(s_failure_reason) - 1);
    s_failure_reason[sizeof(s_failure_reason) - 1] = '\0';
    s_state = State::kFailed;
}

// Real crash found on real hardware, 2026-08-25: replaying a truncated
// (kMaxEdgesPerSignal = 8192) capture triggered a task_wdt abort --
// "IDLE0 (CPU 0)" did not reset in time while "cc1101_tx" was the only task
// running on that core. Root cause: unlike rf433_replay.cpp's own real
// measured worst case (~509ms for 512 edges, comfortably under the 5s
// watchdog window), this loop never yields at all across up to 8192 edges,
// and delayMicroseconds() busy-waits rather than yielding -- a long enough
// truncated signal keeps this task's pinned core continuously busy for
// several real seconds, starving that core's IDLE task (which IS
// watchdog-subscribed by default, unlike this task itself) past its
// timeout. Fixed with a periodic vTaskDelay(1) paced by ELAPSED TIME, not
// edge count, so it fires the same way regardless of a given signal's real
// inter-edge gap distribution: one ~1-2ms scheduler-tick pause inserted
// between two pulse transitions every kYieldIntervalUs, letting IDLE0 run
// and feed the watchdog. This does introduce a real, one-time timing
// anomaly at each yield point -- accepted as a real, bounded, rare cost
// against an unconditional crash on any long replay.
constexpr uint32_t kYieldIntervalUs = 500000; // 500ms -- 10x margin under a
                                               // typical 5s task_wdt timeout

void transmit_task(void *arg) {
    TransmitArgs *args = static_cast<TransmitArgs *>(arg);
    int gdo0 = Cc1101Hw::gdo0_pin();

    if (args->edge_count > 0) {
        digitalWrite(gdo0, args->edges[0].level ? HIGH : LOW);
        uint32_t last_yield_us = micros();
        for (size_t i = 1; i < args->edge_count; i++) {
            uint32_t delta = args->edges[i].timestamp_us - args->edges[i - 1].timestamp_us;
            if (delta > kMaxSingleDelayUs) delta = kMaxSingleDelayUs;
            delayMicroseconds(delta);
            digitalWrite(gdo0, args->edges[i].level ? HIGH : LOW);
            if (micros() - last_yield_us > kYieldIntervalUs) {
                vTaskDelay(1);
                last_yield_us = micros();
            }
        }
    }
    digitalWrite(gdo0, LOW);
    Cc1101Hw::idle();

    delete args;
    s_task_done = true;
    vTaskDelete(nullptr);
}

} // namespace

void transmit(const Cc1101Scan::CapturedSignal &sig) {
    if (s_task_running) {
        Serial.println("quarky-tab5: [cc1101-replay] transmit() REFUSED -- a "
                        "replay is already in flight");
        return;
    }
    if (Cc1101Scan::is_capturing()) {
        Serial.println("quarky-tab5: [cc1101-replay] transmit() REFUSED -- a "
                        "CC1101 capture is currently active on the same GDO0 pin");
        set_failure("CC1101 capture in progress -- stop it before replaying");
        return;
    }
    if (sig.edge_count == 0) {
        Serial.printf("quarky-tab5: [cc1101-replay] transmit() REFUSED -- signal "
                      "#%u has no edges\n", (unsigned)sig.capture_id);
        set_failure("Signal has no edges");
        return;
    }
    if (!Cc1101Hw::is_present()) {
        Cc1101Hw::init();
    }
    Cc1101Hw::set_frequency_mhz(sig.freq_hz / 1000000.0f);
    if (!Cc1101Hw::enable_async_tx()) {
        Serial.println("quarky-tab5: [cc1101-replay] transmit() REFUSED -- "
                        "enable_async_tx() failed (module not present?)");
        set_failure("CC1101 not present / async TX setup failed");
        return;
    }
    pinMode(Cc1101Hw::gdo0_pin(), OUTPUT);
    digitalWrite(Cc1101Hw::gdo0_pin(), LOW);

    s_last_truncated = sig.truncated;
    if (sig.truncated) {
        Serial.printf("quarky-tab5: [cc1101-replay] transmit() signal #%u is "
                      "truncated -- replaying only its captured prefix (%u edges)\n",
                      (unsigned)sig.capture_id, (unsigned)sig.edge_count);
    }

    TransmitArgs *args = new TransmitArgs();
    args->edge_count = sig.edge_count;
    std::memcpy(args->edges, sig.edges, sizeof(Cc1101Scan::EdgeSample) * sig.edge_count);

    s_state = State::kTransmitting;
    s_task_done = false;
    s_task_running = true;

    BaseType_t created = xTaskCreatePinnedToCore(transmit_task, "cc1101_tx", 4096,
                                                  args, kTransmitTaskPriority,
                                                  nullptr, kTransmitTaskCore);
    if (created != pdPASS) {
        delete args;
        s_task_running = false;
        s_last_truncated = false;
        Serial.println("quarky-tab5: [cc1101-replay] xTaskCreatePinnedToCore() "
                        "FAILED -- out of memory?");
        set_failure("Failed to start transmit task (out of memory?)");
    }
}

bool is_busy() { return s_task_running; }
State state() { return s_state; }
const char *failure_reason() { return s_failure_reason; }
bool last_transmit_was_truncated() { return s_last_truncated; }

void poll() {
    if (!s_task_running) return;
    if (!s_task_done) return;
    s_task_running = false;
    s_task_done = false;
    s_state = State::kDone;
    Serial.println("quarky-tab5: [cc1101-replay] transmit complete");
}

} // namespace Cc1101Replay
