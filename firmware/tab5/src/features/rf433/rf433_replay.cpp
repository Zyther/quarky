#include "rf433_replay.h"
#include "rf433_common.h"
#include "../../hal/gpio53_arbiter.h"
#include "../../../boards/tab5/pins_config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

// ===========================================================================
// WHY the bit-bang transmit loop runs on its own FreeRTOS task, and not
// synchronously inside the Replay click handler -- a real architecture
// decision, not a style preference (this plan's Task 6 controller notes
// rule on this explicitly).
//
// transmit() reproduces a captured signal's edge timing via
// digitalWrite() + delayMicroseconds(), the same technique Bruce's
// rf_send.cpp/emit.cpp (via rc-switch) uses -- microsecond-precision OOK
// timing cannot be done through poll()'s tick-based scheduling, which runs
// at an irregular multi-millisecond cadence mixed with LVGL/C2/other
// feature polling. That part of the plan's Context line is correct and
// necessary. But WHERE that loop runs is a separate question:
//
// Task 1's real-hardware capture recorded ~1,000 edges/second sustained
// (512 edges spanning 509ms of continuous transmission), so a single real
// CapturedSignal can legitimately take hundreds of milliseconds to replay
// (same edge count, same inter-edge delays, reproduced via
// delayMicroseconds()). An LVGL click handler runs synchronously on the
// main/LVGL task inside loop() -- the SAME task this project's task
// watchdog (enableLoopWDT(), ~5s budget) expects fed every tick.
// features/wifi/wifi_connect.cpp documents a REAL on-device crash (confirmed
// task_wdt abort of "loopTask (CPU 1)") from exactly this mistake: a
// blocking call (WiFi connect, up to 15s) invoked synchronously from a click
// handler. Task 5's RF433 scan capture hit the same failure CLASS from the
// receive side (an unbounded poll()-tick drain loop -- see rf433_scan.cpp's
// kMaxEdgesPerPoll/kMaxFinalizesPerPoll comments, added to fix it).
//
// A single replay burst is comfortably inside wifi_connect.cpp's up-to-15s
// failure window, so this follows that file's established fix exactly:
// move the blocking work into its own FreeRTOS task (xTaskCreate), publish
// the result through a few `volatile` cross-task scalars, and drain/report
// it from a poll() called out of main.cpp's loop() -- same shape, same
// stack-size reasoning (4096, matching connect_task's driver-layer-call
// headroom reasoning).
//
// The transmit task is deliberately never subscribed to the task watchdog
// (only loopTask and the idle tasks are, by default, in this Arduino-ESP32
// framework -- nothing here calls esp_task_wdt_add() for it), so its own
// several-hundred-ms busy-wait cannot trip that watchdog either. No
// deviation from the plan's default recommendation was needed here.
//
// ---- WHY PRIORITY/CORE ARE *NOT* COPIED FROM connect_task (Task 6 review
// fix, 2026-08-20) --------------------------------------------------------
// connect_task (wifi_connect.cpp) spends its whole life blocked inside
// delay() calls -- it never needs to preempt anything, so priority 1
// (matching loopTask) and no core pin (tskNO_AFFINITY, xTaskCreate's
// default) are both fine for it. This task's correctness is the opposite
// shape: transmit_task() is a tight digitalWrite()/delayMicroseconds()
// busy-wait reproducing microsecond-precision OOK pulse timing, for up to
// ~509ms per Task 1's real-hardware measurement (512 edges in 509ms
// sustained). delayMicroseconds() busy-waits rather than yielding, but that
// only protects the CPU it is actually running ON -- at priority 1 with no
// core affinity, the FreeRTOS scheduler is free to place this task on
// EITHER core and to preempt it mid-pulse for any other ready priority-1
// task on the same core.
//
// The concrete collision: loopTask -- the task that runs both LVGL's
// millisecond-scale render tick and this project's own poll() functions --
// is itself created at priority 1 (verified, not assumed: cores/esp32/
// main.cpp:113 in framework-arduinoespressif32,
// `xTaskCreateUniversal(loopTask, "loopTask", ..., 1, &loopTaskHandle,
// ARDUINO_RUNNING_CORE)`), and pinned to
// ARDUINO_RUNNING_CORE == CONFIG_ARDUINO_RUNNING_CORE == 1 (verified in the
// prebuilt target's own config: framework-arduinoespressif32-libs/esp32p4/
// sdkconfig:804). A same-priority, no-affinity transmit_task can therefore
// land on core 1 right alongside loopTask and get round-robin preempted by
// an LVGL tick mid-burst, stretching a real ~300us pulse by an order of
// magnitude -- real OOK receivers reject that.
//
// FIXED by xTaskCreatePinnedToCore(): pin transmit_task to the core loopTask
// is NOT on (computed from ARDUINO_RUNNING_CORE rather than hardcoded, so
// this stays correct if that sdkconfig value ever changes), and raise its
// priority above loopTask's 1. This project has no prior precedent for an
// elevated-priority task, so kTransmitTaskPriority's value was chosen, not
// copied: configMAX_PRIORITIES is 25 on this target (verified:
// framework-arduinoespressif32-libs/esp32p4/include/freertos/config/include/
// freertos/FreeRTOSConfig.h:93), so priority 5 sits comfortably above every
// priority-1 task in this codebase (loopTask, connect_task) while leaving
// deep headroom below the range ESP-IDF's own internal driver/ISR-adjacent
// tasks typically occupy -- no need to crowd that ceiling for a task whose
// real defense is running on an otherwise-idle core, not raw priority.
// Priority is kept as defense-in-depth (in case anything else is ever
// scheduled onto that core), not the primary fix -- pinning to a different
// PHYSICAL core than loopTask means an LVGL tick literally cannot preempt
// this task, regardless of relative priority, because they are running in
// parallel on separate cores rather than time-sliced on one.
// ===========================================================================

namespace Rf433Replay {
namespace {

struct TransmitArgs {
    Rf433Common::EdgeSample edges[Rf433Scan::kMaxEdgesPerSignal];
    size_t edge_count;
};

// Defensive ceiling on a single inter-edge delay passed to
// delayMicroseconds(). Every real inter-edge gap inside one CapturedSignal
// is already bounded well under this by rf433_scan.cpp's
// kBurstGapThresholdUs (25ms): any larger gap is exactly where
// finalize_burst() would have split the burst into two signals instead, so
// this never fires on real captured data. It exists purely so a future
// change to that invariant (or a hand-edited/library-loaded signal, once
// Task 10 lands) degrades into a bounded pause rather than an unbounded
// busy-wait inside a task that -- deliberately, see header comment above --
// is not watchdog-subscribed.
constexpr uint32_t kMaxSingleDelayUs = 1000000; // 1s

// Task priority/core -- see this file's header comment ("WHY PRIORITY/CORE
// ARE NOT COPIED FROM connect_task") for the full real-hardware-preemption
// citation trail behind both of these.
constexpr UBaseType_t kTransmitTaskPriority = 5; // > loopTask's 1 (cores/esp32/
                                                  // main.cpp:113); configMAX_
                                                  // PRIORITIES is 25 on this
                                                  // target (FreeRTOSConfig.h:93)
// The core loopTask (LVGL tick + this project's poll() functions) runs on --
// computed from the real ARDUINO_RUNNING_CORE macro (== CONFIG_ARDUINO_
// RUNNING_CORE == 1 on this target's sdkconfig:804) rather than hardcoded, so
// pinning transmit_task to the OTHER core stays correct if that value ever
// changes.
constexpr BaseType_t kLoopTaskCore = ARDUINO_RUNNING_CORE;
constexpr BaseType_t kTransmitTaskCore = (kLoopTaskCore == 0) ? 1 : 0;

volatile bool s_task_running = false; // true from a successful task launch
                                       // until poll() processes its completion
volatile bool s_task_done = false;    // set by transmit_task() right before
                                       // it exits; poll() clears it

// Touched only by the main task (transmit() and poll()), never by the
// background transmit task -- so, unlike the two flags above, these need no
// `volatile` qualification.
State s_state = State::kIdle;
char s_failure_reason[80] = "";
bool s_last_truncated = false; // see last_transmit_was_truncated()

void set_failure(const char *reason) {
    std::strncpy(s_failure_reason, reason, sizeof(s_failure_reason) - 1);
    s_failure_reason[sizeof(s_failure_reason) - 1] = '\0';
    s_state = State::kFailed;
}

void transmit_task(void *arg) {
    TransmitArgs *args = static_cast<TransmitArgs *>(arg);

    if (args->edge_count > 0) {
        digitalWrite(TAB5_RF433T_PIN, args->edges[0].level ? HIGH : LOW);
        for (size_t i = 1; i < args->edge_count; i++) {
            uint32_t delta = args->edges[i].timestamp_us - args->edges[i - 1].timestamp_us;
            if (delta > kMaxSingleDelayUs) delta = kMaxSingleDelayUs; // see constant's comment
            delayMicroseconds(delta);
            digitalWrite(TAB5_RF433T_PIN, args->edges[i].level ? HIGH : LOW);
        }
    }
    // Idle-safe: leave the transmitter LOW when done, matching
    // Rf433Gpio::init()'s own idle default for this pin.
    digitalWrite(TAB5_RF433T_PIN, LOW);

    delete args;
    s_task_done = true; // publish before exiting; poll() reads this on the main task
    vTaskDelete(nullptr);
}

} // namespace

void transmit(const Rf433Scan::CapturedSignal &sig) {
    if (s_task_running) {
        Serial.println("quarky-tab5: [rf433-replay] transmit() REFUSED -- a "
                        "replay is already in flight");
        return; // leave the in-flight transmit's state alone
    }

    if (Rf433Common::is_capturing()) {
        // Refused -- see this file's transmit() doc comment (rf433_replay.h)
        // and rf433_common.cpp's capture_start() for the full RX/TX
        // exclusion writeup. Do NOT touch pinMode()/digitalWrite(): a live
        // capture's attachInterrupt() handler is still armed on this exact
        // pin.
        Serial.println("quarky-tab5: [rf433-replay] transmit() REFUSED -- an "
                        "RF433 capture is currently active on the same pin");
        set_failure("RF433 capture in progress -- stop it before replaying");
        return;
    }

    if (sig.edge_count == 0) {
        Serial.printf("quarky-tab5: [rf433-replay] transmit() REFUSED -- signal "
                      "#%u has no edges\n", (unsigned)sig.capture_id);
        set_failure("Signal has no edges");
        return;
    }

    if (!Gpio53Arbiter::claim(Gpio53Arbiter::Owner::kRf433)) {
        // Refused -- the external I2C bus (NFC/RFID2) currently owns GPIO53.
        // Do NOT touch pinMode()/digitalWrite(): that would tear down Wire1
        // out from under an active NFC/RFID2 session, exactly the hazard
        // this arbiter exists to prevent. Mirrors capture_start()'s
        // identical refusal in rf433_common.cpp.
        Serial.printf("quarky-tab5: [rf433-replay] transmit() REFUSED -- GPIO%d "
                      "is currently owned by the external I2C bus (NFC/RFID2 "
                      "session in progress). Close that screen and retry.\n",
                      TAB5_RF433T_PIN);
        set_failure("GPIO53 busy (I2C/NFC session in progress)");
        return;
    }

    // Truncated signals are replayed, not refused -- see this file's header
    // comment (rf433_replay.h's transmit() doc) for why: truncation only
    // drops the TAIL of a burst, so the recorded prefix is still a valid
    // partial reconstruction, and real captures truncate often enough that
    // outright refusal left ordinary button-holds unreplayable. Set the
    // query flag the UI reads to show a warning instead of silently
    // replaying a partial burst as if it were the whole thing.
    s_last_truncated = sig.truncated;
    if (sig.truncated) {
        Serial.printf("quarky-tab5: [rf433-replay] transmit() signal #%u is "
                      "truncated -- replaying only its captured prefix (%u "
                      "edges); the real burst's tail was not recorded\n",
                      (unsigned)sig.capture_id, (unsigned)sig.edge_count);
    }

    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);

    TransmitArgs *args = new TransmitArgs();
    args->edge_count = sig.edge_count;
    std::memcpy(args->edges, sig.edges,
                sizeof(Rf433Common::EdgeSample) * sig.edge_count);

    s_state = State::kTransmitting;
    s_task_done = false;
    s_task_running = true;

    // Priority/core: see this file's header comment ("WHY PRIORITY/CORE ARE
    // NOT COPIED FROM connect_task") for the full citation trail. Pinned to
    // kTransmitTaskCore (the core loopTask/LVGL is NOT on) so an LVGL render
    // tick cannot preempt a pulse mid-burst; priority kTransmitTaskPriority
    // (> loopTask's 1) is defense-in-depth on top of that.
    BaseType_t created = xTaskCreatePinnedToCore(transmit_task, "rf433_tx", 4096,
                                                  args, kTransmitTaskPriority,
                                                  nullptr, kTransmitTaskCore);
    if (created != pdPASS) {
        // xTaskCreatePinnedToCore() failed (heap exhaustion) -- undo
        // everything transmit() already did so this doesn't leave the
        // arbiter claimed or the UI stuck showing "Transmitting..." forever
        // with no task to finish it.
        delete args;
        s_task_running = false;
        s_last_truncated = false; // this launch failed outright; not a truncation issue
        Gpio53Arbiter::release(Gpio53Arbiter::Owner::kRf433);
        Serial.println("quarky-tab5: [rf433-replay] xTaskCreatePinnedToCore() "
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
    // Gpio53Arbiter::claim()/release() are main-task-only (see
    // hal/gpio53_arbiter.h) -- this is why the release lives here, in the
    // main-task poll() drain, and not at the end of transmit_task() itself.
    Gpio53Arbiter::release(Gpio53Arbiter::Owner::kRf433);
    s_state = State::kDone;
    Serial.println("quarky-tab5: [rf433-replay] transmit complete, GPIO53 "
                    "arbiter claim released");
}

} // namespace Rf433Replay
