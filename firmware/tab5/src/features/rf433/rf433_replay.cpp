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
// headroom reasoning), same priority (1, matching loopTask -- no reason to
// preempt the UI).
//
// The transmit task is deliberately never subscribed to the task watchdog
// (only loopTask and the idle tasks are, by default, in this Arduino-ESP32
// framework -- nothing here calls esp_task_wdt_add() for it), so its own
// several-hundred-ms busy-wait cannot trip that watchdog either. No
// deviation from the plan's default recommendation was needed here.
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

volatile bool s_task_running = false; // true from a successful task launch
                                       // until poll() processes its completion
volatile bool s_task_done = false;    // set by transmit_task() right before
                                       // it exits; poll() clears it

// Touched only by the main task (transmit() and poll()), never by the
// background transmit task -- so, unlike the two flags above, these need no
// `volatile` qualification.
State s_state = State::kIdle;
char s_failure_reason[80] = "";

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

    if (sig.truncated) {
        Serial.printf("quarky-tab5: [rf433-replay] transmit() REFUSED -- signal "
                      "#%u is truncated (its edges[] do not represent the whole "
                      "burst)\n", (unsigned)sig.capture_id);
        set_failure("Truncated signal -- refusing to replay incomplete data");
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

    pinMode(TAB5_RF433T_PIN, OUTPUT);
    digitalWrite(TAB5_RF433T_PIN, LOW);

    TransmitArgs *args = new TransmitArgs();
    args->edge_count = sig.edge_count;
    std::memcpy(args->edges, sig.edges,
                sizeof(Rf433Common::EdgeSample) * sig.edge_count);

    s_state = State::kTransmitting;
    s_task_done = false;
    s_task_running = true;

    BaseType_t created = xTaskCreate(transmit_task, "rf433_tx", 4096, args, 1, nullptr);
    if (created != pdPASS) {
        // xTaskCreate() failed (heap exhaustion) -- undo everything transmit()
        // already did so this doesn't leave the arbiter claimed or the UI
        // stuck showing "Transmitting..." forever with no task to finish it.
        delete args;
        s_task_running = false;
        Gpio53Arbiter::release(Gpio53Arbiter::Owner::kRf433);
        Serial.println("quarky-tab5: [rf433-replay] xTaskCreate() FAILED -- "
                        "out of memory?");
        set_failure("Failed to start transmit task (out of memory?)");
    }
}

bool is_busy() { return s_task_running; }

State state() { return s_state; }

const char *failure_reason() { return s_failure_reason; }

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
