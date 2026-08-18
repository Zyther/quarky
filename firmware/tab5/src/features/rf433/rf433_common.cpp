#include "rf433_common.h"
#include "../../../boards/tab5/pins_config.h"
#include <Arduino.h>
#include <driver/gpio.h> // gpio_intr_disable() -- the ISR self-disarm below

// ===========================================================================
// HAZARD: A CAPTURE CAN BE INVALIDATED FROM OUTSIDE, SILENTLY (2026-08-18)
// ===========================================================================
// If you are here because a capture returned zero edges, or returned edges
// that decode to nonsense, read this before debugging the ring buffer.
//
// TAB5_RF433R_PIN is GPIO53, which is ALSO TAB5_EXTERNAL_I2C_SDA_GPIO -- the
// SDA line of the external HY2.0 PORT.A I2C bus. One physical pin, because
// PORT.A is one physical socket holding one unit at a time. The full write-up
// of the sharing and its consequences is the canonical block in
// hal/rf433_gpio.cpp.
//
// The direction that matters HERE is I2C taking the pin back. capture_start()
// claims GPIO53 with pinMode(), which tears down the Wire1 peripheral. The NFC
// side detects that and re-initializes Wire1 on its next access
// (hal/nfc_pn532.cpp's ensureExternalI2CBegun()), and that re-init routes
// GPIO53 to the I2C peripheral again -- possibly while a capture is still
// running.
//
// THIS MODULE IS NOT NOTIFIED. s_capturing stays true and the attachInterrupt()
// handler stays installed, because the peripheral manager's GPIO deinit
// callback (gpioDetachBus(), cores/esp32/esp32-hal-gpio.c:105-107) is a no-op
// returning true -- it detaches nothing. So this module keeps believing it
// owns a pin that I2C now drives. Depending on whether the I2C driver's pin
// setup masks the GPIO interrupt (inside the prebuilt i2c_new_master_bus();
// not verified), the capture then either records ~0 edges or -- worse --
// timestamps I2C bus traffic and returns fake but plausible-looking pulse
// data. Neither case sets overrun() or any other error flag.
//
// Not defended against in code on purpose: the fix is proper arbitration of
// GPIO53 (a claim/release owner token shared by both subsystems), not a
// back-reference from either side. Unreachable in practice today -- both RF433
// and NFC are serial-trigger spikes with no launcher tile, so nothing can run
// concurrently with a capture -- and it becomes reachable the moment either
// grows a UI that stays open while the other runs. Whoever builds that owns
// this problem; do not assume the current quiet is a guarantee.
// ===========================================================================

namespace Rf433Common {
namespace {

constexpr size_t kRingSize = 512; // generous headroom for one OOK burst --
                                   // typical fixed-code remotes send a few
                                   // hundred edges per press (repeated ~4-8x)

// Hard ceiling on how many edges one capture session may service before the
// ISR disarms itself. This is a runaway-interrupt guard, not a capture-length
// setting -- see the self-disarm block in isr_edge() for the full rationale
// and for how this number was derived from the 2026-08-18 real-hardware run.
constexpr uint32_t kMaxEdgesPerCapture = 100000;

// NOT declared `volatile`, deliberately. The brief's sketch had
// `volatile EdgeSample s_ring[kRingSize]`, which does not compile: the
// implicitly-declared copy-assignment operator of a plain struct is
// `EdgeSample &operator=(const EdgeSample &)`, which cannot be invoked on a
// volatile lvalue, so both `s_ring[i] = {...}` in the ISR and
// `out[i] = s_ring[...]` in capture_read() are ill-formed. The options were
// (a) hand-write volatile-qualified assignment operators, or (b) drop
// volatile from the array. (b) is correct here and not merely convenient:
// every access to this array happens inside a portMUX critical section, and
// portENTER_CRITICAL/portEXIT_CRITICAL are full compiler barriers (they
// contain `__asm__ volatile ... ::: "memory"` clobbers) plus a real spinlock,
// so the compiler may not hoist or sink accesses across them and the two
// cores may not interleave inside them. `volatile` would add nothing to that
// guarantee -- it is not a synchronization primitive. The scalar indices
// below keep `volatile` purely as cheap future-proofing against someone
// later adding an unlocked peek at them (an "is there anything to drain?"
// fast path is the obvious candidate); as of today every access to both is
// already inside a critical section, so it is not load-bearing either.
EdgeSample s_ring[kRingSize];
volatile size_t s_head = 0;  // ISR-owned write index, wraps
volatile size_t s_count = 0; // number of valid unread samples, capped at kRingSize
volatile uint32_t s_edges_this_capture = 0; // total serviced since capture_start(),
                                            // NOT reset by a drain -- this is the
                                            // runaway guard's odometer
volatile bool s_overrun = false;            // set by the ISR when it self-disarms
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
bool s_capturing = false;

void IRAM_ATTR isr_edge() {
    // micros() and digitalRead() are both ISR-safe on ESP32 (micros() reads
    // the system timer; digitalRead() is a register read). Reading the level
    // AFTER the edge rather than inferring it by alternation is deliberate:
    // if an edge is ever missed (ISR latency during a long critical section
    // elsewhere), an inferred/alternating level would stay wrong for the rest
    // of the capture, whereas a real read self-corrects on the next edge.
    portENTER_CRITICAL_ISR(&s_mux);

    // ---- runaway-interrupt guard -----------------------------------------
    // Added 2026-08-18 after a real incident: a caller left this ISR armed
    // indefinitely (a second 'r' after the auto-stop had already fired
    // re-STARTED a capture, and nothing ever stopped it) while a 433MHz
    // transmitter nearby was still looping, and the Tab5 was observed to
    // crash and reboot shortly afterwards. That crash was NOT reproduced on
    // retry and no panic/backtrace was captured, so this is deliberately not
    // presented as a root-cause fix -- it is a bound on the failure CLASS
    // (unbounded ISR servicing with no main-task participation), which is
    // real regardless of whether it caused that particular crash.
    //
    // WHY a count and not a timeout: a fixed edge budget converts to a
    // SHORTER wall-clock bound the faster the interrupt is firing, which is
    // exactly the discrimination wanted here. At the ~1000 edges/s measured
    // during the real 2026-08-18 capture (512 edges spanning 509ms of
    // sustained transmission) 100k edges is ~100 seconds -- 5x the longest
    // sanctioned capture, so legitimate use can never trip it. Against a
    // floating-pin/AGC-noise storm at tens of kHz it disarms in a couple of
    // seconds. A small ceiling would invert that: 5-10k would fire partway
    // into a legitimate 20s capture of real traffic (~20k edges) while
    // buying only milliseconds in the storm case.
    //
    // WHY gpio_intr_disable() and not detachInterrupt() -- verified against
    // this exact framework, not assumed:
    //   * driver/gpio.h documents gpio_intr_disable() as callable from ISR
    //     context, needing CONFIG_GPIO_CTRL_FUNC_IN_IRAM only for the
    //     cache-disabled case. That config is NOT set here, but it does not
    //     need to be: Arduino installs the GPIO ISR service with
    //     ARDUINO_ISR_FLAG, which is (0) rather than ESP_INTR_FLAG_IRAM
    //     because CONFIG_ARDUINO_ISR_IRAM is likewise not set in this
    //     framework's esp32p4 sdkconfig. A non-IRAM ISR never runs with
    //     cache disabled, so reaching a flash-resident driver function from
    //     here is safe.
    //   * detachInterrupt() is NOT safe from here. Per
    //     cores/esp32/esp32-hal-gpio.c it calls gpio_isr_handler_remove(),
    //     gpio_wakeup_disable() and gpio_set_intr_type(), and then clears
    //     __pinInterruptHandlers[pin] -- which is the very struct passed as
    //     `arg` to the handler that is executing at that moment. Tearing a
    //     handler down from inside itself is not behavior the driver
    //     specifies as safe.
    // So the ISR does the minimum that stops the bleeding (mask the source)
    // and leaves the full teardown to capture_stop() on the main task, which
    // the overrun() flag prompts a caller to do.
    if (s_edges_this_capture >= kMaxEdgesPerCapture) {
        gpio_intr_disable((gpio_num_t)TAB5_RF433R_PIN);
        s_overrun = true;
        portEXIT_CRITICAL_ISR(&s_mux);
        return; // record nothing: past the ceiling this is not data worth having
    }
    s_edges_this_capture = s_edges_this_capture + 1; // `= x + 1`, not `++`; see below
    // ----------------------------------------------------------------------

    s_ring[s_head].timestamp_us = micros();
    s_ring[s_head].level = (digitalRead(TAB5_RF433R_PIN) != LOW);
    s_head = (s_head + 1) % kRingSize;
    // Spelled out as `= s_count + 1` rather than `s_count++`: ++/-- on a
    // volatile-qualified lvalue is deprecated in C++20 (P1152R4) and this
    // toolchain warns about it (-Wvolatile). An explicit load-add-store is
    // identical here (we are inside the critical section) and warning-free.
    if (s_count < kRingSize) s_count = s_count + 1;
    portEXIT_CRITICAL_ISR(&s_mux);
}

} // namespace

bool capture_start() {
    if (s_capturing) return true;
    pinMode(TAB5_RF433R_PIN, INPUT);
    portENTER_CRITICAL(&s_mux);
    s_head = 0;
    s_count = 0;
    s_edges_this_capture = 0;
    s_overrun = false;
    portEXIT_CRITICAL(&s_mux);
    attachInterrupt(digitalPinToInterrupt(TAB5_RF433R_PIN), isr_edge, CHANGE);
    // Explicitly re-enable rather than trusting attachInterrupt to undo a
    // previous ISR self-disarm. gpio_isr_handler_add() (which attachInterrupt
    // funnels into) does appear to enable the interrupt for the calling core,
    // but that driver ships prebuilt in this framework -- only its headers are
    // inspectable here -- and after a gpio_intr_disable() from ISR context the
    // enable-bit state is not something to infer from an unread implementation.
    // Enabling an already-enabled GPIO interrupt is a no-op, so this costs
    // nothing and removes the guesswork.
    gpio_intr_enable((gpio_num_t)TAB5_RF433R_PIN);
    s_capturing = true;
    Serial.printf("quarky-tab5: [rf433] edge capture started on GPIO%d "
                  "(self-disarms after %u edges)\n",
                  TAB5_RF433R_PIN, (unsigned)kMaxEdgesPerCapture);
    return true;
}

void capture_stop() {
    if (!s_capturing) return;
    detachInterrupt(digitalPinToInterrupt(TAB5_RF433R_PIN));
    s_capturing = false;
    Serial.printf("quarky-tab5: [rf433] edge capture stopped (%u edges serviced"
                  " this session)%s\n",
                  (unsigned)s_edges_this_capture,
                  s_overrun ? " -- ISR HAD SELF-DISARMED ON THE EDGE CEILING" : "");
}

bool is_capturing() { return s_capturing; }

bool overrun() { return s_overrun; }

uint32_t edges_this_capture() { return s_edges_this_capture; }

size_t capture_capacity() { return kRingSize; }

size_t capture_read(EdgeSample *out, size_t max) {
    if (!out || max == 0) return 0;
    portENTER_CRITICAL(&s_mux);
    // Read out oldest-first. s_head is the next WRITE slot and s_count is how
    // many unread samples sit behind it, so the oldest unread sample is
    // always exactly s_count slots back from the head -- modulo the ring.
    // That single expression is correct for every state: a fresh partial fill
    // (head == count, so start == 0), a saturated buffer (count == kRingSize,
    // so start == head), AND a partial fill that follows an earlier drain
    // (head is wherever the ISR left it, unrelated to count).
    //
    // That last case is why this is NOT the "start at 0 unless the buffer is
    // full" test it used to be. Consuming does not rewind s_head, so after a
    // drain at head==300 the next 10 edges land in slots 300-309 with
    // s_count==10 -- and the old test, seeing s_count < kRingSize, would have
    // copied out slots 0-9 and reported pre-drain samples as the new ones.
    // Harmless for the 'r' trigger (which only ever stops and drains once),
    // but this file is the shared receive front-end for the later scan/decode
    // tasks, which drain repeatedly during an open capture and would have
    // silently decoded stale data.
    //
    // Overflow behavior, disclosed: once s_count saturates at kRingSize the
    // ISR keeps writing and silently overwrites the OLDEST unread samples.
    // For this task that is the right trade (the interesting part of an OOK
    // burst is the most recent edges, and a burst is re-sent 4-8x anyway),
    // but a caller that reads exactly capture_capacity() samples should
    // assume it lost older ones.
    size_t n = s_count < max ? s_count : max;
    size_t start = (s_head + kRingSize - s_count) % kRingSize;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(start + i) % kRingSize];
    }
    // Consume only what was actually copied. A short read (max < s_count)
    // therefore leaves the NEWEST s_count - n samples in the ring for the
    // next call instead of throwing them away, which is what `s_count = 0`
    // used to do -- and for OOK capture the recent edges are usually the ones
    // the caller wanted. This is only expressible now that `start` is derived
    // from head-minus-count rather than special-cased.
    s_count = s_count - n; // not `-=`, for the same -Wvolatile reason as the ISR
    portEXIT_CRITICAL(&s_mux);
    return n;
}

} // namespace Rf433Common
