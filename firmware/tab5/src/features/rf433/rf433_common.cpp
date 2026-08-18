#include "rf433_common.h"
#include "../../../boards/tab5/pins_config.h"
#include <Arduino.h>

namespace Rf433Common {
namespace {

constexpr size_t kRingSize = 512; // generous headroom for one OOK burst --
                                   // typical fixed-code remotes send a few
                                   // hundred edges per press (repeated ~4-8x)

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
// below DO stay volatile: they are also read (via is_capturing()'s sibling
// paths and future poll()-side "is there anything to drain?" checks) in ways
// where a stale cached value would be a real bug.
EdgeSample s_ring[kRingSize];
volatile size_t s_head = 0;  // ISR-owned write index, wraps
volatile size_t s_count = 0; // number of valid unread samples, capped at kRingSize
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
    s_ring[s_head].timestamp_us = micros();
    s_ring[s_head].level = (digitalRead(TAB5_RF433R_PIN) != LOW);
    s_head = (s_head + 1) % kRingSize;
    if (s_count < kRingSize) s_count++;
    portEXIT_CRITICAL_ISR(&s_mux);
}

} // namespace

bool capture_start() {
    if (s_capturing) return true;
    pinMode(TAB5_RF433R_PIN, INPUT);
    portENTER_CRITICAL(&s_mux);
    s_head = 0;
    s_count = 0;
    portEXIT_CRITICAL(&s_mux);
    attachInterrupt(digitalPinToInterrupt(TAB5_RF433R_PIN), isr_edge, CHANGE);
    s_capturing = true;
    Serial.printf("quarky-tab5: [rf433] edge capture started on GPIO%d\n",
                  TAB5_RF433R_PIN);
    return true;
}

void capture_stop() {
    if (!s_capturing) return;
    detachInterrupt(digitalPinToInterrupt(TAB5_RF433R_PIN));
    s_capturing = false;
    Serial.println("quarky-tab5: [rf433] edge capture stopped");
}

bool is_capturing() { return s_capturing; }

size_t capture_capacity() { return kRingSize; }

size_t capture_read(EdgeSample *out, size_t max) {
    if (!out || max == 0) return 0;
    portENTER_CRITICAL(&s_mux);
    // Read out oldest-first: s_head is the next WRITE slot, so with a full
    // buffer the oldest sample is at s_head itself; with a partially-filled
    // buffer (s_count < kRingSize) the oldest is at index 0 (capture_start()
    // reset s_head to 0, so the first s_count writes landed at 0..s_count-1
    // in order -- no wraparound has happened yet).
    //
    // Overflow behavior, disclosed: once s_count saturates at kRingSize the
    // ISR keeps writing and silently overwrites the OLDEST unread samples.
    // For this task that is the right trade (the interesting part of an OOK
    // burst is the most recent edges, and a burst is re-sent 4-8x anyway),
    // but a caller that reads exactly capture_capacity() samples should
    // assume it lost older ones.
    size_t n = s_count < max ? s_count : max;
    size_t start = (s_count < kRingSize) ? 0 : s_head;
    for (size_t i = 0; i < n; i++) {
        out[i] = s_ring[(start + i) % kRingSize];
    }
    s_count = 0; // consumed
    portEXIT_CRITICAL(&s_mux);
    return n;
}

} // namespace Rf433Common
