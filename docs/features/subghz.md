# Sub-GHz (CC1101)

Tab5-native sub-GHz radio suite built around a CC1101 module wired to the
Tab5's M-Bus connector (a dedicated interface — this hardware does not
share pins with NFC/RF433/IR's HY2.0 PORT.A arbiter). Five launcher tiles
live under **Sub-GHz**: **CC1101 Scan/Decode**, **CC1101 Spectrum**,
**CC1101 Bruteforce**, **CC1101 Jammer**, **CC1101 KeeLoq**.

The module's antenna/PA matching is built for 855-925MHz, but the chip
itself can tune 300-348MHz and 387-464MHz too (real CC1101 silicon limits,
not a software restriction) — expect degraded range/sensitivity outside
855-925MHz, not a hard failure.

## CC1101 Scan/Decode

Capture, decode, save, replay, and combine sub-GHz signals — the hub screen
this category is built around.

### Using it

1. Open **Sub-GHz > CC1101 Scan/Decode**.
2. Tap **Next Freq** to cycle the working frequency (315.00, 433.92, 868.00,
   868.35, 915.00, 925.00 MHz) — disabled while a capture or Hot/Cold session
   is running.
3. Tap **Start Capture** to arm the receiver. Any OOK burst on the air is
   recorded; a 25ms gap with no edges closes one signal and starts listening
   for the next, so multiple presses of a remote during one capture session
   show up as separate rows. Tap **Stop Capture** to disarm.
4. Each captured signal appears in the list as `Sig #N: <edges> edges,
   ~<duration>ms @ <freq>MHz`, with `[truncated]` appended if it hit the
   131,072-edge-per-signal cap. If the signal matches a known fixed-code
   protocol, a decode line appears above the list (`Sig #N decoded: <name>
   key=0x... (<bits> bits)`).
5. Tap a row to select it (used by Replay Selected / Save as .sub).
6. **Replay Selected** re-transmits the selected signal's real captured
   timing on the current frequency. **Save as .sub** writes it to
   `/quarky/sub/captures/capture_<id>.sub`.
7. **Load from SD** opens a deep folder browser rooted at `/quarky/sub`
   (same recursive, dotfile-filtering navigation as the IR screens' Flipper
   folder browser) — filtered to `.sub` files, including the bundled
   `/quarky/sub/flipperdb` signal database. **Replay Loaded** transmits
   whatever was last loaded this way.
8. **Hot/Cold: Start** puts the radio into a continuous RSSI-sampling mode
   instead of edge capture (mutually exclusive with Start Capture — stop one
   before starting the other). It spends the first ~3 seconds (30 samples)
   learning a baseline noise floor, then shows live `RSSI: X dBm (baseline
   Y, delta +Z)` with a 0-100 bar — higher/closer to full means you're
   physically closer to whatever is transmitting. Useful for walking down a
   signal source by hand rather than reading raw dBm numbers.

### Chaining signals into one file

1. Tap **Select** to enter select mode (label changes to `Select: ON`); a
   `Chain (n/8)` status line appears listing the signal IDs you've tapped,
   in the order you tapped them.
2. Tap captured rows to add/remove them from the chain, up to 8 signals.
3. Tap **Combine -> .sub**. The chained signals' real edge timing is
   concatenated in tap order (each segment separated by the same 25ms gap
   used to delimit bursts during capture) into one synthetic signal, saved
   as `/quarky/sub/captures/chain_<id>.sub`. The combined signal is capped
   at 65,536 edges — long enough for realistic chains, but a chain
   combining several near-maximum-length captures can truncate (flagged
   `[truncated]` in the status line if so).
4. A successful combine clears the chain and exits nothing — tap **Select**
   again to build a new chain.

### Known limits

- Signal list holds a bounded number of captures; capturing past that
  silently evicts the oldest to make room for the newest.
- Starting a capture while a CC1101 Replay transmit is in flight (from this
  screen, or from KeeLoq's Replay +1) is refused — the two share the radio
  and GDO0 pin.
- A capture overrun (radio noise/interference producing edges faster than
  the ISR ring buffer drains) stops the capture automatically and shows
  `Status: Overrun -- check antenna`.

## CC1101 Spectrum

A live RSSI bar chart sweeping the module's full documented range.

### Using it

Open **Sub-GHz > CC1101 Spectrum**. The chart auto-starts: 71 bars, one per
1MHz step from 855 to 925MHz (the module's full documented range, not the
narrower list of common frequencies the other screens use), each retuned
and read roughly every 30ms. The three labels under the chart (855 / 890 /
925) anchor the start, middle, and end of the sweep — there's no room for a
label under every bar. Leave the screen to stop; the radio returns to idle.

## CC1101 Bruteforce

Fixed-code rolling sweep against six common garage/gate remote protocols
(Came, Nice, Ansonic, Holtek — 12-bit; Linear — 10-bit; Chamberlain — 9-bit).
**Authorized targets only.**

### Using it

1. Open **Sub-GHz > CC1101 Bruteforce**.
2. Tap a protocol button to start immediately — there's no separate
   "confirm" step, and no frequency picker (fixed at 433.92MHz, the
   standard band for these fixed-code remotes).
3. While running, the status line shows `<protocol>: code <n> / <total>`
   with a matching progress bar. A full 12-bit sweep is 4096 codes, each
   sent twice (the fixed repeat count this screen uses).
4. Tap **Stop** to abort early — the running task also stops automatically
   if you back out of the screen.

Only one sweep can run at a time; tapping a protocol button while another
is already running does nothing (use Stop first).

## CC1101 Jammer

Continuous or intermittent RF jamming on the current 433.92MHz-fixed
frequency. **Real regulatory exposure — confirm authorization before
starting**, as the on-screen note itself states.

### Using it

1. Open **Sub-GHz > CC1101 Jammer**.
2. Tap **Full** (continuous transmission) or **Intermittent** (pulsed) to
   start immediately.
3. While active, a red `*** ACTIVELY TRANSMITTING -- JAMMING ***` banner is
   shown alongside a live `<MODE>: <pulses> pulses, <ms> ms` counter — this
   screen is designed so it's never ambiguous whether the radio is
   currently on the air.
4. Tap **Stop Jamming** to end the session. Leaving the screen also stops
   it.

Both jamming modes run with no built-in time limit — they run until you
stop them.

## CC1101 KeeLoq

Rolling-code KeeLoq decode and single-step replay attack, layered on top of
CC1101 Scan's captures. **Owner-authorized equipment only.**

### Using it

1. Capture a signal with **CC1101 Scan/Decode** first — this screen has no
   capture capability of its own.
2. Open **Sub-GHz > CC1101 KeeLoq**. The manufacturer-key count loaded from
   `/quarky/keeloq/mfcodes` is shown at the top; the signal list below shows
   every capture currently held by the Scan screen.
3. Select a signal, tap **Decode Selected**. Three outcomes:
   - `Not a KeeLoq frame` — the capture doesn't match KeeLoq's protocol
     shape at all.
   - `KeeLoq frame found, btn=... serial=0x... -- no matching manufacturer
     key` — a real KeeLoq frame, but no loaded key can decrypt the rolling
     portion (button/serial are readable in the clear; counter value is
     not).
   - `KeeLoq: mfr=<name> btn=<n> serial=0x... cnt=<n>` — fully decoded,
     including the current counter value. Only this outcome enables Replay.
4. On a full decode, **Replay +1 (ACTIVELY ATTACKING)** becomes tappable.
   Tapping it transmits a freshly-encoded frame with the counter
   incremented by one (a real rolling-code step, not a raw replay of the
   captured bytes) behind the same unambiguous red "ACTIVELY ATTACKING"
   banner style the Jammer screen uses.

This screen has not been verified against a real captured rolling-code
frame on real hardware as of this writing — treat decode/replay results as
unverified until confirmed against a real target.
