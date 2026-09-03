# IR

Tab5-native infrared suite built around the M5Stack "Unit IR" (SKU U002), a
plain GPIO transistor-driven 940nm TX LED plus an IRM-3638T 38kHz-demodulating
RX module — not an I2C device, no chip protocol at all. Both wires ride the
Tab5's single HY2.0 PORT.A connector (GPIO53=TX, GPIO54=RX), the same
connector NFC/RFID2 and RF433 use, arbitrated so only one owner holds it at a
time. Four launcher tiles live under **IR**: **IR Learn**, **IR Clone
(Flipper-IRDB)**, **IR Jammer**, **TV-B-Gone**.

### Shared limit: PORT.A is single-owner

Every IR screen claims PORT.A (via the same GPIO53 arbiter NFC/RFID2 and
RF433 use) before it can transmit or capture, and releases it right after.
If NFC/RFID2 or RF433 already holds it, the IR screen's action fails
outright with an on-screen message naming the conflict (e.g. "PORT.A busy
(NFC/RFID2 or RF433)" or "Failed to start — PORT.A is held by another owner
(NFC/RFID2 or RF433)") rather than queuing or silently doing nothing. Close
the other feature's screen first.

Carrier frequency (38kHz) and duty cycle (1/3) are the same well-established
consumer-IR defaults across every screen in this category — none of them are
measured from real hardware, because the IRM-3638T strips the carrier in
hardware before a receive ever reaches GPIO54, and this class of IR receiver
is broadly duty-cycle-tolerant enough that the defaults work regardless.

## IR Learn

Captures one raw IR signal from a real remote and can save it to SD as a
`.ir` file. No named-protocol decode (no NEC/RC5/Sony bit parsing) — this is
a raw pulse-timing recorder only.

### Using it

1. Open **IR > IR Learn**.
2. Tap **Start Capture**. The status line changes to "Waiting for
   remote...", and the button becomes **Cancel**.
3. Point a real remote at the IR unit's receiver and press a button.
   Capture ends automatically 8ms after the last edge (a starting value,
   not yet hardware-confirmed against every remote's repeat-frame timing —
   see below).
4. On success, the status line reads "Captured N pulses (~X.Y ms)", with
   carrier/duty noted as assumed (38000 Hz, 0.33) since they can't be
   measured from this receiver. Tap **Save to SD (.ir)** to write it to
   `/quarky/captures/ir/learn_<ms>.ir`, named `Learned_<ms>` inside the
   file. The save button is disabled until a capture completes.
5. Tap **Cancel** at any point during capture to abort, or **Back** to
   leave — both release the RMT receiver and the PORT.A claim immediately
   so NFC/RFID2/RF433 aren't locked out.

### Status line meanings

| Status | Meaning |
|---|---|
| Idle | No capture started yet this session. |
| Waiting for remote... | Armed; nothing captured yet. |
| Captured N pulses (~X.Y ms) | Success. `[TRUNCATED]` appended if the signal exceeded the 1024-pulse cap. |
| Failed — PORT.A busy (NFC/RFID2 or RF433) | Another feature holds PORT.A; capture never armed. |
| Failed — No signal detected (timed out) | The 8ms idle gap fired before any real mark was seen — nothing was pressed, or the remote's signal never registered. |

### Real limits and gotchas

- **Idle-gap threshold (8ms) is a starting value, not yet confirmed on real
  hardware.** It's sized to clear a typical NEC-family ~4.5ms header space
  with margin while staying under typical auto-repeat gaps (tens of ms), but
  if a specific remote's capture cuts off mid-frame or merges multiple
  button presses into one capture, this is the value to revisit.
- **Capture caps at 1024 pulses** (matching the on-disk `.ir` format's own
  raw-data limit). Anything beyond that is dropped and the result is flagged
  `[TRUNCATED]`, not rejected.
- **Only one capture is held at a time.** Starting a new capture overwrites
  the previous result; there's no history list like some other capture
  screens in this project keep.
- Carrier/duty are always the assumed defaults (38000 Hz, 1/3), never a real
  measurement — noted explicitly in the status line so this isn't mistaken
  for a real reading.
- Closing the screen mid-capture (Back) cancels it the same as tapping
  Cancel; nothing is left running in the background.

## IR Clone (Flipper-IRDB)

A universal-remote screen that browses the community Flipper-IRDB corpus
bundled on the SD card and transmits any command from it. This is the
category's deepest screen — a real recursive folder browser, not the
project's generic flat file picker.

### Using it

1. Open **IR > IR Clone (Flipper-IRDB)**. This opens a folder browser rooted
   at `/quarky/ir`.
2. Tap a folder row to descend into it — the real on-SD corpus is
   inconsistently deep (some categories are 2 levels under the root, others
   4+), so how many taps it takes to reach a real `.ir` file varies by
   category. Tap **Back** to go up one level; there's no separate "up"
   button, Back doubles as it.
3. Tap a `.ir` file to open it. Each named signal inside the file becomes
   its own button in a 4-column grid, labeled with that signal's real name
   (a real file typically holds ~11-12 signals, but some hold more).
4. Tap a signal button to transmit it immediately. The status line reports
   `Sent '<name>' (raw)` or `Sent '<name>' (<protocol>)` on success, or a
   failure reason otherwise.
5. Tap **Back** repeatedly to retrace the folder path, or back out to the
   launcher.

### What's filtered from the browser

- **Dotfiles are always hidden**, including macOS AppleDouble sidecar files
  (`._Something.ir`) that come from the corpus having been written to the
  SD card via Finder — every real `.ir` file on this card has one of these
  sitting next to it, and they'd otherwise pass the same `.ir` extension
  filter as the real file if not excluded.
- Each directory listing is capped at 128 subdirectories and 128 `.ir`
  files. If a real directory somehow exceeds that, entries past the cap are
  silently omitted with no on-screen indication — a known, disclosed gap,
  not expected to trigger against the real bundled corpus.
- A directory scan visits at most 512 raw entries (including the dotfile
  sidecars, which double the real per-file cost) before giving up on
  finding more matches — again, sized well above the real corpus's largest
  observed folder.
- A file screen shows at most 32 signal buttons; a file with more signals
  than that is flagged "This file had more signals than fit — some are not
  shown."
- If a directory read fails outright (shown as "Could not read this
  directory — SD card read failed...") rather than showing "Empty
  directory," that's a distinct, real condition — usually a busy WiFi/BLE
  radio session leaving too little DMA-capable memory free for the SD
  read, not an actually-empty folder. Try again or close other radio
  features first.

### Transmit protocol support

- `raw`-type signals transmit directly, whatever their captured timing.
- `parsed`-type signals are only supported for **NEC** and **NECext**
  protocols (encoded fresh from the stored address/command bytes using
  real Flipper-firmware-derived timing). Any other protocol name found in
  the corpus (Samsung32, RC5, RC6, SIRC, Kaseikyo, Pioneer, etc.) is
  refused outright with an on-screen "Protocol not supported: <name>"
  message — not silently ignored, not mis-encoded. The real bundled corpus
  is overwhelmingly NEC/NECext, so most files work; anything else won't
  transmit from this screen.
- Directory nesting has a real practical ceiling: root + `flipperdb` + up
  to several more folder levels + a file's own signal screen all count
  against the app's overall screen-stack depth limit. The deepest real
  paths confirmed in this corpus fit under that limit but with little
  headroom — a branch even one level deeper than what's been checked could
  silently stop tracking depth rather than fail cleanly. Not expected in
  practice, but worth knowing if navigation ever behaves oddly on an
  unusually deep folder.

## IR Jammer

Transmits continuous randomized IR "noise" intended to disrupt a nearby real
IR receiver's ability to lock onto a coherent signal. No named protocol —
just short, randomized mark/space bursts at the standard 38kHz carrier,
timed to stay inside the range real consumer-IR receivers can't filter out
as background noise.

### Using it

1. Open **IR > IR Jammer**.
2. Point the IR unit's TX LED at the target receiver.
3. Tap **Start Jamming**. The status line reads "Jamming — transmitting
   continuous IR noise." while active.
4. Tap **Stop** to end the session, or **Back** to leave — either stops
   transmission and releases the PORT.A claim. A session left running when
   the screen is closed is stopped automatically, not left running in the
   background.

### Gotchas

- If PORT.A is already held by NFC/RFID2 or RF433, Start fails immediately
  with "Failed to start — PORT.A is held by another owner (NFC/RFID2 or
  RF433)" and nothing transmits.
- Each burst is a fixed 20 mark/space pairs with each duration randomized
  between 150-900µs, sent back-to-back once per main loop tick for as long
  as jamming is active — this is a continuous session, not a fixed-duration
  or fixed-count sweep. It runs until you stop it.
- If a transmit call fails mid-session (e.g. a hardware fault), jamming
  stops itself automatically and logs the reason to serial; there's no
  separate on-screen error state for this beyond reverting to the idle
  status text.

## TV-B-Gone

Sweeps the classic Gen3 TV-B-Gone power-off code database over the IR TX
LED — the same database (and unpacking algorithm) as the original TV-B-Gone
keychain device, targeting one of two regional code sets.

### Using it

1. Open **IR > TV-B-Gone**.
2. Point the IR unit's TX LED at the target device.
3. Pick a region: **NA** (137 codes) or **EU** (148 codes). Region can only
   be changed while idle — both buttons disable during an active sweep.
4. Tap **Start**. The status line shows `Sending <region> code X / N`, and
   the progress bar fills as the sweep advances.
5. Codes are sent roughly 205ms apart (the same inter-code gap the original
   TV-B-Gone hardware used, giving a real TV time to react before the next
   code). A full NA sweep is on the order of 30+ seconds.
6. Tap **Stop** at any point to end early — the status line reports how far
   the sweep got (`Stopped by user at X / N`). Tap **Back** to leave; a
   sweep in progress is stopped automatically, not left running.
7. On natural completion, the status line reads "Done — sent all N <region>
   codes."

### What this screen does not do

- **There is no brand/manufacturer selection.** This is a database-wide
  region sweep only — NA or EU, nothing narrower. If you know the target's
  brand, there's no way to target just that brand's codes from this screen.
- **No feedback on whether any code actually worked.** Like the original
  TV-B-Gone hardware, this transmits blind — there's no receive-side
  confirmation that a given TV responded. Watch the target device itself.
- If PORT.A is held by NFC/RFID2 or RF433 when Start is tapped, the sweep
  never begins and the status line reports "Failed to start — PORT.A is
  held by another owner (NFC/RFID2 or RF433)."
- If a transmit fails partway through (hardware fault), the sweep stops
  itself and reports "Stopped — transmit failed at code X" rather than
  continuing to send blind or retrying.
