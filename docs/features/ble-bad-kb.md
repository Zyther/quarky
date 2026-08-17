# BLE Bad-KB

Tab5-native BLE HID keyboard (BLE category, launcher tile "BLE Bad-KB"). The
Tab5 advertises itself as a Bluetooth keyboard named `QuarkyKB`; once a host
pairs with it, a Ducky-script-style payload typed into the on-screen textarea
is sent to that host as real keystrokes.

## Using it

1. Open **BLE > BLE Bad-KB**. The Tab5 starts advertising as `QuarkyKB`.
2. Pair from the target host (macOS/Windows/Linux/Android Bluetooth settings).
   The status label tells you where you are:
   - `Advertising as QuarkyKB...` — waiting for a host to pair.
   - `Paired` — a host is connected; keystrokes can be delivered.
   - `Not paired -- cannot send` — advertising never started. Real causes:
     the BLE host never came up (the "radios disabled for this boot" mode), or
     a previous session's connection is still tearing down. This label is
     derived from the actual result of the advertise call, not assumed — if it
     says this, nothing is on the air.
3. Type a script into the textarea, tap **Send**.
4. Tap **Back** to stop advertising, disconnect any host, and leave.

Typing is spread across main-loop ticks (one action per tick), not sent in one
blocking burst, so the UI stays responsive during a long script. A long script
therefore takes visibly longer than a "paste" would — that is by design.

## The supported script subset

This is a **reduced** Ducky-script dialect, not the full language. What is
actually implemented:

| Line | Effect |
|---|---|
| `STRING <text>` | Types `<text>` literally, up to the end of the line |
| `ENTER` | Sends the Enter key |
| (a bare blank line) | Also sends Enter |
| anything else | **Silently skipped** — not typed, not reported |

Known limits, all real and all worth planning around before you rely on a
payload:

- **No shift, so no uppercase and no symbols.** `A`–`Z` are mapped to the same
  keycodes as `a`–`z` with no shift modifier, so `STRING Hello` arrives as
  `hello`. Only `a`–`z`, space and Enter produce output at all; digits,
  punctuation and everything else map to keycode 0 and are dropped.
- **No `DELAY`.** There is no way to wait for a target window to open, a
  Spotlight/Run box to appear, etc. Pacing is fixed per keystroke.
- **No modifiers** (`CTRL`, `ALT`, `GUI`/`WINDOWS`, `SHIFT`) and no key names
  beyond `ENTER`.
- **Unrecognized lines vanish silently.** A payload written for a full Ducky
  interpreter will not error — it will simply do less than you expect. Check
  what actually arrives on the target rather than assuming the script ran.
- Scripts are capped at 511 characters, enforced by the textarea itself.

A fuller command set (modifiers, `DELAY`, a shift-aware keycode table) is real,
considered future work, not a placeholder — the reduced set ships a working HID
typer today.

## Side effects on other features

- **BLE C2 goes down while this screen is open.** Legacy BLE advertising is
  single-instance system-wide, so advertising as `QuarkyKB` stops the Tab5's
  own C2 advertisement (`Quarky-Tab5`). It is restored automatically when you
  leave the screen (`c2link_ble_rearm_advertising()`), so this is a
  for-the-duration outage, not a for-the-boot one.
- **Run BLE Clone / Karma / Sour Apple / Find My first and pairing may
  break.** Those four features set a host-wide random BLE identity that has no
  "unset" API, and this feature infers its own advertising address from it. A
  host that already bonded with `QuarkyKB` will not recognise the leftover
  identity and will treat it as a new device needing pairing — and the bond
  store holds only 3 entries. **Reboot the Tab5 before a Bad-KB session that
  depends on an existing pairing.**
- The HID GATT service (0x1812) stays in the ATT database for the whole boot
  once registered — there is no safe runtime removal. Both its Report Map and
  Report characteristics require encryption, so a peer must pair before it can
  read the descriptor or subscribe to keystrokes.
