# WiFi Evil Portal

Tab5-native captive-portal credential-capture tool (WiFi category, launcher
tile "Evil Portal"). Broadcasts an open AP with a configurable name, serves
a login-style landing page to anyone who connects, and logs whatever they
submit.

## Using it

1. Open **WiFi > Evil Portal**.
2. Enter an SSID (optional — defaults to `QuarkyPortal` if left blank).
3. Pick a template from the dropdown: **Built-in (Wi-Fi Login)**, or any
   custom `.html` file found on the SD card (see below).
4. Tap **Launch Portal**. The AP comes up, and the on-screen status updates
   to show the live SSID.
5. Anyone who joins the AP and submits the form has their input captured —
   shown live in the on-screen list, and written to a CSV file on SD (see
   *Captured data*, below).
6. Tap **Stop Portal** to take the AP down without leaving the screen (so
   you can review the captured list, or configure a new SSID/template and
   launch again). Tapping **Back** also stops the portal, in addition to
   leaving the screen.

### Side effect: this takes the WiFi C2 link down for the rest of the boot

ESP32 WiFi supports exactly one SoftAP configuration at a time, so launching
the portal takes over the Tab5's AP identity: the WiFi C2 link's own
`Quarky-Tab5-Test` SoftAP — the transport the Cardputer-ADV satellite joins —
goes down as soon as the portal comes up.

**It does not come back by itself.** Stopping the portal (Stop Portal, or
Back) calls `WiFi.softAPdisconnect(true)`, which tears the portal's AP down
and disables AP mode entirely; nothing re-initializes `c2link_wifi`, so the
C2 SoftAP stays down until the Tab5 is rebooted. Plan around that: finish
what you need the WiFi C2 link for before opening this screen, or reboot
afterwards. (If the satellite is paired over the BLE C2 transport instead,
that link is unaffected — this is a WiFi-only side effect.)

Rebooting with the portal running does not preserve it either: the portal is
never restarted automatically at boot, so a reboot mid-session comes up with
the normal C2 SoftAP and no portal. Reopen this screen and tap Launch Portal
again if you want the portal back.

## Custom templates

Drop `.html` files into `/quarky/portals/` on the Tab5's SD card. Every
`.html` file found there appears in the template dropdown by filename, with
no reflash needed — the SD card is rescanned every time the Evil Portal
screen opens.

**A custom template must follow this exact contract to work with this
firmware:**

- The login form must `POST` to `/submit` — e.g. `<form method="POST"
  action="/submit">`. Any other action path will not be handled (the
  firmware serves your page at every request except `/submit`, matching
  standard captive-portal catch-all behavior, so navigating anywhere shows
  your page — but only `/submit` is wired to actually capture input).
- The two fields you want captured must be named exactly `user` and `pass`
  — e.g. `<input name="user" ...>` and `<input name="pass" ...>`. Any other
  field name is silently ignored; the firmware only reads `user`/`pass`
  from the POST body. You don't have to use both — a form with just one of
  the two still works, the other is simply logged as an empty string.
- Field values are read as-submitted with no validation or sanitization
  beyond what the browser's own `required`/`type` attributes enforce
  client-side — a real user can submit empty strings, arbitrary length
  input, etc. Don't rely on the firmware to enforce input shape.

Minimal working template:

```html
<!DOCTYPE html>
<html>
<body>
<form method="POST" action="/submit">
  <input name="user" placeholder="Email">
  <input name="pass" type="password" placeholder="Password">
  <button type="submit">Sign in</button>
</form>
</body>
</html>
```

**Limits:**
- Template files are read fully into RAM at Launch time, capped at 8KB.
  Anything larger is rejected (logged to serial, falls back silently to
  the built-in template) — keep custom templates lightweight; this is a
  simple login-form page, not a place for large embedded images/fonts.
- Up to 8 `.html` files in `/quarky/portals/` are listed in the dropdown.
  Extras beyond that are ignored by the scan.

## Captured data

Every submission is:
- Shown live in the on-screen list as `user / pass`.
- Logged to the serial console.
- Appended as a CSV row (`ms,user,pass`) to
  `/quarky/captures/wifi/evil_portal_<launch-timestamp>.csv` on the SD
  card — one file per Launch session (stopping and re-launching starts a
  new file, so separate sessions never interleave into one log).

Pull the SD card (or add SD access via the C2 link, once available) to
retrieve captured credential files.
