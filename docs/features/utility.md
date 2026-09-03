# Utility

One launcher tile, **Ping Satellite**, under **Utility**.

## Ping Satellite

Sends a single C2 ping frame to the paired Cardputer-ADV satellite over
whichever transport is currently connected — WiFi C2 is preferred if it's
up, otherwise BLE C2. There is no dedicated screen: tapping the tile sends
the ping immediately and returns to the launcher.

Result is reported on the serial console only (`ping sent via WiFi/BLE, OK`
or `FAILED`, or `ping not sent, no transport connected` if neither link is
up) — there is no on-screen confirmation. Use this to sanity-check that
pairing succeeded and a transport is actually connected before relying on
satellite-dependent features elsewhere in the app.
