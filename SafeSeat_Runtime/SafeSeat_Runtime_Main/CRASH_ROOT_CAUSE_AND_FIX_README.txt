SAFESEAT MAIN HUB — CRASH ROOT CAUSE + NO-REBOOT SERIAL FIX
2026-08-23
============================================================

WHAT THE LOG PROVED
-------------------
The crash was NOT a sensor/model failure.

Immediately before the reboot:
- MPU stayed about 79.8-80.0 Hz.
- FSR stayed about 4.5 Hz.
- heap stayed essentially flat around 200 kB.
- MLX baseline remained 30/30.
- FSR/MPU model windows continued advancing.

The log then explicitly reported:

  task_wdt: Task watchdog got triggered
  loopTask ... did not reset the watchdog in time
  Guru Meditation Error
  Rebooting...

The previous diagnostic build had subscribed Arduino loopTask to an
8-second panic watchdog. It also retained the very large periodic Serial
dashboard. During one dashboard print the loop stopped making progress
long enough for OUR added watchdog to force a panic/reboot.

THIS PACKAGE
------------
1. Removes the application-added task watchdog completely.
   It will not intentionally reboot a passenger session because a debug
   print took too long.

2. Keeps runtime Serial at 460800.

3. Keeps the compact [LIVE] heartbeat every 1 second.

4. Adds compact sensor diagnostics every 5 seconds:
   [DETAIL]
   [MLX]
   [FSR9]
   [MPU]

5. Disables the giant periodic multi-kilobyte Serial dashboard by default.

6. Keeps the local HTTP API active, so full structured telemetry remains
   available without dumping several KB repeatedly over UART.

7. Adds a 1 ms cooperative yield in loop() for Wi-Fi/ESP-NOW/system tasks.

UNCHANGED
---------
FSR gain/scale and mapping
FSR empty-seat adaptive baseline
FSR occupancy hysteresis
MLX occupancy gate
MLX geometry guard
MLX IF/OCSVM
MPU 80 Hz scheduler
MPU IF/OCSVM
Fusion semantics

SETTINGS
--------
Tools > Upload Speed : 115200 recommended
Serial Monitor       : 460800

EXPECTED
--------
You should continuously see:
  [LIVE] t=...
once per second, and compact detail every five seconds.

There should be NO:
  [WATCHDOG]
  task_wdt panic caused by SafeSeat
  intentional SafeSeat watchdog reboot

The ESP32/Arduino platform's own low-level safety mechanisms remain intact.
