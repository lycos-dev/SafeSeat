SAFESEAT SERIAL STALL GUARD / LIVE HEARTBEAT — 2026-08-23

The prior combined run produced valid data for roughly a minute and then
Serial output stopped completely. The log contains no reset banner or exception,
so it cannot distinguish a true firmware stall from a 921600 USB-UART/Serial
Monitor issue.

Changes:
- Runtime Serial: 460800
- Recommended Tools > Upload Speed: 115200 (independent of runtime Serial)
- TX buffer: 8192 bytes
- Compact [LIVE] line every 1 second
- Full dashboard every 10 seconds
- 8-second ESP32 task watchdog after startup calibration
- RTC-retained runtime stage for post-reset stall diagnosis

If firmware truly blocks for >8 s, it reboots and the next boot prints:
  [STALL-DIAG] Previous reset/stall stage: N - <stage>

This patch does NOT change FSR, MLX, MPU model logic or sensor thresholds.
