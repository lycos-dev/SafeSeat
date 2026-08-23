SAFESEAT MAIN HUB — STEP 5.9.8.14 COMPILATION FIX
2026-08-23
===================================================

This package fixes the three compile failures in Step 5.9.8.13:

1. readyText() was accidentally removed while stripping the watchdog code.
   Restored exactly.

2. mapSensorHealth() was accidentally removed in the same edit.
   Restored exactly.

3. Compact MPU telemetry incorrectly used:
      m.dynamicAccelG
   The actual MPUReading field is:
      m.dynamicAcceleration
   Corrected.

IMPORTANT
---------
The no-reboot design is preserved:
- no SafeSeat-added task watchdog
- no intentional SafeSeat watchdog panic/reboot
- compact [LIVE] telemetry every 1 second
- compact detail telemetry every 5 seconds
- giant periodic Serial dashboard disabled
- Serial runtime baud remains 460800
- MPU 80 Hz scheduling logic unchanged
- FSR/MLX/model/fusion logic unchanged

Recommended:
  Tools > Upload Speed = 115200
  Serial Monitor       = 460800
