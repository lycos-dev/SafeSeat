SAFESEAT STEP 5.9.8.20
ROBUST FSR EXIT + MPU STARTUP RETRY + 460800 MINIMAL SERIAL
2026-08-23
============================================================

WHAT THE 5.9.8.19 LOG ACTUALLY SHOWED
-------------------------------------
- No Guru Meditation / LoadProhibited / reboot appeared in the supplied log.
- FSR acquisition itself continued steadily at about 4.54-4.55 Hz.
- The pressure occupancy latch stayed YES after the passenger left.
- MPU did not run because its very first 14-byte I2C startup read failed.

Therefore this patch does NOT add another crash watchdog or large diagnostic
system. It fixes the two concrete issues visible in that run.

FSR EXIT
--------
Entry stays unchanged:
  total >= 12000 AND >=2 active sensors for 3 frames.

Exit now accepts any sustained condition for 6 FSR frames (~1.3 s):
1) hard empty:
     total <= 2500
2) residual mechanical relaxation:
     total <= 12000 AND <=1 active sensor
3) departure collapse:
     peak seated load >= 20000
     current total <= max(12000, 30% of seated peak)
     backrest total <= 2000
     <=3 active sensors

The seated peak is private FSRSensor state only; FSRReading layout/model input
is unchanged. This avoids interpreting a slowly relaxing cushion as a passenger.

MPU STARTUP
-----------
A single failed shared-I2C read no longer disables the MPU for the whole run.
The exact 14-byte read is retried up to 5 times with wake/range reassertion.

SERIAL
------
Tools > Upload Speed : 115200 recommended
Serial Monitor       : 460800

Only one compact line per second:
  [LIVE] t=... MPU=... FSR=... occ=... P=... act=... exit=x/6 MLX=... base=...

No giant dashboard. No FSR9 dump. No watchdog.
