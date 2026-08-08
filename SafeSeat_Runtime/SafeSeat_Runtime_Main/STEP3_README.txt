SafeSeat Runtime Main - Step 3 FSR Restoration
================================================

Changed:
- FSR.h
- FSR.cpp

Preserved unchanged:
- SafeSeat_Runtime_Main.ino from Step 1
- Config.h from Step 1
- C1001 module
- MLX Step 2 module
- MPU module
- Fusion.cpp / Fusion.h

Restored directly from the user's proven combined sketch:
- ADS1115 #1 at 0x48
- ADS1115 #2 at 0x49
- GAIN_ONE
- no forced 860 SPS data rate
- GPIO34 CushionRight
- 5-sample median on every channel
- adaptive press/release filter
- 20-round empty-seat calibration after 3-second delay
- 20 ADC-count noise margin
- old empty-seat baseline drift behavior
- old pressure/posture context

Critical fix:
The failed modular FSR code rejected the entire nine-sensor sample if ANY
ADS1115 median value was negative. Near-zero ADS1115 single-ended readings
can occasionally be slightly negative due to offset/noise. The proven old
sketch never rejected these samples, so Step 3 removes that whole-frame
negative-value rejection.

Test:
Keep the chair completely empty during startup calibration.

Next:
Step 4 restores MPU6050 from the proven raw-I2C implementation.
