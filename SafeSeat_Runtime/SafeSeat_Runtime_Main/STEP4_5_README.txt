SafeSeat Runtime Main - Step 4.5 Scheduling Fix
=================================================

Purpose:
Fix timing BEFORE Sensor Fusion without changing proven sensor acquisition.

Changes:
1. C1001 UART update moved to ESP32 core 0.
   - Existing C1001.cpp/.h are byte-for-byte unchanged.
   - Its own 1 Hz gate, warm-up, filtering, motion handling remain unchanged.
   - This prevents slow UART library calls from starving I2C sensors.

2. FSR cooperative 9-channel scheduler.
   - ADS1115 restored at 860 SPS now that the real calibration bug
     (negative whole-frame rejection) is already fixed.
   - One FSR channel is sampled per update() call.
   - MPU gets loop time between FSR channels.
   - Full FSR frame target remains 80 ms (~12.5 Hz).
   - 5-sample median and adaptive filtering are preserved.

3. MPU sampling diagnostics reset AFTER FSR startup calibration.
   - Startup blocking time no longer lowers the displayed runtime rate.

4. Backrest-to-cushion ratio finite guard.
   - denominator is cushionTotal + 1.0, matching the proven old sketch.
   - no more 'ovf' when cushion pressure is zero.

Files changed:
- SafeSeat_Runtime_Main.ino
- FSR.cpp / FSR.h
- MPU.cpp / MPU.h

Files intentionally unchanged:
- C1001.cpp / C1001.h
- MLX.cpp / MLX.h
- Fusion.cpp / Fusion.h

Expected after upload:
- FSR full-frame Fs should move toward ~12.5 Hz.
- MPU should become dramatically higher than the previous ~0.3 Hz.
  Exact MPU rate depends on shared-I2C FSR workload.
- Back/Cushion must remain finite.
- All 4 sensors should remain READY.

If MPU still cannot approach the trained-model target after this cooperative
scheduler, that is a separate shared-I2C throughput constraint to solve before
embedded MPU feature inference; it does not block Fusion state architecture.
