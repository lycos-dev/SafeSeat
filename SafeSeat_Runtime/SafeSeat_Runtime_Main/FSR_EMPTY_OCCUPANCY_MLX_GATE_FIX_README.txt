SAFESEAT COMBINED RUNTIME — FSR EMPTY-SEAT / MLX OCCUPANCY FIX
2026-08-23
================================================================

OBSERVED IN THE COMBINED LOG
----------------------------
After the GAIN_ONE correction, MPU timing recovered to ~79.0-79.5 Hz and
FSR frame cadence recovered to ~4.5 Hz.

However, with nobody sitting on the seat, the cushion channels drifted above
their just-calibrated empty baselines. The old runtime had two overly-permissive
occupancy shortcuts:

1) pressure occupancy could trigger at only 1800 total units with two channels;
2) Main/Fusion/FSRML also treated >300 units as enough in fallback paths.

That allowed unloaded FSR drift to falsely mark the seat OCCUPIED, which then
opened the MLX personal-baseline session even though nobody was present.

FIXES
-----
1. Kept ADS1115 GAIN_ONE and 860 SPS.
2. Kept MPU target timing and cooperative service checkpoints (~80 Hz).
3. Extended empty-seat calibration:
     4 s settle
     60 samples
4. Added fast adaptive empty-seat baseline tracking:
     alpha = 0.12
   It runs only while:
     - C1001 does not confirm a person,
     - FSR pressure occupancy is not latched,
     - that channel is below a strong-press freeze level.
5. Strong deliberate FSR presses are NOT learned into the baseline.
6. Added pressure-occupancy hysteresis/persistence:
     ENTER: >= 12000 total common units
            AND >= 2 active sensors
            for 3 consecutive FSR frames
     EXIT : <= 2500 total
            for 8 consecutive FSR frames
7. Removed the old 300-unit occupancy shortcuts from:
     Main MLX session gate
     Fusion occupancy
     FSR ML occupancy gate
8. MLX physical sensor acquisition may still run in the background, but:
     - no MLX personal baseline is built,
     - no MLX model inference session starts,
   until either:
     - debounced FSR occupancy is confirmed, or
     - fresh C1001 presence is confirmed.

WHY THE MLX STILL PRINTS AMBIENT/OBJECT WHEN EMPTY
---------------------------------------------------
Those are physical sensor telemetry values. They are NOT treated as occupant
temperature evidence while the thermal session says:

  DISABLED - SEAT EMPTY / NO OCCUPANCY

The important behavior is that baseline/model/fusion thermal evidence remains
gated until occupancy is real.

SERIAL NOTE
-----------
The application runs at 921600 baud to preserve MPU timing. The ESP32 ROM boot
message itself is emitted at 115200, so a short garbage line BEFORE the SafeSeat
banner can appear when the Serial Monitor is set to 921600. This is harmless.
After the SafeSeat banner, output must be readable.

VALIDATION TARGET
-----------------
Empty seat:
  FSR Pressure occ   = EMPTY / NOT CONFIRMED
  FSR Baseline track = ACTIVE (when drift exists)
  MLX Thermal session= DISABLED
  MLX baseline       = 0/30
  MPU Actual Fs      = about 79-81 Hz

Then sit normally:
  pressure occupancy latches after ~3 FSR frames
  MLX thermal session becomes ENABLED
  MLX 30-second personal baseline begins
  FSR baseline tracking freezes

This patch does NOT retrain any FSR, MLX, or MPU model.
