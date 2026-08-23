SAFESEAT STEP 5.9.8.22 — FSR ANTI-REBOUND RE-ARM
2026-08-23
==================================================

The live Step 5.9.8.21 log showed:
- t=139: exit=2/6
- t=140: occ=NO at ~23.7k, 2 active FSRs
- t=141: occ=YES again at ~22.9k, 2 active FSRs

That immediate re-entry was residual foam/FSR pressure, not a new passenger.

FIX:
After any confirmed exit:
- occupancy remains NO
- new occupancy detection is temporarily disarmed
- re-arm only after <=1 active FSR persists for 6 completed FSR frames
  (~1.3 s at ~4.5 Hz)
- after re-arm, a real occupant still needs the original >=12k total
  + >=2 active FSRs for 3 frames

The physical pressure is NOT forced to zero. A residual 14k-16k on one FSR
may still print while occupancy correctly stays NO.

UNCHANGED:
FSR ML model/features, MLX behavior, MPU, Fusion, tiny ML heartbeat,
Serial 460800, no application watchdog.
