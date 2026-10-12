SafeSeat Runtime Main R5.9 - MLX Elevated Temperature Verification
=================================================================
Base: R5.8 Centered Reference + Practical Timing Final
Date: 2026-10-12

Scope
-----
This revision changes ONLY the MLX temperature-model wrapper/diagnostics needed to
reduce false temperature escalation observed during validation. Fusion severity,
FSR, C1001, MPU, camera, occupancy, centered-reference, Warning persistence and
Emergency persistence are unchanged from R5.8.

Observed issue
--------------
The native MLX IF + OCSVM model is baseline-relative. In the supplied validation
recording, a session baseline around the low-30 C range allowed a later ~32-33 C
surface reading to remain a strong model anomaly long enough to reach WARNING.
That is too sensitive for the intended SafeSeat thermal use case: a small upward
change from baseline must not be enough by itself to create a temperature Warning.

A separate issue was that a deliberately hot physical object could make a sudden
jump to ~38 C and be classified as TARGET GEOMETRY DEGRADED because the existing
FOV guard correctly treats fast IR steps as possible target movement.

R5.9 MLX-only behavior
----------------------
1. Positive/warming model anomalies below 37.5 C are suppressed from Fusion.
   - The IF + OCSVM scores can still be calculated internally.
   - The deployed anomaly flags are published as NORMAL below the floor.
   - Example: a baseline near 30 C followed by ~32 C no longer becomes a
     temperature Warning simply because the baseline-relative model is sensitive.

2. A genuinely elevated positive anomaly at/above 37.5 C enters a 10-second
   VERIFYING ELEVATED TEMPERATURE state.
   - The 10 seconds are ten continuous stable one-second MLX blocks.
   - If the elevated/anomalous condition disappears, the verification resets.
   - The threshold is an engineering surface-temperature false-positive control,
     NOT a fever/core-temperature diagnostic threshold.

3. A rapid upward transition that is already at/above 37.5 C is no longer trapped
   in TARGET GEOMETRY DEGRADED solely because it rose quickly.
   - It is allowed into the same 10-second elevated-temperature verification path.
   - Rapid non-elevated shifts keep the original R5.8 geometry/FOV guard.

4. The existing three-block model persistence remains conceptually intact and is
   satisfied inside the stricter 10-second verification window. No extra 3-second
   delay is added after verification completes.

5. The existing Fusion timing is unchanged:
   - Once MLX becomes trusted strong anomaly evidence, Fusion still requires the
     existing 10 continuous seconds before WARNING.
   - After WARNING, the existing 30 continuous seconds remain required before
     EMERGENCY.

Practical timing note
---------------------
Because the user explicitly requested the new MLX verification without changing
Fusion, a sustained high-temperature event now has approximately:

  ~10 s MLX elevated verification
  + existing 10 s Fusion Warning persistence
  = ~20 s before WARNING (plus normal block/filter scheduling tolerance)

The 30-second post-WARNING Emergency timer is unchanged.

Unchanged modules / policies
----------------------------
- Fusion.cpp / Fusion.h: unchanged byte-for-byte from R5.8
- FSR acquisition/model and centered-reference behavior: unchanged
- C1001 acquisition/model and motion-artifact gate: unchanged
- MPU acquisition/model/context: unchanged
- Camera event-triggered corroboration: unchanged
- Occupancy/session gating: unchanged
- FSR posture 12 s persistence: unchanged
- Monitoring Limited behavior: unchanged
- Warning persistence: 10 s unchanged
- Emergency persistence after Warning: 30 s unchanged

Validation-console diagnostics
------------------------------
The MLX runtime API now also reports:
- elevated_temperature_qualified
- elevated_temperature_verifying
- elevated_verification_blocks
- elevated_verification_required_blocks
- elevated_temperature_floor_c

The Validation recorder exports these fields so the 10-second verification can be
shown directly in future CSV evidence.

Build note
----------
Host-side C++ syntax and behavioral harness checks were run for the modified MLX
wrapper. A final ESP32 board Verify/Compile should still be run in the user's
Arduino/ESP32 environment before flashing.
