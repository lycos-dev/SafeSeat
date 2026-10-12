SafeSeat Runtime Main R5.7 - Adaptive Validation + Critical Sensor Escalation
============================================================================
Base: R5.6 System Validation Final

Purpose
-------
This revision addresses issues observed during the physical vehicle recording:
- validation no longer needs a manual Begin step;
- returning to live readings no longer exits validation or discards completed baselines;
- physical MLX hot-water transitions are allowed through the validation transition profile without weakening the normal production target gate;
- strong trusted MLX or C1001 evidence may escalate independently;
- FSR posture evidence alone remains WATCH-only;
- FSR posture detection is passenger-reference-relative instead of depending only on one exact hardcoded pressure pattern;
- prolonged forward posture exposes Monitoring Limited rather than a false emergency.

Validation console
------------------
- Open http://192.168.4.1/validation (legacy /uat still works).
- Opening the page automatically starts a validation session in LIVE READINGS mode.
- Apply Test Condition enables selected test inputs.
- Live Readings disables all optional test inputs but keeps validation active.
- Completed MLX baselines/model windows are preserved when returning to Live Readings.
- All 9 FSR values remain visible.
- The page uses neutral validation terminology and contains no visible Controlled / Inject / UAT wording.

MLX physical validation
-----------------------
- Normal runtime target sanity remains 20..50 C.
- While the validation console is active, the physical target sanity window is widened to 10..90 C so a deliberate hot-water reference is not mistaken for target loss.
- Backrest contact remains required for a usable MLX target.
- Only the rapid geometry-transition quarantine is bypassed in validation.
- The existing 1-second stability gate, 30-second personal baseline, IF+OCSVM inference, 3-block anomaly persistence, and Fusion warning persistence remain active.
- Once the model baseline is complete, the console reports the actual runtime state (Baseline Ready / Stabilizing Target / Anomaly Candidate / Strong Anomaly) rather than treating 30/30 as a new baseline build.

Fusion severity policy
----------------------
- Trusted STRONG MLX -> WARNING after 4 s persistence.
- Trusted STRONG C1001 -> WARNING after 4 s persistence, provided the C1001 motion-artifact gate says the vitals are reliable.
- FSR strong posture/model evidence alone -> WATCH only.
- Camera -> corroboration only.
- MPU -> motion/context only.
- Once WARNING is confirmed, the same qualifying MLX/C1001 critical condition must remain continuously active for another 30 s before EMERGENCY.
- If the critical condition clears, the Emergency timer resets.
- Existing ~3 s elevated-state recovery hysteresis remains.

Adaptive FSR posture behavior
-----------------------------
- A passenger-specific seated pressure reference is learned after approximately 12 s of stable cushion + backrest contact.
- A sustained left/right pressure shift or sustained forward loss of backrest contact is recognized relative to that passenger reference.
- The posture must persist for approximately 8 s before it becomes a strong FSR posture vote.
- FSR alone remains WATCH-only even after the strong posture vote.
- Brief reaching/adjusting movements do not reach the 8 s strong-vote persistence.
- Forward posture for >=30 s reports Monitoring Limited.
- Forward posture for >=60 s reports persistent Monitoring Limited.
- Monitoring Limited never creates WARNING or EMERGENCY by itself.
- If the passenger enters already far forward and never establishes a normal backrest reference, the Runtime does not learn that poor geometry as normal; it reports Monitoring Limited after the sustained timeout instead.

Compatibility
-------------
- Legacy /uat route remains available.
- Legacy API aliases remain available for the current app.
- Legacy strong_multisensor_candidate / elapsed telemetry keys remain present for app compatibility, but R5.7 also exposes clearer critical/monitoring fields.
- No app code is included or modified in this Runtime package.

Recommended defense flow
------------------------
1. Open /validation: it is automatically ready on live physical readings.
2. Let the passenger settle and allow the MLX baseline to complete.
3. Optional FSR posture demo: sustained slump -> WATCH only.
4. MLX hot-water physical demo: stable strong MLX anomaly -> WARNING.
5. Hold the critical MLX condition after WARNING for 30 s -> EMERGENCY.
6. Press Live Readings whenever you want to clear optional test inputs without exiting validation.

Important
---------
This package passed host-side syntax checks for the modified Fusion, validation-input, and API sources plus browser JavaScript/DOM checks and behavioral Fusion tests. A full ESP32 Arduino board compile still needs to be run on the actual development machine before flashing because the board toolchain is not installed in the packaging environment.
