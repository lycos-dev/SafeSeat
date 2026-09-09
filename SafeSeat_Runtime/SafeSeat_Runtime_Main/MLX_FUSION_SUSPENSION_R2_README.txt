SafeSeat Main Hub 5.9.9-R2
MLX90614 obstruction-aware Fusion suspension
Date: 2026-09-09

PURPOSE
-------
During UAT, long hair or another obstruction can prevent the headrest-mounted
MLX90614 from observing the intended exposed skin region. This condition must
not be interpreted as normal temperature evidence, abnormal temperature
evidence, or a reason for the entire SafeSeat Fusion state to remain WATCH
when the other required sensing modalities are usable.

R2 BEHAVIOR
-----------
1. Seat-session occupancy remains authoritative from calibrated FSR pressure,
   with C1001 presence only as the existing degraded fallback.

2. MLX target qualification remains a quality/FOV gate. The firmware does NOT
   attempt to identify hair specifically. Hair, clothing, posture, leaning, or
   another obstruction can all produce the same target-unavailable condition.

3. While occupied, if the intended MLX target becomes unusable:
   - MLX Fusion contribution is SUSPENDED.
   - MLX is not counted as valid Fusion evidence.
   - MLX normal/anomaly votes are disabled.
   - Other SafeSeat modalities continue through Fusion normally.
   - If the native 30-second MLX session baseline already exists, it is
     preserved and is not rebuilt from background/chair/hair samples.

4. When the target becomes usable again:
   - MLX remains suspended for 3 continuous seconds.
   - No samples from that reacquisition hold are fed into the native MLX model.
   - After 3 stable seconds, MLX automatically resumes its existing session.
   - If no baseline existed yet, normal 30-second baseline collection then
     proceeds from a qualified target.

5. A true seat-session exit retains the existing behavior and resets the
   occupant-scoped MLX session/baseline.

SERIAL HEARTBEAT LABELS
-----------------------
MLX=IDLE       : no occupied seat session
MLX=OCCLUDED   : occupied, target unusable; MLX Fusion vote suspended
MLX=REACQ      : target returned; 3-second stable reacquisition in progress
MLX=TARGET     : target usable and MLX allowed to participate normally

MLXML=HOLD         : completed baseline preserved while target unavailable
MLXML=NO-TARGET    : no usable target and no completed baseline yet
MLXML=REACQ-HOLD   : stable target reacquisition interval

API ADDITIONS (/api/v1/sensors -> mlx90614)
-------------------------------------------
target_visible
fusion_contribution_suspended
target_reacquiring
native_mlx_model_fusion_role

The app can later use these backend fields to present user-friendly wording
such as "Analyzing" or a target-obstruction prompt without changing the
underlying sensor/fusion semantics.

UNCHANGED
---------
- FSR model and occupancy logic
- C1001 remote runtime/model
- MPU model and road-motion context
- Camera runtime and ESP-NOW protocol
- Existing Main Hub auto-recovery/freeze supervisor
- Retained FSR empty-seat calibration recovery behavior
- MLX trained IF/OCSVM model data and thresholds
