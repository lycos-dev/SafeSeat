SAFESEAT MLX90614 OCCUPANCY-GATED AMBIENT/DELTA FIX — 2026-08-23
================================================================

ISSUE FOUND DURING INT-HW-02
----------------------------
In the combined home-seat run, the MLX ambient channel gradually increased
while the occupant remained close to the headrest sensor. This caused the
Object-Ambient difference to repeatedly cross the former +2 C / +1 C target
thresholds even though the seat remained occupied.

The MLX90614 ambient channel is the sensor/package thermal reference used for
IR compensation. It is not a dedicated cabin-air thermometer. Local sensor
self-heating, enclosure/mount warming, reduced airflow, and proximity to a warm
occupant can make Ta drift even when the room itself did not suddenly change.

Therefore Object-Ta is too fragile to be the authority for "person present".

NEW COMBINED-RUNTIME POLICY
---------------------------
1. FSR pressure occupancy (or fresh C1001 presence when available) establishes
   the occupant/session gate.
2. The native MLX model continues to use only:
      object_delta_from_session_baseline
      object_abs_delta_from_session_baseline
3. The personal/session baseline is built from the existing filtered MLX object
   signal while the seat is occupied.
4. Ambient temperature and Object-Ta remain visible diagnostic/context values.
5. LOW Object-Ta contrast does NOT:
      - reset the personal baseline
      - stop baseline collection
      - block IF/OCSVM inference
      - claim the occupant disappeared
6. A filtered 1-second transition guard remains to avoid learning abrupt FOV
   transitions. The guard is now <= 1.00 C rather than the earlier 0.50 C.
7. The separate MLXContext module no longer builds a competing baseline. It
   shares the authoritative native-MLX personal baseline.
8. Leaving the seat / losing occupancy starts a new MLX session and resets the
   personal baseline.

WHY THIS FITS THE MODEL
-----------------------
The trained native MLX model is baseline-relative. Ambient temperature and
Object-Ta were explicitly kept outside the ML feature vector. This change
therefore removes an over-aggressive runtime gate without retraining or
changing the IF/OCSVM model.

IMPORTANT LIMIT
---------------
Low Object-Ta can still indicate poor FOV/target geometry. It remains visible
as LOW THERMAL CONTRAST context. Later vehicle testing can use FSR posture/back
contact to decide when a lean-away should hold MLX evidence. It is no longer
allowed to repeatedly destroy a valid occupied session.

NO MODEL RETRAINING
-------------------
IF unchanged.
OCSVM unchanged.
Scaler unchanged.
Two model features unchanged.
30-second personal baseline unchanged.
