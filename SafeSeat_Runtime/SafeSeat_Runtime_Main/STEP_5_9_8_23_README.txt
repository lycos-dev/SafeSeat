SafeSeat Main Hub - Step 5.9.8.23
===================================

Purpose
-------
Make remote-node connectivity and model state visible in the existing compact
1-second [LIVE] heartbeat without restoring the giant dashboard.

Changes
-------
1. C1001 live telemetry added to [LIVE]:
   - C1=WAIT / ON / OFF
   - packet age and sequence
   - presence, motion code, MoveRange
   - trusted filtered RR/HR when available; raw RR/HR before trust
   - C1001 sensor runtime status
   - exact remote ML status (WAIT-SAMPLE, WAIT-OCC, WARM, BUILD, NORMAL,
     WEAK, STRONG, MOTION-HOLD, REACQ-HOLD, etc.)
   - ML window progress
   - IF and OCSVM decision values + N/A anomaly marker when model output is valid

2. Camera telemetry added in advance to [LIVE]:
   - CAM=WAIT / ON / OFF
   - packet age
   - camera hardware readiness
   - PSRAM readiness
   - camera model readiness
   - IDLE / VERIFY / BUSY state
   - request ID while a verification transaction is active
   - last posture + confidence after a result is received

3. Main-Hub C1001 status enum now mirrors the remote node's appended
   REACQUIRING_TARGET and REACQUIRING_REBASELINE codes. This prevents these
   valid remote statuses from appearing as UNKNOWN.

4. Initialization summary now explicitly says C1001Link/CameraLink READY means
   the transport initialized. Actual remote-node connectivity is shown in [LIVE].

No model/fusion behavior changed
-------------------------------
These additions are telemetry/display only. C1001 packet layout/version, camera
packet layout/version, sensor preprocessing, ML decisions, Fusion thresholds,
and camera trigger logic are unchanged.

Expected examples
-----------------
Before remote nodes are powered:
  ... C1=WAIT CAM=WAIT FUS=WATCH

C1001 online but warming/building:
  ... C1=ON age=120ms seq=18 Pres=Y Mot=... Rng=... RRraw=... HRraw=...
      C1ML=WARM win=0/30 CAM=WAIT FUS=WATCH

C1001 trusted + ML ready:
  ... C1=ON age=85ms seq=73 Pres=Y ... RR=16.0 HR=74.0
      C1ML=NORMAL win=30/30 IF=...N SVM=...N CAM=WAIT ...

Camera later powered and ready:
  ... CAM=ON age=90ms HW=OK PS=OK CML=READY CSTATE=IDLE ...

Important
---------
The uploaded SafeSeat_Runtime_C1001 package already transmits all fields needed
for this display, so no C1001 wire-protocol change is required.
