SafeSeat Main Hub R5.4 - Final Defense Fusion/UAT revision

Changes from the supplied latest SafeSeat_Runtime.zip:

1. Controlled UAT hard-side FSR pattern must persist 10,000 ms before the
   dedicated UAT FSR strong vote is emitted. While that controlled pattern is
   active but not yet qualified, raw FSR IF/OCSVM output remains visible in
   telemetry but cannot bypass the controlled UAT timer.

2. Camera is event-triggered CORROBORATION ONLY. UPRIGHT/NON_UPRIGHT results
   are retained in /uat, API telemetry, CSV and reports, but camera results do
   not create, cancel, downgrade, clear or block Fusion severity.

3. Two independent strong anomaly votes start the strong multisensor timer.
   WARNING can occur from the normal warning persistence path. The camera is
   requested in parallel during a strong multisensor WARNING. If >=2 strong
   anomaly votes remain continuously active for 30,000 ms, Fusion escalates to
   EMERGENCY without needing any camera result. If the strong condition breaks,
   the 30 s timer resets.

4. MPU motion remains useful as an escalation/artifact context gate, but motion
   alone no longer prevents an otherwise clean occupied session from returning
   to SAFE after the existing 3 s clean recovery period. Camera availability or
   posture likewise does not block recovery.

5. API exposes strong_multisensor_elapsed_ms,
   emergency_persistence_required_ms, camera_fusion_role=corroboration_only,
   and generic user-facing visual-confirmation state/message. Detailed camera
   posture remains available for /uat/research evidence.

6. /uat keeps actual camera baseline progress (0/5 ... 5/5), controlled camera
   UPRIGHT/NON_UPRIGHT/real modes, and session CSV logging. It now also records
   the multisensor emergency timer and camera corroboration role.

Only the Main Hub folder is changed in this package. C1001 and ESP32-S3 camera
firmware are not changed by R5.4.
