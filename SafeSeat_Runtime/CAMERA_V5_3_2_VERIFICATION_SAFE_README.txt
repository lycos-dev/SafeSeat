SAFESEAT CAMERA V5.3.2 — ESP-NOW VERIFICATION-SAFE INTEGRATION
================================================================

Purpose
-------
V5.3.2 mirrors the final standalone V4.3.2 verification-safe camera contract
inside the ESP-NOW camera runtime.

Safety contract
---------------
- UNKNOWN is never equivalent to UPRIGHT and can never clear a sensor-triggered
  verification request.
- UPRIGHT can be sent only when the CURRENT raw pose is explicitly NORMAL,
  the temporal filter is NORMAL, and the integrated request has at least two
  consecutive clean NORMAL observations with no active abnormal streak.
- Confirmed deviation returns NON_UPRIGHT.
- Ambiguous identity, missing occupant, blur/low confidence, or unusable pose
  remain UNKNOWN if a conclusive result is not obtained within the request.

Preserved from V5.3.1
---------------------
- IF + OCSVM model/export/thresholds: unchanged.
- Canonical 7D feature logic: unchanged.
- V4.2 missing-nose forward fallback: unchanged.
- V4.2 temporal filter: unchanged.
- V5.3 seat-occupant anchor/background-person rejection: unchanged.
- ESP-IDF 5.5.5 Xtensa EPS compile fix: preserved.
- ESP-NOW wire protocol version/packet layouts: unchanged (version 2).
- Passenger-session calibration lifecycle: unchanged.
- NVS schema/key remains baseline_v53, so V5.3/V5.3.1 passenger calibration
  can be reused when the same session is still active.

Important integration note
--------------------------
This ZIP updates the ESP32-S3 CAMERA runtime only. It does not change Main Hub,
C1001, Fusion, FSR, MLX, or MPU firmware. The full hardware integration test
must still confirm that the Main Hub treats only CameraPostureClass::UPRIGHT as
camera clearance and never treats UNKNOWN/NOT_READY/PENDING as clearance.

Standalone evidence carried forward
------------------------------------
The camera model and pose feature files are byte-identical to V4.3.2/V5.3.1.
The final standalone validation used the 750-image SafeSeat dataset and reported
0 unsafe non-upright clearances in the verification-contract replay. The live
V4.3.2 ESP32-S3 test also demonstrated CLEAR_UPRIGHT for trusted upright,
HOLD_DEVIATION for abnormal posture, and HOLD_UNKNOWN when pose was unavailable.
