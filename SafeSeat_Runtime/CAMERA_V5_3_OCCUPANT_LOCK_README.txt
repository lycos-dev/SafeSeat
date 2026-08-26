SAFESEAT CAMERA V5.3 — ESP-NOW INTEGRATED SEAT-OCCUPANT LOCK
=================================================================

This supersedes V5.2 camera runtime. Main Hub, C1001, Fusion, FSR, MLX, MPU, and the ESP-NOW wire protocol are unchanged.

Camera change only
------------------
- The 5-pose passenger calibration now also builds a seat-occupant anchor.
- Verification selects the detection matching that passenger anchor instead of accepting any visible person.
- Tiny/distant background people are rejected. If the seat occupant is missed and only a background person remains, result is UNKNOWN.
- Multiple people are okay when one clearly matches the calibrated seat occupant; genuinely ambiguous identity stays UNKNOWN.
- V4.2 forward fallback and temporal handling are retained unchanged.

Passenger/session lifecycle remains the same
--------------------------------------------
Stable FSR occupancy -> Main Hub creates new session -> CALIBRATE_UPRIGHT -> camera collects 5 valid upright poses + occupant anchor -> READY.
Stable seat exit -> RESET_SESSION -> camera clears session baseline/anchor.
Next occupant -> new session -> new calibration.

V5.3 uses NVS key baseline_v53. Older V5.2 saved camera baselines are ignored because they do not contain the occupant anchor.
