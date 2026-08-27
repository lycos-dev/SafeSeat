SAFESEAT CAMERA V5.3.2 — ESP-NOW VERIFICATION-SAFE INTEGRATED RUNTIME
====================================================================

This supersedes V5.3.1 camera runtime.

Camera behavior
---------------
- Triggered only by Main Hub VERIFY_POSTURE requests; no continuous pose ML.
- Passenger calibration remains 5 valid upright poses plus seat-occupant anchor.
- Runtime selects the calibrated seat occupant and rejects background people.
- V4.2 forward fallback and temporal filtering remain unchanged.
- V4.3.2 verification-safe contract is now explicit in the ESP-NOW runtime:
    UNKNOWN -> never clear
    confirmed deviation -> NON_UPRIGHT
    UPRIGHT -> only after current RAW=NORMAL + FILTER=NORMAL + two clean normals

Passenger/session lifecycle
---------------------------
Stable FSR occupancy -> Main Hub starts new session -> CALIBRATE_UPRIGHT ->
5 valid upright poses + occupant anchor -> READY.

Stable seat exit -> RESET_SESSION -> camera clears passenger session state.
Next occupant -> new session -> fresh calibration.

Compatibility
-------------
- ESP-NOW camera wire protocol remains version 2.
- NVS baseline schema/key remains baseline_v53.
- IF/OCSVM model, thresholds, canonical 7D features, forward fallback,
  temporal filter, occupant anchor, camera pins, partitions, and SDK defaults
  are unchanged from V5.3.1.
