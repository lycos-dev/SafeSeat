SAFESEAT CAMERA V5.2 — ESP-NOW INTEGRATED
==============================================
This package supersedes V5.1 camera runtime. Main Hub and C1001 source are byte-for-byte unchanged.

Camera changes
--------------
- Keeps the same V4.1 canonical 7D IF+OCSVM model and baseline semantics.
- Adds V4.2 conservative missing-nose forward fallback: both shoulder span and
  person-box area must grow >=1.30x versus the calibrated upright baseline.
- Overlapping duplicate pose detections may be collapsed; distinct people remain UNKNOWN.
- Temporal pending evidence survives one intervening NORMAL (A-N-A can confirm).
- Integrated VERIFY no longer returns UPRIGHT after one normal frame. It requires
  two clean normals. Mixed normal/abnormal evidence ends PENDING/UNKNOWN rather than
  falsely clearing the sensor-triggered emergency.

The ESP-NOW protocol is unchanged (wire version 2). No Main Hub protocol change is required.

ACTIVE NODES
------------
1) SafeSeat_Runtime_Main — unchanged from V5.1
2) SafeSeat_Runtime_C1001 — unchanged from V5.1
3) SafeSeat_Runtime_ESP32S3_CAM — updated V5.2 camera runtime

Camera verification remains conservative: blur, truly unusable pose, distinct multiple
people, or insufficient fallback evidence stay UNKNOWN and do not vote anomaly.
