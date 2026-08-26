SafeSeat Final Runtime — Camera ESP-NOW Integration
===================================================
Step 5.9.9 — 2026-08-26

Current deployed architecture
-----------------------------
Main Hub ESP32:
- MLX90614 thermal sensor + existing native IF/OCSVM/runtime context
- 9 FSR pressure sensors + existing trained runtime
- MPU6050 road/seat-motion context + existing trained runtime
- Fusion state machine
- SafeSeat Wi-Fi SoftAP / telemetry API
- ESP-NOW transport for C1001 and camera

Remote C1001 ESP32:
- presence / respiration / heartbeat-presence / motion context
- trained IF + One-Class SVM
- ESP-NOW to Main Hub

ESP32-S3 Camera:
- OV2640 + Espressif YOLO11n-Pose
- Robust 7D calibrated pose anomaly verification
- Isolation Forest + One-Class SVM
- nose + both shoulders required; eyes optional
- 3-frame sharpness selection before one YOLO inference
- 5-valid-pose upright passenger-session baseline
- temporal abnormal confirmation
- command-driven ESP-NOW runtime; idle between calibration/verification jobs

Camera role
-----------
The camera is verification-only. It does not independently diagnose a medical
emergency. Main Hub Fusion is the trigger authority.

Fusion mapping
--------------
UPRIGHT_CONFIRMED       -> valid normal camera verification
NON_UPRIGHT_CONFIRMED   -> valid abnormal camera verification
DEVIATION_PENDING       -> no Fusion vote yet
UNKNOWN / NOT_READY     -> no Fusion vote

Removed from final deployment
-----------------------------
PVDF/Piezo respiration node/protocol/evidence. C1001 remains the respiration
source. Historical Piezo datasets/training files may remain elsewhere for
thesis reproducibility, but are not in this runtime package.
