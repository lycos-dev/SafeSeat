SafeSeat Final Runtime — R5 UAT Value-Injection Extension
=========================================================
Step 5.9.9-R5 — 2026-10-05

Current deployed architecture
-----------------------------
Main Hub ESP32:
- MLX90614 thermal sensor + native IF/OCSVM/runtime context
- 9 FSR pressure sensors + trained runtime
- MPU6050 road/seat-motion context + trained runtime
- authoritative Fusion state machine
- SafeSeat Wi-Fi SoftAP / telemetry API / researcher /uat console
- ESP-NOW transport for C1001 and camera

Remote C1001 ESP32:
- presence / respiration / heart rate / motion context
- trained IF + One-Class SVM
- ESP-NOW to Main Hub
- R5 temporary UAT input command; deployed C1001 ML still evaluates the
  controlled 1-Hz values on the remote node

ESP32-S3 Camera:
- OV2640 + Espressif YOLO11n-Pose
- Robust 7D calibrated pose anomaly verification
- Isolation Forest + One-Class SVM
- command-driven/event-triggered verification

Camera role
-----------
The camera is verification-only. It does not independently diagnose a medical
emergency. Main Hub Fusion is the trigger authority.

R5 UAT camera behavior preserves that architecture: a configured synthetic
UPRIGHT/NON_UPRIGHT result is emitted only after Fusion requests verification.
It cannot originate a Warning/emergency candidate.

UAT value injection
-------------------
Open http://192.168.4.1/uat to view live telemetry, enter controlled FSR/MLX/
C1001/MPU values, and monitor model-window progress and the final Fusion state.
No web path directly sets WARNING or EMERGENCY.

See UAT_SENSOR_VALUE_INJECTION_R5_README.txt for exact injection boundaries,
flashing requirements, and validation notes.
