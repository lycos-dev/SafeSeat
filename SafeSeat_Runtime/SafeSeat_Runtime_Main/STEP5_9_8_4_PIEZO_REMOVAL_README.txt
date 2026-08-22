SafeSeat Step 5.9.8.4 - Final Runtime Architecture Update
===========================================================

Final deployment removes the PVDF/Piezo module completely.

Main Hub changes:
- Removed PiezoComm and PiezoProtocol source files.
- Removed Piezo packet handling from SafeSeatNow.
- Removed Piezo fields from FusionInput and telemetry snapshots.
- Removed Piezo respiration-support logic from Fusion.
- Removed Piezo section from the local telemetry API.
- Removed Piezo startup/update/dashboard code.
- C1001 is the deployed respiration source.

Preserved:
- C1001 remote IF/OCSVM node
- MLX90614 context policy
- 9-FSR model/runtime
- MPU6050 road-motion context/model
- ESP32-S3 INT8 posture verifier
- ESP-NOW C1001/camera communication
- SafeSeat Wi-Fi SoftAP
- Local read-only telemetry API
- Arduino String overload hotfix

Historical Piezo datasets/models may remain in SafeSeat_ML for thesis history,
but they are not referenced by this final runtime.
