SafeSeat Final Runtime - Piezo Removed
========================================

Current deployed nodes:
1. SafeSeat_Runtime_Main
   - Main Hub ESP32
   - MLX90614
   - 9 FSRs through ADS1115/native ADC
   - MPU6050
   - Fusion
   - SafeSeat Wi-Fi SoftAP
   - local read-only telemetry API
   - ESP-NOW receiver/transport for C1001 and camera transactions

2. SafeSeat_Runtime_C1001
   - dedicated C1001 ESP32 node
   - C1001 acquisition/filtering
   - trained IF + One-Class SVM
   - ESP-NOW to Main Hub

3. SafeSeat_Runtime_ESP32S3_CAM
   - ESP32-S3-WROOM-1-N16R8 camera node
   - 8 MB OPI PSRAM
   - INT8 5-class posture classifier
   - joins SafeSeat Wi-Fi
   - ESP-NOW camera trigger/result

REMOVED FROM FINAL DEPLOYMENT:
- PVDF/Piezo sensor
- Piezo ESP32 node
- Piezo protocol
- Piezo Fusion evidence
- Piezo telemetry/API fields

C1001 is the deployed respiration source.

Historical Piezo datasets/models/training files may remain in SafeSeat_ML
for thesis history and reproducibility. They are not loaded by this runtime.

Arduino verification:
- Main Hub: verify after replacing the folder with this version.
- C1001: functional logic unchanged from Step 5.8.
- ESP32-S3 camera: functional logic unchanged from Step 5.9.5.
