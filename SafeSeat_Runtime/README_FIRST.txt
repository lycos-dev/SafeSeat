SAFESEAT RUNTIME — CAMERA ESP-NOW V5 INTEGRATED
================================================
Step: 5.9.9
Date: 2026-08-26

WHAT THIS PACKAGE IS
--------------------
This is the matched SafeSeat runtime package for the Main Hub, remote C1001
node, and ESP32-S3 camera node. It is based on the uploaded current runtime and
adds the validated Robust 7D camera as a passenger-session ESP-NOW verifier.

ACTIVE NODES
------------
1) SafeSeat_Runtime_Main
   Arduino Main Hub. Existing FSR / MLX90614 / MPU6050 / Fusion behavior is
   preserved. CameraComm + CameraProtocol + SafeSeatNow were updated for the
   new command/session protocol.

2) SafeSeat_Runtime_C1001
   Existing C1001 ESP-NOW node. Functional files are unchanged.

3) SafeSeat_Runtime_ESP32S3_CAM
   ESP-IDF camera node. Uses the exact validated V4 Robust 7D model core plus
   passenger-session calibration, true idle, and ESP-NOW commands/results.

PVDF/Piezo is NOT part of the final runtime and is intentionally omitted.

PASSENGER / CAMERA LIFECYCLE
----------------------------
Seat becomes stably occupied
 -> Main Hub creates a passenger camera session
 -> Main sends CALIBRATE_UPRIGHT
 -> camera collects 5 valid upright poses in the background
 -> baseline is tagged to that passenger session and saved to NVS
 -> camera becomes IDLE (no continuous YOLO inference)

Fusion later requests verification
 -> Main sends VERIFY_POSTURE
 -> camera wakes and performs pose verification
 -> UPRIGHT_CONFIRMED can finish after one valid normal inference
 -> non-upright requires two valid abnormal observations
 -> UNKNOWN / low-confidence / blur never votes anomaly
 -> camera returns to IDLE after the transaction

Seat becomes stably empty
 -> Main sends RESET_SESSION
 -> passenger baseline is invalidated
 -> next passenger receives a new calibration session

IMPORTANT ABOUT CALIBRATION TIME
--------------------------------
The camera still uses the proven 5-valid-pose calibration. Each YOLO11n-Pose
inference is slow on ESP32-S3, so first-passenger calibration can take minutes,
especially if a frame is blurred or nose/shoulders are not detected. This is
intentional for reliability. The improvement in V5 is lifecycle: calibration
runs only once per passenger session in the background and is NOT on the
emergency verification path.

FILES TO OPEN
-------------
Main Hub (Arduino IDE):
  SafeSeat_Runtime_Main\SafeSeat_Runtime_Main.ino

C1001 node (Arduino IDE):
  SafeSeat_Runtime_C1001\SafeSeat_Runtime_C1001.ino

Integrated camera (ESP-IDF):
  SafeSeat_Runtime_ESP32S3_CAM\

START WITH
----------
Read:
  STEP_5_9_9_CAMERA_ESPNOW_README.txt
  CAMERA_V5_INTEGRATION_TEST_REPORT.json

Do not merge the separate standalone diagnostic camera package into this
integrated camera folder. They serve different purposes.
