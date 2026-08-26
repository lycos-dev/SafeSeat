SAFESEAT STEP 5.9.9 — CAMERA ESP-NOW PASSENGER-SESSION INTEGRATION
=====================================================================

SCOPE
-----
Integrates the already validated V4 Robust 7D camera verifier into the current
Main Hub without altering the accepted FSR, MLX, MPU, or Fusion algorithms.

MAIN HUB CHANGES
----------------
Active files changed from the uploaded parent runtime:
- CameraProtocol.h
- CameraComm.h / CameraComm.cpp
- SafeSeatNow.h / SafeSeatNow.cpp
- Config.h (camera timing/session constants only)
- SafeSeat_Runtime_Main.ino (camera session service hook + status text)

Confirmed byte-for-byte preserved sensor/Fusion sources include:
FSR*, MLX*, MPU*, and Fusion.cpp/Fusion.h.

PROTOCOL V2
-----------
Main -> Camera commands:
- PING
- CALIBRATE_UPRIGHT
- VERIFY_POSTURE
- CANCEL_VERIFY
- RESET_SESSION

Camera -> Main:
- 1 s status heartbeat
- UPRIGHT_CONFIRMED
- DEVIATION_PENDING
- NON_UPRIGHT_CONFIRMED
- UNKNOWN
- NOT_READY

Packet sizes:
- status  30 bytes
- command 18 bytes
- result  36 bytes
All packets use the existing SafeSeat CRC16 convention.

SESSION SAFETY
--------------
- Stable occupancy entry creates a random non-zero session ID.
- Baseline is valid only when camera remoteSessionId matches Main localSessionId.
- Stable seat exit sends RESET_SESSION and clears old passenger baseline.
- Late/wrong-session/wrong-request results are ignored.
- A camera reboot can reload a saved session-tagged baseline from NVS; Main will
  either accept the matching current session or command a new calibration.

VERIFICATION SAFETY
-------------------
- Main starts VERIFY only when camera/model/PSRAM/baseline/current-session are ready.
- DEVIATION_PENDING stays inside the transaction and never reaches Fusion.
- Only final UPRIGHT or NON_UPRIGHT packets with VALID flag reach Fusion.
- UNKNOWN / blur / low-confidence / missing required pose do not vote anomaly.
- Main retries the same request ID during long inference.
- Camera caches the last result so duplicate VERIFY retries can be answered
  without rerunning YOLO after a request has completed.

TIMING
------
ESP32-S3 YOLO11n-Pose is approximately 27–28 s per inference on the tested
board. Request timeout is therefore long enough for up to three attempts.
Calibration remains 5 valid upright poses and runs after passenger entry on the
separate camera node while Main Hub sensor monitoring continues.

BUILD / FLASH — MAIN HUB
------------------------
Use Arduino IDE and open:
  SafeSeat_Runtime_Main\SafeSeat_Runtime_Main.ino
Compile/flash using the same board/core/library setup as the uploaded parent
runtime. No model retraining is required.

BUILD / FLASH — INTEGRATED CAMERA
---------------------------------
Open a CMD with ESP-IDF 5.5.5 activated, then enter the camera folder:

  set "IDF_TOOLS_PATH=C:\Espressif\tools"
  call "C:\esp\v5.5.5\esp-idf\export.bat"
  cd /d "<this package>\SafeSeat_Runtime_ESP32S3_CAM"
  idf.py fullclean
  idf.py build

Close Arduino Serial Monitor if it owns the camera COM port, then:
  idf.py -p COM8 flash
  idf.py -p COM8 monitor

Use the actual COM port if Windows assigns a different number.

EXPECTED INTEGRATED CAMERA BEHAVIOR
-----------------------------------
At boot:
  SafeSeat ESP32-S3 Camera V5 - ESP-NOW Integrated
  ...
  Camera ready. Idle until Main Hub passenger-session command.

On passenger entry:
  Passenger session <id>: upright calibration started.
  CAL ... 1/5 ... 5/5
  Upright calibration complete ... Camera is now idle until verification.

On Fusion verification:
  VERIFY request=<id> session=<id> started.
  -> UPRIGHT_CONFIRMED
or
  -> DEVIATION_PENDING
  -> NON_UPRIGHT_CONFIRMED
or
  -> UNKNOWN

VALIDATION EVIDENCE
-------------------
See CAMERA_V5_INTEGRATION_TEST_REPORT.json and validation/camera_v5_integration/.
The complete Arduino project still requires compilation in the user's Arduino
environment, and both physical boards still require an end-to-end ESP-NOW test.
