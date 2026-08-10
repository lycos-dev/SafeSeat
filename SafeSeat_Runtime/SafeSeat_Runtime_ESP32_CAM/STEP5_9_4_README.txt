SafeSeat Step 5.9.4 - ESP32-CAM INT8 Verification + ESP-NOW + Fusion
====================================================================

PURPOSE
-------
This step integrates the accepted/validated Step 5.9.3 posture model into the
AI-Thinker ESP32-CAM and connects camera verification to the Main Hub over
ESP-NOW.

RUNTIME PROJECTS IN THIS PACKAGE
--------------------------------
1. SafeSeat_Runtime_Main
   - MLX90614 + FSR + MPU6050 acquisition/model/context
   - remote C1001 evidence over ESP-NOW
   - remote Piezo evidence over ESP-NOW
   - camera trigger/result transactions over ESP-NOW
   - final Fusion

2. SafeSeat_Runtime_C1001
   - dedicated remote C1001 node from Step 5.8, unchanged

3. SafeSeat_Runtime_Piezo
   - dedicated remote Piezo node from Step 5.7.3, unchanged

4. SafeSeat_Runtime_ESP32_CAM
   - AI-Thinker ESP32-CAM + OV2640
   - validated 409,400-byte full-INT8 posture model
   - 160x160 RGB INT8 inference
   - ESP-NOW trigger/status/result communication

ESP32-CAM MODEL
---------------
Classes / indices:
0 = leaning_backward
1 = leaning_left
2 = leaning_right
3 = upright
4 = leaning_forward

SafeSeat camera interpretation:
- upright = normal posture verification
- any leaning class = abnormal posture verification

The camera is a verifier only. It does not independently declare an emergency.

CAMERA EXECUTION POLICY
-----------------------
The ESP32-CAM board stays powered so ESP-NOW can receive requests.
The OV2640 camera sensor is deinitialized/powered down while idle.

On a verification request:
1. initialize OV2640
2. discard two warm-up frames
3. capture/infer three frames
4. require at least two valid frames
5. choose posture by majority vote (ties use summed model confidence)
6. send the result to Main Hub over ESP-NOW
7. deinitialize/power down the OV2640 again

The JPEG image itself is NOT sent over ESP-NOW. Only compact status, trigger,
and posture-result packets are exchanged.

MAIN-HUB CAMERA TRANSACTION SAFETY
----------------------------------
- Each verification request has a request ID.
- Main retries the trigger every 500 ms while a request is active.
- Request timeout: 12 seconds.
- Camera status freshness: 3 seconds.
- Camera result freshness: 5 seconds.
- Late/stale results that do not match the active request are ignored.
- An UPRIGHT result is consumed once and can de-escalate the current candidate.
- A LEANING result latches camera-abnormal verification only while the strong
  underlying multisensor candidate remains active.

ESP-NOW
-------
The existing SafeSeat Hub beacon/channel-discovery scheme remains active.
Piezo, C1001 and ESP32-CAM can all find/follow the Main Hub channel.
Default channel before infrastructure Wi-Fi is added: 6.

AI-THINKER ESP32-CAM / FTDI PROGRAMMING
----------------------------------------
Use the user's current FTDI programming setup:

FTDI 5V  -> ESP32-CAM 5V
FTDI GND -> ESP32-CAM GND
FTDI TX  -> ESP32-CAM U0R / RX0
FTDI RX  <- ESP32-CAM U0T / TX0
GPIO0    -> GND ONLY while entering flash/upload mode

After upload:
- disconnect GPIO0 from GND
- reset or power-cycle the ESP32-CAM

The 5V connection above is the board power input. UART GPIO signaling itself
must remain ESP32-compatible logic; do not intentionally drive ESP32 GPIO with
5V logic.

ARDUINO SETTINGS
----------------
ESP32-CAM:
- Board: AI Thinker ESP32-CAM
- PSRAM: enabled/available (required by this runtime)
- Partition: Huge APP is recommended for the model + TFLM runtime
- Upload through FTDI using the normal GPIO0 boot procedure

Main Hub:
- Keep Huge APP (3MB No OTA / 1MB SPIFFS)

TENSORFLOW LITE MICRO LIBRARY
-----------------------------
SafeSeat_Runtime_ESP32_CAM requires TensorFlow Lite Micro Arduino headers.
If Arduino reports that tensorflow/lite/micro/micro_interpreter.h is missing,
install a compatible ESP32-capable TensorFlow Lite Micro Arduino library before
compiling. A practical first choice in Arduino Library Manager is
Chirale_TensorFlowLite (ESP32 architecture). Because AI-Thinker ESP32-CAM is
not one of that library's explicitly tested boards, Arduino Verify on this exact
board remains required. Do not install several competing TFLM ports at once.
The runtime intentionally emits a clear #error when the expected headers are
absent rather than silently compiling without inference.

PSRAM / MEMORY
--------------
The ESP32-CAM runtime allocates from PSRAM:
- 2 MiB TensorFlow Lite Micro tensor arena
- RGB888 decode buffer for a 160x120 JPEG frame

The model itself is stored as a C++ byte array in program flash.

VALIDATION COMPLETED BEFORE PACKAGING
-------------------------------------
- Step 5.9.3 full-INT8 deployment validation: PASS
- INT8 test accuracy: 85.80%
- INT8 macro F1: 86.14%
- Upright-vs-any-leaning accuracy: 92.39%
- False-normal rate: 6.35%
- C++ INT8 pixel quantization LUT matches the Python formula for all 256
  possible uint8 pixel values: 0 mismatches
- Shared CameraProtocol.h identical between Main and ESP32-CAM
- C1001/Piezo runtime projects copied unchanged from their verified versions
- Host C++ syntax checks for changed camera/Main sources: PASS

IMPORTANT
---------
A full Arduino build was NOT performed in the packaging environment.
The authoritative next check is Arduino IDE Verify for:
1. SafeSeat_Runtime_ESP32_CAM
2. SafeSeat_Runtime_Main

After both Verify successfully, SafeSeat's five sensor pipelines plus the
camera verifier are code-integrated and the next phase is the combined
hardware/ESP-NOW test.
