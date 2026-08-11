SafeSeat Step 5.9.5 - ESP32-S3 WROOM Camera Runtime Rebuild
===========================================================

PURPOSE
-------
Rebuild the accepted Step 5.9.4 camera-verifier runtime for the user's actual
final camera hardware: ESP32-S3 WROOM camera board, not AI-Thinker ESP32-CAM.

WHAT IS PRESERVED
-----------------
- Accepted Step 5.9.3 full-INT8 posture model (409,400 bytes)
- 160x160 RGB INT8 model input
- Five classes:
  0 leaning_backward
  1 leaning_left
  2 leaning_right
  3 upright
  4 leaning_forward
- Three verification frames, minimum two valid frames
- Majority vote with confidence tie-break
- Step 5.9.4 CameraTrigger/CameraResult transaction IDs
- ESP-NOW status/trigger/result transport
- No JPEG image is sent through ESP-NOW

ESP32-S3 HARDWARE BASELINE
--------------------------
This runtime uses the exact pin map from the user's confirmed-working camera
hardware diagnostic:

SIOD  GPIO4
SIOC  GPIO5
VSYNC GPIO6
HREF  GPIO7
Y4    GPIO8
Y3    GPIO9
Y5    GPIO10
Y2    GPIO11
Y6    GPIO12
PCLK  GPIO13
XCLK  GPIO15
Y9    GPIO16
Y8    GPIO17
Y7    GPIO18
PWDN  -1
RESET -1

Upload/debug: USB-UART, matching the user's working ESP32-S3 setup.

CAMERA EVENT POLICY
-------------------
The user's ESP32-S3 camera wiring has no PWDN GPIO (PWDN=-1). Therefore this
step does NOT pretend the OV sensor can be electrically power-gated in code.
For reliability, esp_camera is initialized once at boot and remains initialized.

Event-driven behavior is still preserved:
- no continuous streaming in the production runtime
- no frame capture during normal monitoring
- no CNN inference during normal monitoring
- capture + inference only after a valid Main Hub CAMERA_TRIGGER

This is the practical equivalent of "camera verification only" on this board.

WI-FI FOUNDATION
----------------
The camera no longer creates SafeSeat-Camera-Test in the production runtime.
It is configured as WIFI_STA and targets:

SSID: SafeSeat
Password: safeseat123

Step 5.9.6 will make the Main Hub create this SoftAP. Until then, the S3 camera
tries to join for 5 seconds, then disconnects the association attempt so ESP-NOW
channel discovery can resume. It retries every 30 seconds.

This allows Step 5.9.5 to remain compatible with the existing Step 5.9.4 Main
Hub while preparing for the final SafeSeat local network.

WI-FI + ESP-NOW CHANNEL RULE
----------------------------
- When Wi-Fi STA is connected/associating, Wi-Fi owns the S3 radio channel and
  CameraComm must not force esp_wifi_set_channel().
- When no SafeSeat AP association is active, the existing ESP-NOW hub-beacon
  channel discovery resumes.
- ESP-NOW broadcast peer uses peer.channel=0 (current radio channel).

The CameraProtocol.h packet layout and constants are preserved exactly from
Step 5.9.4, so the existing Main Hub remains wire-compatible without changes.

MODEL / MEMORY
--------------
- Model: 409,400-byte full INT8 TFLite C array in flash
- Tensor arena: 2 MiB PSRAM
- Production camera frame: QQVGA JPEG (160x120)
- RGB decode safety buffer: up to QVGA (320x240x3) in PSRAM
- PSRAM is required for the posture model runtime

LIVE STREAM NOTE
----------------
The standalone diagnostic sketch supplied by the user already proves camera +
PSRAM + local AP + browser streaming on this exact board. This production
Step 5.9.5 runtime intentionally does not run a continuous /stream endpoint,
because continuous capture conflicts with SafeSeat's event-verification policy.
The diagnostic sketch should remain available as a separate hardware test.

ARDUINO VERIFY
--------------
Open:
  SafeSeat_Runtime_ESP32S3_CAM/SafeSeat_Runtime_ESP32S3_CAM.ino

Use the exact ESP32-S3 WROOM board/settings that compiled the user's working
camera diagnostic. Keep PSRAM enabled. Use USB-UART for upload/Serial.

TensorFlow Lite Micro dependency is the same dependency that already allowed
Step 5.9.4 camera runtime to compile. Do not change TFLM libraries during this
hardware migration unless Arduino reports a missing/incompatible header.

EXPECTED SERIAL STARTUP
-----------------------
- ESP32-S3 camera initialization successful
- Camera self-test frame successful
- INT8 model READY
- ESP-NOW READY
- Wi-Fi attempts SafeSeat

Before Step 5.9.6 exists, "SafeSeat AP not available yet" is EXPECTED and is
not a camera/ESP-NOW failure.

NEXT STEP
---------
After Arduino Verify succeeds for this ESP32-S3 project:
Step 5.9.6 = Main Hub creates SafeSeat Wi-Fi SoftAP while preserving ESP-NOW.
