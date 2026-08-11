SafeSeat Runtime Main - Step 5.9.8
Local Telemetry / API Foundation
=================================

PURPOSE
-------
Step 5.9.8 adds a backend-agnostic, read-only HTTP API to the Main Hub.
It builds directly on Step 5.9.6 SafeSeat SoftAP + ESP-NOW.

This is NOT the final frontend/backend implementation.
It gives the future frontend one stable local data contract while preserving:
- Main Hub Fusion as the authoritative decision layer
- C1001 ESP-NOW evidence
- Piezo ESP-NOW evidence
- ESP32-S3 camera ESP-NOW trigger/result transactions
- SafeSeat Wi-Fi SoftAP
- all existing sensor/model code

NETWORK
-------
SSID      : SafeSeat
Password  : safeseat123
Main IP   : 192.168.4.1
Channel   : 6
Internet  : not required

API CONTRACT
------------
Schema version: 1.0.0
Read-only: YES

Canonical endpoints:
  GET http://192.168.4.1/health
  GET http://192.168.4.1/api/v1/status
  GET http://192.168.4.1/api/v1/sensors
  GET http://192.168.4.1/api/v1/camera
  GET http://192.168.4.1/api/v1/network

Development aliases:
  /status
  /sensors
  /camera
  /network

Opening http://192.168.4.1/ shows a tiny diagnostic page linking to the API.
It is not intended to be the final SafeSeat frontend.

IMPORTANT ARCHITECTURE RULES
----------------------------
1. Fusion remains authoritative.
   The frontend reads Fusion state; it does not independently decide emergencies.

2. The API is read-only.
   There are no POST/control endpoints and no API route can trigger/cancel the
   camera, modify Fusion, or trigger an alert.

3. MLX WESAD IF/OCSVM remains diagnostic-only in the API because the deployment
   Fusion policy uses MLX context rather than the mismatched WESAD model vote.

4. MPU model output is exposed as road-motion/artifact context, not occupant
   health evidence.

5. Camera posture is verification-only.

6. No Firebase, WebSocket server, cloud database, or final backend technology is
   assumed in this step.

7. The API uses null for unavailable/non-finite measurements instead of fake
   zero values.

MAIN FILES ADDED
----------------
SafeSeatTelemetry.h/.cpp
  - Copies the latest FusionInput/FusionReading and link/network status into a
    read-only telemetry snapshot.

SafeSeatApi.h/.cpp
  - Runs the Main Hub local HTTP server on port 80.
  - Generates JSON without requiring ArduinoJson or another external library.

ARDUINO DEPENDENCIES
--------------------
No new third-party library is required.
WebServer.h is provided by the ESP32 Arduino core.

VERIFY / TEST
-------------
1. Open SafeSeat_Runtime_Main.ino.
2. Keep the same Main Hub ESP32 board configuration used previously.
3. Keep Partition Scheme = Huge APP.
4. Arduino Verify.
5. Upload to Main Hub.
6. Connect phone/laptop Wi-Fi to SafeSeat / safeseat123.
   A phone may warn that the network has no Internet; remain connected.
7. Open:
       http://192.168.4.1/health
   Expected conceptually:
       {"ok":true,...,"telemetry_ready":true,"read_only":true}
8. Open:
       http://192.168.4.1/api/v1/status
   A full JSON snapshot should appear.

This API test DOES NOT require C1001, Piezo, or the camera to be physically
available. Missing remote nodes should appear as unavailable/disconnected in
telemetry. Full coexistence testing remains Step 5.9.7 and can be performed
later when the hardware is available.

FRONTEND GUIDANCE
-----------------
The future frontend should primarily consume:
  /api/v1/status

Recommended initial polling rate for the prototype:
  1 Hz to 2 Hz

Do not poll at camera-frame rates. This is status telemetry, not a video stream.

SUGGESTED COMMIT AFTER ARDUINO VERIFY + API TEST
------------------------------------------------
feat(api): add read-only local SafeSeat telemetry endpoints
