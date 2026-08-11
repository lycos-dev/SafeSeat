SafeSeat Step 5.9.6 - Main Hub SafeSeat SoftAP
================================================

PURPOSE
-------
The Main Hub now creates the final local SafeSeat Wi-Fi network while
preserving the existing ESP-NOW sensor/camera transport.

NETWORK
-------
SSID       : SafeSeat
Password   : safeseat123
Main IP    : 192.168.4.1
Channel    : 6
Internet   : NOT required

The password is currently the same known-working development password used
for the ESP32-S3 camera diagnostic. Change it later if desired, but keep the
Main and ESP32-S3 camera Config values identical.

ARCHITECTURE
------------
C1001 ESP32 ---- ESP-NOW ----\
                               \
Piezo ESP32 ---- ESP-NOW ------> Main Hub ESP32
                                 - Fusion
ESP32-S3 Camera - ESP-NOW ----/ - SafeSeat Wi-Fi SoftAP
                     ^           - 192.168.4.1
                     |                |
                     |                +---- Phone joins SafeSeat
                     +---- camera also joins SafeSeat as Wi-Fi STA

The ESP32-S3 camera's Step 5.9.5 production runtime already targets:
  SSID: SafeSeat
  Password: safeseat123
so no camera-side modification is required for Step 5.9.6.

WHAT CHANGED
------------
1. Added NetworkConfig.h.
2. Added SafeSeatAccessPoint.h/.cpp.
3. Main starts WIFI_AP_STA and creates SafeSeat before ESP-NOW begins.
4. SafeSeatNow no longer switches the radio back to WIFI_STA, which would
   disable the SoftAP.
5. The SoftAP is fixed to the existing ESP-NOW default channel (6).
6. Serial dashboard shows AP state, IP, channel, and connected client count.
7. No backend, Firebase, WebSocket, HTTP API, or telemetry endpoint is added
   in this step.

ARDUINO CHECK
-------------
Open:
  SafeSeat_Runtime_Main/SafeSeat_Runtime_Main.ino

Use the same Main Hub ESP32 board settings that compiled Step 5.9.4.
Keep the Huge APP partition scheme already required by the Main runtime.

Expected startup includes:
  [NETWORK] SafeSeat local Wi-Fi started.
  [NETWORK] SSID    : SafeSeat
  [NETWORK] IP      : 192.168.4.1
  [NETWORK] Channel : 6

Then:
1. On a phone, look for Wi-Fi "SafeSeat".
2. Join with password "safeseat123".
3. The Main dashboard should change Wi-Fi clients from 0 to 1.
4. If the Step 5.9.5 ESP32-S3 camera is powered, it should also join the AP,
   so the client count can become 2.

IMPORTANT
---------
Step 5.9.6 proves the Main Hub can CREATE the local network without removing
ESP-NOW. Step 5.9.7 is the deliberate coexistence/channel test for C1001,
Piezo, camera ESP-NOW, SafeSeat Wi-Fi, and phone simultaneously.

No full seat/sensor behavior tuning should be performed from this network-only
step; physical design/integration remains separate.
