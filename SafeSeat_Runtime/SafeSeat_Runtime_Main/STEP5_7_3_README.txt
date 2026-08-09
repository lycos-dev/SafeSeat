SafeSeat Step 5.7.3 - ESP-NOW Wireless Piezo Link
=================================================

This replaces ONLY the Piezo <-> Main Hub UART transport.
The Step 5.7 Piezo models and the Fusion policy are unchanged.

CURRENT LINK
------------
Piezo ESP32:
  PVDF -> 25 Hz -> 30 s model window -> 16 features -> IF + OCSVM
       -> 40-byte Piezo evidence packet -> ESP-NOW broadcast

Main Hub:
  SafeSeatNow -> receives latest Piezo packet -> CRC16 validation
              -> Piezo ModelEvidence -> Fusion

No Piezo/Main signal wire is required.
No shared communication ground is required when both boards are independently powered.

CHANNEL DISCOVERY
-----------------
Main Hub starts on channel 6 while no frontend Wi-Fi is configured.
It broadcasts an 8-byte SafeSeatHubBeacon every 250 ms.

Piezo starts at channel 6 and listens for the hub beacon.
If no valid beacon is heard for 2 seconds, it scans channels 1..13,
350 ms per channel. When a valid hub beacon is heard it locks to that channel.

This is intentional preparation for later frontend Wi-Fi. If the Main Hub later
joins a 2.4 GHz AP and moves to the AP channel, the Piezo can lose the old beacon,
rescan, and find the Main Hub on the new channel.

ESP32-CAM
---------
The same SafeSeat hub-beacon/channel-discovery transport can later be reused by
the ESP32-CAM. ESP-NOW should carry small camera trigger/status packets, not image
frames. Image upload/streaming should use normal Wi-Fi.

FRONTEND PLAN (NOT IMPLEMENTED IN THIS ZIP)
-------------------------------------------
Recommended later Main Hub transport:
  Wi-Fi = primary server/backend connection
  BLE   = local fallback frontend connection

BLE should be treated as a local-device fallback, not a direct replacement for an
Internet/server path unless a phone/computer acts as a gateway.

Wi-Fi, BLE, and ESP-NOW all share the ESP32 2.4 GHz radio, so later frontend code
must be tested under simultaneous load. Keep Piezo ESP-NOW traffic low (2 packets/s).

VERIFY
------
Main Hub:
  Keep Tools -> Partition Scheme -> Huge APP (3MB No OTA / 1MB SPIFFS)

Piezo:
  Default APP is okay if it fits; Huge APP is also okay.

Arduino ESP32 core target used for this transport: 3.3.11.

Expected Piezo dashboard after both boards are powered:
  ESP-NOW link      : LOCKED
  Wi-Fi channel     : <same as Main Hub>
  Hub beacons       : increasing
  Packets sent      : increasing

Expected Main dashboard:
  Transport      : ESP-NOW
  Connected      : YES
  Packets RX     : increasing
  Wi-Fi channel  : same as Piezo

Do not connect the old GPIO17 -> GPIO25 UART wire for Step 5.7.3.
