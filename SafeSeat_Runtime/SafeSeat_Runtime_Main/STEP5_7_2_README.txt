SafeSeat Step 5.7.2 - Piezo Communication + Full Sensor Fusion
================================================================

BASE
----
This Main Hub folder extends the confirmed Step 5.6 MPU runtime.
It also includes the Arduino-core macro fix:
  MPUML::DEG_TO_RAD -> MPUML::DEG_TO_RAD_FACTOR

PIEZO LINK
----------
One-way UART:
  Separate Piezo ESP32 GPIO17 (TX) -> Main Hub GPIO25 (RX)
  Separate Piezo ESP32 GND         -> Main Hub GND

Baud: 115200
Packet interval: 500 ms
Main freshness timeout: 2500 ms
Packet: 40-byte packed binary payload with CRC16.

The Main Hub does NOT embed a second copy of the Piezo model.
The Piezo ESP32 runs its own signal processing + IF/OCSVM and sends only
current evidence to the Main Hub.

FUSION POLICY
-------------
Piezo is a WESAD RespiBAN -> PVDF surrogate respiration-pattern model.

- Piezo either-only anomaly:
    supporting context only.
- Piezo both-model anomaly without C1001 anomaly:
    WATCH context only; no WARNING/EMERGENCY/camera.
- Piezo both-model anomaly + C1001 weak anomaly:
    adds an independent anomaly vote, but not a strong Piezo vote.
- Piezo both-model anomaly + C1001 strong anomaly:
    adds a strong corroborating anomaly vote.
- Auxiliary Piezo 15 s no-breath timer:
    context/state only; never a standalone anomaly vote.

MPU remains artifact/road-motion context only.
MLX WESAD model remains diagnostic-only; MLXContext is fused.

ARDUINO
-------
Board: ESP32 Dev Module
Partition Scheme: Huge APP (3MB No OTA / 1MB SPIFFS)

Verify the Main Hub sketch before hardware testing.

CURRENT SENSOR-FUSION STATUS
----------------------------
C1001 : model -> Fusion
MLX   : context -> Fusion
FSR   : model -> Fusion
MPU   : road/motion context -> Fusion
Piezo : remote model evidence -> Fusion

Camera remains the post-fusion verifier and is not changed by Step 5.7.2.
