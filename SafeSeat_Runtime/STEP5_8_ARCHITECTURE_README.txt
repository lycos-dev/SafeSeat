SafeSeat Step 5.8 - Updated Physical/Runtime Architecture
========================================================

M1A C1001 (dashboard/front module)
  C1001 -> dedicated ESP32 -> C1001 filtering + ML -> ESP-NOW -> Main Hub

M1B MLX90614 (headrest/apron module)
  MLX90614 -> wired I2C -> Main Hub

M2 FSR apron
  6 backrest + 3 cushion FSRs -> junction/interface box -> Main Hub

M4 Main Hub / MPU
  Main ESP32 + MPU6050 in rigid junction box
  Main runs Fusion and receives remote ESP-NOW evidence

Piezo seatbelt node
  PVDF -> dedicated ESP32 -> Piezo processing + ML -> ESP-NOW -> Main Hub

ESP32-CAM
  Remains separate for later wireless posture-verification integration.

CURRENT ESP-NOW LINKS
  C1001 node  ---> Main Hub
  Piezo node  ---> Main Hub

The Main Hub broadcasts a channel beacon every 250 ms. Remote nodes
scan 2.4 GHz Wi-Fi channels 1..13 until they find the Main Hub, then
lock to its current channel. This is designed to remain compatible
with a later infrastructure Wi-Fi frontend connection.

IMPORTANT
  This Step 5.8 package does NOT yet add frontend Wi-Fi/BLE and does
  NOT yet integrate ESP32-CAM communication. It only corrects the
  runtime files to match the newly approved C1001 physical design.
