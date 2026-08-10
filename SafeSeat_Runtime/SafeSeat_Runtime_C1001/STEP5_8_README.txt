SafeSeat Step 5.8 - Remote C1001 Node
=====================================

Physical architecture:
  C1001 -> dedicated ESP32 -> ESP-NOW -> Main Hub

This project contains the SAME C1001 acquisition/filtering and
SAME trained 64-feature IF + OCSVM model previously compiled into
the Main Hub. They are now executed locally on the dashboard C1001
node instead.

Wiring on the C1001 node:
  C1001 TX -> ESP32 GPIO16 (RX)
  C1001 RX -> ESP32 GPIO17 (TX)
  C1001 GND -> node ESP32 GND
  Power according to the tested C1001 module setup.

No C1001 signal wire runs to the seat/Main Hub.

Wireless:
  ESP-NOW, 2.4 GHz
  Main Hub beacon discovery across channels 1..13
  C1001 evidence packet every 500 ms
  Main Hub freshness timeout: 2500 ms

The Main Hub continues to own final Fusion.
