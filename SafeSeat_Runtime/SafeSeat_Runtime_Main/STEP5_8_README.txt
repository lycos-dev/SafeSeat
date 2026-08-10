SafeSeat Step 5.8 - Main Hub with Remote C1001
==============================================

Architecture change:
  OLD: C1001 -> UART -> Main ESP32, with C1001 ML compiled on Main.
  NEW: C1001 -> dedicated ESP32 -> local C1001 ML -> ESP-NOW -> Main.

Main Hub now physically owns:
  - MLX90614
  - 9 FSRs (ADS1115 x2 + native ADC)
  - MPU6050
  - Fusion
  - ESP-NOW receiver/beacon service

Remote nodes:
  - C1001 node: acquisition + warm-up/filter + 64-feature IF/OCSVM
  - Piezo node: 25 Hz signal processing + 16-feature IF/OCSVM

Main Hub no longer contains:
  - C1001 UART pins
  - DFRobot C1001 hardware acquisition implementation
  - C1001 feature extractor/model arrays/inference implementation
  - C1001 FreeRTOS acquisition task

C1001 evidence still enters Fusion through the SAME C1001FusionInput
contract, so Fusion.cpp decision logic is unchanged.

Freshness:
  Remote C1001 packets are invalidated after 2500 ms without a valid
  packet. Stale presence/vitals/model results are not fused.

FSR occupancy gate:
  Fresh remote C1001 presence can qualify an FSR window, but the FSR
  model still retains its own calibrated occupied/back-contact gates.

Main Hub should continue using Huge APP because FSR + MPU + other
runtime code/models are still compiled locally.
