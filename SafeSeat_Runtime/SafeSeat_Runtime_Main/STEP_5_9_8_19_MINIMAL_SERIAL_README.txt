SAFESEAT STEP 5.9.8.19 — 115200 MINIMAL SERIAL
2026-08-23
================================================

This build is a stability-first simplification.

Use:
  Tools > Upload Speed : 115200
  Serial Monitor       : 115200

Runtime Serial now prints ONLY one short heartbeat every 2 seconds:
  [LIVE] t=... MPU=...Hz FSR=...Hz occ=... MLX=... base=.../30

Removed from runtime Serial:
- 460800 baud
- custom 8192-byte TX buffer
- verbose diagnostic block
- nine-FSR array dump
- verbose MLX/MPU status text
- stack high-water print
- giant dashboard remains disabled

Preserved:
- current FSR acquisition/model logic
- FSR stack-safety change
- MPU 80 Hz scheduler/model
- current MLX target/FOV hold behavior
- Fusion
- ESP-NOW/API code
- no application watchdog
