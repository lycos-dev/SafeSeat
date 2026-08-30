SafeSeat Main Hub - Controlled UAT Warning Control
Date: 2026-08-30

Purpose
-------
Adds researcher-only controls to http://192.168.4.1/uat so participant UAT can be performed with the Main Hub battery-powered and untethered from a laptop.

Controls
--------
- Simulate Warning: activates the already validated SYS02 one-strong-FSR-model-vote stimulus.
- Clear Simulation: removes injected evidence; Fusion recovers using normal production hysteresis.
- Stimulus status is visible in /uat and recorded in downloaded evaluator CSV as uat_controlled_stimulus.

Safety / interpretation
-----------------------
- Fusion remains authoritative.
- The participant app state is never directly forced.
- No trained model, threshold, sensor acquisition path, camera safety rule, or Fusion decision rule is changed.
- /uat deliberately has NO Emergency injection control.
- Simulate Warning is rejected unless the latest Fusion occupancy is OCCUPIED.
- POST is used for control actions to prevent accidental activation by simply opening a URL.
- Serial engineering commands SYS02_ON, SYS03_ON, TEST_OFF, TEST_STATUS remain available for regression/debugging only.

Recommended UAT wording in the thesis
--------------------------------------
Describe this as a controlled simulated Warning stimulus used to evaluate interface comprehension/usability. Do not report it as a naturally detected medical/anomaly event.

Changed source files
--------------------
- SafeSeat_Runtime_Main.ino
- SafeSeatApi.h
- SafeSeatApi.cpp
- SafeSeatUatPage.h
- UAT_CONTROLLED_WARNING_README.txt
