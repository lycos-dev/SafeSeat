SafeSeat Runtime Main - Step 5.2A-1 Fusion Foundation
======================================================

Changed:
- Fusion.cpp

Unchanged:
- Fusion.h from Step 5.1
- SafeSeat_Runtime_Main.ino
- Config.h
- C1001.cpp/.h
- MLX.cpp/.h
- FSR.cpp/.h
- MPU.cpp/.h

Implemented:
- FusionEngine constructor
- FusionEngine::begin()
- FusionEngine::update() compile-safe shell
- FusionEngine::getReading()
- all Fusion enum-to-text helper functions

Current behavior:
- Fusion starts in WATCH
- confidence = 0
- no camera trigger
- no alert trigger
- no sensor context is classified yet
- no Isolation Forest / OCSVM logic is added

This is intentional.

Next:
Step 5.2A-2
- sensor health interpretation
- occupancy evaluation
- motion evaluation
