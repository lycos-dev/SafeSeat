SafeSeat Runtime Main - Step 5.1 Fusion Header
=================================================

Changed:
- Fusion.h

Intentionally unchanged:
- Fusion.cpp
- SafeSeat_Runtime_Main.ino
- Config.h
- C1001.cpp/.h
- MLX.cpp/.h
- FSR.cpp/.h
- MPU.cpp/.h

Purpose:
Define the data contracts for Sensor Fusion BEFORE implementing any
decision logic.

Fusion.h now contains:
- FusionSensorHealth
- ModelEvidence
- FusionOccupancyState
- FusionMotionState
- FusionVitalsState
- FusionPressureState
- FusionTemperatureState
- FusionRespirationState
- FusionLevel
- PiezoFusionEvidence placeholder
- CameraFusionEvidence placeholder
- C1001/MLX/FSR/MPU Fusion input wrappers
- FusionInput
- FusionEvidenceSummary
- FusionReading
- FusionEngine declaration

Important:
No Isolation Forest or One-Class SVM implementation is added here.
Those models stay with each sensor pipeline and later populate
ModelEvidence.

Compile expectation:
Because SafeSeat_Runtime_Main.ino does not include/use Fusion yet and
Fusion.cpp remains untouched, this step should not alter runtime behavior.

Next:
Step 5.2 - implement Fusion.cpp decision/context engine.
