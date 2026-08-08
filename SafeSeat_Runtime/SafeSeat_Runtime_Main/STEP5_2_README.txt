SafeSeat Runtime Main - Step 5.2 Fusion Decision Engine
=========================================================

Status: COMPLETE / FROZEN for current integration stage.

Fusion architecture:
- Sensor acquisition remains inside each sensor module.
- Sensor-specific ML remains outside Fusion.
- Fusion consumes ModelEvidence only.
- One physical sensor contributes one independent model vote.
- IF + OCSVM both anomalous = strong sensor evidence.
- Only one model anomalous = weak sensor evidence.
- MPU raw movement is artifact/context information, not a medical vote.
- Warning/emergency persistence is millis()-based.
- Strong persistent multi-sensor concern requests camera verification.
- Final EMERGENCY requires a valid abnormal camera result.
- Piezo and Camera communication remain future integration steps.
