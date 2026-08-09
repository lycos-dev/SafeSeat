SafeSeat Runtime Main - Step 5.4.2
=====================================

Purpose
-------
Embed the corrected Step 5.4.1 MLX90614 anomaly model into the ESP32 runtime.

Runtime MLX path
----------------
MLX object temperature @ 4 Hz
 -> Object-Ambient warm-target qualification
 -> 30-second / 120-sample qualified window
 -> limited interpolation (limit 8, matching Python)
 -> subtract window median
 -> 38 relative/temporal features
 -> median imputer
 -> RobustScaler
 -> Isolation Forest + One-Class SVM
 -> ModelEvidence -> Fusion

Important
---------
- The model does NOT consume ambient temperature.
- Object-Ambient is only the warm-target/environment gate/context.
- Warm target threshold +2.0 C is provisional and must be revalidated on the final seat/headrest geometry.
- This replaces the old 44-feature absolute-temperature MLX model.
- C1001 ML is preserved.
- Fusion.cpp was not redesigned.
- Arduino/ESP32 toolchain compile was NOT performed here.

Embedded model
--------------
Feature count: 38
IF trees: 300
IF total nodes: 30502
OCSVM support vectors: 297
OCSVM gamma: 0.0111471517012

Host parity
-----------
Test feature rows: 1441
IF prediction matches: 1441 / 1441
OCSVM prediction matches: 1441 / 1441

Expected bench behavior
-----------------------
1. Point MLX at room/background:
   Target gate = NOT QUALIFIED
   Window = 0/120
   No MLX anomaly evidence.

2. Point MLX steadily at palm/forehead:
   Target gate = QUALIFIED
   Window progresses toward 120/120.

3. Hold target steady for 30+ seconds.
   First inference should occur.

A stable warm target is no longer rejected merely because its absolute
temperature level differs from WESAD wrist temperature. The model still
may classify a window anomalous if its relative/temporal pattern is unusual.
