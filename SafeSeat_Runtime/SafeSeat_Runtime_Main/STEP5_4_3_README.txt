SafeSeat Runtime - Step 5.4.3 MLX Live Feature Diagnostics

PURPOSE
-------
Diagnose why a stable real MLX90614 palm/skin window is still classified as anomalous after Step 5.4.1 median-centered retraining.

THIS PACKAGE DOES NOT RETRAIN OR RETUNE ANY MODEL.
It does not change Fusion, C1001, FSR, MPU, the MLX warm-target gate, or the MLX IF/OCSVM parameters.

WHAT WAS ADDED
--------------
1. MLXFeatureDiagnostics.h/.cpp
2. A diagnostic call after MLX inference.
3. WESAD p01/p05/p50/p95/p99 references calculated from the user's actual retrained train_features.csv (5,678 windows, 38 features).
4. MLX_WESAD_FEATURE_REFERENCE.json for audit/reproducibility.

SERIAL BEHAVIOR
---------------
For the FIRST FOUR successful MLX inference windows only, Serial prints:
  - IF/SVM decisions
  - count of live features outside WESAD p05-p95
  - count outside WESAD p01-p99
  - all 38 live features
  - WESAD p01, p50, p99 for each
  - OUTSIDE_P99 / outside_p95 / inside flag

After four windows, detailed dumps stop automatically so normal runtime timing is not continuously burdened. ML inference continues.

IMPORTANT
---------
The MLX model input remains rawObjectC at 4 Hz. The dashboard's Object value is filteredObjectC. This diagnostic intentionally examines the exact features created from the raw model input; it does NOT switch the model to filtered input yet.

TEST
----
1. Upload this package.
2. Point MLX at empty room: MLX ML should remain WAITING FOR WARM TARGET.
3. Point it steadily at palm/forehead and avoid changing distance/angle.
4. Keep it there for at least ~75 seconds after the target qualifies, so four windows are captured (first at ~30 s, then ~45, ~60, ~75 s).
5. Copy the [MLX-DIAG] sections / full Serial log.

NEXT DECISION
-------------
Use those live-vs-WESAD feature comparisons to decide whether the next correction is:
- matching WESAD/MLX smoothing + downsampling,
- removing sensor-quantization-sensitive features,
- both,
without guessing or weakening anomaly thresholds.
