SAFESEAT RUNTIME MAIN - STEP 5.4
MLX90614 EMBEDDED ML INTEGRATION
================================

WHAT THIS STEP ADDS
-------------------
Step 5.4 embeds the trained MLX90614/WESAD temperature anomaly models
into the Main Hub runtime while preserving Step 5.3 C1001 ML and the
Step 5.1/5.2 Fusion architecture.

Runtime path:

    MLX90614 hardware
        -> raw object temperature @ 4 Hz
        -> 30-second / 120-sample window
        -> 44 training-matched temporal features
        -> Median Imputer
        -> RobustScaler
        -> Isolation Forest
        -> One-Class SVM
        -> ModelEvidence
        -> Fusion

IMPORTANT DATA-MAPPING RULE
---------------------------
The trained WESAD model uses wearable skin-temperature patterns.

For embedded deployment:
- MLX90614 OBJECT temperature is the model stream.
- MLX90614 AMBIENT temperature is NOT passed into the trained model.
- Ambient and object-minus-ambient remain environmental/context fields
  for Fusion and later real-seat compensation.

This follows the training metadata deployment note.

TRAINING SOURCE OF TRUTH
------------------------
Dataset: WESAD Empatica E4 TEMP
Sampling rate: 4 Hz
Window: 30 s
Samples/window: 120
Overlap: 50%
Stride: 15 s / 60 samples
Features: 44

Isolation Forest:
- 300 trees
- contamination = 0.05
- max_samples = 256

One-Class SVM:
- RBF
- nu = 0.05
- gamma = scale in training
- exported learned gamma = 0.013309710746992448
- 296 support vectors

FILES ADDED
-----------
MLXML.h / MLXML.cpp
MLXFeatureExtractor.h / MLXFeatureExtractor.cpp
MLXInference.h / MLXInference.cpp
MLXModelData.h
MLXModelData_Preprocessor.cpp
MLXModelData_IF_Structure.cpp
MLXModelData_IF_Children.cpp
MLXModelData_IF_Thresholds.cpp
MLXModelData_OCSVM.cpp

Also included:
MLX_FEATURE_COLUMNS.json
MLX_FEATURE_MANIFEST.json
MLX_TRAINING_METADATA.json
MLX_EMBEDDED_MODEL_VALIDATION.json
STEP5_4_CHECKS.json

VALIDATION
----------
Embedded model-only prediction parity on all 1,441 stored test feature
rows:
- Isolation Forest prediction match: 100%
- One-Class SVM prediction match: 100%

End-to-end raw WESAD full-window validation on 1,438 complete 120-sample
windows:
- C++ feature extraction -> embedded preprocessing -> embedded IF/OCSVM
- Isolation Forest prediction match: 100%
- One-Class SVM prediction match: 100%
- maximum absolute feature error: < 0.000057

Synthetic missing-data interpolation cases were also checked against the
Python feature-engineering behavior.

WHAT TO EXPECT ON SERIAL
------------------------
Immediately after startup:

    [MLX-ML] Embedded model initialized.
    [MLX-ML] Window: 30 s @ 4 Hz, stride: 15 s.
    [MLX-ML] Source: raw MLX90614 object temperature.

The dashboard then shows:

    MLX ML:
      Status       : COLLECTING 30 s WINDOW
      Window       : x / 120
      Next infer   : ...
      Windows      : 0
      Model result : not ready

After roughly 30 seconds of uninterrupted MLX sampling, the first result
appears. New inferences occur every 60 new samples (~15 seconds).

TESTING NOTE
------------
Until the actual SafeSeat seat/headrest geometry exists, this is primarily
an EMBEDDED-INFERENCE FUNCTIONAL TEST.

The WESAD source was wearable skin temperature. A room/wall/empty target
is not equivalent to the final MLX90614 human-surface deployment, so a live
ANOMALY result during bench testing should not be interpreted as a final
model-quality verdict or as a medical emergency.

When the physical seat exists, the harder deployment phase begins:
placement, target geometry, ambient compensation, domain shift, real-seat
baseline validation, multi-sensor timing, and Fusion behavior.

No Piezo communication, FSR ML, MPU ML, or camera communication is added
in this step.


STEP 5.4 TARGET-GATE PATCH
==========================

Problem observed during bench validation:
- MLX ML began filling a WESAD skin-temperature window while the sensor
  was pointed at room/background (~24 C), so IF and OCSVM correctly saw
  that deployment window as far outside the learned skin-temperature
  distribution.

Patch:
- MLX ML now requires a provisional warm-target qualification before it
  starts/continues the 30 s / 120-sample model window.
- Qualification: filtered Object-Ambient >= +2.0 C.
- If the target is not qualified, the ML window is cleared and
  ModelEvidence remains invalid/unavailable to Fusion.
- When a warm target appears, collection starts from 0/120, preventing
  room/background samples from contaminating the human-temperature window.
- The trained model input is STILL raw object temperature only. Ambient and
  Object-Ambient remain context/gating information, not trained features.

IMPORTANT:
+2.0 C is a provisional engineering gate based on the current bench log,
where background was around -0.7..0 C delta and forehead targeting reached
about +5..+8 C. It is NOT a medical threshold and MUST be recalibrated with
the final SafeSeat headrest geometry, distance, cabin temperature, and AC
conditions.
