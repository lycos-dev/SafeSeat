SafeSeat Runtime Main - Step 5.3 C1001 Embedded ML
=====================================================

PURPOSE
-------
Connect the trained C1001 Isolation Forest + One-Class SVM pipeline to
Fusion as real ModelEvidence.

TRAINING SOURCE OF TRUTH
------------------------
features/C1001/feature_manifest.json:
- Sampling rate: 1 Hz
- Window: 30 seconds
- Overlap: 50%
- Runtime stride: 15 new samples
- Features: 64

The stale config/c1001_config.json "window_size": 10 value is NOT used.
The generated feature manifest and trained artifacts are the source of truth.

RUNTIME FLOW
------------
C1001 hardware (1 Hz)
  -> existing 60 s warm-up / motion filtering remains untouched
  -> C1001ML collects clean raw HR + RR monitor values
  -> 30-sample feature window
  -> exact 64-feature extraction
  -> median imputer
  -> RobustScaler
  -> 300-tree Isolation Forest
  -> 85-support-vector RBF One-Class SVM
  -> C1001 ModelEvidence
  -> Fusion

FIRST RESULT TIMING
-------------------
After C1001's existing 60-second warm-up completes, the ML pipeline needs
30 clean 1-Hz samples for its first inference.

So under uninterrupted clean conditions:
- C1001 warm-up: about 60 s
- First ML window: another 30 s
- First C1001 model result: about 90 s after the warm-up timer initially started
- Later model results: every 15 new clean samples

MOTION / INVALID-SAMPLE POLICY
------------------------------
The model training source contained cleaned physiological monitor values.
Therefore:
- strong/moderate C1001 motion or motion recovery resets the ML window
- sensor values outside the broad BIDMC training-validity bounds reset the ML window
- stale model evidence is immediately invalidated on reset

This prevents movement artifacts or invalid sensor codes from becoming
physiological model votes.

FILES ADDED
-----------
- C1001ML.h/.cpp
- C1001FeatureExtractor.h/.cpp
- C1001Inference.h/.cpp
- C1001ModelData.h
- C1001ModelData_Preprocessor.cpp
- C1001ModelData_IF_Structure.cpp
- C1001ModelData_IF_Children.cpp
- C1001ModelData_IF_Thresholds.cpp
- C1001ModelData_OCSVM.cpp
- C1001_FEATURE_COLUMNS.json
- C1001_TRAINING_METADATA.json
- C1001_EMBEDDED_MODEL_VALIDATION.json

FILES MODIFIED
--------------
- C1001.h/.cpp
  Added sampleSequence/sampleTimestampMillis so the fast main loop can
  consume each completed 1-Hz C1001 hardware poll exactly once.
  Existing acquisition/filter logic is otherwise preserved.

- SafeSeat_Runtime_Main.ino
  Runs C1001ML, passes the resulting ModelEvidence into Fusion, and prints
  model-window/decision diagnostics.

- Fusion.h/.cpp
  Replaced with the final Step 5.1/5.2 versions approved immediately
  before this integration.

UNCHANGED SENSOR ACQUISITION MODULES
------------------------------------
- MLX.cpp/.h
- FSR.cpp/.h
- MPU.cpp/.h
- Config.h

VALIDATION
----------
See C1001_EMBEDDED_MODEL_VALIDATION.json.

The generated C++ implementation was tested against the Python pipeline:
- 100 feature windows checked: max feature error < 4e-6
- 327 saved feature rows: IF prediction match = 100%
- 327 saved feature rows: OCSVM prediction match = 100%
- 316 complete raw 30-s windows end-to-end:
  IF prediction match = 100%
  OCSVM prediction match = 100%

The generated C++ feature/model files also passed a host C++17 syntax build.

WHAT TO TEST ON THE ESP32
-------------------------
1. Compile/upload.
2. Confirm all four Main Hub sensors still initialize.
3. Sit normally and wait through the existing C1001 warm-up.
4. Watch:
   C1001 ML -> Status
   C1001 ML -> Window
   C1001 ML -> IF decision
   C1001 ML -> SVM decision
   C1001 ML -> Fusion vote
5. First inference should appear after 30 clean post-warm-up samples.
6. Later inference should occur every 15 clean samples.
7. Move strongly and confirm the C1001 ML window resets instead of using
   the motion-contaminated window.

IMPORTANT
---------
The models are anomaly detectors trained from surrogate/public datasets.
Their anomaly outputs are system evidence, not a medical diagnosis.
