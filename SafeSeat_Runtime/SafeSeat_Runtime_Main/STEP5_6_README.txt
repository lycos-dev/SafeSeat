SafeSeat Runtime Main - Step 5.6
MPU6050 Embedded ML + Sensor Fusion
========================================

PURPOSE
-------
This package integrates the final runtime-aligned MPU6050 Isolation Forest
and One-Class SVM models into the SafeSeat Main Hub and connects their output
to Fusion as road/vehicle-motion context.

TRAINED MODEL CONTRACT
----------------------
Sampling rate : 80 Hz deployment model cadence
Window        : 80 samples (~1.0 second)
Stride        : 40 samples (~0.5 second)
Features      : 198
Preprocessor  : median imputer + RobustScaler
Isolation Forest:
  trees        : 400
  contamination: 0.01
  max_features : 0.75
  max_samples  : 256 fitted
One-Class SVM:
  kernel       : RBF
  nu           : 0.05
  gamma        : 0.0005
  support vecs : 105

The exported arrays were generated from the user's canonical Step 5.6.1
artifacts after gap-aware 80-Hz retraining.

RUNTIME UNIT / OFFSET BRIDGE
----------------------------
The current SafeSeat MPU acquisition exposes accelerometer values in g and
gyroscope values in deg/s. Before inference, MPUML performs a stationary
startup baseline using the first 80 unique MPU samples, then converts:

  accel_model = (accel_g - startup_baseline_g) * 9.80665
  gyro_model  = (gyro_deg_s - startup_baseline_deg_s) * pi / 180

This approximates the calibrated Road Data training domain (acceleration in
m/s^2 and gyroscope angular rate in rad/s) without changing the raw MPU
readings shown elsewhere in the dashboard.

IMPORTANT STARTUP REQUIREMENT
-----------------------------
During the MPU startup baseline, keep the ESP32/seat/MPU physically still.
The baseline needs 80 unique MPU samples (about one second at the intended
runtime cadence). After that, the model begins collecting its first 80-sample
window.

SAMPLING NOTE
-------------
MPU.h intentionally keeps SAMPLE_INTERVAL_US = 10000 (100-Hz request).
In the combined SafeSeat main loop this previously yielded about 80-81 Hz in
practice. Do not change it to 12500 us before the final combined test; doing so
could push the achieved cadence below the 80-Hz model contract. The final
all-sensor test should confirm Actual Fs remains near 80 Hz.

FUSION POLICY
-------------
MPU model evidence is CONTEXT ONLY.

  IF normal + SVM normal       -> normal road/motion context
  exactly one model anomaly    -> weak road/motion context
  both models anomaly          -> strong road/motion context

Neither weak nor strong MPU evidence increments occupant anomalyEvidenceCount.
Only strong MPU model agreement is allowed to set motionArtifactPossible and
therefore gate/suppress escalation from transient occupant-sensor changes.
Weak MPU model anomaly is supporting context only.

Raw instantaneous MPU magnitude checks remain as a fast fallback while the
one-second model window is not yet ready.

PHYSICAL AXIS ORIENTATION
-------------------------
For final installation, align the MPU axes consistently with the road-data
reference convention as closely as practical:
  X = vehicle longitudinal direction
  Y = vehicle lateral direction
  Z = vehicle vertical direction

Do not rotate/remap axes in software unless the physical mounting requires it
and the mapping is documented.

VALIDATION COMPLETED BEFORE PACKAGING
-------------------------------------
1. Exact 198-feature C++ extractor checked against Python reference windows.
   Maximum observed feature difference: approximately 2.46e-6.

2. Embedded model prediction parity on all 4,312 held-out test feature rows:
   Isolation Forest: 4,312 / 4,312 predictions matched.
   One-Class SVM   : 4,312 / 4,312 predictions matched.
   Combined labels : 4,312 / 4,312 matched.

   Maximum decision-function differences observed:
   IF    : approximately 5.2513e-4
   OCSVM : approximately 1.612e-6

3. Host C++ syntax checks passed for:
   - MPUFeatureExtractor.cpp
   - MPUInference.cpp
   - MPUML.cpp
   - all MPUModelData_*.cpp files
   - Fusion.cpp
   - SafeSeat_Runtime_Main.ino using local Arduino stubs

A full ESP32 Arduino build was NOT run in the packaging environment.

WHAT TO DO NOW
--------------
1. Copy/replace this package over the current SafeSeat_Runtime_Main folder.
2. In Arduino IDE keep:
     Board             : ESP32 Dev Module
     Partition Scheme  : Huge APP (3MB No OTA / 1MB SPIFFS)
3. Click Verify/Compile only.
4. Do NOT run the full hardware/system test yet. Continue with the remaining
   sensor integration first, then test all fused sensors together.

EXPECTED SERIAL STARTUP ADDITIONS
---------------------------------
[MPU-ML] Embedded Step 5.6.1 model initialized.
[MPU-ML] Model contract: 80 samples / 1.0 s, stride 40 samples.
[MPU-ML] Features: 198, RobustScaler + IF + One-Class SVM.
[MPU-ML] Startup calibration: first 80 runtime samples estimate stationary accel/gyro offsets.
[MPU-ML] Fusion role: vehicle/road-motion artifact context only.

The dashboard now contains:
  MPU ML (ROAD/MOTION CONTEXT)
with baseline/window progress, IF/SVM decisions, and the Fusion context role.

SUGGESTED COMMIT
----------------
feat(mpu): integrate runtime-aligned road-motion context into fusion
