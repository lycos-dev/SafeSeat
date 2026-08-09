SAFESEAT STEP 5.4.5 - MLX SENSOR-FUSION FINALIZATION
====================================================

PURPOSE
-------
Finish the MLX90614 fusion role before moving to the next sensor.
This step deliberately separates:

1. MLX hardware acquisition
2. old WESAD surrogate IF/OCSVM diagnostics
3. deployment-safe MLX Fusion context

WHY THE OLD MLX MODEL IS NOT FUSED
----------------------------------
Step 5.4.3 showed that the WESAD E4 contact-temperature feature
space does not transfer safely to the real non-contact MLX90614.
The model may still run for traceability/diagnostics, but its
NORMAL/ANOMALY result is NOT allowed to contribute anomaly votes.
This prevents a known domain-mismatched model from contaminating
system-level Fusion.

STEP 5.4.5 MLX FUSION ALGORITHM
-------------------------------
Primary temperature signal:
    filtered MLX OBJECT temperature

MLX Ambient/Ta:
    context only; it is sensor/package temperature Ta

Object-Ta:
    thermal-contrast quality gate only
    it is NOT body temperature

Qualified target rule:
    Object-Ta >= +2.0 C
    provisional engineering gate only; not medical

Session baseline:
    120 accepted filtered samples @ 4 Hz = 30 seconds
    baseline = median(filtered object temperature)

Context-change reference:
    abs(current filtered object - baseline) > 1.85 C

The 1.85 C value is rounded from the FDA Step 5.4.4 p99
within-subject repeated-round surface-temperature range of 1.8258 C.
It is a BROAD CONTEXT marker only, not a medical abnormal-temperature
threshold and not claimed to equal nape physiology.

FUSION POLICY
-------------
MLX status can become:
    UNKNOWN
    INVALID
    NO THERMAL TARGET
    BASELINE BUILDING
    STABLE
    CONTEXT CHANGE

STABLE:
    contributes normal context.

CONTEXT CHANGE:
    contributes supportingContextCount only.
    It CANNOT create anomalyEvidenceCount by itself.
    Therefore MLX alone cannot create WARNING or EMERGENCY.

WESAD IF/OCSVM:
    retained as diagnostic-only output.
    Fusion ignores its anomaly votes in Step 5.4.5.

WHY THIS IS CONSERVATIVE
------------------------
- avoids forcing contact-E4 behavior onto non-contact MLX
- keeps Object temperature as the actual primary measurement
- avoids treating Object-Ta as body temperature
- uses the FDA non-contact dataset only where it is defensible:
  broad surface-temperature context/repeatability
- leaves final deployment-domain anomaly-model calibration for the
  real headrest + nape + car geometry after all sensors are fused

FILES ADDED
-----------
MLXContext.h
MLXContext.cpp
MLX_FDA_CONTEXT_REFERENCE.json
STEP5_4_5_README.txt
STEP5_4_5_CHECKS.json

FILES MODIFIED
--------------
Fusion.h
Fusion.cpp
MLXML.cpp
SafeSeat_Runtime_Main.ino

STEP 5.4.3 VERBOSE FEATURE DUMPS
--------------------------------
Disabled in MLXML.cpp for Step 5.4.5. The old model still runs, but
its detailed 38-feature dump no longer floods Serial. This keeps the
runtime ready for the later all-sensor combined test.

TESTING POLICY
--------------
Do NOT perform the next standalone MLX test now.
Proceed to the next sensor integration. Hardware testing will be done
once every sensor has its final fusion/evidence path connected, so the
full SafeSeat dashboard can be evaluated together.
