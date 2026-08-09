SafeSeat Runtime Main - Step 5.5
FSR Runtime-Aligned Embedded ML + Fusion
=========================================

WHAT THIS STEP DOES
-------------------
1. Embeds the user's final tuned Step 5.5 FSR models:
   - StandardScaler
   - Isolation Forest
   - One-Class SVM

2. Mirrors the retrained runtime representation:
   - ~4.5 Hz completed FSR frames
   - 23-frame window (~4.9-5 s)
   - 5-frame stride (~1.1 s)
   - 93 features
   - per-frame nine-sensor pressure shares
   - absolute pressure magnitude is NOT model input

3. Adds FSRML to the Main Hub.

4. Connects FSR ModelEvidence to Fusion:
   - both IF + OCSVM anomaly => strong FSR anomaly evidence
   - either one anomaly      => weak FSR anomaly evidence
   - both normal             => normal FSR model evidence
   - no valid model window   => do not falsely call FSR normal

5. Empty/zero-pressure frames are not sent into the anomaly model.
   C1001 presence, FSR occupied, or FSR back-contact is used only to
   qualify whether a seated-pressure window is meaningful.

IMPORTANT INTERPRETATION
------------------------
- High raw FSR totals are NOT anomalies.
- The model learns pressure distribution and redistribution.
- Raw contact-loss/asymmetry values remain contextual diagnostics.
- One FSR sensor alone cannot create EMERGENCY. Fusion still requires
  persistent strong multi-sensor evidence for the camera/emergency path.

MODEL ARTIFACTS USED
--------------------
Feature count          : 93
Target runtime rate    : 4.5 Hz
Window                 : 23 completed frames
Stride                 : 5 completed frames
Isolation Forest       : 200 trees, contamination=0.01
One-Class SVM          : nu=0.01, gamma=0.005
OCSVM support vectors  : 66

USER'S HELD-OUT RESULTS
-----------------------
Isolation Forest anomaly rate : 1.56%
One-Class SVM anomaly rate    : 6.43%
Both-model anomaly rate       : 0.91%
Either-model anomaly rate     : 7.08%

EMBEDDED PARITY
---------------
All 1,540 held-out feature rows were checked against the exported
float32 embedded-model simulation:
- IF prediction parity   : 1540 / 1540
- OCSVM prediction parity: 1540 / 1540
- max IF score difference: ~1.19e-8
- max SVM score diff     : ~6.68e-7

A compiled host C++ check also passed for:
- FSR feature extractor
- IF + OCSVM inference
- FSRML source syntax

See FSR_EMBEDDED_MODEL_VALIDATION.json for details.

WHAT TO DO NOW
--------------
1. Copy/extract the contents of this folder into your current
   SafeSeat_Runtime_Main folder, replacing matching files.
2. Open SafeSeat_Runtime_Main.ino in Arduino IDE.
3. Click Verify/Compile only.
4. Do NOT start the full sensor hardware test yet. Per the current
   workflow, continue integrating MPU6050 and Piezo into Fusion first.

EXPECTED STARTUP ADDITIONS
--------------------------
[FSR-ML] Embedded Step 5.5 model initialized.
[FSR-ML] Runtime alignment: 23 completed FSR frames @ ~4.5 Hz.
[FSR-ML] Stride: 5 completed frames (~1.1 s).
[FSR-ML] Representation: per-frame 9-sensor pressure shares.
[FSR-ML] Absolute pressure magnitude is NOT model input.
[FSR-ML] Isolation Forest + One-Class SVM ready.

EXPECTED DASHBOARD ADDITION
---------------------------
FSR ML:
  Status       : ...
  Window       : x / 23
  Next infer   : x frame(s)
  Windows      : x
  Input        : 9-sensor pressure shares (absolute scale removed)
  IF decision  : ...
  SVM decision : ...
  Fused result : NORMAL / WEAK ANOMALY / STRONG ANOMALY

NOTE
----
This package was host-validated but not compiled with the actual ESP32
Arduino toolchain in the assistant environment. Arduino IDE Verify on
the user's machine remains the compile proof for the final runtime.
