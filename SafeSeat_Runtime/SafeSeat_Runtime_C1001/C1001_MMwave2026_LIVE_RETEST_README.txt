SAFESEAT C1001 — 2026 60 GHz MMWAVE CANDIDATE — LIVE RETEST
=================================================================

THIS FOLDER IS READY TO OPEN IN ARDUINO IDE.

Open:
    SafeSeat_Runtime_C1001.ino

WHAT CHANGED
------------
The existing 2026-08-22 C1001 motion-gate-fixed runtime is preserved.

Only the embedded C1001 ML model parameters were replaced with the
new 2026 60 GHz radar-domain candidate, plus traceability text/metadata.

The motion-gate behavior remains:
- MoveRange 15-29 does not block an otherwise valid HR/RR sample.
- An isolated MoveRange >=30 holds/skips that one sample.
- A confirmed motion-artifact/recovery interval holds samples.
- The existing 30-second ML window is PRESERVED instead of reset.

NEW EMBEDDED MODEL
------------------
Features:                    64
IF trees:                    300
IF nodes:                    19118
IF max_samples:              75
OCSVM support vectors:       4
OCSVM nu:                    0.01
OCSVM gamma:                 0.0001

OFFLINE COMBINED VERIFICATION
-----------------------------
Held-out healthy windows:    19
Isolation Forest NORMAL:     19/19
Tuned OCSVM NORMAL:          19/19
Both models NORMAL:          19/19
Model agreement:             100.00%

Float32 embedded parity:
IF max decision error:       1.110e-08
SVM max decision error:      2.890e-08
Prediction match:            100% / 100%

C++ end-to-end compile/run:  PASS

OLD FALSE-ANOMALY WINDOW REPLAY
-------------------------------
The exact 30 raw HR/RR samples from your 2026-08-22 physical C1001 run,
which the old BIDMC model classified STRONG ANOMALY, were replayed
through the new embedded pipeline.

New result:
IF decision:                 +0.136897  NORMAL
SVM decision:                +0.004673  NORMAL
Expected combined status:    READY - NORMAL

This is a strong pre-flash check, but it is NOT a substitute for a new
physical-sensor live validation.

EXPECTED STARTUP
----------------
Look for:

[C1001-ML] Embedded model initialized.
[C1001-ML] Window: 30 s @ 1 Hz, stride: 15 s.
[C1001-ML] Isolation Forest + tuned One-Class SVM ready.
[C1001-ML] Model source: 2026 60 GHz mmWave radar candidate.
[C1001-ML] MoveRange is context only; motion samples are held, window preserved.

LIVE RETEST
-----------
1. Upload this exact folder to the C1001 ESP32.
2. Sit normally at the same known-working C1001 distance/orientation.
3. Wait for valid HR + RR.
4. Let the configured 60-second warm-up finish.
5. Stay normally seated until ML reaches 30/30.
6. Keep it running through at least 3 inference windows if practical.
7. Save/send the COMPLETE Serial Monitor log.

PRIMARY PASS TARGET
-------------------
During stable normal sitting:
- ML reaches complete inference windows.
- Anomaly does not remain saturated.
- Ideally both decisions are >= 0 and status is READY - NORMAL for
  most/all stable windows.

DO NOT YET
----------
Do NOT overwrite/promote SafeSeat_ML/models/C1001 canonical joblib models.
We promote this candidate only after the physical live retest passes.
