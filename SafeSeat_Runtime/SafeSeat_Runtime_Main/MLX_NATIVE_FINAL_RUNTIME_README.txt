SafeSeat MLX90614 Native Model - FINAL RUNTIME INTEGRATION
============================================================
Date: 2026-08-23

STATUS
------
The previous WESAD / Empatica E4 surrogate has been RETIRED from deployment.
The Main Hub now uses the live-validated actual-MLX90614 external-data model.

TRAINING / VALIDATION
---------------------
Primary external dataset:
  CorrectionForeheadTemperature human MLX90614 dataset

SafeSeat filtered dataset:
  238 eligible subjects
  190 train subjects
  48 untouched held-out subjects
  3349 train feature rows
  935 held-out feature rows

Isolation Forest:
  300 trees
  contamination 0.025
  held-out normal acceptance 97.54%

One-Class SVM:
  nu 0.02
  gamma 0.03
  held-out normal acceptance 98.72%

Combined both-normal held-out acceptance:
  97.54%

Python -> embedded float32 prediction parity:
  IF 100%
  OCSVM 100%

LIVE VALIDATION
---------------
2026-08-23 air-conditioned exposed-nape testing passed:
  - background correctly rejected as no warm target
  - stable nape built a 30-second baseline around 31.28 C
  - repeated stable IF + OCSVM outputs were NORMAL
  - movement/lean-away produced unstable-block holds or target-loss reset
  - after a later baseline around 30.48 C, readings around 29.2-30.3 C
    remained model-normal while target geometry was still thermally qualified

MODEL CONTRACT
--------------
Physical MLX acquisition remains 4 Hz.
Every 4 accepted samples -> one 1-second block.

Quality gates:
  Object-Ta >= +2.0 C
  1-second object-temperature std <= 0.50 C

30 stable seconds establish a personal/session baseline.

ONLY model features:
  1. current stable 1-second object mean - session baseline
  2. absolute value of feature 1

Ambient temperature, Object-Ta and measurement stability are NOT model inputs.
They remain quality/context signals.

FUSION ROLE
-----------
The native MLX model is now active conservative evidence:
  both IF+OCSVM anomaly -> one strong MLX sensor vote
  either-only anomaly   -> one weak MLX sensor vote
  both normal           -> one normal MLX sensor vote

MLX context change remains supporting context only; it is never counted as a
second independent anomaly vote from the same temperature signal family.

POSTURE / GEOMETRY FOLLOW-UP (INTENTIONALLY NOT IMPLEMENTED YET)
---------------------------------------------------------------
Live testing showed that leaning forward/away can lower the MLX object reading
while the person is still seated. A later Fusion update should use validated
FSR posture/contact evidence to mark MLX targeting as geometry-degraded and
preserve/hold the baseline during brief lean-away events.

Do NOT feed FSR values into the MLX IF/OCSVM model itself. This belongs in the
quality/fusion layer and should be implemented only after combined live testing.
