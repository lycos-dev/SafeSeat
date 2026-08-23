SAFESEAT MLX90614 FILTERED-STABILITY FIX — 2026-08-23
=======================================================

WHY THIS FIX EXISTS
-------------------
During INT-HW-02 combined-hardware testing, the occupant remained seated with
the MLX aimed at the nape, but the MLX native wrapper repeatedly reported:

  UNSTABLE TARGET - BLOCK HELD
  1-s obj std ≈ 2–3 C
  baseline 0 / 30

At the same time, the separate MLXContext path — which uses the trusted
median+EMA filtered object temperature — was able to accumulate/build a
temperature baseline and later report a stable baseline around 30.09 C.

That is strong evidence that "UNSTABLE TARGET" was not a statement that the
person was moving. It came from using four instantaneous RAW object readings
for the native model's hard 0.50 C stability gate while the production sensor
path already had a validated median+EMA filter.

FIX
---
The native MLX model wrapper still:
- consumes each NEW accepted physical 4-Hz sample only once;
- forms 4 samples -> 1-second blocks;
- uses the same +2.0 C acquire / +1.0 C hold / 3-second loss persistence;
- uses the same 30 stable-second personal/session baseline;
- uses the same two model features:
    object_delta_from_baseline_c
    object_abs_delta_from_baseline_c
- uses the same IF and OCSVM models.

The only preprocessing correction is:

  BEFORE:
    native block input = rawObjectC / rawAmbientC

  NOW:
    native block input = filteredObjectC / filteredAmbientC
    (the existing median-of-three + EMA production signal)

The 0.50 C stability threshold is NOT loosened in this fix.

Raw sensor values are still retained for diagnostics.

WHY THIS IS JUSTIFIED
---------------------
The trained model is baseline-relative and does not use absolute ambient,
Object-Ta, or stability as ML features. The external training rows are
measurement-level MLX90614 object-temperature records with repeatability
screening, rather than single noisy instantaneous IR reads.

Using the already-trusted filtered runtime object temperature therefore avoids
letting instantaneous FOV jitter prevent the personal baseline from ever
forming, without changing the trained model.

NEXT TEST
---------
Repeat INT-HW-02 with the person seated normally and the MLX fixed on the nape.

Expected:
- target gate qualifies;
- "1-s filt std" should be much lower than the prior raw 2–3 C values;
- baseline should advance toward 30 / 30;
- short thermal-contrast dips should be HELD, not hard reset;
- no NaN/I2C lockup.

If the FILTERED 1-second std still repeatedly exceeds 0.50 C, then the remaining
issue is genuinely physical/FOV geometry or the filter itself, not the user's
ability to sit still.
