SAFESEAT C1001 — POST-MOTION / TARGET REACQUISITION GATE
===========================================================

WHY THIS PATCH EXISTS
---------------------
The 2026-08-22 disturbance test showed a specific deployment failure:

- Strong motion/obstruction was detected correctly at first.
- MoveRange then quickly fell back to small values.
- C1001 HR/RR stayed falsely elevated for many seconds.
- The old 3-sample sustained-change rule eventually accepted the artifact.
- Contaminated raw HR/RR entered the 30-second ML window and later produced
  a WEAK ANOMALY.

The ML model itself is unchanged. This patch improves signal-quality handling
BEFORE raw C1001 HR/RR is allowed into the model.

NEW RUNTIME POLICY
------------------
1. ANY MoveRange >= 30 immediately starts target reacquisition.
2. The current ML window is PRESERVED — never reset by this quarantine.
3. Last trusted filtered HR/RR is held.
4. New raw HR/RR is NOT fed to ML during reacquisition.
5. Fast recovery requires 5 consecutive samples with:
     - MoveRange < 15
     - valid HR/RR
     - RR within 5 BPM of the pre-motion trusted RR
     - HR within 15 BPM of the pre-motion trusted HR
6. If the old baseline is not recovered within 30 post-motion samples, the
   runtime switches to a controlled fresh-baseline rebuild using 5 clean,
   low-motion samples. ML remains held during that rebuild.

This avoids the exact observed failure where values like:
    23 / 109
    24 / 112
    24 / 115
    23 / 118
    23 / 121
were smooth enough to fool the old 3-sample confirmation rule.

WHAT YOU SHOULD SEE DURING RETEST
---------------------------------
After a strong obstruction/body shift/leave-reenter event:

    Status         : REACQUIRING TARGET - HOLD
    Reacquisition  : ACTIVE x / 5 stable, age y s

    ML status      : HOLD - TARGET REACQUISITION (WINDOW PRESERVED)
    ML window      : stays where it was
    ML reacq holds : increases

IMPORTANT:
Raw RR/HR may still jump high. That is the C1001 sensor estimator itself.
The success condition is that FILTERED values remain held and those bad raw
values do NOT enter ML.

When five safe samples are seen:

    Reacquisition  : IDLE
    ML status      : COLLECTING 30 s WINDOW

and the existing window continues rather than starting over.

If recovery stays far from the old baseline for 30 seconds, expect:

    Status         : REACQUIRING NEW BASELINE
    Reacquisition  : REBASELINE x / 5 stable ...

After five clean low-motion samples, a fresh median baseline is initialized
and ML resumes. This prevents a genuine lasting physiological shift from
being hidden forever.

UPLOAD
------
Open and upload the COMPLETE folder:

    SafeSeat_Runtime_C1001/
        SafeSeat_Runtime_C1001.ino
        ...all companion .cpp/.h files...

Do not copy only the .ino file. The 2026 60 GHz IF + tuned OCSVM model is
already included and unchanged from the live-normal PASS package.

RETEST ORDER
------------
Use the same disturbance sequence that exposed the problem:

A. Stable normal sitting.
B. Brief obstruction around ML window 15/30.
C. Longer obstruction after the first completed ML window.
D. Body shift, then return to normal.
E. Leave the target area, then re-enter and sit normally.

Send the full Serial Monitor log afterward.

EXPECTED KEY RESULT
-------------------
The bad post-motion raw HR/RR may still appear, but should remain quarantined.
You should NOT see them become SUSTAINED CHANGE CONFIRMED immediately after
strong motion, and they should not contaminate the ML window.

GIT COMMIT AFTER RETEST PASSES
------------------------------
fix(c1001): add post-motion vital reacquisition gate
