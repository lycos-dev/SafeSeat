SAFESEAT MLX90614 TARGET HYSTERESIS UPDATE — 2026-08-23
========================================================

WHY THIS UPDATE
---------------
Combined-hardware testing in a warm room showed Object-Ta repeatedly crossing
the old fixed +2.0 C threshold. That caused repeated baseline resets even
though the MLX continued to see a plausible warm human target.

THIS UPDATE CHANGES ONLY TARGET QUALITY/RUNTIME GATING.
The native MLX IF/OCSVM model, feature order, preprocessing, and trained
parameters are unchanged.

NEW TARGET LOGIC
----------------
Acquire a new target:
  Object-Ta >= +2.0 C

Retain an already-acquired target:
  Object-Ta >= +1.0 C

Persistent target loss:
  below +1.0 C for 3 consecutive 1-second ML blocks
  / 12 consecutive 4-Hz context samples (~3 seconds)

During the hysteresis band (+1.0 to +2.0 C) or a short low-contrast grace
period, the target remains latched but the baseline/model are HELD. No new
baseline samples are learned from lower-confidence data.

A true persistent target loss still resets the personal/session baseline.

STABILITY GATE
--------------
The existing 1-second object-temperature stability requirement (std <= 0.50 C)
is intentionally unchanged. If the target is physically unstable/FOV-mixed,
the ML block will still be held. This update fixes threshold flapping; it does
not hide a real targeting/alignment problem.

EXPECTED SERIAL STATES
----------------------
TARGET CONTRAST DEGRADED - BASELINE HELD
  = short/partial thermal contrast loss; baseline preserved

WAITING FOR WARM TARGET
  = no target acquired yet, or persistent target loss confirmed

UNSTABLE TARGET - BLOCK HELD
  = target contrast exists, but 1-second raw object readings are too unstable

TEST NEXT
---------
Repeat INT-HW-02 with all Main Hub sensors connected. Let the room/aircon be
normal rather than artificially cold. Sit normally, keep the MLX aimed at the
intended exposed-skin target, and watch for whether baseline progress is now
preserved across brief Object-Ta dips.

FUSION STATE
------------
During a retained-but-degraded target, Fusion now reports:
  Temperature = TARGET DEGRADED / HELD

No MLX anomaly/normal vote is used while the target is degraded. This prevents
a stale model decision from being counted while thermal confidence is reduced.
