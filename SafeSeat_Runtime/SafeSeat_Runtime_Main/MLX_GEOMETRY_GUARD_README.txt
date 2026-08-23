SAFESEAT MLX90614 TARGET-GEOMETRY / FOV GUARD — 2026-08-23
================================================================

WHY THIS PATCH EXISTS
---------------------
Combined testing established a personal nape baseline around 30.41 C. A small
neck bend then moved the MLX field of view away from the original nape region.
The filtered object measurement dropped toward ~28 C and the unchanged IF +
OCSVM correctly classified the large baseline deviation as anomalous. Because
that change occurred on a movement/FOV timescale rather than a physiological
temperature-change timescale, it must be treated as target geometry loss first.

PATCH
-----
1. Model, scaler and two baseline-relative features are unchanged.
2. Occupancy remains the authoritative session gate.
3. After the 30-second personal baseline is ready, the runtime watches the
   filtered object signal for rapid departures:
      >= 0.90 C in 1 second, or
      >= 1.40 C across 2 seconds,
   while already >= 1.25 C away from the baseline.
4. Such a rapid departure enters TARGET GEOMETRY DEGRADED. The MLX baseline is
   preserved and the IF/OCSVM vote is held.
5. Reacquisition requires 3 consecutive stable 1-second blocks within +/-1.00 C
   of the original personal baseline. No 30-second rebaseline is required.
6. A gradual temperature change that does NOT look like a rapid FOV step may
   still reach the model. Even then, 3 consecutive trusted anomalous blocks are
   required before MLX is allowed to vote anomaly into Fusion.
7. One anomalous block followed by NORMAL is discarded as a transient.
8. MLXContext suppresses its broad context-change vote while geometry is
   degraded/reacquiring OR while an anomaly is still only a persistence
   candidate, preventing the same MLX excursion from appearing twice.

WHY THERE IS NO AUTOMATIC TIMEOUT
---------------------------------
If the nape remains outside the original field of view, the runtime cannot
reliably distinguish a different surface from true nape temperature. It is safer
to mark MLX target quality degraded and rely on the other SafeSeat sensors than
to manufacture a temperature emergency. A slow genuine thermal trend can still
be evaluated because it does not trigger the rapid-step geometry rule.

RETEST
------
A. Sit normally and let baseline reach 30/30.
B. Confirm repeated READY - NORMAL.
C. Bend/tilt the neck enough to reproduce the previous drop. Expected:
      TARGET GEOMETRY DEGRADED - ML HELD
      Fusion Temperature: TARGET DEGRADED
      no STRONG/WEAK MLX Fusion vote
D. Return nape to the original target. Expected 1/3 -> 2/3 -> 3/3 reacquisition,
   then READY - NORMAL without rebuilding the 30-second baseline.
E. A single trusted model anomaly should show ANOMALY CANDIDATE 1/3 and remain
   held from Fusion.
