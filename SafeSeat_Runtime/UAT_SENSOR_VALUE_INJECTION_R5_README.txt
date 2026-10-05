SAFESEAT RUNTIME R5.1 — UAT RECORDER + MLX DEMO POLICY
=======================================================
Step: 5.9.9-R5.1
Date: 2026-10-05
Based on: SafeSeat_Runtime_R5_UAT_Value_Injection.zip

PURPOSE
-------
R5.1 keeps the R5 researcher-only value-injection test bench and adds two
focused fixes for defense/UAT work:

1. MLX Object-Ta below 2 C is now diagnostic information only. It is still
   visible and recorded, but it does NOT by itself make Fusion report
   TARGET DEGRADED / HELD.
2. While synthetic MLX injection is active in /uat, the rapid 1-2 second
   FOV/geometry transition hold is bypassed so an intentional controlled
   temperature step (for example 31 -> 40 C) can reach the real
   baseline-relative MLX model.

The live physical MLX path is NOT relaxed. Its rapid-transition/FOV geometry
guard remains enabled outside active synthetic MLX injection.

R5.1 also adds a browser-side Record Session feature to /uat so a complete
CSV can be sent back for post-test analysis.

IMPORTANT MODEL/FUSION BEHAVIOR
-------------------------------
- Fusion remains authoritative. No web endpoint writes WARNING or EMERGENCY.
- The MLX still uses its 30-second session baseline.
- The 1-second MLX stability check remains active.
- The real MLX Isolation Forest + One-Class SVM remain active.
- The 3-consecutive-block anomaly persistence remains active.
- Object-Ta is retained in telemetry/CSV for diagnostic analysis only.
- A single strong MLX anomaly is still only one sensor vote. A full persistent
  Fusion Warning/camera verification still requires the existing independent
  multi-sensor evidence rules.

WHY OBJECT-Ta < 2 C NO LONGER DEGRADES FUSION
----------------------------------------------
The deployed native MLX model does not use Object-Ta as an ML feature. Its
features are the current filtered object temperature relative to the learned
personal/session baseline. In a warm vehicle, ambient/Ta can approach the
surface reading even when the sensor and model are otherwise usable. Treating
that low contrast as TARGET DEGRADED / HELD made the maintenance UI look as if
MLX inference was disabled when it was not.

R5.1 therefore keeps Object-Ta as useful context but removes its authority to
change the Fusion temperature state. True target loss, contribution suspension,
and true geometry/reacquisition degradation can still report a degraded/held
state.

UAT-ONLY RAPID TEMPERATURE STEP
-------------------------------
When BOTH conditions are true:
- a /uat test session is active, and
- MLX injection is enabled,

the native MLX rapid-transition/FOV hold is bypassed. This is specifically for
deterministic synthetic defense demonstrations such as changing an injected
object value from a stable baseline to 40 C.

This bypass does NOT apply to live physical MLX sensing. It also does not skip
the baseline, model, stability, persistence, target-visible range, or Fusion
logic. An update that lands in the middle of a 1-second 4-sample block may cause
that mixed block to be held by the normal stability check; the following full
stable block will proceed normally.

The convenience button "MLX jump -> 40 C" changes only the MLX object field to
40 C. It does not reset the already learned baseline. Press Update Values to
apply it.

RECORD SESSION
--------------
Open http://192.168.4.1/uat and use the Live Monitor tab.

1. Enter a label such as UAT-P10 or DEMO-01.
2. Tap Record Session.
3. Run the controlled sequence from Inject Values / Simulation Monitor.
4. When finished, tap Stop & Download CSV.
5. Send that CSV for analysis.

While recording, the page requests a consolidated status snapshot about once
per second. The CSV includes:
- Fusion state, confidence, occupancy and evidence counters
- C1001 HR/RR, link/model status, IF/OCSVM outputs and model window
- all 9 FSR values, totals, IF/OCSVM outputs and model window
- MLX object/Ta/Object-Ta, baseline/deviation, target and geometry state,
  IF/OCSVM outputs, persistence and baseline counters
- MPU motion values/model context
- camera request/result state
- UAT session id/revision, configured injected values and whether the MLX
  rapid-transition bypass was active

The recorder runs in the browser and is intentionally simple. Keep /uat open
until the CSV is downloaded; reloading/closing the page clears an unfinished
recording.

RECOMMENDED THERMAL DEMO FLOW
-----------------------------
1. Open /uat and start Record Session.
2. On Inject Values, choose Normal seated (or your chosen normal MLX value),
   then Start Test.
3. Wait until the MLX baseline shows 30 / 30 and the native model is ready.
4. Tap "MLX jump -> 40 C" and then Update Values.
5. Watch Simulation Monitor for the real MLX IF/OCSVM/persistence result.
6. If demonstrating the complete Fusion Warning/camera path, also provide a
   second independent strong sensor anomaly using the existing controlled UAT
   inputs; MLX alone intentionally does not force Warning.
7. Restore normal values if desired and observe recovery.
8. Stop Test to restore live sensors, then Stop & Download CSV.

THESIS / DEFENSE WORDING
------------------------
Describe the rapid thermal step as a controlled synthetic UAT stimulus used to
exercise the deployed model/Fusion pipeline. Do not describe 31 -> 40 C as a
physiologically realistic human temperature transition. The live deployment
retains the geometry/FOV protection that the UAT-only synthetic path bypasses.

FIRMWARE THAT MUST BE UPDATED
-----------------------------
For these R5.1 changes, flash SafeSeat_Runtime_Main.

R5 already changed SafeSeat_Runtime_C1001 for remote value injection. If the
C1001 node is not already on the matching R5 firmware from this package, flash
it as well. The ESP32-S3 camera firmware is unchanged by R5.1.

VALIDATION PERFORMED IN THIS PACKAGE
------------------------------------
- Embedded /uat JavaScript syntax checked with Node.js.
- Source-level checks confirm low Object-Ta is informational only in Fusion.
- Source-level checks confirm the rapid-transition bypass is active only for an
  active synthetic MLX UAT session.
- Recorder controls/functions and 40 C preset are present.
- Main/C1001 shared ESP-NOW protocol header remains byte-identical.
- Package manifest/SHA-256 list regenerated after the changes.

LIMITATION
----------
This environment does not contain the complete Arduino/ESP32 board toolchain
used by the physical SafeSeat hardware, so the package has not been flashed or
bench-tested here. Compile/upload the Main Hub and run one short controlled
bench session before relying on it during the defense.
