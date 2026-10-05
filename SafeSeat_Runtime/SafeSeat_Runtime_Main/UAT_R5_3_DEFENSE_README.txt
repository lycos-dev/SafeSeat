SafeSeat Main Hub R5.3 - Defense UAT
====================================

PACKAGE SCOPE
-------------
This package contains the COMPLETE SafeSeat_Runtime_Main folder only.
Only Main Hub files were changed for this revision.

Flash:
  SafeSeat_Runtime_Main/SafeSeat_Runtime_Main.ino

C1001 / camera firmware:
  No new firmware change is required by R5.3.
  Keep the matching R5 C1001 and current camera firmware already in use.

WHAT R5.3 CHANGES
-----------------
1. Controlled UAT start preserves the already-built model history.
   - Start Controlled Test does NOT rebuild the MLX 30-second baseline.
   - Start Controlled Test does NOT clear the FSR rolling ML window.
   - Apply / Update values does NOT reset those windows.
   - Stopping UAT clears UAT-contaminated local model history before live use.

2. MLX controlled test behavior.
   - Object-Ta remains visible as diagnostic context only.
   - Low Object-Ta does NOT hold or invalidate the baseline-relative MLX model.
   - While UAT-PXX is armed and the sudden-change bypass is checked, a sudden
     PHYSICAL or INJECTED MLX change is allowed through the FOV transition
     guard and into the real baseline-relative MLX model.
   - The real 1-second stability gate, IF + OCSVM, 3-block anomaly persistence,
     and normal Fusion persistence still apply. Warning is not directly forced.

3. FSR controlled physical defense test.
   - The existing trained FSR IF/OCSVM remains active and visible separately.
   - UAT-PXX also has a transparent controlled hard-slump detector based on the
     installed seat behavior observed in the supplied UAT recording:
       * loaded backrest side remains strongly loaded,
       * at least two sensors on the opposite backrest side unload near zero,
       * left/right pressure redistribution is large,
       * pattern must remain for about 2 seconds.
   - That sustained controlled pattern contributes ONE strong FSR sensor vote.
   - Fusion then still requires its normal 4-second warning persistence.
   - Expected hard-slump Warning timing is therefore about 6 seconds; hold the
     posture for 8-10 seconds during defense for margin.
   - UAT-only hard-slump evidence is not suppressed by the movement created by
     performing the intentional slump itself. Normal/live motion gating remains
     unchanged outside controlled UAT.

4. FSR injection presets use realistic occupied-seat values.
   - Loaded contacts: about 20k-26k.
   - Unloaded side: near 0.
   - Presets: hard LEFT, hard RIGHT, and alternating hard lean.

5. Camera baseline is visible again in /uat.
   - Live Monitor and Simulation Monitor show the real camera calibration count
     from the camera status packet (for example 0 / 5 ... 5 / 5).
   - The recorder CSV also includes camera_calibrating,
     camera_baseline_count, camera_baseline_target, and camera_baseline_ready.

6. Record Session remains independent of UAT control.
   - Recording never starts/stops a test and never changes a baseline.
   - Start recording before the normal baseline if you want one CSV containing
     before -> controlled stimulus -> Warning -> recovery.

DEFENSE FLOW - PHYSICAL FSR, NO INJECTION
-----------------------------------------
1. Sit normally.
2. Wait for normal monitoring / FSR window readiness.
3. In /uat, leave "Inject FSR pressure values" UNCHECKED.
4. Press "Arm Controlled Test - KEEP baseline/windows".
5. Lean VERY hard to one side so multiple opposite backrest FSRs collapse
   toward 0 while the loaded side stays high.
6. Hold 8-10 seconds.
7. Watch:
     Controlled hard-slump pattern -> LEFT/RIGHT + timer
     Controlled UAT vote          -> STRONG
     Fusion                       -> WARNING
8. Return to normal sitting and allow recovery.

DEFENSE FLOW - PHYSICAL MLX, NO INJECTION
-----------------------------------------
1. Sit normally until MLX baseline is 30 / 30.
2. Leave "Inject MLX values" UNCHECKED.
3. Keep "Controlled-test MLX sudden-change bypass" CHECKED.
4. Arm Controlled Test.
5. Place the warm/hot test object in the MLX field of view so the object
   reading rises substantially (for example ~31 C -> ~40 C).
6. Hold it steady for about 8-10 seconds.
7. The real MLX model and Fusion decide the Warning; the baseline is preserved.
8. Remove the object and allow the reading to return near baseline.

DEFENSE FLOW - INJECTION BACKUP
-------------------------------
MLX:
  Use "Inject MLX sustained 40 C" -> Apply / Update.

FSR:
  Use hard LEFT / hard RIGHT / alternating hard lean -> Apply / Update.

Injection replaces the selected sensor value at the accepted reading boundary.
It does not directly write the final Fusion level.

CAMERA
------
Camera remains event-triggered / verification-only. The baseline counter shown
in /uat is the actual remote camera calibration count, not a simulated counter.

VALIDATION NOTE
---------------
This revision received host-side C++ syntax checks for Fusion.cpp,
SafeSeatUatInjection.cpp, MLXML.cpp and MLXContext.cpp, plus Node.js syntax
validation of the embedded /uat JavaScript. The supplied UAT CSV was also used
to verify that the new controlled FSR pattern condition matches the recorded
hard-side pressure behavior.

A full ESP32 Arduino board compile/flash cannot be performed in this environment.
Compile the sketch once in your normal Arduino/ESP32 environment before flashing.
