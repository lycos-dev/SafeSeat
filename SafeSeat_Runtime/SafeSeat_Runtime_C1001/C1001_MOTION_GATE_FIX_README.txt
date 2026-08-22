SAFESEAT C1001 MOTION-GATE FIX — 2026-08-22
================================================

Purpose
-------
Fix the live C1001 runtime behavior observed during ML Runtime Validation:
isolated MoveRange spikes (including 30, 37, 64, 72, 79, 100) repeatedly
caused the 30-second HR/RR ML window to reset to 0/30 even while the
participant was effectively stationary.

Important architecture point
----------------------------
MoveRange is NOT an input feature to the trained BIDMC C1001 IF/OCSVM model.
The model still receives only the BIDMC-aligned HR/RR window/features.

Changes
-------
1. MODERATE MoveRange (15..29)
   - no longer blocks HR/RR collection solely because of MoveRange.

2. ISOLATED STRONG MoveRange (>=30)
   - the current HR/RR sample is held/skipped;
   - the existing ML window is preserved;
   - no full motion-recovery state is entered from a single spike.

3. CONFIRMED SUSTAINED STRONG MOTION
   - requires 2 consecutive MoveRange >=30 samples before the sensor enters
     STRONG_MOTION / MOTION_RECOVERY.

4. ML motion behavior
   - motion-related samples now show:
       HOLD - MOTION SAMPLE (WINDOW PRESERVED)
   - they DO NOT reset windowCount/writeIndex/stride progress.
   - ML anomaly evidence is temporarily invalid for the held sample so stale
     model evidence is not sent as current.

5. Diagnostics
   - Serial now prints:
       ML motion holds: <count>

What remains unchanged
----------------------
- 60-second warm-up starts only after a valid RR+HR pair.
- 30 samples @ 1 Hz first ML window.
- 15-sample stride.
- BIDMC-aligned feature extractor.
- Isolation Forest + One-Class SVM model data.
- C1001 ESP-NOW wire packet layout/version.
- Current Main Hub protocol compatibility.

Compatibility
-------------
The C1001 wire protocol in the supplied current Main Hub was checked against
the remote C1001 node protocol and is byte-for-byte identical. Packet size
remains 62 bytes and protocol version remains 1.

What to upload NOW
------------------
For individual C1001 ML validation, upload:

    SafeSeat_Runtime_C1001/SafeSeat_Runtime_C1001.ino

You do NOT need to upload the Main Hub for this standalone validation.

Expected behavior after warm-up
-------------------------------
Example noisy radar sequence:

    MoveRange: 4, 7, 64, 6, 5

Old behavior:
    64 -> RESET - MOTION ARTIFACT -> ML window 0/30

New behavior:
    64 -> HOLD - MOTION SAMPLE (WINDOW PRESERVED)
    next clean sample resumes from the previous ML window count.

Example true sustained motion:

    MoveRange: 38, 44, 51

New behavior:
    first strong sample: held only
    second consecutive strong sample: confirmed STRONG_MOTION
    ML window remains preserved while motion samples are held.

Validation target
-----------------
Remain seated normally and let the node pass warm-up. The important proof is:

    ML window: 1/30, 2/30, ... 30/30

even if occasional isolated MoveRange spikes occur.

Then capture:
    IF decision
    SVM decision
    READY - NORMAL / WEAK ANOMALY / STRONG ANOMALY

Main Hub
--------
The folder SafeSeat_Runtime_Main included in this package is the user's
supplied current Main Hub reference and was not behaviorally modified by this
patch.
