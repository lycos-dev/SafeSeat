SAFESEAT MAIN HUB — FSR UPDATE OVERLAY
Date: 2026-08-21

WHAT THIS ZIP IS
================
This is a DROP-IN OVERLAY for your existing latest SafeSeat_Runtime_Main project.

It intentionally contains only the FSR files that must change:
    SafeSeat_Runtime_Main/FSR.h
    SafeSeat_Runtime_Main/FSR.cpp

Do NOT delete your existing Main Hub folder.

Extract/copy the included SafeSeat_Runtime_Main folder OVER your current:
    SafeSeat_Runtime_Main/

Allow Windows to replace:
    FSR.h
    FSR.cpp

KEEP every other current file exactly as it is, including your already-working:
    SafeSeat_Runtime_Main.ino
    Config.h
    Fusion.*
    FSRML.* / existing embedded FSR model
    MLX*
    MLXContext / MLX diagnostic model
    MPU*
    MPU ML runtime
    C1001 remote/ESP-NOW files
    Camera link
    API / SoftAP / network files
    and any other files in your current latest Main Hub.

WHY THIS IS AN OVERLAY INSTEAD OF A REBUILT OLD ZIP
====================================================
The previous SafeSeat_Runtime_Main ZIP is referenced in the project history,
but its original ZIP bytes are not mounted in this execution environment.
Rebuilding unrelated modules from memory would risk regressing your already-
working Main Hub. This overlay changes only the FSR acquisition/runtime layer.

API COMPATIBILITY
=================
The earlier Main Hub API is preserved:

    FSRSensor fsr;
    fsr.begin();
    fsr.update(occupantPresent);
    fsr.getReading();
    fsr.hasValidReading();
    fsr.getStatusText();
    fsr.getSensorLabel(index);

Existing FSRReading fields used by Fusion/dashboard are retained:

    connected
    valid
    status
    actualSamplingRateHz
    pressure[9]
    backrestTotal
    cushionTotal
    wholeSeatTotal
    backrestLRBalance
    cushionLRBalance
    backrestToCushionRatio

NEW FSR FIXES
=============
1) VALIDATED ELECTRICAL MAP IS RETAINED

ADS1115 #1 0x48:
    A0 = electrical Backrest FSR1
    A1 = electrical Backrest FSR2
    A2 = electrical Backrest FSR3
    A3 = electrical Backrest FSR4

ADS1115 #2 0x49:
    A0 = electrical Backrest FSR5
    A1 = electrical Backrest FSR6
    A2 = Cushion FSR1
    A3 = Cushion FSR2

GPIO34:
    Cushion FSR3

2) PHYSICAL BACKREST LEFT/RIGHT MIRROR IS CORRECTED

The 2026-08-21 lean validation showed:
    physically lean LEFT  -> electrical/currently-labeled FSR4-FSR6 dominate
    physically lean RIGHT -> electrical/currently-labeled FSR1-FSR3 dominate

The code therefore keeps raw electrical identity for diagnostics, but exposes
reading.pressure[] in SafeSeat's LOGICAL / MODEL physical order:

    Logical Backrest FSR1 = physical LEFT top    <- electrical FSR4
    Logical Backrest FSR2 = physical LEFT middle <- electrical FSR5
    Logical Backrest FSR3 = physical LEFT bottom <- electrical FSR6

    Logical Backrest FSR4 = physical RIGHT top    <- electrical FSR1
    Logical Backrest FSR5 = physical RIGHT middle <- electrical FSR2
    Logical Backrest FSR6 = physical RIGHT bottom <- electrical FSR3

This is important because the ChairPose/TDSD model topology was trained as:
    left top/middle/bottom, right top/middle/bottom, cushion L/C/R.

No physical rewiring is required.

3) ADS1115 vs GPIO34 SCALE IS FIXED

ADS1115 and GPIO34 raw values are NOT directly summed anymore.

Each channel is:
    raw ADC
    -> empty-seat baseline subtraction
    -> deadband
    -> channel ADC-range normalization
    -> EMA filtering
    -> common 0..26000 pressure-unit scale

This prevents GPIO34's 0..4095 ADC range from being underweighted against
ADS1115 values around ~25,000.

The original raw ADC values are still available in:
    reading.electricalRaw[]
    reading.electricalBaseline[]

4) FSR ML REPRESENTATION IS PRESERVED

The deployed FSR model contract remains:
    23 completed live FSR frames @ about 4.5 Hz
    stride = 5 completed frames
    9-sensor pressure-share representation
    Isolation Forest + One-Class SVM

This FSR acquisition revision does NOT retrain or replace the trained models.

New:
    reading.modelShare[9]

The shares use the corrected physical/model order and the corrected ADC scale.
If your current FSRML.cpp already computes shares from reading.pressure[],
it will also receive the corrected logical/model order because pressure[] is
now exposed in that order.

5) SUDDEN-ZERO MAINTENANCE / AUTO-RECOVERY

The module now:
    - probes ADS1115 0x48 and 0x49 periodically;
    - distinguishes disconnected/degraded acquisition from valid zero pressure;
    - retries ADS initialization if communication disappears;
    - detects a sustained unexpected all-zero ADS collapse when an occupant or
      previous meaningful load is expected;
    - reinitializes the ADS devices WITHOUT erasing the accepted baseline;
    - exposes recoveryCount and maintenanceActive diagnostics.

6) BAD STARTUP BASELINE IS REJECTED

If an empty-seat calibration is near ADC saturation (for example the earlier
failure mode where many ADS values were ~25k and GPIO34 was 4095), the code
does NOT silently learn that as the normal zero.

It reports:
    CALIBRATION FAILED

Keep the seat empty, fix the wiring/power/pressure condition, then restart
or call:
    fsr.forceRecalibration();

7) BASELINE DRIFT DOES NOT LEARN A SEATED OCCUPANT

Very slow baseline drift correction is allowed only when:
    occupantPresent == false
and the channel is effectively unloaded.

This prevents the runtime from slowly subtracting away a real seated load.

EXPECTED BOOT
=============
You should still see the familiar flow:

    [MAIN] Starting FSR array...
    [FSR] Initializing validated 9-channel array...
    ADS1115 #1 Ready ...
    ADS1115 #2 Ready ...
    GPIO34 Ready ...

    FSR EMPTY-SEAT CALIBRATION
    Keep the backrest and cushion EMPTY.
    ...

    [FSR] Runtime module ready.

Then your EXISTING FSR ML should continue with its own initialization:
    [FSR-ML] Embedded Step 5.5 model initialized.
    [FSR-ML] Runtime alignment: 23 completed FSR frames @ ~4.5 Hz.
    [FSR-ML] Stride: 5 completed frames (~1.1 s).

IMPORTANT FIRST TEST
====================
1. Boot with ALL FSRs completely unloaded.
2. Confirm both ADS1115 devices initialize.
3. Confirm calibration does NOT report CALIBRATION FAILED.
4. Press each sensor individually.
5. Confirm logical physical location is correct:
       left-backrest press/lean -> BackLeft group
       right-backrest press/lean -> BackRight group
6. Check Cushion Left / Center / Right.
7. Confirm Actual Fs settles around ~4 to 5 Hz.
8. Only after that judge FSR-ML normal/anomaly output.

DO NOT change the trained model parameters merely because the hardware
acquisition code was corrected. First retest live preprocessing/model behavior.

DEPENDENCY
==========
Arduino library required:
    Adafruit ADS1X15

The Main Hub already used this library in the earlier validated runtime.
