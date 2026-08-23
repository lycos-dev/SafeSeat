SAFESEAT COMBINED RUNTIME — FSR GAIN + MPU 80 HZ FIX — 2026-08-23
=================================================================

WHY FSR ADS CHANNELS STOPPED AROUND 16-17k
-------------------------------------------
The Main runtime configured both ADS1115 boards with GAIN_TWOTHIRDS, whose
full-scale input range is +/-6.144 V. SafeSeat's divider is powered from 3.3 V.
Therefore the largest physically possible positive ADC code was approximately:

    32767 * 3.3 / 6.144 = 17600 counts

That exactly explains the ~16-17k ceiling observed during INT-HW-03. This was
not the FSR losing sensitivity; it was an ADC gain/range mismatch.

FIX:
    ADS1115 gain -> GAIN_ONE (+/-4.096 V)

At 3.3 V this gives approximately:

    32767 * 3.3 / 4.096 = 26400 counts

which matches the previously validated ~25-26k pressure range. Runtime output
still clamps to the existing common 0..26000 pressure scale, so the FSR ML
representation (per-sensor pressure shares) does not change and no FSR model
retraining is required.

Cushion physical/logical mapping remains:
    LEFT   = GPIO34 / electrical Cushion FSR3
    CENTER = ADS2 A3 / electrical Cushion FSR2
    RIGHT  = ADS2 A2 / electrical Cushion FSR1

WHY MPU SETTLED AROUND 40-42 HZ
-------------------------------
The MPU model requires 80 physical samples per second. The combined runtime
was being starved by two major blockers:

1. The very large full Serial dashboard was emitted every 1 s at 115200 baud.
   Several kilobytes of output can block for hundreds of milliseconds.
2. ADS1115 single-ended reads were using the library's slow/default conversion
   rate. Eight sequential FSR conversions periodically occupied the loop/I2C
   path for too long.

FIXES:
    MPU schedule interval      = 12,500 us (exact 80 Hz contract)
    ADS1115 conversion rate    = 860 SPS
    FSR completed-frame period = unchanged at 220 ms (~4.5 Hz target)
    Main Serial baud           = 921600
    Full dashboard interval    = 5 seconds
    Extra cooperative MPU service checkpoints added between heavier modules

IMPORTANT:
Set Arduino Serial Monitor to 921600 baud after uploading this package.

ACCEPTANCE RETEST
-----------------
1. INT-HW-03 FSR quick scale check:
   - Press one ADS-backed backrest FSR firmly.
   - Expected strong response can now approach ~25-26k instead of ~17k.
   - GPIO34 cushion-left already used its own 12-bit normalization and remains
     on the same common 0..26000 output scale.

2. MPU timing check before INT-HW-04:
   - Keep the module still after startup baseline.
   - Observe 'Actual Fs' for at least 20-30 s.
   - Target: approximately 80 Hz under the FULL combined runtime.
   - Do not accept INT-HW-04 if it remains near the old ~40-42 Hz.

NO MODEL RETRAINING
-------------------
FSR IF/OCSVM: unchanged.
MPU IF/OCSVM: unchanged.
MLX model/geometry guard: unchanged.
C1001/camera code: unchanged.
