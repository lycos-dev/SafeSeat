#include "FSR.h"

#include <math.h>

// ============================================================
// SAFESEAT FSR ARRAY - IMPLEMENTATION
// ============================================================

namespace
{
    static float clamp01(float value)
    {
        if (value < 0.0f) return 0.0f;
        if (value > 1.0f) return 1.0f;
        return value;
    }

    static float safeBalance(float left, float right)
    {
        const float total = left + right;
        if (total < 1.0f)
        {
            return 0.0f;
        }
        return (left - right) / total;
    }
}


FSRSensor::FSRSensor()
{
}


bool FSRSensor::i2cProbe(uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}


bool FSRSensor::initADS1()
{
    bool ok = ads1.begin(ADS1_ADDRESS, &Wire);

    if (ok)
    {
        ads1.setGain(GAIN_TWOTHIRDS);
    }

    reading.ads1Connected = ok;
    return ok;
}


bool FSRSensor::initADS2()
{
    bool ok = ads2.begin(ADS2_ADDRESS, &Wire);

    if (ok)
    {
        ads2.setGain(GAIN_TWOTHIRDS);
    }

    reading.ads2Connected = ok;
    return ok;
}


bool FSRSensor::begin()
{
    setStatus(FSRStatus::UNINITIALIZED);

    Serial.println();
    Serial.println("[FSR] Initializing validated 9-channel array...");

    pinMode(CUSHION_RIGHT_GPIO, INPUT);

    const bool ads1OK = initADS1();
    const bool ads2OK = initADS2();

    Serial.println(
        ads1OK
            ? "ADS1115 #1 Ready - electrical FSR1..FSR4"
            : "ADS1115 #1 FAILED at 0x48"
    );

    Serial.println(
        ads2OK
            ? "ADS1115 #2 Ready - electrical FSR5/6 + Cushion FSR1/2"
            : "ADS1115 #2 FAILED at 0x49"
    );

    Serial.println("GPIO34 Ready - Cushion FSR3");

    initialized = ads1OK && ads2OK;
    reading.connected = initialized;

    // Keep Main Hub alive even when calibration fails because the
    // hardware can still be diagnosed and recovered without changing
    // unrelated MLX/MPU/C1001 runtime code.
    if (!initialized)
    {
        setStatus(FSRStatus::DEGRADED);
        reading.valid = false;

        Serial.println(
            "[FSR] WARNING: one or both ADS1115 devices are unavailable."
        );
        Serial.println(
            "[FSR] Runtime will continue health checks/recovery attempts."
        );

        // Return true so the earlier Main Hub keeps calling update().
        // Fusion still sees connected=false / valid=false.
        return true;
    }

    setStatus(FSRStatus::CALIBRATING);

    baselineReady = calibrateEmptySeat();
    reading.baselineValid = baselineReady;

    if (!baselineReady)
    {
        setStatus(FSRStatus::CALIBRATION_FAILED);
        reading.valid = false;

        Serial.println(
            "[FSR] Calibration rejected. Keep the seat EMPTY and restart,"
        );
        Serial.println(
            "[FSR] or call fsr.forceRecalibration() from a maintenance path."
        );

        return true;
    }

    setStatus(FSRStatus::READING);
    reading.valid = false;  // becomes true after first completed frame

    Serial.println();
    Serial.println("[FSR] Runtime module ready.");
    Serial.println(
        "[FSR] Logical/model mapping corrected for validated mirrored backrest."
    );
    Serial.println(
        "[FSR] Electrical FSR4-6 -> physical LEFT; FSR1-3 -> physical RIGHT."
    );
    Serial.println(
        "[FSR] ADS1115 and GPIO34 pressure are normalized before aggregation."
    );

    return true;
}


bool FSRSensor::acquireElectricalRaw(float out[FSR_COUNT])
{
    bool ads1OK = reading.ads1Connected;
    bool ads2OK = reading.ads2Connected;

    if (ads1OK)
    {
        out[0] = static_cast<float>(ads1.readADC_SingleEnded(0));
        out[1] = static_cast<float>(ads1.readADC_SingleEnded(1));
        out[2] = static_cast<float>(ads1.readADC_SingleEnded(2));
        out[3] = static_cast<float>(ads1.readADC_SingleEnded(3));
    }
    else
    {
        out[0] = out[1] = out[2] = out[3] = 0.0f;
    }

    if (ads2OK)
    {
        out[4] = static_cast<float>(ads2.readADC_SingleEnded(0));
        out[5] = static_cast<float>(ads2.readADC_SingleEnded(1));
        out[6] = static_cast<float>(ads2.readADC_SingleEnded(2));
        out[7] = static_cast<float>(ads2.readADC_SingleEnded(3));
    }
    else
    {
        out[4] = out[5] = out[6] = out[7] = 0.0f;
    }

    out[8] = static_cast<float>(analogRead(CUSHION_RIGHT_GPIO));

    for (int i = 0; i < FSR_COUNT; ++i)
    {
        if (!isfinite(out[i]))
        {
            return false;
        }
    }

    return ads1OK && ads2OK;
}


void FSRSensor::mapElectricalToLogical(
    const float electrical[FSR_COUNT],
    float logical[FSR_COUNT]
) const
{
    // --------------------------------------------------------
    // BACKREST MIRROR CORRECTION
    //
    // Validated physical installation:
    //   electrical FSR4 = physical LEFT top
    //   electrical FSR5 = physical LEFT middle
    //   electrical FSR6 = physical LEFT bottom
    //
    //   electrical FSR1 = physical RIGHT top
    //   electrical FSR2 = physical RIGHT middle
    //   electrical FSR3 = physical RIGHT bottom
    //
    // Model/design order:
    //   FSR1-3 = LEFT top/middle/bottom
    //   FSR4-6 = RIGHT top/middle/bottom
    // --------------------------------------------------------

    logical[BACKREST_FSR1] = electrical[3];  // ADS1 A3, electrical FSR4
    logical[BACKREST_FSR2] = electrical[4];  // ADS2 A0, electrical FSR5
    logical[BACKREST_FSR3] = electrical[5];  // ADS2 A1, electrical FSR6

    logical[BACKREST_FSR4] = electrical[0];  // ADS1 A0, electrical FSR1
    logical[BACKREST_FSR5] = electrical[1];  // ADS1 A1, electrical FSR2
    logical[BACKREST_FSR6] = electrical[2];  // ADS1 A2, electrical FSR3

    // Cushion left/right mapping corrected from combined live validation
    // on 2026-08-23. Left/right are from the SEATED OCCUPANT perspective.
    //
    // Physical LEFT  = electrical cushion FSR3 = GPIO34
    // Physical CENTER = electrical cushion FSR2 = ADS2 A3
    // Physical RIGHT = electrical cushion FSR1 = ADS2 A2
    logical[CUSHION_FSR1] = electrical[8];   // GPIO34, physical left
    logical[CUSHION_FSR2] = electrical[7];   // ADS2 A3, physical center
    logical[CUSHION_FSR3] = electrical[6];   // ADS2 A2, physical right
}


bool FSRSensor::calibrateEmptySeat()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println(" FSR EMPTY-SEAT CALIBRATION");
    Serial.println("========================================");
    Serial.println("Keep the backrest and cushion EMPTY.");
    Serial.println("Calibration begins in 3 seconds...");

    delay(CALIBRATION_SETTLE_MS);

    float sums[FSR_COUNT] = {0};
    int accepted = 0;

    for (int sample = 0; sample < CALIBRATION_SAMPLES; ++sample)
    {
        float electrical[FSR_COUNT] = {0};

        if (acquireElectricalRaw(electrical))
        {
            for (int i = 0; i < FSR_COUNT; ++i)
            {
                sums[i] += electrical[i];
            }

            accepted++;
        }

        Serial.print(".");
        delay(20);
    }

    Serial.println();

    if (accepted < (CALIBRATION_SAMPLES * 3) / 4)
    {
        Serial.println(
            "[FSR] ERROR: not enough valid calibration frames were acquired."
        );
        return false;
    }

    for (int i = 0; i < FSR_COUNT; ++i)
    {
        reading.electricalBaseline[i] =
            sums[i] / static_cast<float>(accepted);

        electricalFilteredLoad[i] = 0.0f;
    }

    mapElectricalToLogical(
        reading.electricalBaseline,
        reading.baseline
    );

    Serial.println("FSR baseline calibration complete.");

    for (int i = 0; i < FSR_COUNT; ++i)
    {
        Serial.print("  ");
        Serial.print(getSensorLabel(i));
        Serial.print(" baseline: ");
        Serial.println(reading.baseline[i], 1);
    }

    if (!calibrationLooksValid())
    {
        Serial.println();
        Serial.println(
            "[FSR] ERROR: empty-seat baseline contains a saturated/stuck channel."
        );
        Serial.println(
            "[FSR] Do NOT accept this as a normal baseline."
        );
        Serial.println(
            "[FSR] Check seat pressure, FSR divider wiring, GND/3.3V, ADS power,"
        );
        Serial.println(
            "[FSR] and GPIO34. Then recalibrate with the seat fully unloaded."
        );
        return false;
    }

    return true;
}


bool FSRSensor::calibrationLooksValid() const
{
    for (int e = 0; e < FSR_COUNT; ++e)
    {
        const float maxExpected =
            (e == 8)
                ? GPIO_EXPECTED_PRESS_MAX
                : ADS_EXPECTED_PRESS_MAX;

        if (
            reading.electricalBaseline[e]
            >=
            maxExpected * SATURATED_BASELINE_FRACTION
        )
        {
            return false;
        }
    }

    return true;
}


float FSRSensor::normalizeElectricalChannel(
    uint8_t electricalIndex,
    float raw,
    float baseline
) const
{
    const bool gpioChannel = electricalIndex == 8;

    const float expectedMax =
        gpioChannel
            ? GPIO_EXPECTED_PRESS_MAX
            : ADS_EXPECTED_PRESS_MAX;

    const float deadband =
        gpioChannel
            ? GPIO_DEADBAND_RAW
            : ADS_DEADBAND_RAW;

    float delta = raw - baseline - deadband;

    if (delta <= 0.0f)
    {
        return 0.0f;
    }

    // Channel-specific denominator starts at its measured empty baseline.
    // This is the critical ADS1115-vs-GPIO34 scale correction.
    float availableRange = expectedMax - baseline - deadband;

    const float minimumRange =
        gpioChannel
            ? 500.0f
            : 1200.0f;

    if (availableRange < minimumRange)
    {
        availableRange = minimumRange;
    }

    return clamp01(delta / availableRange);
}


void FSRSensor::processFrame(
    const float electricalRaw[FSR_COUNT],
    bool occupantPresent
)
{
    for (int e = 0; e < FSR_COUNT; ++e)
    {
        reading.electricalRaw[e] = electricalRaw[e];

        float normalized =
            normalizeElectricalChannel(
                static_cast<uint8_t>(e),
                electricalRaw[e],
                reading.electricalBaseline[e]
            );

        const float commonLoad =
            normalized * COMMON_PRESSURE_SCALE;

        electricalFilteredLoad[e] =
            FILTER_ALPHA * commonLoad
            +
            (1.0f - FILTER_ALPHA) * electricalFilteredLoad[e];

        // Very slow baseline drift compensation ONLY while the C1001 says
        // the seat is not occupied and this channel is effectively unloaded.
        //
        // This avoids the dangerous behavior of learning an occupant's
        // seated pressure as the new zero.
        if (
            !occupantPresent
            &&
            electricalFilteredLoad[e] < 120.0f
        )
        {
            reading.electricalBaseline[e] =
                (1.0f - BASELINE_DRIFT_ALPHA)
                * reading.electricalBaseline[e]
                +
                BASELINE_DRIFT_ALPHA
                * electricalRaw[e];
        }
    }

    float logicalRaw[FSR_COUNT] = {0};
    float logicalBaseline[FSR_COUNT] = {0};
    float logicalPressure[FSR_COUNT] = {0};

    mapElectricalToLogical(
        reading.electricalRaw,
        logicalRaw
    );

    mapElectricalToLogical(
        reading.electricalBaseline,
        logicalBaseline
    );

    mapElectricalToLogical(
        electricalFilteredLoad,
        logicalPressure
    );

    for (int i = 0; i < FSR_COUNT; ++i)
    {
        reading.raw[i] = logicalRaw[i];
        reading.baseline[i] = logicalBaseline[i];
        reading.pressure[i] = logicalPressure[i];
        reading.normalized[i] =
            clamp01(logicalPressure[i] / COMMON_PRESSURE_SCALE);
    }

    updateDerivedQuantities();
}


void FSRSensor::updateDerivedQuantities()
{
    reading.backrestLeftTotal =
        reading.pressure[BACKREST_FSR1]
        + reading.pressure[BACKREST_FSR2]
        + reading.pressure[BACKREST_FSR3];

    reading.backrestRightTotal =
        reading.pressure[BACKREST_FSR4]
        + reading.pressure[BACKREST_FSR5]
        + reading.pressure[BACKREST_FSR6];

    reading.backrestTotal =
        reading.backrestLeftTotal
        + reading.backrestRightTotal;

    reading.cushionLeft =
        reading.pressure[CUSHION_FSR1];

    reading.cushionCenter =
        reading.pressure[CUSHION_FSR2];

    reading.cushionRight =
        reading.pressure[CUSHION_FSR3];

    reading.cushionTotal =
        reading.cushionLeft
        + reading.cushionCenter
        + reading.cushionRight;

    reading.wholeSeatTotal =
        reading.backrestTotal
        + reading.cushionTotal;

    reading.backrestLRBalance =
        safeBalance(
            reading.backrestLeftTotal,
            reading.backrestRightTotal
        );

    reading.cushionLRBalance =
        safeBalance(
            reading.cushionLeft,
            reading.cushionRight
        );

    reading.backrestToCushionRatio =
        reading.backrestTotal
        /
        ((reading.cushionTotal > 1.0f)
            ? reading.cushionTotal
            : 1.0f);

    reading.activeSensorCount = 0;

    for (int i = 0; i < FSR_COUNT; ++i)
    {
        if (
            reading.normalized[i]
            >=
            ACTIVE_NORMALIZED_THRESHOLD
        )
        {
            reading.activeSensorCount++;
        }
    }

    reading.occupiedByPressure =
        reading.wholeSeatTotal >= OCCUPANCY_TOTAL_THRESHOLD
        &&
        reading.activeSensorCount >= OCCUPANCY_ACTIVE_SENSOR_MIN;

    if (reading.wholeSeatTotal > 1.0f)
    {
        for (int i = 0; i < FSR_COUNT; ++i)
        {
            reading.modelShare[i] =
                reading.pressure[i]
                /
                reading.wholeSeatTotal;
        }
    }
    else
    {
        for (int i = 0; i < FSR_COUNT; ++i)
        {
            reading.modelShare[i] = 0.0f;
        }
    }
}


void FSRSensor::updateSamplingRate(unsigned long now)
{
    if (previousFrameMillis != 0UL)
    {
        const unsigned long dt =
            now - previousFrameMillis;

        if (dt > 0UL)
        {
            const float instantaneous =
                1000.0f / static_cast<float>(dt);

            if (reading.actualSamplingRateHz <= 0.0f)
            {
                reading.actualSamplingRateHz =
                    instantaneous;
            }
            else
            {
                reading.actualSamplingRateHz =
                    0.90f * reading.actualSamplingRateHz
                    +
                    0.10f * instantaneous;
            }
        }
    }

    previousFrameMillis = now;
}


bool FSRSensor::frameLooksSuddenlyZero(
    const float electricalRaw[FSR_COUNT],
    bool occupantPresent
) const
{
    bool allADSNearZero = true;

    for (int e = 0; e < 8; ++e)
    {
        if (fabsf(electricalRaw[e]) > 1.5f)
        {
            allADSNearZero = false;
            break;
        }
    }

    if (!allADSNearZero)
    {
        return false;
    }

    // A sudden full-ADS collapse is suspicious when either:
    // 1) C1001 still says an occupant is present, or
    // 2) the immediately previous pressure frame had meaningful load.
    return occupantPresent || previousWholeSeatTotal > 1500.0f;
}


void FSRSensor::checkAndRecoverADS()
{
    const unsigned long now = millis();

    if (
        now - lastHealthCheckMillis
        <
        HEALTH_CHECK_INTERVAL_MS
    )
    {
        return;
    }

    lastHealthCheckMillis = now;

    const bool probe1 = i2cProbe(ADS1_ADDRESS);
    const bool probe2 = i2cProbe(ADS2_ADDRESS);

    reading.ads1Connected = probe1;
    reading.ads2Connected = probe2;
    reading.connected = probe1 && probe2;

    if (probe1 && probe2)
    {
        if (
            reading.status == FSRStatus::DEGRADED
            ||
            reading.status == FSRStatus::RECOVERING
        )
        {
            setStatus(
                baselineReady
                    ? FSRStatus::READING
                    : FSRStatus::CALIBRATION_FAILED
            );
        }

        return;
    }

    if (
        now - lastRecoveryAttemptMillis
        <
        RECOVERY_COOLDOWN_MS
    )
    {
        return;
    }

    lastRecoveryAttemptMillis = now;
    reading.maintenanceActive = true;
    setStatus(FSRStatus::RECOVERING);

    Serial.println();
    Serial.println("[FSR-MAINT] ADS health fault detected.");

    if (!probe1)
    {
        Serial.println("[FSR-MAINT] Reinitializing ADS1115 #1 (0x48)...");
        reading.ads1Connected = initADS1();
    }

    if (!probe2)
    {
        Serial.println("[FSR-MAINT] Reinitializing ADS1115 #2 (0x49)...");
        reading.ads2Connected = initADS2();
    }

    reading.connected =
        reading.ads1Connected
        &&
        reading.ads2Connected;

    if (reading.connected)
    {
        reading.recoveryCount++;
        reading.maintenanceActive = false;

        Serial.println(
            "[FSR-MAINT] ADS communication recovered; preserving existing baseline."
        );

        setStatus(
            baselineReady
                ? FSRStatus::READING
                : FSRStatus::CALIBRATION_FAILED
        );
    }
    else
    {
        Serial.println(
            "[FSR-MAINT] Recovery incomplete. Check ADS power, SDA/SCL, GND and jumpers."
        );

        setStatus(FSRStatus::DEGRADED);
    }
}


void FSRSensor::update(bool occupantPresent)
{
    if (!initialized)
    {
        // begin() may have returned true intentionally so Main Hub continues
        // calling us while one ADS device is temporarily unavailable.
        checkAndRecoverADS();

        initialized =
            reading.ads1Connected
            &&
            reading.ads2Connected;

        if (!initialized)
        {
            reading.valid = false;
            return;
        }

        if (!baselineReady)
        {
            setStatus(FSRStatus::CALIBRATION_FAILED);
            reading.valid = false;
            return;
        }
    }

    checkAndRecoverADS();

    if (
        !reading.ads1Connected
        ||
        !reading.ads2Connected
    )
    {
        reading.valid = false;
        setStatus(FSRStatus::DEGRADED);
        return;
    }

    if (!baselineReady)
    {
        reading.valid = false;
        setStatus(FSRStatus::CALIBRATION_FAILED);
        return;
    }

    const unsigned long now = millis();

    if (
        now - reading.lastSampleMillis
        <
        SAMPLE_INTERVAL_MS
    )
    {
        return;
    }

    float electrical[FSR_COUNT] = {0};

    if (!acquireElectricalRaw(electrical))
    {
        reading.valid = false;
        setStatus(FSRStatus::DEGRADED);
        return;
    }

    if (frameLooksSuddenlyZero(electrical, occupantPresent))
    {
        suddenZeroStreak++;
    }
    else
    {
        suddenZeroStreak = 0;
    }

    // A short transient is ignored. Sustained all-zero collapse while an
    // occupant/previous load is expected triggers an ADS reinitialization.
    if (suddenZeroStreak >= 6)
    {
        Serial.println();
        Serial.println(
            "[FSR-MAINT] Sustained unexpected all-zero ADS frame detected."
        );
        Serial.println(
            "[FSR-MAINT] Reinitializing both ADS1115 devices without erasing baseline."
        );

        setStatus(FSRStatus::RECOVERING);
        reading.maintenanceActive = true;

        const bool ok1 = initADS1();
        const bool ok2 = initADS2();

        reading.connected = ok1 && ok2;
        reading.recoveryCount++;
        reading.maintenanceActive = false;

        suddenZeroStreak = 0;

        if (!reading.connected)
        {
            setStatus(FSRStatus::DEGRADED);
            reading.valid = false;
            return;
        }

        setStatus(FSRStatus::READING);
        return;
    }

    processFrame(electrical, occupantPresent);

    reading.sampleCount++;
    reading.lastSampleMillis = now;
    reading.valid =
        reading.connected
        &&
        baselineReady;

    reading.baselineValid = baselineReady;

    updateSamplingRate(now);

    previousWholeSeatTotal =
        reading.wholeSeatTotal;

    if (reading.valid)
    {
        setStatus(FSRStatus::READING);
    }
}


void FSRSensor::forceRecalibration()
{
    if (
        !reading.ads1Connected
        ||
        !reading.ads2Connected
    )
    {
        Serial.println(
            "[FSR-MAINT] Cannot recalibrate: ADS1115 hardware is not fully connected."
        );
        return;
    }

    setStatus(FSRStatus::CALIBRATING);
    reading.valid = false;
    baselineReady = false;
    reading.baselineValid = false;

    baselineReady = calibrateEmptySeat();
    reading.baselineValid = baselineReady;

    if (baselineReady)
    {
        for (int i = 0; i < FSR_COUNT; ++i)
        {
            electricalFilteredLoad[i] = 0.0f;
            reading.pressure[i] = 0.0f;
            reading.normalized[i] = 0.0f;
            reading.modelShare[i] = 0.0f;
        }

        previousWholeSeatTotal = 0.0f;
        suddenZeroStreak = 0;
        reading.valid = false;

        setStatus(FSRStatus::READING);

        Serial.println(
            "[FSR-MAINT] Empty-seat recalibration completed successfully."
        );
    }
    else
    {
        setStatus(FSRStatus::CALIBRATION_FAILED);
        Serial.println(
            "[FSR-MAINT] Recalibration rejected. Do not use this baseline."
        );
    }
}


const FSRReading& FSRSensor::getReading() const
{
    return reading;
}


bool FSRSensor::hasValidReading() const
{
    return reading.valid;
}


void FSRSensor::setStatus(FSRStatus status)
{
    reading.status = status;
}


const char* FSRSensor::getStatusText() const
{
    switch (reading.status)
    {
        case FSRStatus::UNINITIALIZED:
            return "UNINITIALIZED";

        case FSRStatus::CALIBRATING:
            return "CALIBRATING";

        case FSRStatus::READING:
            return "READING";

        case FSRStatus::DEGRADED:
            return "DEGRADED";

        case FSRStatus::RECOVERING:
            return "RECOVERING";

        case FSRStatus::CALIBRATION_FAILED:
            return "CALIBRATION FAILED";

        default:
            return "UNKNOWN";
    }
}


const char* FSRSensor::getSensorLabel(int logicalIndex) const
{
    switch (logicalIndex)
    {
        case BACKREST_FSR1:
            return "BackLeftTop (FSR1 logical)";

        case BACKREST_FSR2:
            return "BackLeftMiddle (FSR2 logical)";

        case BACKREST_FSR3:
            return "BackLeftBottom (FSR3 logical)";

        case BACKREST_FSR4:
            return "BackRightTop (FSR4 logical)";

        case BACKREST_FSR5:
            return "BackRightMiddle (FSR5 logical)";

        case BACKREST_FSR6:
            return "BackRightBottom (FSR6 logical)";

        case CUSHION_FSR1:
            return "CushionLeft (FSR1)";

        case CUSHION_FSR2:
            return "CushionCenter (FSR2)";

        case CUSHION_FSR3:
            return "CushionRight (FSR3)";

        default:
            return "Unknown FSR";
    }
}


const char* FSRSensor::getElectricalSource(int logicalIndex) const
{
    switch (logicalIndex)
    {
        case BACKREST_FSR1:
            return "ADS1 A3 / electrical Backrest FSR4";

        case BACKREST_FSR2:
            return "ADS2 A0 / electrical Backrest FSR5";

        case BACKREST_FSR3:
            return "ADS2 A1 / electrical Backrest FSR6";

        case BACKREST_FSR4:
            return "ADS1 A0 / electrical Backrest FSR1";

        case BACKREST_FSR5:
            return "ADS1 A1 / electrical Backrest FSR2";

        case BACKREST_FSR6:
            return "ADS1 A2 / electrical Backrest FSR3";

        case CUSHION_FSR1:
            return "ADS2 A2";

        case CUSHION_FSR2:
            return "ADS2 A3";

        case CUSHION_FSR3:
            return "GPIO34";

        default:
            return "unknown";
    }
}


void FSRSensor::printMaintenanceDiagnostics(Stream& out) const
{
    out.println();
    out.println("========================================");
    out.println(" SAFESEAT FSR MAINTENANCE DIAGNOSTICS");
    out.println("========================================");

    out.print("Status          : ");
    out.println(getStatusText());

    out.print("ADS1 0x48       : ");
    out.println(reading.ads1Connected ? "OK" : "FAULT");

    out.print("ADS2 0x49       : ");
    out.println(reading.ads2Connected ? "OK" : "FAULT");

    out.print("Baseline valid  : ");
    out.println(reading.baselineValid ? "YES" : "NO");

    out.print("Recovery count  : ");
    out.println(reading.recoveryCount);

    out.print("Actual Fs       : ");
    out.println(reading.actualSamplingRateHz, 2);

    out.println();
    out.println("Logical/model channels:");

    for (int i = 0; i < FSR_COUNT; ++i)
    {
        out.print("  ");
        out.print(i + 1);
        out.print(" | ");
        out.print(getSensorLabel(i));
        out.print(" | source=");
        out.print(getElectricalSource(i));
        out.print(" | raw=");
        out.print(reading.raw[i], 1);
        out.print(" | base=");
        out.print(reading.baseline[i], 1);
        out.print(" | pressure=");
        out.print(reading.pressure[i], 1);
        out.print(" | share=");
        out.println(reading.modelShare[i], 5);
    }

    out.println();
    out.println("Electrical raw channels:");

    for (int e = 0; e < FSR_COUNT; ++e)
    {
        out.print("  E");
        out.print(e);
        out.print(" raw=");
        out.print(reading.electricalRaw[e], 1);
        out.print(" baseline=");
        out.println(reading.electricalBaseline[e], 1);
    }

    out.println("========================================");
}
