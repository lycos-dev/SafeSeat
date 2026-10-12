#include "Fusion.h"

#include <math.h>


namespace
{
    float clamp01(
        float value
    )
    {
        if (
            value
            <
            0.0f
        )
        {
            return 0.0f;
        }

        if (
            value
            >
            1.0f
        )
        {
            return 1.0f;
        }

        return value;
    }


    bool isUsableHealth(
        FusionSensorHealth health
    )
    {
        return
            health
            ==
            FusionSensorHealth::VALID
            ||
            health
            ==
            FusionSensorHealth::DEGRADED;
    }


    bool hasModelEvidence(
        const ModelEvidence& model
    )
    {
        return
            model.available
            &&
            model.valid;
    }


    bool hasStrongModelAnomaly(
        const ModelEvidence& model
    )
    {
        return
            hasModelEvidence(
                model
            )
            &&
            model.bothModelsAnomaly;
    }


    bool hasWeakModelAnomaly(
        const ModelEvidence& model
    )
    {
        return
            hasModelEvidence(
                model
            )
            &&
            model.eitherModelAnomaly
            &&
            !model.bothModelsAnomaly;
    }
}


// ============================================================
// SAFESEAT SENSOR FUSION CORE
// STEP 5.2 - CONSERVATIVE DECISION ENGINE
//
// This revision keeps Fusion as a consumer of sensor-model
// results and uses the existing sensor acquisition modules as
// contextual evidence. It does not embed any model logic.
// ============================================================


// ============================================================
// CONSTRUCTOR
// ============================================================

FusionEngine::FusionEngine()
{
    reading =
        FusionReading{};
}


// ============================================================
// BEGIN
// ============================================================

void FusionEngine::begin()
{
    reading =
        FusionReading{};


    reading.valid =
        false;


    reading.occupancy =
        FusionOccupancyState::UNKNOWN;


    reading.motion =
        FusionMotionState::UNKNOWN;


    reading.vitals =
        FusionVitalsState::UNKNOWN;


    reading.pressure =
        FusionPressureState::UNKNOWN;


    reading.temperature =
        FusionTemperatureState::UNKNOWN;


    reading.respiration =
        FusionRespirationState::UNKNOWN;


    reading.level =
        FusionLevel::WATCH;


    reading.confidence =
        0.0f;


    reading.triggerCamera =
        false;


    reading.triggerAlert =
        false;


    reading.lastUpdateMillis =
        millis();


    warningCandidateStartMillis =
        0UL;

    emergencyCandidateStartMillis =
        0UL;

    clearStateStartMillis =
        0UL;

    previousLevel =
        FusionLevel::WATCH;

    lastCameraCorroborationResultId = 0;
    cameraCorroborationReceivedForCandidate = false;

    lastMpuMotionSampleCount = 0UL;
    mpuActivePersistenceSamples = 0U;
    mpuStrongPersistenceSamples = 0U;

    fsrPostureReferenceReady = false;
    fsrPostureReferenceStartMillis = 0UL;
    fsrPostureCandidateStartMillis = 0UL;
    fsrForwardStartMillis = 0UL;
    lastFsrPostureSampleCount = 0UL;
    fsrPostureReferenceSamples = 0U;
    fsrReferenceBackrestTotal = 0.0f;
    fsrReferenceCushionTotal = 0.0f;
    fsrReferenceBackrestBalance = 0.0f;
    fsrReferenceBackrestRatio = 0.0f;
    fsrReferenceBackrestAccum = 0.0f;
    fsrReferenceCushionAccum = 0.0f;
    fsrReferenceBalanceAccum = 0.0f;
    fsrReferenceRatioAccum = 0.0f;
    fsrPostureCandidateCode = 0;


    Serial.println();
    Serial.println(
        "[FUSION] Core initialized."
    );

    Serial.println(
        "[FUSION] Conservative decision engine active."
    );
}


// ============================================================
// UPDATE
// ============================================================

void FusionEngine::update(
    const FusionInput& input
)
{
    const unsigned long now =
        input.timestampMillis
            !=
            0
                ? input.timestampMillis
                : millis();


    reading.lastUpdateMillis =
        now;


    reading.triggerCamera =
        false;


    reading.triggerAlert =
        false;


    reading.valid =
        false;


    reading.occupancy =
        FusionOccupancyState::UNKNOWN;


    reading.motion =
        FusionMotionState::UNKNOWN;


    reading.vitals =
        FusionVitalsState::UNKNOWN;


    reading.pressure =
        FusionPressureState::UNKNOWN;


    reading.temperature =
        FusionTemperatureState::UNKNOWN;


    reading.respiration =
        FusionRespirationState::UNKNOWN;


    reading.level =
        FusionLevel::WATCH;


    reading.confidence =
        0.0f;


    reading.evidence =
        FusionEvidenceSummary{};

    reading.criticalStrongCandidate = false;
    reading.fsrPostureReferenceReady = fsrPostureReferenceReady;
    reading.fsrPostureSettlingMs = 0U;
    reading.fsrPosturePatternActive = false;
    reading.fsrPostureStrongVote = false;
    reading.fsrPostureCode = 0;
    reading.fsrPostureCandidateMs = 0U;
    reading.monitoringLimited = false;
    reading.monitoringLimitedPersistent = false;
    reading.monitoringLimitedMs = 0U;


    const bool c1001Available =
        input.c1001.health
        !=
        FusionSensorHealth::UNAVAILABLE;

    const bool c1001Usable =
        isUsableHealth(
            input.c1001.health
        );

    const bool mlxAvailable =
        input.mlx.health
        !=
        FusionSensorHealth::UNAVAILABLE;

    const bool mlxUsable =
        isUsableHealth(
            input.mlx.health
        );

    const bool fsrAvailable =
        input.fsr.health
        !=
        FusionSensorHealth::UNAVAILABLE;

    const bool fsrUsable =
        isUsableHealth(
            input.fsr.health
        );

    const bool mpuAvailable =
        input.mpu.health
        !=
        FusionSensorHealth::UNAVAILABLE;

    const bool mpuUsable =
        isUsableHealth(
            input.mpu.health
        );


    if (
        c1001Usable
    )
    {
        reading.evidence.validSensorCount++;
    }
    else
    {
        reading.evidence.unavailableSensorCount++;
    }


    // Electrical sensor health and usable passenger target are different.
    // A healthy MLX whose intended skin target is obstructed must not count
    // as a valid Fusion modality until the Main Hub reacquires that target.
    const bool mlxFusionUsable =
        mlxUsable
        && !input.mlx.contributionSuspended;

    if (
        mlxFusionUsable
    )
    {
        reading.evidence.validSensorCount++;
    }
    else
    {
        reading.evidence.unavailableSensorCount++;
    }


    if (
        fsrUsable
    )
    {
        reading.evidence.validSensorCount++;
    }
    else
    {
        reading.evidence.unavailableSensorCount++;
    }


    if (
        mpuUsable
    )
    {
        reading.evidence.validSensorCount++;
    }
    else
    {
        reading.evidence.unavailableSensorCount++;
    }




    // --------------------------------------------------------
    // Occupancy: C1001 + FSR
    // --------------------------------------------------------

    // Conservative occupancy fusion.
    //
    // A sensor that is unavailable/unusable must never be
    // treated as implying EMPTY. EMPTY is only declared when
    // BOTH independent occupancy sources are usable and agree
    // the seat is empty. If only one usable source reports
    // "empty" while the other source cannot be evaluated, the
    // correct decision is UNKNOWN, not EMPTY.

    const bool c1001Present =
        c1001Usable
        &&
        input.c1001.reading.present;

    const bool fsrOccupied =
        fsrUsable
        &&
        input.fsr.reading.occupiedByPressure;

    const bool fsrSaysEmpty =
        fsrUsable
        &&
        input.fsr.reading.wholeSeatTotal
        <
        100.0f;

    if (
        fsrOccupied
    )
    {
        // FSR usable + occupied -> OCCUPIED.
        // (Covers both "FSR occupied" and "C1001 present AND
        // FSR occupied", since FSR occupied alone is already
        // sufficient positive evidence.)
        reading.occupancy =
            FusionOccupancyState::OCCUPIED;
    }
    else if (
        c1001Usable
        &&
        c1001Present
        &&
        fsrUsable
        &&
        fsrSaysEmpty
    )
    {
        // C1001 usable + present AND FSR usable + empty ->
        // the two usable sources disagree.
        reading.occupancy =
            FusionOccupancyState::CONFLICT;
    }
    else if (
        c1001Usable
        &&
        !c1001Present
        &&
        fsrUsable
        &&
        fsrSaysEmpty
    )
    {
        // C1001 usable + not present AND FSR usable + empty ->
        // both usable sources agree the seat is empty.
        reading.occupancy =
            FusionOccupancyState::EMPTY;
    }
    else
    {
        // Either only one source is usable (the other source is
        // unavailable/unusable), or the usable evidence is not
        // strong enough (e.g. FSR in the ambiguous mid-range).
        // A lone empty-leaning source with the other source
        // unavailable/unusable must NOT imply EMPTY.
        reading.occupancy =
            FusionOccupancyState::UNKNOWN;
    }


    // --------------------------------------------------------
    // FSR posture reference + sustained posture context - R5.7
    //
    // Learn a passenger-specific seated reference only after the passenger
    // has both cushion and backrest contact for a stable settling period.
    // This avoids judging the first few seconds while someone is entering,
    // reaching, adjusting a bag, or sitting forward.
    //
    // Sustained left/right shifts and sustained forward loss of backrest
    // contact can become ONE strong FSR vote after 8 s. FSR remains
    // WATCH-only by itself; it never creates WARNING/EMERGENCY alone.
    //
    // Forward posture also exposes a separate Monitoring Limited state after
    // 30 s because MLX/C1001 may become less reliable when the passenger is
    // far away from their intended sensing geometry.
    // --------------------------------------------------------
    if (!fsrUsable
        || reading.occupancy != FusionOccupancyState::OCCUPIED)
    {
        fsrPostureReferenceReady = false;
        fsrPostureReferenceStartMillis = 0UL;
        fsrPostureCandidateStartMillis = 0UL;
        fsrForwardStartMillis = 0UL;
        lastFsrPostureSampleCount = 0UL;
        fsrPostureReferenceSamples = 0U;
        fsrReferenceBackrestTotal = 0.0f;
        fsrReferenceCushionTotal = 0.0f;
        fsrReferenceBackrestBalance = 0.0f;
        fsrReferenceBackrestRatio = 0.0f;
        fsrReferenceBackrestAccum = 0.0f;
        fsrReferenceCushionAccum = 0.0f;
        fsrReferenceBalanceAccum = 0.0f;
        fsrReferenceRatioAccum = 0.0f;
        fsrPostureCandidateCode = 0;
    }
    else
    {
        const bool freshFsrPostureSample =
            input.fsr.reading.sampleCount != lastFsrPostureSampleCount;

        if (freshFsrPostureSample)
        {
            lastFsrPostureSampleCount = input.fsr.reading.sampleCount;
        }

        const float currentBackrest =
            input.fsr.reading.backrestTotal;

        const float currentCushion =
            input.fsr.reading.cushionTotal;

        const float currentBalance =
            input.fsr.reading.backrestLRBalance;

        const float currentRatio =
            input.fsr.reading.backrestToCushionRatio;

        if (!fsrPostureReferenceReady)
        {
            const bool referencePostureUsable =
                currentBackrest >= FSR_REFERENCE_MIN_BACKREST
                && currentCushion >= FSR_REFERENCE_MIN_CUSHION
                && isfinite(currentRatio)
                && currentRatio >= FSR_REFERENCE_MIN_RATIO;

            if (referencePostureUsable)
            {
                if (fsrPostureReferenceStartMillis == 0UL)
                {
                    fsrPostureReferenceStartMillis = now;
                    fsrPostureReferenceSamples = 0U;
                    fsrReferenceBackrestAccum = 0.0f;
                    fsrReferenceCushionAccum = 0.0f;
                    fsrReferenceBalanceAccum = 0.0f;
                    fsrReferenceRatioAccum = 0.0f;
                }

                reading.fsrPostureSettlingMs =
                    static_cast<uint32_t>(
                        now - fsrPostureReferenceStartMillis
                    );

                if (freshFsrPostureSample)
                {
                    fsrPostureReferenceSamples++;
                    fsrReferenceBackrestAccum += currentBackrest;
                    fsrReferenceCushionAccum += currentCushion;
                    fsrReferenceBalanceAccum += currentBalance;
                    fsrReferenceRatioAccum += currentRatio;
                }

                if (
                    reading.fsrPostureSettlingMs >= FSR_POSTURE_SETTLE_MS
                    && fsrPostureReferenceSamples >= 8U
                )
                {
                    const float denom =
                        static_cast<float>(fsrPostureReferenceSamples);

                    fsrReferenceBackrestTotal =
                        fsrReferenceBackrestAccum / denom;

                    fsrReferenceCushionTotal =
                        fsrReferenceCushionAccum / denom;

                    fsrReferenceBackrestBalance =
                        fsrReferenceBalanceAccum / denom;

                    fsrReferenceBackrestRatio =
                        fsrReferenceRatioAccum / denom;

                    fsrPostureReferenceReady = true;
                    fsrPostureCandidateStartMillis = 0UL;
                    fsrPostureCandidateCode = 0;
                }
            }
            else
            {
                // Do not learn a forward/unstable entry posture as normal.
                fsrPostureReferenceStartMillis = 0UL;
                fsrPostureReferenceSamples = 0U;
                fsrReferenceBackrestAccum = 0.0f;
                fsrReferenceCushionAccum = 0.0f;
                fsrReferenceBalanceAccum = 0.0f;
                fsrReferenceRatioAccum = 0.0f;
                reading.fsrPostureSettlingMs = 0U;
            }

            // If the passenger never establishes backrest contact, avoid
            // calling it an emergency. After 30 s simply expose that sensor
            // monitoring is limited by the sustained forward posture.
            const bool forwardBeforeReference =
                currentCushion >= FSR_REFERENCE_MIN_CUSHION
                &&
                (
                    currentBackrest < FSR_REFERENCE_MIN_BACKREST
                    ||
                    (
                        isfinite(currentRatio)
                        && currentRatio < 0.35f
                    )
                );

            if (forwardBeforeReference)
            {
                if (fsrForwardStartMillis == 0UL)
                {
                    fsrForwardStartMillis = now;
                }
            }
            else
            {
                fsrForwardStartMillis = 0UL;
            }
        }
        else
        {
            const float sideShift =
                currentBalance - fsrReferenceBackrestBalance;

            const bool forwardPattern =
                currentBackrest
                    <=
                    fsrReferenceBackrestTotal
                    *
                    FSR_FORWARD_BACKREST_FRACTION
                &&
                currentCushion
                    >=
                    fsrReferenceCushionTotal
                    *
                    FSR_FORWARD_CUSHION_FRACTION
                &&
                isfinite(currentRatio)
                &&
                currentRatio
                    <=
                    fsrReferenceBackrestRatio
                    *
                    FSR_FORWARD_RATIO_FRACTION;

            const bool leftPattern =
                !forwardPattern
                &&
                sideShift >= FSR_SIDE_SHIFT_THRESHOLD
                &&
                currentBackrest
                    >=
                    fsrReferenceBackrestTotal * 0.35f;

            const bool rightPattern =
                !forwardPattern
                &&
                sideShift <= -FSR_SIDE_SHIFT_THRESHOLD
                &&
                currentBackrest
                    >=
                    fsrReferenceBackrestTotal * 0.35f;

            const int8_t postureCode =
                forwardPattern
                    ? 2
                    : leftPattern
                        ? -1
                        : rightPattern
                            ? 1
                            : 0;

            if (postureCode == 0)
            {
                fsrPostureCandidateStartMillis = 0UL;
                fsrPostureCandidateCode = 0;
            }
            else
            {
                if (
                    fsrPostureCandidateStartMillis == 0UL
                    ||
                    fsrPostureCandidateCode != postureCode
                )
                {
                    fsrPostureCandidateStartMillis = now;
                    fsrPostureCandidateCode = postureCode;
                }

                reading.fsrPosturePatternActive = true;
                reading.fsrPostureCode = postureCode;
                reading.fsrPostureCandidateMs =
                    static_cast<uint32_t>(
                        now - fsrPostureCandidateStartMillis
                    );
                reading.fsrPostureStrongVote =
                    reading.fsrPostureCandidateMs
                    >=
                    FSR_POSTURE_PERSIST_MS;
            }

            if (forwardPattern)
            {
                if (fsrForwardStartMillis == 0UL)
                {
                    fsrForwardStartMillis = now;
                }
            }
            else
            {
                fsrForwardStartMillis = 0UL;
            }
        }

        if (fsrForwardStartMillis != 0UL)
        {
            reading.monitoringLimitedMs =
                static_cast<uint32_t>(
                    now - fsrForwardStartMillis
                );

            reading.monitoringLimited =
                reading.monitoringLimitedMs
                >=
                FORWARD_LIMITED_NOTICE_MS;

            reading.monitoringLimitedPersistent =
                reading.monitoringLimitedMs
                >=
                FORWARD_LIMITED_PERSIST_MS;
        }
    }

    reading.fsrPostureReferenceReady =
        fsrPostureReferenceReady;


    // --------------------------------------------------------
    // Motion: MPU6050 vehicle/road context
    // 2026-08-23 V4 BENCH-VALIDATED SEMANTICS
    // --------------------------------------------------------
    //
    // SafeSeat MPU role:
    // vehicle / seat-frame vibration context that can help
    // explain temporary FSR pressure artifacts.
    //
    // It is NOT occupant anomaly evidence.
    //
    // Order:
    // 1) qualify real physical motion;
    // 2) require 3 consecutive fresh MPU samples;
    // 3) only then use IF/OCSVM to characterize road pattern.
    //
    // A stationary OCSVM outlier therefore cannot create
    // vehicle-motion context by itself.
    // --------------------------------------------------------

    bool mpuPhysicalMotion = false;
    bool mpuStrongPhysicalMotion = false;

    bool mpuStrongRoadMotion = false;
    bool mpuWeakRoadMotion = false;

    bool mpuModerateInstantMotion = false;

    if (
        !mpuUsable
        ||
        !input.mpu.reading.valid
    )
    {
        lastMpuMotionSampleCount = 0UL;
        mpuActivePersistenceSamples = 0U;
        mpuStrongPersistenceSamples = 0U;
    }
    else
    {
        const unsigned long currentSampleCount =
            input.mpu.reading.sampleCount;

        if (
            currentSampleCount
            !=
            lastMpuMotionSampleCount
        )
        {
            lastMpuMotionSampleCount =
                currentSampleCount;

            const bool activeSample =
                input.mpu.reading.dynamicAcceleration
                >=
                MPU_ACTIVE_ACCEL_G
                ||
                input.mpu.reading.gyroMagnitude
                >=
                MPU_ACTIVE_GYRO_DPS;

            const bool strongSample =
                input.mpu.reading.dynamicAcceleration
                >=
                MPU_STRONG_ACCEL_G
                ||
                input.mpu.reading.gyroMagnitude
                >=
                MPU_STRONG_GYRO_DPS;

            if (activeSample)
            {
                if (
                    mpuActivePersistenceSamples
                    <
                    MPU_MOTION_PERSIST_SAMPLES
                )
                {
                    mpuActivePersistenceSamples++;
                }
            }
            else
            {
                mpuActivePersistenceSamples = 0U;
            }

            if (strongSample)
            {
                if (
                    mpuStrongPersistenceSamples
                    <
                    MPU_MOTION_PERSIST_SAMPLES
                )
                {
                    mpuStrongPersistenceSamples++;
                }
            }
            else
            {
                mpuStrongPersistenceSamples = 0U;
            }
        }

        mpuPhysicalMotion =
            mpuActivePersistenceSamples
            >=
            MPU_MOTION_PERSIST_SAMPLES;

        mpuStrongPhysicalMotion =
            mpuStrongPersistenceSamples
            >=
            MPU_MOTION_PERSIST_SAMPLES;

        mpuModerateInstantMotion =
            input.mpu.reading.dynamicAcceleration
            >
            0.12f
            ||
            input.mpu.reading.gyroMagnitude
            >
            20.0f;

        if (
            mpuPhysicalMotion
            &&
            hasStrongModelAnomaly(
                input.mpu.model
            )
        )
        {
            mpuStrongRoadMotion = true;
        }
        else if (
            mpuPhysicalMotion
            &&
            hasWeakModelAnomaly(
                input.mpu.model
            )
        )
        {
            mpuWeakRoadMotion = true;
        }
    }

    if (mpuPhysicalMotion)
    {
        // Ordinary vehicle vibration is useful supporting
        // context even if both road models call it normal.
        reading.evidence.supportingContextCount++;
    }

    if (!mpuUsable)
    {
        reading.motion =
            FusionMotionState::UNKNOWN;
    }
    else if (!mpuPhysicalMotion)
    {
        reading.motion =
            FusionMotionState::STILL;
    }
    else if (
        mpuStrongPhysicalMotion
        ||
        mpuStrongRoadMotion
    )
    {
        reading.motion =
            FusionMotionState::HIGH_MOTION;
    }
    else if (
        mpuModerateInstantMotion
        ||
        mpuWeakRoadMotion
    )
    {
        reading.motion =
            FusionMotionState::MODERATE_MOTION;
    }
    else
    {
        reading.motion =
            FusionMotionState::LOW_MOTION;
    }


    // --------------------------------------------------------
    // C1001 / vitals: one primary anomaly vote only
    // --------------------------------------------------------

    bool c1001StrongAnomaly = false;
    bool c1001WeakAnomaly = false;
    bool c1001NormalContext = false;

    if (
        !c1001Available
    )
    {
        reading.vitals =
            FusionVitalsState::UNKNOWN;
    }
    else if (
        !c1001Usable
        ||
        !input.c1001.reading.trustedVitalsAvailable
    )
    {
        reading.vitals =
            FusionVitalsState::NOT_READY;
    }
    else
    {
        if (
            hasStrongModelAnomaly(
                input.c1001.model
            )
        )
        {
            c1001StrongAnomaly =
                true;
        }
        else if (
            hasWeakModelAnomaly(
                input.c1001.model
            )
        )
        {
            c1001WeakAnomaly =
                true;
        }
        else if (
            hasModelEvidence(
                input.c1001.model
            )
            &&
            !input.c1001.model.bothModelsAnomaly
            &&
            !input.c1001.model.eitherModelAnomaly
        )
        {
            c1001NormalContext =
                true;
        }

        reading.vitals =
            c1001StrongAnomaly
            ||
            c1001WeakAnomaly
                ? FusionVitalsState::ANOMALOUS
                : FusionVitalsState::NORMAL;
    }


    if (
        c1001NormalContext
    )
    {
        reading.evidence.normalEvidenceCount++;
    }
    else if (
        c1001StrongAnomaly
    )
    {
        reading.evidence.anomalyEvidenceCount++;
        reading.evidence.strongAnomalyEvidenceCount++;
    }
    else if (
        c1001WeakAnomaly
    )
    {
        reading.evidence.anomalyEvidenceCount++;
    }

    reading.evidence.c1001StrongAnomaly = c1001StrongAnomaly;
    reading.evidence.c1001WeakAnomaly = c1001WeakAnomaly;


    // --------------------------------------------------------
    // FSR / pressure - Step 5.5 embedded anomaly evidence
    //
    // FSR model input is scale-invariant pressure distribution,
    // not raw ADC magnitude. Raw contact/asymmetry fields remain
    // contextual diagnostics and do not become independent
    // anomaly votes by themselves.
    // --------------------------------------------------------

    bool fsrStrongAnomaly = false;
    bool fsrWeakAnomaly = false;
    bool fsrNormalContext = false;

    if (
        !fsrAvailable
        ||
        !fsrUsable
    )
    {
        reading.pressure =
            FusionPressureState::UNKNOWN;
    }
    else if (
        reading.occupancy
        ==
        FusionOccupancyState::EMPTY
    )
    {
        reading.pressure =
            FusionPressureState::EMPTY;

        // Empty-seat agreement is normal occupancy context. The
        // FSR anomaly model itself is intentionally not run on
        // empty pressure windows.
        fsrNormalContext =
            true;
    }
    else
    {
        const bool modelEvidenceReady =
            hasModelEvidence(
                input.fsr.model
            );

        const bool contactLoss =
            input.fsr.reading.backrestTotal
            <
            100.0f
            &&
            input.fsr.reading.cushionTotal
            <
            100.0f;

        const float asymmetry =
            fabsf(
                input.fsr.reading.backrestLRBalance
            )
            +
            fabsf(
                input.fsr.reading.cushionLRBalance
            );

        const bool stronglyAsymmetric =
            asymmetry
            >
            0.28f;

        // R5.7 posture evidence can supply ONE strong FSR vote after a
        // sustained baseline-relative left/right/forward posture change.
        // The legacy validation pattern vote remains accepted for backwards
        // compatibility with existing presets. FSR remains WATCH-only by
        // itself regardless of which FSR path produced the vote.
        if (
            reading.fsrPostureStrongVote
            ||
            input.fsr.controlledUatStrongVote
        )
        {
            fsrStrongAnomaly =
                true;
        }
        else if (
            hasStrongModelAnomaly(
                input.fsr.model
            )
        )
        {
            fsrStrongAnomaly =
                true;
        }
        else if (
            hasWeakModelAnomaly(
                input.fsr.model
            )
        )
        {
            fsrWeakAnomaly =
                true;
        }

        if (
            contactLoss
        )
        {
            reading.pressure =
                FusionPressureState::CONTACT_LOSS;
        }
        else if (
            fsrStrongAnomaly
            ||
            fsrWeakAnomaly
        )
        {
            reading.pressure =
                FusionPressureState::ANOMALOUS;
        }
        else if (
            stronglyAsymmetric
        )
        {
            reading.pressure =
                FusionPressureState::ASYMMETRIC;
        }
        else if (
            modelEvidenceReady
        )
        {
            reading.pressure =
                FusionPressureState::NORMAL;

            fsrNormalContext =
                true;
        }
        else
        {
            // Occupied pressure is present, but the 23-frame
            // model window is still collecting or invalid.
            // Do not call this NORMAL until model evidence exists.
            reading.pressure =
                FusionPressureState::UNKNOWN;
        }
    }


    if (
        fsrNormalContext
    )
    {
        reading.evidence.normalEvidenceCount++;
    }
    else if (
        fsrStrongAnomaly
    )
    {
        reading.evidence.anomalyEvidenceCount++;
        reading.evidence.strongAnomalyEvidenceCount++;
    }
    else if (
        fsrWeakAnomaly
    )
    {
        reading.evidence.anomalyEvidenceCount++;
    }

    reading.evidence.fsrStrongAnomaly = fsrStrongAnomaly;
    reading.evidence.fsrWeakAnomaly = fsrWeakAnomaly;


    // --------------------------------------------------------
    // MLX90614 native temperature model + context
    // 2026-08-23 FINAL ML RUNTIME INTEGRATION
    // --------------------------------------------------------
    //
    // The previous WESAD / Empatica E4 surrogate is retired.
    // input.mlx.model now comes from an actual-MLX90614 external
    // human dataset and operates only on change relative to the
    // occupant/session baseline.
    //
    // The model is still conservative evidence, NOT diagnosis:
    // - both MLX models anomaly -> one STRONG MLX sensor vote
    // - either-only anomaly     -> one WEAK MLX sensor vote
    // - both normal             -> one normal MLX sensor vote
    //
    // MLX ambient temperature and Object-Ta remain contextual
    // signals only. Combined-runtime testing showed Ta can drift
    // with local sensor/package/body heating, so Object-Ta has
    // NO authority to create/destroy the occupant session or
    // suppress the baseline-relative IF/OCSVM model.
    // --------------------------------------------------------

    // Object-Ta / low thermal contrast is diagnostic context only.
    // The native MLX model is baseline-relative, so low contrast must not
    // be translated into TARGET_DEGRADED / HELD by Fusion. True geometry
    // degradation and target-loss suspension are handled separately below.

    const bool mlxGeometryDegraded =
        input.mlx.context.geometryDegraded
        ||
        input.mlx.context.reacquiring;

    const bool mlxModelReady =
        mlxFusionUsable
        && input.mlx.context.baselineReady
        && hasModelEvidence(input.mlx.model);

    const bool mlxStrongAnomaly =
        mlxModelReady
        && hasStrongModelAnomaly(input.mlx.model);

    const bool mlxWeakAnomaly =
        mlxModelReady
        && hasWeakModelAnomaly(input.mlx.model);

    const bool mlxModelNormal =
        mlxModelReady
        && !input.mlx.model.eitherModelAnomaly;

    const bool mlxContextChanged =
        input.mlx.context.valid
        && input.mlx.context.contextChange;

    if (!mlxAvailable || !mlxUsable)
    {
        reading.temperature = FusionTemperatureState::UNKNOWN;
    }
    else if (
        reading.occupancy
        !=
        FusionOccupancyState::OCCUPIED
    )
    {
        reading.temperature = FusionTemperatureState::NO_THERMAL_TARGET;
    }
    else if (input.mlx.contributionSuspended)
    {
        // Hair/clothing/posture/FOV obstruction is not temperature evidence.
        // Keep the channel visible diagnostically, but exclude it from all
        // normal/anomaly voting until stable target reacquisition completes.
        reading.temperature = FusionTemperatureState::TARGET_DEGRADED;
    }
    else if (!isfinite(input.mlx.reading.filteredAmbientC)
        || !isfinite(input.mlx.reading.filteredObjectC))
    {
        reading.temperature = FusionTemperatureState::INVALID;
    }
    else if (mlxGeometryDegraded)
    {
        reading.temperature = FusionTemperatureState::TARGET_DEGRADED;
    }
    else if (mlxStrongAnomaly || mlxWeakAnomaly)
    {
        reading.temperature = FusionTemperatureState::ANOMALOUS;
    }
    else if (!input.mlx.context.baselineReady || !mlxModelReady)
    {
        reading.temperature = FusionTemperatureState::BASELINE_BUILDING;
    }
    else if (mlxContextChanged)
    {
        reading.temperature = FusionTemperatureState::CONTEXT_CHANGE;
    }
    else
    {
        reading.temperature = FusionTemperatureState::STABLE;
    }

    if (mlxStrongAnomaly)
    {
        reading.evidence.anomalyEvidenceCount++;
        reading.evidence.strongAnomalyEvidenceCount++;
    }
    else if (mlxWeakAnomaly)
    {
        reading.evidence.anomalyEvidenceCount++;
    }
    else if (mlxModelNormal)
    {
        reading.evidence.normalEvidenceCount++;
    }

    reading.evidence.mlxStrongAnomaly = mlxStrongAnomaly;
    reading.evidence.mlxWeakAnomaly = mlxWeakAnomaly;

    // Same MLX signal family: context-change is supporting context
    // only and never becomes a second independent anomaly vote.
    if (
        mlxContextChanged
        && !input.mlx.contributionSuspended
        && !mlxGeometryDegraded
        && !mlxStrongAnomaly
        && !mlxWeakAnomaly
    )
    {
        reading.evidence.supportingContextCount++;
    }


    // --------------------------------------------------------
    // Respiration: C1001 only - FINAL ARCHITECTURE
    //
    // C1001 is the final deployed respiration source available
    // to Fusion.
    //
    // The C1001 anomaly model is multivariate across HR, RR and
    // their dynamics, so an anomalous model window cannot be
    // attributed specifically to respiration. In that case the
    // respiration state remains UNKNOWN rather than claiming a
    // respiratory diagnosis.
    // --------------------------------------------------------

    if (
        !c1001Available
        ||
        !c1001Usable
    )
    {
        reading.respiration =
            FusionRespirationState::NOT_AVAILABLE;
    }
    else if (
        !input.c1001.reading.trustedVitalsAvailable
    )
    {
        reading.respiration =
            FusionRespirationState::UNKNOWN;
    }
    else if (
        c1001StrongAnomaly
        ||
        c1001WeakAnomaly
    )
    {
        reading.respiration =
            FusionRespirationState::UNKNOWN;
    }
    else
    {
        reading.respiration =
            FusionRespirationState::NORMAL;
    }


    // --------------------------------------------------------
    // Motion artifact gating
    // --------------------------------------------------------

    bool motionArtifactPossible = false;

    if (
        c1001Usable
        &&
        input.c1001.reading.motionArtifactActive
    )
    {
        motionArtifactPossible =
            true;
    }

    if (
        mpuUsable
        &&
        (
            mpuStrongPhysicalMotion
            ||
            mpuStrongRoadMotion
        )
    )
    {
        // Only persistent STRONG vehicle/road motion can gate
        // escalation as a motion-artifact possibility.
        //
        // A one-sample spike, weak model-only outlier, or
        // stationary OCSVM outlier cannot suppress FSR/occupant
        // evidence.
        motionArtifactPossible =
            true;
    }

    reading.evidence.motionArtifactPossible =
        motionArtifactPossible;

    // R5.7 keeps the agreement diagnostic, but WARNING no longer requires
    // agreement in every case. A trusted STRONG MLX or C1001 anomaly is a
    // critical sensor condition and may escalate by itself after persistence.
    // FSR remains posture/pressure evidence and cannot create WARNING alone.
    reading.evidence.multiSensorAgreement =
        reading.evidence.strongAnomalyEvidenceCount
        >=
        2;


    // --------------------------------------------------------
    // Persistence / hysteresis - R5.7
    // --------------------------------------------------------

    // C1001 is suppressed from standalone critical escalation while its own
    // motion-artifact detector says vitals are unreliable. The anomaly remains
    // visible diagnostically, but must first become trusted again.
    const bool c1001CriticalStrong =
        c1001StrongAnomaly
        &&
        !input.c1001.reading.motionArtifactActive;

    // MLX is already target/baseline/model gated before mlxStrongAnomaly can
    // become true, so a STRONG MLX anomaly is eligible by itself.
    const bool mlxCriticalStrong =
        mlxStrongAnomaly;

    const bool strongCandidate =
        c1001CriticalStrong
        ||
        mlxCriticalStrong;

    const bool warningCandidate =
        strongCandidate;

    const bool isolatedAnomalyPresent =
        reading.evidence.anomalyEvidenceCount
        >=
        1
        ||
        reading.monitoringLimited
        ||
        reading.occupancy
        ==
        FusionOccupancyState::CONFLICT;

    if (warningCandidate)
    {
        if (warningCandidateStartMillis == 0UL)
        {
            warningCandidateStartMillis = now;
        }
    }
    else
    {
        warningCandidateStartMillis = 0UL;
    }

    const bool persistentWarning =
        warningCandidate
        &&
        warningCandidateStartMillis != 0UL
        &&
        now - warningCandidateStartMillis >= WARNING_PERSIST_MS;

    // The 30-second Emergency timer begins only after WARNING is actually
    // confirmed. If the critical condition clears, the timer resets.
    if (persistentWarning)
    {
        if (emergencyCandidateStartMillis == 0UL)
        {
            emergencyCandidateStartMillis = now;
        }
    }
    else
    {
        emergencyCandidateStartMillis = 0UL;
    }

    const bool persistentEmergencyCandidate =
        persistentWarning
        &&
        emergencyCandidateStartMillis
        !=
        0UL
        &&
        now - emergencyCandidateStartMillis >= EMERGENCY_PERSIST_MS;

    reading.warningCandidateMs =
        warningCandidate && warningCandidateStartMillis != 0UL
            ? static_cast<uint32_t>(now - warningCandidateStartMillis)
            : 0U;

    reading.strongCandidateMs =
        persistentWarning && emergencyCandidateStartMillis != 0UL
            ? static_cast<uint32_t>(now - emergencyCandidateStartMillis)
            : 0U;

    reading.emergencyThresholdMs =
        static_cast<uint32_t>(EMERGENCY_PERSIST_MS);

    // Legacy app field retained. It now means "critical strong candidate",
    // not strictly "two-sensor candidate".
    reading.strongMultisensorCandidate =
        strongCandidate;

    reading.criticalStrongCandidate =
        strongCandidate;

    reading.cameraCorroborationOnly =
        true;

    // Camera result handling is transaction bookkeeping ONLY. One fresh
    // result (UPRIGHT or NON_UPRIGHT) completes corroboration for the current
    // critical strong candidate, preventing repeated camera requests. It
    // never changes the candidate timers or final severity.
    if (!strongCandidate)
    {
        cameraCorroborationReceivedForCandidate = false;
    }
    else if (
        input.camera.resultValid
        && input.camera.resultId != 0U
        && input.camera.resultId != lastCameraCorroborationResultId
    )
    {
        lastCameraCorroborationResultId = input.camera.resultId;
        cameraCorroborationReceivedForCandidate = true;
    }

    // concernActive is the current WARNING-eligible critical condition.
    // When it clears, the elevated state is held for the normal 3 s recovery
    // hysteresis, then falls to WATCH if some non-critical evidence remains.
    const bool concernActive =
        warningCandidate;

    const bool previousLevelElevated =
        previousLevel
        ==
        FusionLevel::WARNING
        ||
        previousLevel
        ==
        FusionLevel::EMERGENCY;

    if (
        concernActive
    )
    {
        // Concern returned - cancel any in-progress clear timer.
        // The elevated state is retained (see final decision
        // cascade below) until persistence re-confirms it or a
        // fresh continuous clear period completes.
        clearStateStartMillis =
            0UL;
    }
    else if (
        previousLevelElevated
    )
    {
        if (
            clearStateStartMillis
            ==
            0UL
        )
        {
            clearStateStartMillis =
                now;
        }
    }
    else
    {
        clearStateStartMillis =
            0UL;
    }

    // True only once WARNING-level agreement has remained absent
    // (concernActive == false, uninterrupted) for at least CLEAR_STABLE_MS.
    // An isolated anomaly may still remain and will then resolve to WATCH.
    const bool clearPeriodComplete =
        clearStateStartMillis
        !=
        0UL
        &&
        (
            now
            -
            clearStateStartMillis
        )
        >=
        CLEAR_STABLE_MS;

    // True while we are still inside a continuous clear period
    // (clean so far, but not yet long enough) and the previous
    // state was elevated - the elevated state must be held here.
    const bool inClearHold =
        !concernActive
        &&
        previousLevelElevated
        &&
        !clearPeriodComplete;


    // --------------------------------------------------------
    // Final decision
    // --------------------------------------------------------

    FusionLevel effectiveLevel =
        FusionLevel::WATCH;

    // --------------------------------------------------------
    // Final severity authority
    //
    // Camera is deliberately NOT part of this decision. It is requested
    // in parallel for event/report corroboration only. UPRIGHT cannot clear
    // a real sensor concern; NON_UPRIGHT cannot create an Emergency.
    // --------------------------------------------------------

    if (persistentEmergencyCandidate)
    {
        // A qualifying critical MLX/C1001 condition remained active for
        // the full 30-second post-WARNING emergency persistence window.
        effectiveLevel =
            FusionLevel::EMERGENCY;
        reading.triggerAlert =
            true;
        reading.triggerCamera =
            false;
    }
    else if (persistentWarning)
    {
        // WARNING is confirmed after persistence of a trusted STRONG MLX or
        // C1001 condition. Request the camera once in parallel as corroborative
        // evidence while the 30 s emergency escalation timer continues.
        effectiveLevel =
            FusionLevel::WARNING;
        reading.triggerCamera =
            strongCandidate
            && !cameraCorroborationReceivedForCandidate;
    }
    else if (
        concernActive
        &&
        previousLevelElevated
    )
    {
        // Concern has returned (candidate flickered back on, or
        // its persistence timer just restarted) but has not yet
        // re-confirmed persistence. Do not drop out of the
        // previous elevated state on a single flicker - hold it
        // until persistence re-confirms (handled above) or a
        // full continuous CLEAR_STABLE_MS clean period elapses.
        effectiveLevel =
            previousLevel;
    }
    else if (
        inClearHold
    )
    {
        // WARNING-level agreement has remained absent so far, but the
        // de-escalation period has not yet run for CLEAR_STABLE_MS - keep
        // holding the previous elevated state.
        effectiveLevel =
            previousLevel;
    }
    else if (isolatedAnomalyPresent)
    {
        // FSR-only posture/model concerns, weak anomalies, Monitoring Limited,
        // and other non-critical isolated evidence remain WATCH. A trusted
        // STRONG MLX/C1001 condition has already been handled above.
        effectiveLevel =
            FusionLevel::WATCH;
    }
    else if (
        reading.occupancy
        ==
        FusionOccupancyState::EMPTY
        &&
        reading.evidence.validSensorCount
        >=
        1
    )
    {
        effectiveLevel =
            FusionLevel::SAFE;
    }
    else if (
        reading.occupancy
        ==
        FusionOccupancyState::OCCUPIED
        &&
        reading.evidence.validSensorCount
        >=
        2
        &&
        reading.vitals
        !=
        FusionVitalsState::UNKNOWN
        &&
        reading.vitals
        !=
        FusionVitalsState::NOT_READY
        &&
        reading.pressure
        !=
        FusionPressureState::UNKNOWN
        &&
        reading.respiration
        !=
        FusionRespirationState::UNKNOWN
    )
    {
        effectiveLevel =
            FusionLevel::SAFE;
    }
    else
    {
        effectiveLevel =
            FusionLevel::WATCH;
    }


    reading.level =
        effectiveLevel;


    reading.valid =
        reading.evidence.validSensorCount
        >
        0
        &&
        (
            reading.level
            !=
            FusionLevel::WATCH
            ||
            reading.occupancy
            !=
            FusionOccupancyState::UNKNOWN
        );


    const unsigned int modelEvidenceCount =
        (
            hasModelEvidence(
                input.c1001.model
            )
                ? 1U
                : 0U
        )
        +
        (
            hasModelEvidence(
                input.fsr.model
            )
                ? 1U
                : 0U
        )
        +
        (
            hasModelEvidence(
                input.mlx.model
            )
                ? 1U
                : 0U
        );

    reading.confidence =
        clamp01(
            0.12f
            +
            0.08f
            *
            static_cast<float>(
                reading.evidence.validSensorCount
            )
            +
            0.07f
            *
            static_cast<float>(
                modelEvidenceCount
            )
            /
            3.0f
            +
            (
                reading.evidence.multiSensorAgreement
                ? 0.06f
                : 0.0f
            )
            +
            0.02f
            *
            static_cast<float>(
                reading.evidence.supportingContextCount
            )
            +
            (
                persistentWarning
                ? 0.03f
                : 0.0f
            )
            +
            (
                persistentEmergencyCandidate
                ? 0.05f
                : 0.0f
            )
            -
            0.08f
            *
            static_cast<float>(
                reading.evidence.unavailableSensorCount
            )
            -
            (
                motionArtifactPossible
                ? 0.08f
                : 0.0f
            )
        );

    previousLevel =
        reading.level;
}


// ============================================================
// READING ACCESS
// ============================================================

const FusionReading&
FusionEngine::getReading() const
{
    return reading;
}


// ============================================================
// SENSOR HEALTH TEXT
// ============================================================

const char*
FusionEngine::getSensorHealthText(
    FusionSensorHealth health
)
{
    switch (
        health
    )
    {
        case FusionSensorHealth::UNAVAILABLE:
            return "UNAVAILABLE";


        case FusionSensorHealth::INITIALIZING:
            return "INITIALIZING";


        case FusionSensorHealth::WARMING_UP:
            return "WARMING UP";


        case FusionSensorHealth::VALID:
            return "VALID";


        case FusionSensorHealth::DEGRADED:
            return "DEGRADED";


        case FusionSensorHealth::INVALID:
            return "INVALID";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// OCCUPANCY TEXT
// ============================================================

const char*
FusionEngine::getOccupancyText(
    FusionOccupancyState state
)
{
    switch (
        state
    )
    {
        case FusionOccupancyState::UNKNOWN:
            return "UNKNOWN";


        case FusionOccupancyState::EMPTY:
            return "EMPTY";


        case FusionOccupancyState::OCCUPIED:
            return "OCCUPIED";


        case FusionOccupancyState::CONFLICT:
            return "CONFLICT";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// MOTION TEXT
// ============================================================

const char*
FusionEngine::getMotionText(
    FusionMotionState state
)
{
    switch (
        state
    )
    {
        case FusionMotionState::UNKNOWN:
            return "UNKNOWN";


        case FusionMotionState::STILL:
            return "STILL";


        case FusionMotionState::LOW_MOTION:
            return "LOW MOTION";


        case FusionMotionState::MODERATE_MOTION:
            return "MODERATE MOTION";


        case FusionMotionState::HIGH_MOTION:
            return "HIGH MOTION";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// VITALS TEXT
// ============================================================

const char*
FusionEngine::getVitalsText(
    FusionVitalsState state
)
{
    switch (
        state
    )
    {
        case FusionVitalsState::UNKNOWN:
            return "UNKNOWN";


        case FusionVitalsState::NOT_READY:
            return "NOT READY";


        case FusionVitalsState::NORMAL:
            return "NORMAL";


        case FusionVitalsState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// PRESSURE TEXT
// ============================================================

const char*
FusionEngine::getPressureText(
    FusionPressureState state
)
{
    switch (
        state
    )
    {
        case FusionPressureState::UNKNOWN:
            return "UNKNOWN";


        case FusionPressureState::EMPTY:
            return "EMPTY";


        case FusionPressureState::NORMAL:
            return "NORMAL";


        case FusionPressureState::ASYMMETRIC:
            return "ASYMMETRIC";


        case FusionPressureState::CONTACT_LOSS:
            return "CONTACT LOSS";


        case FusionPressureState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// TEMPERATURE TEXT
// ============================================================

const char*
FusionEngine::getTemperatureText(
    FusionTemperatureState state
)
{
    switch (
        state
    )
    {
        case FusionTemperatureState::UNKNOWN:
            return "UNKNOWN";


        case FusionTemperatureState::INVALID:
            return "INVALID";


        case FusionTemperatureState::NO_THERMAL_TARGET:
            return "NO THERMAL TARGET";


        case FusionTemperatureState::TARGET_DEGRADED:
            return "TARGET DEGRADED / HELD";


        case FusionTemperatureState::BASELINE_BUILDING:
            return "BASELINE BUILDING";


        case FusionTemperatureState::STABLE:
            return "STABLE";


        case FusionTemperatureState::CONTEXT_CHANGE:
            return "CONTEXT CHANGE";


        case FusionTemperatureState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// RESPIRATION TEXT
// ============================================================

const char*
FusionEngine::getRespirationText(
    FusionRespirationState state
)
{
    switch (
        state
    )
    {
        case FusionRespirationState::UNKNOWN:
            return "UNKNOWN";


        case FusionRespirationState::NOT_AVAILABLE:
            return "NOT AVAILABLE";


        case FusionRespirationState::NORMAL:
            return "NORMAL";


        case FusionRespirationState::IRREGULAR:
            return "IRREGULAR";


        case FusionRespirationState::NO_BREATH:
            return "NO BREATH";


        case FusionRespirationState::ANOMALOUS:
            return "ANOMALOUS";


        default:
            return "UNKNOWN";
    }
}


// ============================================================
// FINAL LEVEL TEXT
// ============================================================

const char*
FusionEngine::getLevelText(
    FusionLevel level
)
{
    switch (
        level
    )
    {
        case FusionLevel::SAFE:
            return "SAFE";


        case FusionLevel::WATCH:
            return "WATCH";


        case FusionLevel::WARNING:
            return "WARNING";


        case FusionLevel::EMERGENCY:
            return "EMERGENCY";


        default:
            return "WATCH";
    }
}