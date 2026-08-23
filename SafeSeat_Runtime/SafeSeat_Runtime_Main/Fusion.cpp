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

    lastCameraRequestId = 0;
    lastCameraResultId = 0;
    cameraAbnormalLatched = false;

    lastMpuMotionSampleCount = 0UL;
    mpuActivePersistenceSamples = 0U;
    mpuStrongPersistenceSamples = 0U;


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


    if (
        mlxUsable
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
        (
            input.fsr.reading.occupiedByPressure
            ||
            input.fsr.reading.wholeSeatTotal
            >
            300.0f
        );

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

        if (
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
            stronglyAsymmetric
        )
        {
            reading.pressure =
                FusionPressureState::ASYMMETRIC;
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
    // MLX ambient temperature, Object-Ta and broad context-change
    // remain contextual/quality signals; they are NOT additional
    // independent anomaly votes and are not model inputs.
    // --------------------------------------------------------

    const bool mlxModelReady =
        mlxUsable
        && input.mlx.context.thermalContrastQualified
        && input.mlx.context.baselineReady
        && hasModelEvidence(input.mlx.model);

    const bool mlxStrongAnomaly =
        hasStrongModelAnomaly(input.mlx.model);

    const bool mlxWeakAnomaly =
        hasWeakModelAnomaly(input.mlx.model);

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
    else if (!isfinite(input.mlx.reading.filteredAmbientC)
        || !isfinite(input.mlx.reading.filteredObjectC))
    {
        reading.temperature = FusionTemperatureState::INVALID;
    }
    else if (!input.mlx.context.thermalContrastQualified)
    {
        reading.temperature = FusionTemperatureState::NO_THERMAL_TARGET;
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

    // Same MLX signal family: context-change is supporting context
    // only and never becomes a second independent anomaly vote.
    if (mlxContextChanged && !mlxStrongAnomaly && !mlxWeakAnomaly)
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

    reading.evidence.multiSensorAgreement =
        reading.evidence.anomalyEvidenceCount
        >=
        2
        ||
        reading.evidence.strongAnomalyEvidenceCount
        >=
        2;


    // --------------------------------------------------------
    // Persistence / hysteresis
    // --------------------------------------------------------

    const bool warningCandidate =
        (
            reading.evidence.anomalyEvidenceCount
            >=
            1
            &&
            !motionArtifactPossible
        )
        ||
        reading.occupancy
        ==
        FusionOccupancyState::CONFLICT;

    const bool strongCandidate =
        reading.evidence.strongAnomalyEvidenceCount
        >=
        2
        &&
        !motionArtifactPossible;

    if (
        warningCandidate
    )
    {
        if (
            warningCandidateStartMillis
            ==
            0UL
        )
        {
            warningCandidateStartMillis =
                now;
        }
    }
    else
    {
        warningCandidateStartMillis =
            0UL;
    }

    if (
        strongCandidate
    )
    {
        if (
            emergencyCandidateStartMillis
            ==
            0UL
        )
        {
            emergencyCandidateStartMillis =
                now;
        }
    }
    else
    {
        emergencyCandidateStartMillis =
            0UL;
    }

    const bool persistentWarning =
        warningCandidate
        &&
        warningCandidateStartMillis
        !=
        0UL
        &&
        now
        -
        warningCandidateStartMillis
        >=
        WARNING_PERSIST_MS;

    const bool persistentEmergencyCandidate =
        strongCandidate
        &&
        emergencyCandidateStartMillis
        !=
        0UL
        &&
        now
        -
        emergencyCandidateStartMillis
        >=
        EMERGENCY_PERSIST_MS;

    // concernActive represents "something is flagging right now",
    // independent of whether it has persisted long enough yet.
    // This is used (rather than the raw persistence flags) so
    // that a single flickered-off update, or a persistence timer
    // restart, does not look identical to a genuinely clean
    // reading for de-escalation purposes.
    const bool concernActive =
        warningCandidate
        ||
        strongCandidate;

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

    // True only once the system has been continuously clean
    // (concernActive == false, uninterrupted) for at least
    // CLEAR_STABLE_MS, measured with millis().
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
    // Camera transaction semantics - Step 5.9.4
    //
    // A normal camera result must be consumed only once; holding
    // the same UPRIGHT packet across many Fusion updates would
    // otherwise keep resetting the emergency persistence timer.
    // An abnormal result is latched only while the underlying
    // strong multisensor candidate remains active, so EMERGENCY
    // does not disappear merely because the one result packet is
    // no longer marked fresh.
    // --------------------------------------------------------

    const bool cameraResultUsable =
        input.camera.available
        && input.camera.connected
        && input.camera.resultValid
        && input.camera.requestId != 0
        && input.camera.resultId != 0;

    const bool newCameraResult =
        cameraResultUsable
        && (
            input.camera.requestId != lastCameraRequestId
            || input.camera.resultId != lastCameraResultId
        );

    bool cameraConfirmedNormal = false;

    if (newCameraResult)
    {
        lastCameraRequestId = input.camera.requestId;
        lastCameraResultId = input.camera.resultId;

        if (input.camera.postureAbnormal)
        {
            cameraAbnormalLatched = true;
        }
        else if (input.camera.postureNormal)
        {
            cameraAbnormalLatched = false;
            cameraConfirmedNormal = persistentEmergencyCandidate;

            // Restart strong-candidate persistence after one
            // authoritative UPRIGHT verification. If the sensor
            // concern remains, it must persist again before a new
            // camera request can be issued.
            emergencyCandidateStartMillis = 0UL;
        }
    }

    if (!strongCandidate)
    {
        cameraAbnormalLatched = false;
    }

    const bool cameraConfirmedAbnormal =
        cameraAbnormalLatched
        && persistentEmergencyCandidate;

    if (
        cameraConfirmedAbnormal
        &&
        persistentEmergencyCandidate
    )
    {
        // camera abnormal + persistent candidate -> EMERGENCY.
        // Verification is already complete, so no further
        // camera trigger is needed.
        effectiveLevel =
            FusionLevel::EMERGENCY;
        reading.triggerAlert =
            true;
        reading.triggerCamera =
            false;
    }
    else if (
        cameraConfirmedNormal
        &&
        persistentEmergencyCandidate
    )
    {
        // Camera normal with a valid result: do NOT escalate to
        // EMERGENCY. Reject/clear this emergency candidate and
        // transition conservatively to WATCH while sensor
        // evidence is re-evaluated on subsequent updates.
        effectiveLevel =
            FusionLevel::WATCH;

        emergencyCandidateStartMillis =
            0UL;
    }
    else if (
        persistentEmergencyCandidate
    )
    {
        // Persistent strong multisensor candidate, no camera
        // verification result yet -> WARNING and request camera.
        // This is the ONLY case that triggers the camera.
        effectiveLevel =
            FusionLevel::WARNING;
        reading.triggerCamera =
            true;
    }
    else if (
        persistentWarning
    )
    {
        // Persistent but weaker (non-strong) concern -> WARNING.
        // Not a persistent strong multi-sensor candidate, so no
        // camera verification is requested.
        effectiveLevel =
            FusionLevel::WARNING;
        reading.triggerCamera =
            false;
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
        // Continuously clean so far, but the clear period has
        // not yet run for CLEAR_STABLE_MS - keep holding the
        // previous elevated state.
        effectiveLevel =
            previousLevel;
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
        &&
        !motionArtifactPossible
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
                input.mpu.model
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