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

    const bool piezoAvailable =
        input.piezo.available;

    const bool piezoConnected =
        piezoAvailable
        &&
        input.piezo.connected;

    const bool piezoUsable =
        piezoConnected
        &&
        input.piezo.valid
        &&
        input.piezo.signalQualityValid;


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


    if (
        piezoUsable
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
            input.fsr.reading.occupied
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
    // Motion: MPU6050 vehicle/road context + raw fallback
    // --------------------------------------------------------
    //
    // Step 5.6 adds the runtime-aligned MPU IF/OCSVM model.
    // IMPORTANT: MPU anomaly is NOT occupant anomaly evidence.
    // It describes vehicle/seat motion that can explain transient
    // changes seen by pressure/vital sensors.
    //
    // - both MPU models anomaly -> strong road/motion context
    // - either-only anomaly     -> weak road/motion context
    // - instantaneous raw magnitudes remain a fast fallback
    //   before the one-second model window is ready
    // --------------------------------------------------------

    bool mpuStrongRoadMotion = false;
    bool mpuWeakRoadMotion = false;

    if (
        mpuUsable &&
        hasStrongModelAnomaly(
            input.mpu.model
        )
    )
    {
        mpuStrongRoadMotion = true;
    }
    else if (
        mpuUsable &&
        hasWeakModelAnomaly(
            input.mpu.model
        )
    )
    {
        mpuWeakRoadMotion = true;
    }

    if (
        mpuStrongRoadMotion ||
        mpuWeakRoadMotion
    )
    {
        // Context only. Do not increment anomalyEvidenceCount.
        reading.evidence.supportingContextCount++;
    }

    if (
        !mpuUsable
    )
    {
        reading.motion =
            FusionMotionState::UNKNOWN;
    }
    else
    {
        const bool strongInstantMotion =
            input.mpu.reading.dynamicAcceleration
            >
            0.25f
            ||
            input.mpu.reading.gyroMagnitude
            >
            35.0f;

        const bool moderateInstantMotion =
            input.mpu.reading.dynamicAcceleration
            >
            0.12f
            ||
            input.mpu.reading.gyroMagnitude
            >
            20.0f;

        const bool lowInstantMotion =
            input.mpu.reading.dynamicAcceleration
            >
            0.04f
            ||
            input.mpu.reading.gyroMagnitude
            >
            8.0f;

        if (
            mpuStrongRoadMotion ||
            strongInstantMotion
        )
        {
            reading.motion =
                FusionMotionState::HIGH_MOTION;
        }
        else if (
            mpuWeakRoadMotion ||
            moderateInstantMotion
        )
        {
            reading.motion =
                FusionMotionState::MODERATE_MOTION;
        }
        else if (
            lowInstantMotion
        )
        {
            reading.motion =
                FusionMotionState::LOW_MOTION;
        }
        else
        {
            reading.motion =
                FusionMotionState::STILL;
        }
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
    // MLX temperature - Step 5.4.5
    // --------------------------------------------------------
    //
    // IMPORTANT:
    // The WESAD-derived MLX IF/OCSVM is NOT consumed here.
    // Step 5.4.3 demonstrated that its E4-contact feature
    // distribution does not transfer safely to a non-contact
    // MLX90614. The model remains available for diagnostics only.
    //
    // Fusion instead consumes MLXContextReading:
    // - filtered OBJECT temperature is the primary signal
    // - MLX Ta is context only
    // - Object-Ta is a thermal-contrast quality gate only
    // - a 30-second filtered-object baseline is established
    // - +/-1.85 C is a broad FDA repeated-round p99 context
    //   marker, NOT a medical abnormal-temperature threshold
    //
    // A context change NEVER creates anomalyEvidenceCount by
    // itself. It is supporting context only until real headrest/
    // nape MLX data can calibrate a deployment-domain model.
    // --------------------------------------------------------

    bool mlxStableContext = false;
    bool mlxContextChanged = false;

    if (
        !mlxAvailable
        ||
        !mlxUsable
    )
    {
        reading.temperature =
            FusionTemperatureState::UNKNOWN;
    }
    else if (
        !isfinite(
            input.mlx.reading.filteredAmbientC
        )
        ||
        !isfinite(
            input.mlx.reading.filteredObjectC
        )
    )
    {
        reading.temperature =
            FusionTemperatureState::INVALID;
    }
    else if (
        !input.mlx.context.thermalContrastQualified
    )
    {
        reading.temperature =
            FusionTemperatureState::NO_THERMAL_TARGET;
    }
    else if (
        !input.mlx.context.baselineReady
    )
    {
        reading.temperature =
            FusionTemperatureState::BASELINE_BUILDING;
    }
    else if (
        input.mlx.context.valid
        &&
        input.mlx.context.contextChange
    )
    {
        reading.temperature =
            FusionTemperatureState::CONTEXT_CHANGE;

        mlxContextChanged =
            true;
    }
    else if (
        input.mlx.context.valid
    )
    {
        reading.temperature =
            FusionTemperatureState::STABLE;

        mlxStableContext =
            true;
    }
    else
    {
        reading.temperature =
            FusionTemperatureState::UNKNOWN;
    }


    if (
        mlxStableContext
    )
    {
        reading.evidence.normalEvidenceCount++;
    }
    else if (
        mlxContextChanged
    )
    {
        reading.evidence.supportingContextCount++;
    }


    // --------------------------------------------------------
    // Respiration: C1001 primary vitals + remote Piezo
    // corroboration (Step 5.7.2)
    //
    // The Piezo model is trained on WESAD RespiBAN as a
    // surrogate respiratory-motion source. Therefore:
    //
    // - Piezo either-only anomaly is context only.
    // - Piezo both-model anomaly alone is still context/WATCH,
    //   never a standalone WARNING/EMERGENCY vote.
    // - Piezo both-model anomaly becomes an independent Fusion
    //   anomaly vote only when C1001 is also anomalous.
    // - If C1001 itself is a strong model anomaly, the agreeing
    //   Piezo both-model result is counted as strong corroboration.
    // - The auxiliary 15 s no-breath timer is not a standalone
    //   medical threshold and never creates anomaly evidence.
    // --------------------------------------------------------

    bool piezoStrongPattern = false;
    bool piezoWeakPattern = false;
    bool piezoNormalContext = false;
    bool piezoUncorroboratedConcern = false;

    if (
        piezoUsable
        &&
        hasStrongModelAnomaly(
            input.piezo.model
        )
    )
    {
        piezoStrongPattern = true;
    }
    else if (
        piezoUsable
        &&
        hasWeakModelAnomaly(
            input.piezo.model
        )
    )
    {
        piezoWeakPattern = true;
    }
    else if (
        piezoUsable
        &&
        hasModelEvidence(
            input.piezo.model
        )
        &&
        !input.piezo.model.eitherModelAnomaly
    )
    {
        piezoNormalContext = true;
    }

    if (
        !c1001Available
        ||
        !c1001Usable
    )
    {
        if (
            piezoUsable
            &&
            (
                piezoStrongPattern
                ||
                piezoWeakPattern
            )
        )
        {
            reading.respiration =
                FusionRespirationState::IRREGULAR;
        }
        else if (
            piezoNormalContext
        )
        {
            reading.respiration =
                FusionRespirationState::NORMAL;
        }
        else
        {
            reading.respiration =
                FusionRespirationState::NOT_AVAILABLE;
        }
    }
    else if (
        !input.c1001.reading.trustedVitalsAvailable
    )
    {
        if (
            piezoUsable
            &&
            (
                piezoStrongPattern
                ||
                piezoWeakPattern
            )
        )
        {
            reading.respiration =
                FusionRespirationState::IRREGULAR;
        }
        else if (
            piezoNormalContext
        )
        {
            reading.respiration =
                FusionRespirationState::NORMAL;
        }
        else
        {
            reading.respiration =
                FusionRespirationState::UNKNOWN;
        }
    }
    else if (
        piezoUsable
        &&
        input.piezo.noBreathTimerExceeded
        &&
        (
            c1001StrongAnomaly
            ||
            c1001WeakAnomaly
        )
    )
    {
        // Corroborated timer context. Still not a diagnosis.
        reading.respiration =
            FusionRespirationState::NO_BREATH;
    }
    else if (
        piezoStrongPattern
        ||
        piezoWeakPattern
    )
    {
        reading.respiration =
            FusionRespirationState::IRREGULAR;
    }
    else
    {
        reading.respiration =
            FusionRespirationState::NORMAL;
    }


    if (
        piezoNormalContext
    )
    {
        reading.evidence.normalEvidenceCount++;
    }

    if (
        piezoWeakPattern
    )
    {
        // A single Piezo model anomaly is never elevated to an
        // occupant anomaly vote because of the surrogate-domain
        // limitation.
        reading.evidence.supportingContextCount++;
        piezoUncorroboratedConcern = true;
    }

    if (
        piezoStrongPattern
    )
    {
        // Both Piezo models agreeing is meaningful respiratory
        // pattern context, but still requires an independent
        // C1001 concern before becoming anomaly evidence.
        reading.evidence.supportingContextCount++;

        if (
            c1001StrongAnomaly
            ||
            c1001WeakAnomaly
        )
        {
            reading.evidence.anomalyEvidenceCount++;
            if (
                c1001StrongAnomaly
            )
            {
                reading.evidence.strongAnomalyEvidenceCount++;
            }
        }
        else
        {
            piezoUncorroboratedConcern = true;
        }
    }

    if (
        piezoUsable
        &&
        input.piezo.noBreathTimerExceeded
    )
    {
        // Engineering context only. Do not create an anomaly
        // vote from this provisional peak/no-breath detector.
        reading.evidence.supportingContextCount++;
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
            mpuStrongRoadMotion
            ||
            input.mpu.reading.dynamicAcceleration
            >
            0.25f
            ||
            input.mpu.reading.gyroMagnitude
            >
            35.0f
        )
    )
    {
        // Only STRONG MPU model agreement gates escalation.
        // An either-only/weak MPU anomaly is supporting context
        // but is not enough by itself to suppress occupant evidence.
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
        piezoUncorroboratedConcern
    )
    {
        // Surrogate Piezo anomaly without C1001 corroboration:
        // remain at WATCH. Do not create WARNING, EMERGENCY,
        // camera request, or alert from Piezo alone.
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
        )
        +
        (
            hasModelEvidence(
                input.piezo.model
            )
                ? 1U
                : 0U
        )
;

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
            4.0f
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