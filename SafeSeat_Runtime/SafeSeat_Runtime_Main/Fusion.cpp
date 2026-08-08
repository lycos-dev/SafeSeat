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

    const bool seatEmpty =
        fsrUsable
        &&
        input.fsr.reading.wholeSeatTotal
        <
        100.0f;

    if (
        c1001Present
        &&
        fsrOccupied
    )
    {
        reading.occupancy =
            FusionOccupancyState::OCCUPIED;
    }
    else if (
        fsrOccupied
    )
    {
        reading.occupancy =
            FusionOccupancyState::OCCUPIED;
    }
    else if (
        c1001Present
        &&
        seatEmpty
    )
    {
        reading.occupancy =
            FusionOccupancyState::CONFLICT;
    }
    else if (
        seatEmpty
    )
    {
        reading.occupancy =
            FusionOccupancyState::EMPTY;
    }
    else if (
        c1001Usable
        &&
        !c1001Present
    )
    {
        reading.occupancy =
            FusionOccupancyState::EMPTY;
    }
    else
    {
        reading.occupancy =
            FusionOccupancyState::UNKNOWN;
    }


    // --------------------------------------------------------
    // Motion: context and artifact gating only
    // --------------------------------------------------------

    if (
        !mpuUsable
    )
    {
        reading.motion =
            FusionMotionState::UNKNOWN;
    }
    else
    {
        const bool strongVehicleMotion =
            input.mpu.reading.dynamicAcceleration
            >
            0.25f
            ||
            input.mpu.reading.gyroMagnitude
            >
            35.0f;

        const bool moderateVehicleMotion =
            input.mpu.reading.dynamicAcceleration
            >
            0.12f
            ||
            input.mpu.reading.gyroMagnitude
            >
            20.0f;

        if (
            strongVehicleMotion
        )
        {
            reading.motion =
                FusionMotionState::HIGH_MOTION;
        }
        else if (
            moderateVehicleMotion
        )
        {
            reading.motion =
                FusionMotionState::MODERATE_MOTION;
        }
        else if (
            input.mpu.reading.dynamicAcceleration
            >
            0.04f
            ||
            input.mpu.reading.gyroMagnitude
            >
            8.0f
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
    // FSR / pressure: context only; model evidence later
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

        fsrNormalContext =
            true;
    }
    else
    {
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
        else
        {
            reading.pressure =
                FusionPressureState::NORMAL;
            fsrNormalContext =
                true;
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
    // MLX temperature: context only
    // --------------------------------------------------------

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
    else
    {
        reading.temperature =
            FusionTemperatureState::STABLE;
    }


    // --------------------------------------------------------
    // Respiration state: context only until piezo/model arrives
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

    const bool clearCandidate =
        !warningCandidate
        &&
        !strongCandidate
        &&
        (
            previousLevel
            ==
            FusionLevel::WARNING
            ||
            previousLevel
            ==
            FusionLevel::EMERGENCY
        );

    if (
        clearCandidate
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


    // --------------------------------------------------------
    // Final decision
    // --------------------------------------------------------

    FusionLevel effectiveLevel =
        FusionLevel::WATCH;

    const bool cameraConfirmedAbnormal =
        input.camera.available
        &&
        input.camera.connected
        &&
        input.camera.resultValid
        &&
        input.camera.postureAbnormal;

    if (
        cameraConfirmedAbnormal
        &&
        persistentEmergencyCandidate
    )
    {
        effectiveLevel =
            FusionLevel::EMERGENCY;
        reading.triggerAlert =
            true;
    }
    else if (
        persistentWarning
    )
    {
        effectiveLevel =
            FusionLevel::WARNING;
        reading.triggerCamera =
            true;
    }
    else if (
        clearCandidate
        &&
        clearStateStartMillis
        !=
        0UL
        &&
        now
        -
        clearStateStartMillis
        <
        CLEAR_STABLE_MS
    )
    {
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


    if (
        effectiveLevel
        ==
        FusionLevel::WARNING
        ||
        effectiveLevel
        ==
        FusionLevel::EMERGENCY
    )
    {
        reading.triggerCamera =
            true;
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
