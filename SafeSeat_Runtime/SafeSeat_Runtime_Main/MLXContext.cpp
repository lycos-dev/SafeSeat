#include "MLXContext.h"

#include <math.h>


MLXContext::MLXContext()
{
}


void MLXContext::begin()
{
    reading =
        MLXContextReading{};

    reading.available =
        true;

    reading.baselineSamplesRequired =
        BASELINE_SAMPLE_COUNT;

    // Compatibility field only; thermal contrast no longer owns
    // session-loss/reset behavior.
    reading.targetLossGraceSamples =
        0U;

    resetBaseline();
    reading.targetContrastDegraded = false;
    reading.lowContrastSamples = 0;

    Serial.println();
    Serial.println(
        "[MLX-CONTEXT] Step 5.4.5 initialized."
    );
    Serial.println(
        "[MLX-CONTEXT] Primary signal: filtered MLX OBJECT temperature."
    );
    Serial.println(
        "[MLX-CONTEXT] MLX Ta is context only; Object-Ta is NOT body temperature."
    );
    Serial.println(
        "[MLX-CONTEXT] Uses the SAME 30-s native MLX session baseline; no duplicate baseline."
    );
    Serial.println(
        "[MLX-CONTEXT] Context-change reference: +/-1.85 C from baseline (FDA p99 repeated-round range; non-medical)."
    );
    Serial.println(
        "[MLX-CONTEXT] Native MLX IF/OCSVM runs separately; Ta/delta remain context only."
    );
    Serial.println(
        "[MLX-CONTEXT] Object-Ta is LOW/HIGH contrast context only; it never blocks/reset ML."
    );
}


void MLXContext::resetBaseline()
{
    reading.valid =
        false;

    reading.baselineReady =
        false;

    reading.contextChange =
        false;

    reading.baselineObjectC =
        NAN;

    reading.deviationFromBaselineC =
        NAN;

    reading.baselineSamplesCollected =
        0;
}


void MLXContext::update(
    const MLXReading& sensorReading,
    const MLXMLReading& modelReading,
    bool seatOccupied
)
{
    reading.lastUpdateMillis =
        millis();

    if (
        !sensorReading.connected
        ||
        !sensorReading.valid
        ||
        !isfinite(sensorReading.filteredObjectC)
        ||
        !isfinite(sensorReading.filteredAmbientC)
        ||
        !isfinite(sensorReading.objectMinusAmbientC)
    )
    {
        reading.valid = false;
        reading.thermalContrastQualified = false;
        reading.targetContrastDegraded = false;
        reading.status =
            MLXContextStatus::UNAVAILABLE;
        return;
    }

    reading.filteredObjectC =
        sensorReading.filteredObjectC;

    reading.sensorTaC =
        sensorReading.filteredAmbientC;

    reading.thermalContrastC =
        sensorReading.objectMinusAmbientC;

    // Object-Ta is now information, not permission.
    reading.thermalContrastQualified =
        reading.thermalContrastC
        >=
        HIGH_CONTRAST_CONTEXT_C;

    reading.geometryDegraded =
        modelReading.geometryDegraded;

    reading.reacquiring =
        modelReading.reacquiring;

    reading.targetContrastDegraded =
        modelReading.geometryDegraded
        ||
        modelReading.reacquiring
        ||
        !reading.thermalContrastQualified;

    reading.lowContrastSamples =
        reading.targetContrastDegraded
            ? 1U
            : 0U;

    // Share the authoritative native-MLX baseline.
    reading.baselineSamplesRequired =
        BASELINE_SAMPLE_COUNT;

    const uint16_t sharedSamples =
        static_cast<uint16_t>(
            modelReading.baselineBlocksCollected
        )
        *
        4U;

    reading.baselineSamplesCollected =
        sharedSamples > BASELINE_SAMPLE_COUNT
            ? BASELINE_SAMPLE_COUNT
            : sharedSamples;

    reading.baselineReady =
        seatOccupied
        &&
        modelReading.baselineReady
        &&
        isfinite(modelReading.baselineObjectC);

    reading.baselineObjectC =
        reading.baselineReady
            ? modelReading.baselineObjectC
            : NAN;

    if (!seatOccupied)
    {
        reading.valid = false;
        reading.contextChange = false;
        reading.deviationFromBaselineC = NAN;
        reading.status =
            MLXContextStatus::WAITING_FOR_THERMAL_TARGET;
        return;
    }

    if (!reading.baselineReady)
    {
        reading.valid = false;
        reading.contextChange = false;
        reading.deviationFromBaselineC = NAN;
        reading.status =
            MLXContextStatus::BUILDING_BASELINE;
        return;
    }

    reading.valid = true;

    if (
        reading.geometryDegraded
        || reading.reacquiring
        || modelReading.anomalyCandidateBlocks > 0
    )
    {
        // The context layer shares the same physical temperature
        // signal. During geometry quarantine or model anomaly
        // persistence, do not let the same MLX excursion sneak
        // back into Fusion as a separate supporting context vote.
        reading.contextChange = false;
        reading.deviationFromBaselineC =
            reading.filteredObjectC
            -
            reading.baselineObjectC;
        reading.status =
            MLXContextStatus::TARGET_CONTRAST_DEGRADED;
        return;
    }

    reading.deviationFromBaselineC =
        reading.filteredObjectC
        -
        reading.baselineObjectC;

    reading.contextChange =
        fabsf(reading.deviationFromBaselineC)
        >
        FDA_CONTEXT_CHANGE_DELTA_C;

    if (reading.contextChange)
    {
        reading.status =
            MLXContextStatus::CONTEXT_CHANGE;
    }
    else if (reading.targetContrastDegraded)
    {
        // Still valid. Low contrast is only a confidence/context
        // annotation and does not suppress the native model.
        reading.status =
            MLXContextStatus::TARGET_CONTRAST_DEGRADED;
    }
    else
    {
        reading.status =
            MLXContextStatus::STABLE;
    }
}


const MLXContextReading&
MLXContext::getReading() const
{
    return reading;
}


const char*
MLXContext::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case MLXContextStatus::UNAVAILABLE:
            return "UNAVAILABLE";

        case MLXContextStatus::WAITING_FOR_THERMAL_TARGET:
            return "WAITING FOR OCCUPIED SESSION";

        case MLXContextStatus::TARGET_CONTRAST_DEGRADED:
            return "TARGET/FOV DEGRADED - CONTEXT ONLY";

        case MLXContextStatus::BUILDING_BASELINE:
            return "BUILDING 30 s BASELINE";

        case MLXContextStatus::STABLE:
            return "STABLE VS SESSION BASELINE";

        case MLXContextStatus::CONTEXT_CHANGE:
            return "CONTEXT CHANGE VS SESSION BASELINE";

        default:
            return "UNKNOWN";
    }
}
