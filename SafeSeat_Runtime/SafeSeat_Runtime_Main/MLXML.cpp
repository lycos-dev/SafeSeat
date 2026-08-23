#include "MLXML.h"

#include <math.h>

void MLXML::begin()
{
    reading = MLXMLReading{};
    reading.modelAvailable = true;
    reading.baselineBlocksRequired = BASELINE_BLOCK_COUNT;
    reading.targetLossGraceBlocks = 0U;
    resetSessionBaseline(MLXMLStatus::WAITING_FOR_SAMPLE);

    Serial.println();
    Serial.println("[MLX-ML] Native MLX90614 model initialized.");
    Serial.println("[MLX-ML] External source: human MLX90614 dataset (not WESAD/E4).");
    Serial.println("[MLX-ML] Physical acquisition: 4 Hz -> filtered 1-second blocks.");
    Serial.println("[MLX-ML] 30 stable seconds establish personal/session baseline.");
    Serial.println("[MLX-ML] Features: delta from session baseline + absolute delta only.");
    Serial.println("[MLX-ML] Median+EMA filtered object signal feeds baseline/model blocks.");
    Serial.println("[MLX-ML] Ambient/Object-Ta remain quality/context, not ML features.");
    Serial.println("[MLX-ML] OCCUPANCY establishes the thermal session; Object-Ta is NOT a hard gate.");
    Serial.println("[MLX-ML] Low thermal contrast is reported but does NOT reset/block baseline or ML.");
    Serial.println("[MLX-ML] Filtered 1-s transition guard: std <= 1.00 C.");
    Serial.println("[MLX-ML] FOV guard: rapid 1-2 s shifts -> TARGET GEOMETRY DEGRADED.");
    Serial.println("[MLX-ML] Geometry hold preserves baseline; 3 stable near-baseline sec reacquire.");
    Serial.println("[MLX-ML] Anomaly vote requires 3 consecutive trusted anomalous blocks.");
    Serial.println("[MLX-ML] IF + tuned OCSVM active and available to Fusion.");
}

void MLXML::clearDecision()
{
    reading.valid = false;
    reading.deviationFromBaselineC = NAN;
    reading.isolationForestDecision = NAN;
    reading.oneClassSVMDecision = NAN;
    reading.isolationForestAnomaly = false;
    reading.oneClassSVMAnomaly = false;
    reading.bothModelsAnomaly = false;
    reading.eitherModelAnomaly = false;
}

void MLXML::resetSessionBaseline(MLXMLStatus status)
{
    blockCount = 0;
    baselineCount = 0;
    reading.baselineBlocksCollected = 0;
    reading.baselineReady = false;
    reading.baselineObjectC = NAN;
    reading.warmTargetQualified = false;
    reading.targetContrastDegraded = false;
    reading.stabilityQualified = false;
    reading.lowContrastBlocks = 0;
    reading.geometryDegraded = false;
    reading.reacquiring = false;
    reading.geometryReacquireStableBlocks = 0;
    reading.anomalyCandidateBlocks = 0;
    geometryHold = false;
    geometryReacquireCount = 0;
    consecutiveAnomalyBlocks = 0;
    previousStableObjectC = NAN;
    twoBlocksAgoStableObjectC = NAN;
    clearDecision();
    reading.status = status;
    lowContrastBlockCount = 0;
}

float MLXML::mean4(const float values[BLOCK_SAMPLES])
{
    return (values[0] + values[1] + values[2] + values[3]) / 4.0f;
}

float MLXML::std4(const float values[BLOCK_SAMPLES], float mean)
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < BLOCK_SAMPLES; ++i)
    {
        const float d = values[i] - mean;
        sum += d * d;
    }
    return sqrtf(sum / static_cast<float>(BLOCK_SAMPLES));
}

float MLXML::median30(const float values[BASELINE_BLOCK_COUNT])
{
    float working[BASELINE_BLOCK_COUNT];
    for (uint8_t i = 0; i < BASELINE_BLOCK_COUNT; ++i)
        working[i] = values[i];

    for (uint8_t i = 1; i < BASELINE_BLOCK_COUNT; ++i)
    {
        const float key = working[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && working[j] > key)
        {
            working[j + 1] = working[j];
            --j;
        }
        working[j + 1] = key;
    }

    return 0.5f * (working[14] + working[15]);
}

void MLXML::consumeAcceptedSample(float objectC, float ambientC)
{
    if (!isfinite(objectC) || !isfinite(ambientC))
        return;

    blockObject[blockCount] = objectC;
    blockAmbient[blockCount] = ambientC;
    blockCount++;

    if (blockCount >= BLOCK_SAMPLES)
        processOneSecondBlock();
}

void MLXML::processOneSecondBlock()
{
    const float objectMean = mean4(blockObject);
    const float ambientMean = mean4(blockAmbient);
    const float objectStd = std4(blockObject, objectMean);
    const float thermalDelta = objectMean - ambientMean;

    blockCount = 0;

    reading.oneSecondObjectMeanC = objectMean;
    reading.oneSecondAmbientMeanC = ambientMean;
    reading.oneSecondObjectStdC = objectStd;
    reading.targetDeltaC = thermalDelta;

    reading.stabilityQualified =
        isfinite(objectStd)
        && objectStd <= MAX_ONE_SECOND_OBJECT_STD_C;

    // --------------------------------------------------------
    // OCCUPANCY-GATED THERMAL SESSION
    // --------------------------------------------------------
    // Object-Ta is deliberately NOT allowed to decide whether
    // the occupant exists. The MLX Ta channel follows the local
    // sensor/package thermal environment and can drift upward
    // near a warm occupant/enclosure.
    //
    // The occupant session is established outside this wrapper
    // by FSR/C1001 occupancy. Object-Ta is retained only as a
    // diagnostic confidence/context signal.
    // --------------------------------------------------------

    reading.warmTargetQualified =
        reading.seatOccupied;

    reading.targetContrastDegraded =
        !isfinite(thermalDelta)
        || thermalDelta < HIGH_CONTRAST_CONTEXT_C;

    if (reading.targetContrastDegraded)
    {
        if (lowContrastBlockCount < 255U)
            lowContrastBlockCount++;
    }
    else
    {
        lowContrastBlockCount = 0;
    }

    reading.lowContrastBlocks =
        lowContrastBlockCount;

    if (!reading.seatOccupied)
    {
        clearDecision();
        reading.status =
            MLXMLStatus::WAITING_FOR_WARM_TARGET;
        return;
    }

    // A genuinely abrupt filtered transition is still held so
    // it cannot contaminate the personal baseline/model.
    if (!reading.stabilityQualified)
    {
        reading.unstableBlocksHeld++;
        clearDecision();
        consecutiveAnomalyBlocks = 0;
        reading.anomalyCandidateBlocks = 0;
        reading.status =
            MLXMLStatus::UNSTABLE_TARGET;
        return;
    }

    // --------------------------------------------------------
    // POST-BASELINE FOV / TARGET-GEOMETRY GUARD
    // --------------------------------------------------------
    // A rapid step in a non-contact IR surface reading is much
    // more consistent with target/FOV movement than with a real
    // physiological temperature change. We therefore quarantine
    // the MLX channel instead of creating immediate anomaly
    // evidence. The personal baseline is preserved.
    // --------------------------------------------------------
    if (reading.baselineReady)
    {
        const float deviationFromBaseline =
            objectMean - reading.baselineObjectC;

        const bool havePrevious =
            isfinite(previousStableObjectC);

        const bool haveTwoAgo =
            isfinite(twoBlocksAgoStableObjectC);

        const float oneSecondStep =
            havePrevious
                ? objectMean - previousStableObjectC
                : 0.0f;

        const float twoSecondStep =
            haveTwoAgo
                ? objectMean - twoBlocksAgoStableObjectC
                : 0.0f;

        const bool meaningfulDeparture =
            fabsf(deviationFromBaseline)
            >=
            GEOMETRY_MIN_BASELINE_DEVIATION_C;

        const bool rapidGeometryShift =
            meaningfulDeparture
            &&
            (
                (
                    havePrevious
                    &&
                    fabsf(oneSecondStep)
                    >=
                    GEOMETRY_ONE_SEC_STEP_C
                )
                ||
                (
                    haveTwoAgo
                    &&
                    fabsf(twoSecondStep)
                    >=
                    GEOMETRY_TWO_SEC_STEP_C
                )
            );

        if (!geometryHold && rapidGeometryShift)
        {
            geometryHold = true;
            geometryReacquireCount = 0;
            consecutiveAnomalyBlocks = 0;
            reading.anomalyCandidateBlocks = 0;
            reading.geometryEvents++;
        }

        if (geometryHold)
        {
            const bool nearTrustedBaseline =
                fabsf(deviationFromBaseline)
                <=
                GEOMETRY_REACQUIRE_BAND_C;

            if (nearTrustedBaseline)
            {
                if (geometryReacquireCount < GEOMETRY_REACQUIRE_BLOCKS)
                    geometryReacquireCount++;
            }
            else
            {
                geometryReacquireCount = 0;
            }

            reading.geometryReacquireStableBlocks =
                geometryReacquireCount;

            clearDecision();
            reading.geometryDegraded = true;
            reading.reacquiring =
                geometryReacquireCount > 0;

            // Keep recent stable physical values moving so the
            // dashboard remains representative, but do not alter
            // the personal baseline or infer a medical anomaly.
            twoBlocksAgoStableObjectC = previousStableObjectC;
            previousStableObjectC = objectMean;

            if (geometryReacquireCount >= GEOMETRY_REACQUIRE_BLOCKS)
            {
                geometryHold = false;
                geometryReacquireCount = 0;
                reading.geometryReacquireStableBlocks =
                    GEOMETRY_REACQUIRE_BLOCKS;
                reading.geometryDegraded = false;
                reading.reacquiring = true;
                reading.status =
                    MLXMLStatus::REACQUIRING_TARGET;
                return;
            }

            reading.status =
                reading.reacquiring
                    ? MLXMLStatus::REACQUIRING_TARGET
                    : MLXMLStatus::TARGET_GEOMETRY_DEGRADED;
            return;
        }
    }

    if (!reading.baselineReady)
    {
        if (baselineCount < BASELINE_BLOCK_COUNT)
            baselineBlocks[baselineCount++] = objectMean;

        reading.baselineBlocksCollected = baselineCount;
        reading.status = MLXMLStatus::BUILDING_BASELINE;

        if (baselineCount >= BASELINE_BLOCK_COUNT)
        {
            reading.baselineObjectC = median30(baselineBlocks);
            reading.baselineReady = isfinite(reading.baselineObjectC);

            if (reading.baselineReady)
            {
                previousStableObjectC = reading.baselineObjectC;
                twoBlocksAgoStableObjectC = reading.baselineObjectC;
                consecutiveAnomalyBlocks = 0;
                reading.anomalyCandidateBlocks = 0;
            }

            reading.status = reading.baselineReady
                ? MLXMLStatus::BASELINE_READY
                : MLXMLStatus::INFERENCE_ERROR;
        }
        return;
    }

    const float delta = objectMean - reading.baselineObjectC;
    const float features[2] = {delta, fabsf(delta)};

    MLXNativeDecision result = inference.predict(features);

    if (!isfinite(result.isolationForestDecision)
        || !isfinite(result.oneClassSVMDecision))
    {
        clearDecision();
        reading.status = MLXMLStatus::INFERENCE_ERROR;
        return;
    }

    // Preserve the raw model decisions for diagnostics even
    // while an anomaly candidate is being quarantined.
    reading.deviationFromBaselineC = delta;
    reading.isolationForestDecision = result.isolationForestDecision;
    reading.oneClassSVMDecision = result.oneClassSVMDecision;
    reading.isolationForestAnomaly = result.isolationForestAnomaly;
    reading.oneClassSVMAnomaly = result.oneClassSVMAnomaly;
    reading.bothModelsAnomaly = result.bothAnomaly;
    reading.eitherModelAnomaly = result.eitherAnomaly;
    reading.evaluatedBlocks++;
    reading.lastInferenceMillis = millis();
    reading.geometryDegraded = false;
    reading.reacquiring = false;

    // Shift physical history only after the block passed the
    // transition/geometry guards.
    twoBlocksAgoStableObjectC = previousStableObjectC;
    previousStableObjectC = objectMean;

    if (result.eitherAnomaly)
    {
        if (consecutiveAnomalyBlocks < 255U)
            consecutiveAnomalyBlocks++;

        reading.anomalyCandidateBlocks =
            consecutiveAnomalyBlocks;

        if (consecutiveAnomalyBlocks < ANOMALY_PERSISTENCE_BLOCKS)
        {
            // Candidate is visible in diagnostics but is NOT
            // valid model evidence for Fusion yet.
            reading.valid = false;
            reading.status = MLXMLStatus::ANOMALY_CANDIDATE;
            return;
        }

        reading.valid = true;
        reading.status = result.bothAnomaly
            ? MLXMLStatus::READY_STRONG_ANOMALY
            : MLXMLStatus::READY_WEAK_ANOMALY;
        return;
    }

    // Any trusted NORMAL block immediately clears a transient
    // anomaly candidate.
    consecutiveAnomalyBlocks = 0;
    reading.anomalyCandidateBlocks = 0;
    reading.valid = true;
    reading.status = MLXMLStatus::READY_NORMAL;
}

void MLXML::update(const MLXReading &sensorReading, bool seatOccupied)
{
    if (!sensorReading.connected)
    {
        blockCount = 0;
        resetSessionBaseline(MLXMLStatus::SENSOR_UNAVAILABLE);
        return;
    }

    // Consume only each NEW accepted physical 4-Hz sample once.
    if (!sensorReading.currentSampleAccepted
        || sensorReading.acceptedSampleCount == reading.lastProcessedAcceptedSampleCount)
    {
        return;
    }

    reading.lastProcessedAcceptedSampleCount = sensorReading.acceptedSampleCount;
    reading.seatOccupied = seatOccupied;

    // Occupancy, not Object-Ta, owns session lifetime.
    if (!seatOccupied)
    {
        if (targetLatched)
        {
            reading.targetLosses++;
        }

        targetLatched = false;
        resetSessionBaseline(
            MLXMLStatus::WAITING_FOR_WARM_TARGET
        );
        reading.seatOccupied = false;
        return;
    }

    if (!targetLatched)
    {
        // New occupied session: start a clean personal baseline.
        resetSessionBaseline(
            MLXMLStatus::WAITING_FOR_SAMPLE
        );
        targetLatched = true;
        reading.seatOccupied = true;
        reading.warmTargetQualified = true;
    }

    // IMPORTANT:
    // The shared Main Hub already produces a trusted production
    // MLX signal using median-of-three + EMA filtering.
    //
    // Combined-hardware testing showed that the instantaneous RAW
    // IR samples can vary by multiple degrees when the headrest FOV
    // mixes nape/nearby surfaces, even while the FILTERED object
    // signal remains usable and MLXContext can build a stable
    // personal baseline.
    //
    // Therefore the native ML runtime now forms its 1-second
    // blocks from the accepted FILTERED physical samples.
    //
    // Raw values remain available in MLXReading for diagnostics,
    // but are no longer allowed to falsely block baseline creation.
    consumeAcceptedSample(
        sensorReading.filteredObjectC,
        sensorReading.filteredAmbientC
    );
}

const MLXMLReading& MLXML::getReading() const
{
    return reading;
}

const char* MLXML::getStatusText() const
{
    switch (reading.status)
    {
        case MLXMLStatus::WAITING_FOR_SAMPLE:
            return "WAITING FOR SAMPLE";
        case MLXMLStatus::SENSOR_UNAVAILABLE:
            return "SENSOR UNAVAILABLE";
        case MLXMLStatus::WAITING_FOR_WARM_TARGET:
            return "WAITING FOR OCCUPIED THERMAL SESSION";
        case MLXMLStatus::TARGET_CONTRAST_DEGRADED:
            return "LOW THERMAL CONTRAST - CONTEXT ONLY";
        case MLXMLStatus::UNSTABLE_TARGET:
            return "ABRUPT THERMAL TRANSITION - BLOCK HELD";
        case MLXMLStatus::BUILDING_BASELINE:
            return "BUILDING 30 s PERSONAL BASELINE";
        case MLXMLStatus::BASELINE_READY:
            return "BASELINE READY";
        case MLXMLStatus::READY_NORMAL:
            return "READY - NORMAL";
        case MLXMLStatus::READY_WEAK_ANOMALY:
            return "READY - WEAK ANOMALY";
        case MLXMLStatus::READY_STRONG_ANOMALY:
            return "READY - STRONG ANOMALY";
        case MLXMLStatus::TARGET_GEOMETRY_DEGRADED:
            return "TARGET GEOMETRY DEGRADED - ML HELD";
        case MLXMLStatus::REACQUIRING_TARGET:
            return "REACQUIRING TARGET - ML HELD";
        case MLXMLStatus::ANOMALY_CANDIDATE:
            return "ANOMALY CANDIDATE - PERSISTENCE CHECK";
        case MLXMLStatus::INFERENCE_ERROR:
            return "INFERENCE ERROR";
        default:
            return "UNKNOWN";
    }
}
