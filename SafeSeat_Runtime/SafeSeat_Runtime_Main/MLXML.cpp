#include "MLXML.h"

#include <math.h>

void MLXML::begin()
{
    reading = MLXMLReading{};
    reading.modelAvailable = true;
    reading.baselineBlocksRequired = BASELINE_BLOCK_COUNT;
    resetSessionBaseline(MLXMLStatus::WAITING_FOR_SAMPLE);

    Serial.println();
    Serial.println("[MLX-ML] Native MLX90614 model initialized.");
    Serial.println("[MLX-ML] External source: human MLX90614 dataset (not WESAD/E4).");
    Serial.println("[MLX-ML] Physical acquisition: 4 Hz -> stable 1-second blocks.");
    Serial.println("[MLX-ML] 30 stable seconds establish personal/session baseline.");
    Serial.println("[MLX-ML] Features: delta from session baseline + absolute delta only.");
    Serial.println("[MLX-ML] Ambient/Object-Ta/stability remain quality/context, not ML features.");
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
    baselineCount = 0;
    reading.baselineBlocksCollected = 0;
    reading.baselineReady = false;
    reading.baselineObjectC = NAN;
    reading.warmTargetQualified = false;
    reading.stabilityQualified = false;
    clearDecision();
    reading.status = status;
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

    reading.warmTargetQualified =
        isfinite(thermalDelta)
        && thermalDelta >= WARM_TARGET_MIN_DELTA_C;

    reading.stabilityQualified =
        isfinite(objectStd)
        && objectStd <= MAX_ONE_SECOND_OBJECT_STD_C;

    if (!reading.warmTargetQualified)
    {
        reading.targetLosses++;
        resetSessionBaseline(MLXMLStatus::WAITING_FOR_WARM_TARGET);

        // resetSessionBaseline clears only model/baseline state; preserve the
        // just-observed block diagnostics for Serial/API reporting.
        reading.targetDeltaC = thermalDelta;
        reading.oneSecondObjectMeanC = objectMean;
        reading.oneSecondAmbientMeanC = ambientMean;
        reading.oneSecondObjectStdC = objectStd;
        reading.warmTargetQualified = false;
        reading.stabilityQualified = objectStd <= MAX_ONE_SECOND_OBJECT_STD_C;
        return;
    }

    if (!reading.stabilityQualified)
    {
        reading.unstableBlocksHeld++;
        clearDecision();
        reading.status = MLXMLStatus::UNSTABLE_TARGET;
        return;
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

    reading.valid = true;
    reading.deviationFromBaselineC = delta;
    reading.isolationForestDecision = result.isolationForestDecision;
    reading.oneClassSVMDecision = result.oneClassSVMDecision;
    reading.isolationForestAnomaly = result.isolationForestAnomaly;
    reading.oneClassSVMAnomaly = result.oneClassSVMAnomaly;
    reading.bothModelsAnomaly = result.bothAnomaly;
    reading.eitherModelAnomaly = result.eitherAnomaly;
    reading.evaluatedBlocks++;
    reading.lastInferenceMillis = millis();

    reading.status = result.bothAnomaly
        ? MLXMLStatus::READY_STRONG_ANOMALY
        : (result.eitherAnomaly
            ? MLXMLStatus::READY_WEAK_ANOMALY
            : MLXMLStatus::READY_NORMAL);
}

void MLXML::update(const MLXReading &sensorReading)
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

    consumeAcceptedSample(
        sensorReading.rawObjectC,
        sensorReading.rawAmbientC
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
            return "WAITING FOR WARM TARGET - BASELINE RESET";
        case MLXMLStatus::UNSTABLE_TARGET:
            return "UNSTABLE TARGET - BLOCK HELD";
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
        case MLXMLStatus::INFERENCE_ERROR:
            return "INFERENCE ERROR";
        default:
            return "UNKNOWN";
    }
}
