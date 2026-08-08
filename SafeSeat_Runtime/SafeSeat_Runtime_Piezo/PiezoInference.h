#pragma once

#include <Arduino.h>

#include "PiezoFeatureExtractor.h"
#include "PiezoModelData.h"

struct PiezoInferenceResult
{
    bool valid = false;

    // sklearn-compatible decision functions:
    // >= 0 -> inlier / normal
    // <  0 -> anomaly
    float isolationForestDecision = 0.0f;
    float oneClassSVMDecision = 0.0f;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;

    float respirationBPM = 0.0f;
};

class PiezoInference
{
public:
    PiezoInference() = default;

    bool predict(
        const PiezoFeatures &features,
        PiezoInferenceResult &result
    ) const;

private:
    void featuresToArray(
        const PiezoFeatures &features,
        float output[PIEZO_MODEL_FEATURE_COUNT]
    ) const;

    void applyScaler(
        const float input[PIEZO_MODEL_FEATURE_COUNT],
        float output[PIEZO_MODEL_FEATURE_COUNT]
    ) const;

    float averagePathLength(
        uint16_t sampleCount
    ) const;

    float isolationForestDecisionFunction(
        const float scaled[PIEZO_MODEL_FEATURE_COUNT]
    ) const;

    float oneClassSVMDecisionFunction(
        const float scaled[PIEZO_MODEL_FEATURE_COUNT]
    ) const;
};
