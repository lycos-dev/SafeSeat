#pragma once

#include <Arduino.h>
#include "FSRModelData.h"

struct FSRInferenceResult
{
    bool valid = false;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class FSRInference
{
public:
    bool predict(
        const float features[FSR_MODEL_FEATURE_COUNT],
        FSRInferenceResult &result
    ) const;

private:
    bool applyPreprocessor(
        const float input[FSR_MODEL_FEATURE_COUNT],
        float output[FSR_MODEL_FEATURE_COUNT]
    ) const;

    float averagePathLength(
        uint16_t sampleCount
    ) const;

    float isolationForestDecisionFunction(
        const float scaled[FSR_MODEL_FEATURE_COUNT]
    ) const;

    float oneClassSVMDecisionFunction(
        const float scaled[FSR_MODEL_FEATURE_COUNT]
    ) const;
};
