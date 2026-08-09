#pragma once

#include <Arduino.h>
#include "MPUModelData.h"

struct MPUInferenceResult
{
    bool valid = false;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;
    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class MPUInference
{
public:
    bool predict(
        const float input[MPU_MODEL_FEATURE_COUNT],
        MPUInferenceResult &result
    ) const;

private:
    bool applyPreprocessor(
        const float input[MPU_MODEL_FEATURE_COUNT],
        float output[MPU_MODEL_FEATURE_COUNT]
    ) const;

    float averagePathLength(
        uint16_t sampleCount
    ) const;

    float isolationForestDecisionFunction(
        const float scaled[MPU_MODEL_FEATURE_COUNT]
    ) const;

    float oneClassSVMDecisionFunction(
        const float scaled[MPU_MODEL_FEATURE_COUNT]
    ) const;
};
