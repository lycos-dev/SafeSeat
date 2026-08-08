#pragma once

#include <Arduino.h>

#include "MLXModelData.h"

struct MLXInferenceResult
{
    bool valid = false;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class MLXInference
{
public:
    MLXInference() = default;

    bool predict(
        const float features[MLX_MODEL_FEATURE_COUNT],
        MLXInferenceResult &result
    ) const;

private:
    void applyPreprocessor(
        const float input[MLX_MODEL_FEATURE_COUNT],
        float output[MLX_MODEL_FEATURE_COUNT]
    ) const;

    float averagePathLength(
        uint16_t sampleCount
    ) const;

    float isolationForestDecisionFunction(
        const float scaled[MLX_MODEL_FEATURE_COUNT]
    ) const;

    float oneClassSVMDecisionFunction(
        const float scaled[MLX_MODEL_FEATURE_COUNT]
    ) const;
};
