#pragma once

#include <Arduino.h>

#include "C1001ModelData.h"

struct C1001InferenceResult
{
    bool valid = false;

    // sklearn-compatible decision_function values:
    // >= 0 -> inlier / normal
    // <  0 -> anomaly
    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class C1001Inference
{
public:
    C1001Inference() = default;

    bool predict(
        const float features[C1001_MODEL_FEATURE_COUNT],
        C1001InferenceResult &result
    ) const;

private:
    void applyPreprocessor(
        const float input[C1001_MODEL_FEATURE_COUNT],
        float output[C1001_MODEL_FEATURE_COUNT]
    ) const;

    float averagePathLength(
        uint16_t sampleCount
    ) const;

    float isolationForestDecisionFunction(
        const float scaled[C1001_MODEL_FEATURE_COUNT]
    ) const;

    float oneClassSVMDecisionFunction(
        const float scaled[C1001_MODEL_FEATURE_COUNT]
    ) const;
};
