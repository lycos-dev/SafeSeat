#include "MPUInference.h"

#include <math.h>

// ============================================================
// SIMPLEIMPUTER(MEDIAN) + ROBUSTSCALER
//
// Runtime feature extraction should produce finite values. The
// median fallback is nevertheless retained for exact parity with
// the canonical sklearn preprocessor.
// ============================================================

bool MPUInference::applyPreprocessor(
    const float input[MPU_MODEL_FEATURE_COUNT],
    float output[MPU_MODEL_FEATURE_COUNT]
) const
{
    for (uint16_t i = 0; i < MPU_MODEL_FEATURE_COUNT; i++)
    {
        float value = input[i];

        if (!isfinite(value))
        {
            value = MPU_IMPUTER_MEDIAN[i];
        }

        float scale = MPU_SCALER_SCALE[i];

        if (fabsf(scale) < 1.0e-12f)
        {
            scale = 1.0f;
        }

        output[i] =
            (value - MPU_SCALER_CENTER[i]) / scale;
    }

    return true;
}


// ============================================================
// ISOLATION FOREST AVERAGE PATH LENGTH
// ============================================================

float MPUInference::averagePathLength(
    uint16_t sampleCount
) const
{
    if (sampleCount <= 1)
    {
        return 0.0f;
    }

    if (sampleCount == 2)
    {
        return 1.0f;
    }

    constexpr float EULER_GAMMA =
        0.5772156649015329f;

    float n =
        static_cast<float>(sampleCount);

    return
        2.0f *
        (
            logf(n - 1.0f) +
            EULER_GAMMA
        )
        -
        2.0f *
        (n - 1.0f) /
        n;
}


// ============================================================
// ISOLATION FOREST DECISION FUNCTION
//
// max_features=0.75 was handled during export by converting each
// tree-local feature index through sklearn estimators_features_
// into the corresponding GLOBAL 0..197 feature index.
// ============================================================

float MPUInference::isolationForestDecisionFunction(
    const float scaled[MPU_MODEL_FEATURE_COUNT]
) const
{
    double totalDepth = 0.0;

    for (
        uint16_t treeIndex = 0;
        treeIndex < MPU_IF_TREE_COUNT;
        treeIndex++
    )
    {
        const uint32_t treeBase =
            MPU_IF_TREE_OFFSETS[treeIndex];

        int16_t node = 0;
        uint16_t depth = 0;

        while (true)
        {
            const uint32_t globalNode =
                treeBase +
                static_cast<uint32_t>(node);

            const int16_t feature =
                MPU_IF_FEATURE[globalNode];

            if (feature < 0)
            {
                totalDepth +=
                    static_cast<double>(depth) +
                    static_cast<double>(
                        averagePathLength(
                            MPU_IF_N_NODE_SAMPLES[
                                globalNode
                            ]
                        )
                    );

                break;
            }

            if (
                scaled[feature] <=
                MPU_IF_THRESHOLD[globalNode]
            )
            {
                node =
                    MPU_IF_CHILDREN_LEFT[globalNode];
            }
            else
            {
                node =
                    MPU_IF_CHILDREN_RIGHT[globalNode];
            }

            depth++;
        }
    }

    const float denominator =
        static_cast<float>(MPU_IF_TREE_COUNT) *
        averagePathLength(MPU_IF_MAX_SAMPLES);

    if (denominator <= 0.0f)
    {
        return 0.0f;
    }

    const double rawAnomalyMagnitude =
        pow(
            2.0,
            -totalDepth /
            static_cast<double>(denominator)
        );

    const double scoreSamples =
        -rawAnomalyMagnitude;

    return static_cast<float>(
        scoreSamples -
        static_cast<double>(MPU_IF_OFFSET)
    );
}


// ============================================================
// ONE-CLASS SVM RBF DECISION FUNCTION
// ============================================================

float MPUInference::oneClassSVMDecisionFunction(
    const float scaled[MPU_MODEL_FEATURE_COUNT]
) const
{
    double decision =
        static_cast<double>(
            MPU_OCSVM_INTERCEPT
        );

    for (
        uint16_t svIndex = 0;
        svIndex < MPU_OCSVM_SUPPORT_VECTOR_COUNT;
        svIndex++
    )
    {
        double squaredDistance = 0.0;

        const uint32_t base =
            static_cast<uint32_t>(svIndex) *
            MPU_MODEL_FEATURE_COUNT;

        for (
            uint16_t featureIndex = 0;
            featureIndex < MPU_MODEL_FEATURE_COUNT;
            featureIndex++
        )
        {
            const double difference =
                static_cast<double>(
                    scaled[featureIndex]
                )
                -
                static_cast<double>(
                    MPU_OCSVM_SUPPORT_VECTORS[
                        base + featureIndex
                    ]
                );

            squaredDistance +=
                difference * difference;
        }

        const double kernel =
            exp(
                -static_cast<double>(
                    MPU_OCSVM_GAMMA
                )
                *
                squaredDistance
            );

        decision +=
            static_cast<double>(
                MPU_OCSVM_DUAL_COEF[svIndex]
            )
            *
            kernel;
    }

    return static_cast<float>(decision);
}


// ============================================================
// PREDICT
// ============================================================

bool MPUInference::predict(
    const float input[MPU_MODEL_FEATURE_COUNT],
    MPUInferenceResult &result
) const
{
    result =
        MPUInferenceResult{};

    // 198 floats (~792 bytes) are kept out of loopTask stack.
    // Main runs MPU inference serially, so this reusable buffer is safe.
    static float scaled[MPU_MODEL_FEATURE_COUNT];

    if (
        !applyPreprocessor(
            input,
            scaled
        )
    )
    {
        return false;
    }

    result.isolationForestDecision =
        isolationForestDecisionFunction(scaled);

    result.oneClassSVMDecision =
        oneClassSVMDecisionFunction(scaled);

    if (
        !isfinite(result.isolationForestDecision) ||
        !isfinite(result.oneClassSVMDecision)
    )
    {
        return false;
    }

    result.isolationForestAnomaly =
        result.isolationForestDecision < 0.0f;

    result.oneClassSVMAnomaly =
        result.oneClassSVMDecision < 0.0f;

    result.bothModelsAnomaly =
        result.isolationForestAnomaly &&
        result.oneClassSVMAnomaly;

    result.eitherModelAnomaly =
        result.isolationForestAnomaly ||
        result.oneClassSVMAnomaly;

    result.valid = true;

    return true;
}
