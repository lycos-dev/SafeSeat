#include "MLXNativeInference.h"
#include "MLXNativeModelData.h"
#include <math.h>

void MLXNativeInference::preprocess(const float input[2], float output[2]) const
{
    for (uint16_t i = 0; i < MLX_NATIVE_FEATURE_COUNT; ++i)
    {
        float v = input[i];
        if (!isfinite(v))
            v = MLX_NATIVE_IMPUTER_MEDIAN[i];

        float s = MLX_NATIVE_SCALER_SCALE[i];
        if (fabsf(s) < 1.0e-12f)
            s = 1.0f;

        output[i] = (v - MLX_NATIVE_SCALER_CENTER[i]) / s;
    }
}

float MLXNativeInference::averagePathLength(uint16_t n) const
{
    if (n <= 1) return 0.0f;
    if (n == 2) return 1.0f;
    const float EULER_GAMMA = 0.5772156649015329f;
    float x = static_cast<float>(n);
    return 2.0f * (logf(x - 1.0f) + EULER_GAMMA) - 2.0f * (x - 1.0f) / x;
}

float MLXNativeInference::isolationForestDecision(const float scaled[2]) const
{
    double totalDepth = 0.0;

    for (uint16_t t = 0; t < MLX_NATIVE_IF_TREE_COUNT; ++t)
    {
        uint32_t base = MLX_NATIVE_IF_TREE_OFFSETS[t];
        int16_t node = 0;
        uint16_t depth = 0;

        while (true)
        {
            uint32_t global = base + static_cast<uint32_t>(node);
            int8_t feature = MLX_NATIVE_IF_FEATURE[global];

            if (feature < 0)
            {
                totalDepth += static_cast<double>(depth)
                    + static_cast<double>(averagePathLength(MLX_NATIVE_IF_N_NODE_SAMPLES[global]));
                break;
            }

            if (scaled[static_cast<uint8_t>(feature)] <= MLX_NATIVE_IF_THRESHOLD[global])
                node = MLX_NATIVE_IF_CHILDREN_LEFT[global];
            else
                node = MLX_NATIVE_IF_CHILDREN_RIGHT[global];

            depth++;
        }
    }

    float denom = static_cast<float>(MLX_NATIVE_IF_TREE_COUNT)
        * averagePathLength(MLX_NATIVE_IF_MAX_SAMPLES);

    double rawMagnitude = pow(2.0, -totalDepth / static_cast<double>(denom));
    double scoreSamples = -rawMagnitude;
    return static_cast<float>(scoreSamples - static_cast<double>(MLX_NATIVE_IF_OFFSET));
}

float MLXNativeInference::oneClassSVMDecision(const float scaled[2]) const
{
    double sum = 0.0;

    for (uint16_t i = 0; i < MLX_NATIVE_OCSVM_SUPPORT_VECTOR_COUNT; ++i)
    {
        double sq = 0.0;
        uint32_t base = static_cast<uint32_t>(i) * MLX_NATIVE_FEATURE_COUNT;

        for (uint16_t j = 0; j < MLX_NATIVE_FEATURE_COUNT; ++j)
        {
            double d = static_cast<double>(scaled[j])
                - static_cast<double>(MLX_NATIVE_OCSVM_SUPPORT_VECTORS[base + j]);
            sq += d * d;
        }

        double kernel = exp(-static_cast<double>(MLX_NATIVE_OCSVM_GAMMA) * sq);
        sum += static_cast<double>(MLX_NATIVE_OCSVM_DUAL_COEF[i]) * kernel;
    }

    return static_cast<float>(sum + static_cast<double>(MLX_NATIVE_OCSVM_INTERCEPT));
}

MLXNativeDecision MLXNativeInference::predict(const float features[2]) const
{
    float scaled[2];
    preprocess(features, scaled);

    MLXNativeDecision out;
    out.isolationForestDecision = isolationForestDecision(scaled);
    out.oneClassSVMDecision = oneClassSVMDecision(scaled);
    out.isolationForestAnomaly = out.isolationForestDecision < 0.0f;
    out.oneClassSVMAnomaly = out.oneClassSVMDecision < 0.0f;
    out.eitherAnomaly = out.isolationForestAnomaly || out.oneClassSVMAnomaly;
    out.bothAnomaly = out.isolationForestAnomaly && out.oneClassSVMAnomaly;
    return out;
}
