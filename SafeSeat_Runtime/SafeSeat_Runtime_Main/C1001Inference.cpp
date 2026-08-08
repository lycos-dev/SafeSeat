#include "C1001Inference.h"

#include <math.h>

// ============================================================
// PREPROCESSOR
//
// Training pipeline:
//   SimpleImputer(strategy="median", keep_empty_features=True)
//   RobustScaler()
//
// sklearn RobustScaler:
//   z = (x - center_) / scale_
// ============================================================

void C1001Inference::applyPreprocessor(
    const float input[C1001_MODEL_FEATURE_COUNT],
    float output[C1001_MODEL_FEATURE_COUNT]
) const
{
    for (
        uint16_t i = 0;
        i < C1001_MODEL_FEATURE_COUNT;
        i++
    )
    {
        float value =
            input[i];

        if (
            !isfinite(
                value
            )
        )
        {
            value =
                C1001_IMPUTER_MEDIAN[i];
        }

        float scale =
            C1001_SCALER_SCALE[i];

        if (
            fabsf(
                scale
            )
            <
            1.0e-12f
        )
        {
            scale =
                1.0f;
        }

        output[i] =
            (
                value
                -
                C1001_SCALER_CENTER[i]
            )
            /
            scale;
    }
}


// ============================================================
// ISOLATION FOREST AVERAGE PATH LENGTH
//
// Matches sklearn _average_path_length:
// n <= 1 -> 0
// n == 2 -> 1
//
// otherwise:
// 2 * (ln(n - 1) + EulerGamma)
// - 2 * (n - 1) / n
// ============================================================

float C1001Inference::averagePathLength(
    uint16_t sampleCount
) const
{
    if (
        sampleCount <= 1
    )
    {
        return 0.0f;
    }

    if (
        sampleCount == 2
    )
    {
        return 1.0f;
    }

    constexpr float
        EULER_GAMMA =
            0.5772156649015329f;

    float n =
        static_cast<float>(
            sampleCount
        );

    return
        2.0f
        *
        (
            logf(
                n - 1.0f
            )
            +
            EULER_GAMMA
        )
        -
        2.0f
        *
        (
            n - 1.0f
        )
        /
        n;
}


// ============================================================
// ISOLATION FOREST DECISION FUNCTION
//
// sklearn:
//
// raw anomaly magnitude =
// 2 ^ ( -meanDepth / c(max_samples) )
//
// score_samples = -raw
//
// decision_function = score_samples - offset_
//
// prediction:
// decision >= 0 -> normal
// decision <  0 -> anomaly
// ============================================================

float C1001Inference::isolationForestDecisionFunction(
    const float scaled[C1001_MODEL_FEATURE_COUNT]
) const
{
    double totalDepth =
        0.0;

    for (
        uint16_t treeIndex = 0;
        treeIndex < C1001_IF_TREE_COUNT;
        treeIndex++
    )
    {
        uint32_t treeBase =
            C1001_IF_TREE_OFFSETS[
                treeIndex
            ];

        int16_t node =
            0;

        uint16_t depth =
            0;

        while (true)
        {
            uint32_t globalNode =
                treeBase
                +
                static_cast<uint32_t>(
                    node
                );

            int8_t feature =
                C1001_IF_FEATURE[
                    globalNode
                ];

            // sklearn tree leaves use feature = -2.
            if (
                feature < 0
            )
            {
                totalDepth +=
                    static_cast<double>(
                        depth
                    )
                    +
                    static_cast<double>(
                        averagePathLength(
                            C1001_IF_N_NODE_SAMPLES[
                                globalNode
                            ]
                        )
                    );

                break;
            }

            float value =
                scaled[
                    static_cast<uint8_t>(
                        feature
                    )
                ];

            float threshold =
                C1001_IF_THRESHOLD[
                    globalNode
                ];

            if (
                value <= threshold
            )
            {
                node =
                    C1001_IF_CHILDREN_LEFT[
                        globalNode
                    ];
            }
            else
            {
                node =
                    C1001_IF_CHILDREN_RIGHT[
                        globalNode
                    ];
            }

            depth++;
        }
    }

    float denominator =
        static_cast<float>(
            C1001_IF_TREE_COUNT
        )
        *
        averagePathLength(
            C1001_IF_MAX_SAMPLES
        );

    if (
        denominator <= 0.0f
    )
    {
        return 0.0f;
    }

    double rawAnomalyMagnitude =
        pow(
            2.0,
            -totalDepth
            /
            static_cast<double>(
                denominator
            )
        );

    double scoreSamples =
        -rawAnomalyMagnitude;

    return static_cast<float>(
        scoreSamples
        -
        static_cast<double>(
            C1001_IF_OFFSET
        )
    );
}


// ============================================================
// ONE-CLASS SVM DECISION FUNCTION
//
// RBF kernel:
// K(x, sv) = exp(-gamma * ||x-sv||^2)
//
// sklearn decision:
// sum_i dual_coef[i] * K(x, SV_i) + intercept_
//
// decision >= 0 -> normal
// decision <  0 -> anomaly
// ============================================================

float C1001Inference::oneClassSVMDecisionFunction(
    const float scaled[C1001_MODEL_FEATURE_COUNT]
) const
{
    double decision =
        static_cast<double>(
            C1001_OCSVM_INTERCEPT
        );

    for (
        uint16_t svIndex = 0;
        svIndex < C1001_OCSVM_SUPPORT_VECTOR_COUNT;
        svIndex++
    )
    {
        double squaredDistance =
            0.0;

        uint32_t base =
            static_cast<uint32_t>(
                svIndex
            )
            *
            C1001_MODEL_FEATURE_COUNT;

        for (
            uint16_t featureIndex = 0;
            featureIndex < C1001_MODEL_FEATURE_COUNT;
            featureIndex++
        )
        {
            double difference =
                static_cast<double>(
                    scaled[
                        featureIndex
                    ]
                )
                -
                static_cast<double>(
                    C1001_OCSVM_SUPPORT_VECTORS[
                        base
                        +
                        featureIndex
                    ]
                );

            squaredDistance +=
                difference
                *
                difference;
        }

        double kernel =
            exp(
                -static_cast<double>(
                    C1001_OCSVM_GAMMA
                )
                *
                squaredDistance
            );

        decision +=
            static_cast<double>(
                C1001_OCSVM_DUAL_COEF[
                    svIndex
                ]
            )
            *
            kernel;
    }

    return static_cast<float>(
        decision
    );
}


// ============================================================
// PREDICT
// ============================================================

bool C1001Inference::predict(
    const float features[C1001_MODEL_FEATURE_COUNT],
    C1001InferenceResult &result
) const
{
    result =
        C1001InferenceResult{};

    float scaled[
        C1001_MODEL_FEATURE_COUNT
    ];

    applyPreprocessor(
        features,
        scaled
    );

    for (
        uint16_t i = 0;
        i < C1001_MODEL_FEATURE_COUNT;
        i++
    )
    {
        if (
            !isfinite(
                scaled[i]
            )
        )
        {
            return false;
        }
    }

    result.isolationForestDecision =
        isolationForestDecisionFunction(
            scaled
        );

    result.oneClassSVMDecision =
        oneClassSVMDecisionFunction(
            scaled
        );

    if (
        !isfinite(
            result.isolationForestDecision
        )
        ||
        !isfinite(
            result.oneClassSVMDecision
        )
    )
    {
        return false;
    }

    result.isolationForestAnomaly =
        result.isolationForestDecision
        <
        0.0f;

    result.oneClassSVMAnomaly =
        result.oneClassSVMDecision
        <
        0.0f;

    result.bothModelsAnomaly =
        result.isolationForestAnomaly
        &&
        result.oneClassSVMAnomaly;

    result.eitherModelAnomaly =
        result.isolationForestAnomaly
        ||
        result.oneClassSVMAnomaly;

    result.valid =
        true;

    return true;
}
