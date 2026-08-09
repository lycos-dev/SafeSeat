#include "FSRInference.h"

#include <math.h>

// ============================================================
// STANDARD SCALER
//
// Training:
//     StandardScaler()
// z = (x - mean_) / scale_
// ============================================================

bool FSRInference::applyPreprocessor(
    const float input[FSR_MODEL_FEATURE_COUNT],
    float output[FSR_MODEL_FEATURE_COUNT]
) const
{
    for (
        uint16_t i = 0;
        i < FSR_MODEL_FEATURE_COUNT;
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
            return false;
        }

        float scale =
            FSR_SCALER_SCALE[i];

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
                FSR_SCALER_MEAN[i]
            )
            /
            scale;
    }

    return true;
}


// ============================================================
// ISOLATION FOREST AVERAGE PATH LENGTH
// ============================================================

float FSRInference::averagePathLength(
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

    constexpr float EULER_GAMMA =
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
                n
                -
                1.0f
            )
            +
            EULER_GAMMA
        )
        -
        2.0f
        *
        (
            n
            -
            1.0f
        )
        /
        n;
}


// ============================================================
// ISOLATION FOREST DECISION FUNCTION
// ============================================================

float FSRInference::isolationForestDecisionFunction(
    const float scaled[FSR_MODEL_FEATURE_COUNT]
) const
{
    double totalDepth =
        0.0;

    for (
        uint16_t treeIndex = 0;
        treeIndex < FSR_IF_TREE_COUNT;
        treeIndex++
    )
    {
        uint32_t treeBase =
            FSR_IF_TREE_OFFSETS[
                treeIndex
            ];

        int16_t node =
            0;

        uint16_t depth =
            0;

        while (
            true
        )
        {
            uint32_t globalNode =
                treeBase
                +
                static_cast<uint32_t>(
                    node
                );

            int8_t feature =
                FSR_IF_FEATURE[
                    globalNode
                ];

            if (
                feature
                <
                0
            )
            {
                totalDepth +=
                    static_cast<double>(
                        depth
                    )
                    +
                    static_cast<double>(
                        averagePathLength(
                            FSR_IF_N_NODE_SAMPLES[
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
                FSR_IF_THRESHOLD[
                    globalNode
                ];

            if (
                value
                <=
                threshold
            )
            {
                node =
                    FSR_IF_CHILDREN_LEFT[
                        globalNode
                    ];
            }
            else
            {
                node =
                    FSR_IF_CHILDREN_RIGHT[
                        globalNode
                    ];
            }

            depth++;
        }
    }

    float denominator =
        static_cast<float>(
            FSR_IF_TREE_COUNT
        )
        *
        averagePathLength(
            FSR_IF_MAX_SAMPLES
        );

    if (
        denominator
        <=
        0.0f
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

    return
        static_cast<float>(
            scoreSamples
            -
            static_cast<double>(
                FSR_IF_OFFSET
            )
        );
}


// ============================================================
// ONE-CLASS SVM DECISION FUNCTION
// ============================================================

float FSRInference::oneClassSVMDecisionFunction(
    const float scaled[FSR_MODEL_FEATURE_COUNT]
) const
{
    double decision =
        static_cast<double>(
            FSR_OCSVM_INTERCEPT
        );

    for (
        uint16_t svIndex = 0;
        svIndex < FSR_OCSVM_SUPPORT_VECTOR_COUNT;
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
            FSR_MODEL_FEATURE_COUNT;

        for (
            uint16_t featureIndex = 0;
            featureIndex < FSR_MODEL_FEATURE_COUNT;
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
                    FSR_OCSVM_SUPPORT_VECTORS[
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
                    FSR_OCSVM_GAMMA
                )
                *
                squaredDistance
            );

        decision +=
            static_cast<double>(
                FSR_OCSVM_DUAL_COEF[
                    svIndex
                ]
            )
            *
            kernel;
    }

    return
        static_cast<float>(
            decision
        );
}


// ============================================================
// PREDICT
// ============================================================

bool FSRInference::predict(
    const float features[FSR_MODEL_FEATURE_COUNT],
    FSRInferenceResult &result
) const
{
    result =
        FSRInferenceResult{};

    float scaled[
        FSR_MODEL_FEATURE_COUNT
    ];

    if (
        !applyPreprocessor(
            features,
            scaled
        )
    )
    {
        return false;
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
