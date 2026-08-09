#include "PiezoInference.h"

#include <math.h>

// ============================================================
// FEATURE ORDER
//
// MUST match training metadata exactly.
// ============================================================

void PiezoInference::featuresToArray(
    const PiezoFeatures &features,
    float output[PIEZO_MODEL_FEATURE_COUNT]
) const
{
    output[0]  = features.mean;
    output[1]  = features.std;
    output[2]  = features.minimum;
    output[3]  = features.maximum;
    output[4]  = features.range;
    output[5]  = features.median;
    output[6]  = features.iqr;
    output[7]  = features.rms;
    output[8]  = features.energy;
    output[9]  = features.meanAbsDiff;
    output[10] = features.stdDiff;
    output[11] = features.zeroCrossingRate;
    output[12] = features.dominantFrequencyHz;
    output[13] = features.respirationBPM;
    output[14] = features.spectralEntropy;
    output[15] = features.autocorrelationPeak;
}

// ============================================================
// STANDARD SCALER
//
// sklearn StandardScaler:
// z = (x - mean_) / scale_
// ============================================================

void PiezoInference::applyScaler(
    const float input[PIEZO_MODEL_FEATURE_COUNT],
    float output[PIEZO_MODEL_FEATURE_COUNT]
) const
{
    for (
        uint16_t i = 0;
        i < PIEZO_MODEL_FEATURE_COUNT;
        i++
    )
    {
        float scale =
            PIEZO_SCALER_SCALE[i];

        // Do not replace very small learned scales with 1.0.
        // Step 5.7 has a near-zero-but-nonzero median scale
        // (~8e-18) that is part of the fitted StandardScaler.
        // sklearn itself already maps truly constant features to
        // scale_=1. Only zero/non-finite values need a guard.
        if (
            !isfinite(scale)
            ||
            scale == 0.0f
        )
        {
            scale =
                1.0f;
        }

        output[i] =
            (
                input[i]
                -
                PIEZO_SCALER_MEAN[i]
            )
            /
            scale;
    }
}

// ============================================================
// ISOLATION FOREST AVERAGE PATH LENGTH
//
// Matches sklearn _average_path_length:
//
// n <= 1 -> 0
// n == 2 -> 1
//
// otherwise:
// 2 * (ln(n - 1) + EulerGamma)
// - 2 * (n - 1) / n
// ============================================================

float PiezoInference::averagePathLength(
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
// decision_function =
// score_samples - offset_
//
// prediction:
// decision >= 0 -> normal
// decision <  0 -> anomaly
// ============================================================

float PiezoInference::isolationForestDecisionFunction(
    const float scaled[PIEZO_MODEL_FEATURE_COUNT]
) const
{
    double totalDepth =
        0.0;

    for (
        uint16_t treeIndex = 0;
        treeIndex < PIEZO_IF_TREE_COUNT;
        treeIndex++
    )
    {
        uint32_t treeBase =
            PIEZO_IF_TREE_OFFSETS[
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
                PIEZO_IF_FEATURE[
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
                            PIEZO_IF_N_NODE_SAMPLES[
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
                PIEZO_IF_THRESHOLD[
                    globalNode
                ];

            if (
                value <= threshold
            )
            {
                node =
                    PIEZO_IF_CHILDREN_LEFT[
                        globalNode
                    ];
            }
            else
            {
                node =
                    PIEZO_IF_CHILDREN_RIGHT[
                        globalNode
                    ];
            }

            depth++;
        }
    }

    float denominator =
        static_cast<float>(
            PIEZO_IF_TREE_COUNT
        )
        *
        averagePathLength(
            PIEZO_IF_MAX_SAMPLES
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
            PIEZO_IF_OFFSET
        )
    );
}

// ============================================================
// ONE-CLASS SVM DECISION FUNCTION
//
// RBF kernel:
//
// K(x, sv) = exp(-gamma * ||x-sv||^2)
//
// sklearn decision:
//
// sum_i dual_coef[i] * K(x, SV_i)
// + intercept_
//
// decision >= 0 -> normal
// decision <  0 -> anomaly
// ============================================================

float PiezoInference::oneClassSVMDecisionFunction(
    const float scaled[PIEZO_MODEL_FEATURE_COUNT]
) const
{
    double decision =
        static_cast<double>(
            PIEZO_OCSVM_INTERCEPT
        );

    for (
        uint16_t svIndex = 0;
        svIndex < PIEZO_OCSVM_SUPPORT_VECTOR_COUNT;
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
            PIEZO_MODEL_FEATURE_COUNT;

        for (
            uint16_t featureIndex = 0;
            featureIndex < PIEZO_MODEL_FEATURE_COUNT;
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
                    PIEZO_OCSVM_SUPPORT_VECTORS[
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
                    PIEZO_OCSVM_GAMMA
                )
                *
                squaredDistance
            );

        decision +=
            static_cast<double>(
                PIEZO_OCSVM_DUAL_COEF[
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

bool PiezoInference::predict(
    const PiezoFeatures &features,
    PiezoInferenceResult &result
) const
{
    result =
        PiezoInferenceResult{};

    float rawFeatures[
        PIEZO_MODEL_FEATURE_COUNT
    ];

    float scaledFeatures[
        PIEZO_MODEL_FEATURE_COUNT
    ];

    featuresToArray(
        features,
        rawFeatures
    );

    for (
        uint16_t i = 0;
        i < PIEZO_MODEL_FEATURE_COUNT;
        i++
    )
    {
        if (
            !isfinite(
                rawFeatures[i]
            )
        )
        {
            return false;
        }
    }

    applyScaler(
        rawFeatures,
        scaledFeatures
    );

    result.isolationForestDecision =
        isolationForestDecisionFunction(
            scaledFeatures
        );

    result.oneClassSVMDecision =
        oneClassSVMDecisionFunction(
            scaledFeatures
        );

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

    result.respirationBPM =
        features.respirationBPM;

    result.valid =
        true;

    return true;
}
