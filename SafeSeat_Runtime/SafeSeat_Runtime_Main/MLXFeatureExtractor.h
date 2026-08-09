#pragma once

#include <Arduino.h>

#include "MLXModelData.h"

// Training manifest source of truth:
// sampling rate = 4 Hz
// window = 30 seconds
// overlap = 50%
// stride = 60 samples
// feature reference = per-window median-centered object temperature

constexpr uint16_t MLX_ML_WINDOW_SAMPLES = 120;
constexpr uint16_t MLX_ML_STRIDE_SAMPLES = 60;

struct MLXFeatureVector
{
    bool valid = false;

    uint16_t finiteSampleCount = 0;

    float values[
        MLX_MODEL_FEATURE_COUNT
    ] = {0.0f};
};

class MLXFeatureExtractor
{
public:
    MLXFeatureExtractor() = default;

    bool extract(
        const float rawObjectTemperature[MLX_ML_WINDOW_SAMPLES],
        MLXFeatureVector &output
    ) const;

private:
    static constexpr float
        SAMPLING_RATE_HZ = 4.0f;

    static constexpr uint8_t
        INTERPOLATION_LIMIT = 8;

    // These are the broad data-validity bounds from
    // config/mlx_config.json. They are preprocessing bounds,
    // NOT emergency or medical thresholds.
    static constexpr float
        TRAINING_VALID_MIN_C = 20.0f;

    static constexpr float
        TRAINING_VALID_MAX_C = 45.0f;

    void interpolateTemperature(
        const float raw[MLX_ML_WINDOW_SAMPLES],
        float cleaned[MLX_ML_WINDOW_SAMPLES]
    ) const;

    uint16_t collectFinite(
        const float values[MLX_ML_WINDOW_SAMPLES],
        float finiteValues[MLX_ML_WINDOW_SAMPLES]
    ) const;

    void sortValues(
        float values[MLX_ML_WINDOW_SAMPLES],
        uint16_t count
    ) const;

    float quantileSorted(
        const float sortedValues[MLX_ML_WINDOW_SAMPLES],
        uint16_t count,
        float q
    ) const;

    float medianSorted(
        const float sortedValues[MLX_ML_WINDOW_SAMPLES],
        uint16_t count
    ) const;

    float autocorrelation(
        const float values[MLX_ML_WINDOW_SAMPLES],
        uint8_t lag
    ) const;

    float linearSlope(
        const float values[MLX_ML_WINDOW_SAMPLES]
    ) const;

    float linearResidualStd(
        const float values[MLX_ML_WINDOW_SAMPLES]
    ) const;

    uint16_t longestInvalidRun(
        const float raw[MLX_ML_WINDOW_SAMPLES]
    ) const;
};
