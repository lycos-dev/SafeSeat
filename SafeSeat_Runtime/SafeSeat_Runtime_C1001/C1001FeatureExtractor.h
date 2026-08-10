#pragma once

#include <Arduino.h>

#include "C1001ModelData.h"

// Training manifest source of truth:
// sampling rate = 1 Hz
// window = 30 seconds
// overlap = 50%
// stride = 15 samples

constexpr uint16_t C1001_ML_WINDOW_SAMPLES = 30;
constexpr uint16_t C1001_ML_STRIDE_SAMPLES = 15;

struct C1001FeatureVector
{
    bool valid = false;

    float values[
        C1001_MODEL_FEATURE_COUNT
    ] = {0.0f};
};

class C1001FeatureExtractor
{
public:
    C1001FeatureExtractor() = default;

    bool extract(
        const float rawHeartRate[C1001_ML_WINDOW_SAMPLES],
        const float rawRespiration[C1001_ML_WINDOW_SAMPLES],
        C1001FeatureVector &output
    ) const;

private:
    static constexpr uint8_t
        INTERPOLATION_LIMIT = 5;

    void prepareSignal(
        const float raw[C1001_ML_WINDOW_SAMPLES],
        float prepared[C1001_ML_WINDOW_SAMPLES]
    ) const;

    bool isInvalidCode(
        float value
    ) const;

    float quantile(
        const float sortedValues[C1001_ML_WINDOW_SAMPLES],
        uint16_t count,
        float q
    ) const;

    float medianOfValues(
        const float values[C1001_ML_WINDOW_SAMPLES],
        uint16_t count
    ) const;

    float slope(
        const float values[C1001_ML_WINDOW_SAMPLES]
    ) const;

    float lag1Autocorrelation(
        const float values[C1001_ML_WINDOW_SAMPLES]
    ) const;

    void statisticalFeatures(
        const float prepared[C1001_ML_WINDOW_SAMPLES],
        float output[22]
    ) const;

    void rangeFeatures(
        const float raw[C1001_ML_WINDOW_SAMPLES],
        float validMinimum,
        float validMaximum,
        float normalMinimum,
        float normalMaximum,
        float output[8]
    ) const;

    void crossFeatures(
        const float rrPrepared[C1001_ML_WINDOW_SAMPLES],
        const float hrPrepared[C1001_ML_WINDOW_SAMPLES],
        float output[4]
    ) const;

    uint16_t longestTrueRun(
        const bool mask[C1001_ML_WINDOW_SAMPLES]
    ) const;
};
