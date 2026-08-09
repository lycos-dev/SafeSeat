#pragma once

#include <Arduino.h>
#include "FSR.h"
#include "FSRModelData.h"

struct FSRFeatureVector
{
    float values[FSR_MODEL_FEATURE_COUNT] = {0.0f};
};

class FSRFeatureExtractor
{
public:
    bool extract(
        const float pressureWindow[FSR_ML_WINDOW_SAMPLES][NUM_FSR],
        FSRFeatureVector &output
    ) const;

private:
    static constexpr double EPSILON = 1.0e-6;

    double percentile(
        const double values[FSR_ML_WINDOW_SAMPLES],
        double quantile
    ) const;

    void summarize(
        const double values[FSR_ML_WINDOW_SAMPLES],
        bool includeMeanAbsDiff,
        float output[],
        uint16_t &index
    ) const;

    void summarizeMeanStdRange(
        const double values[FSR_ML_WINDOW_SAMPLES],
        float output[],
        uint16_t &index
    ) const;

    void summarizeMeanStd(
        const double values[FSR_ML_WINDOW_SAMPLES],
        float output[],
        uint16_t &index
    ) const;

    double meanAbsDiff(
        const double values[FSR_ML_WINDOW_SAMPLES]
    ) const;
};
