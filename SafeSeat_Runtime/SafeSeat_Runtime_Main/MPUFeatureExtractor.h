#pragma once

#include <Arduino.h>
#include "MPUModelData.h"

struct MPUModelSample
{
    // Calibrated model-domain signals:
    // accel: m/s^2, stationary gravity/bias removed
    // gyro : rad/s, stationary bias removed
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
};

struct MPUFeatureVector
{
    float values[MPU_MODEL_FEATURE_COUNT];
};

class MPUFeatureExtractor
{
public:
    bool extract(
        const MPUModelSample samples[MPU_ML_WINDOW_SAMPLES],
        MPUFeatureVector &output
    ) const;

private:
    struct SignalStats
    {
        float delta = 0.0f;
        float energy = 0.0f;
        float first = 0.0f;
        float iqr = 0.0f;
        float lag1Autocorrelation = 0.0f;
        float last = 0.0f;
        float mad = 0.0f;
        float max = 0.0f;
        float maxAbsChange = 0.0f;
        float mean = 0.0f;
        float meanAbsChange = 0.0f;
        float median = 0.0f;
        float min = 0.0f;
        float q05 = 0.0f;
        float q25 = 0.0f;
        float q75 = 0.0f;
        float q95 = 0.0f;
        float range = 0.0f;
        float rms = 0.0f;
        float slope = 0.0f;
        float std = 0.0f;
        float variance = 0.0f;

        float jerkMaxAbs = 0.0f;
        float jerkMeanAbs = 0.0f;
        float jerkRms = 0.0f;
        float jerkStd = 0.0f;
    };

    float signalValue(
        const MPUModelSample &sample,
        uint8_t signalIndex
    ) const;

    void sortValues(
        float values[MPU_ML_WINDOW_SAMPLES]
    ) const;

    float quantileSorted(
        const float sorted[MPU_ML_WINDOW_SAMPLES],
        float q
    ) const;

    float correlation(
        const float *left,
        const float *right,
        uint16_t count
    ) const;

    bool computeStats(
        const MPUModelSample samples[MPU_ML_WINDOW_SAMPLES],
        uint8_t signalIndex,
        bool computeJerk,
        SignalStats &output
    ) const;
};
