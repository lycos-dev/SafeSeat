#pragma once

#include <Arduino.h>

#include "MPU.h"
#include "MPUFeatureExtractor.h"
#include "MPUInference.h"

enum class MPUMLStatus
{
    WAITING_FOR_SAMPLE,
    CALIBRATING_STATIONARY_BASELINE,
    INVALID_SAMPLE,
    COLLECTING_WINDOW,
    READY_NORMAL,
    READY_WEAK_ROAD_MOTION,
    READY_STRONG_ROAD_MOTION,
    INFERENCE_ERROR
};

struct MPUMLReading
{
    bool modelAvailable = true;
    bool valid = false;

    MPUMLStatus status =
        MPUMLStatus::WAITING_FOR_SAMPLE;

    bool stationaryBaselineReady = false;

    uint16_t baselineSamplesCollected = 0;
    uint16_t baselineSamplesRequired =
        MPU_ML_BASELINE_SAMPLES;

    uint16_t windowSamplesCollected = 0;
    uint16_t windowSamplesRequired =
        MPU_ML_WINDOW_SAMPLES;

    uint16_t samplesUntilNextInference =
        MPU_ML_WINDOW_SAMPLES;

    uint32_t windowsEvaluated = 0;

    unsigned long lastInferenceMillis = 0;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;
    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class MPUML
{
public:
    MPUML() = default;

    void begin();

    void update(
        const MPUReading &sensorReading
    );

    const MPUMLReading&
    getReading() const;

    const char*
    getStatusText() const;

private:
    static constexpr float STANDARD_GRAVITY_MPS2 =
        9.80665f;

    static constexpr float DEG_TO_RAD_FACTOR =
        0.017453292519943295769236907684886f;

    MPUFeatureExtractor featureExtractor;
    MPUInference inference;

    MPUMLReading reading;

    MPUModelSample sampleWindow[
        MPU_ML_WINDOW_SAMPLES
    ];

    uint16_t windowCount = 0;
    uint16_t writeIndex = 0;
    uint16_t newSamplesSinceInference = 0;
    bool firstInferenceCompleted = false;

    unsigned long lastProcessedSampleCount = 0;

    double baselineAccelXSum = 0.0;
    double baselineAccelYSum = 0.0;
    double baselineAccelZSum = 0.0;

    double baselineGyroXSum = 0.0;
    double baselineGyroYSum = 0.0;
    double baselineGyroZSum = 0.0;

    float baselineAccelXG = 0.0f;
    float baselineAccelYG = 0.0f;
    float baselineAccelZG = 0.0f;

    float baselineGyroXDps = 0.0f;
    float baselineGyroYDps = 0.0f;
    float baselineGyroZDps = 0.0f;

    void resetWindow(
        MPUMLStatus status
    );

    void resetBaseline();

    void accumulateBaseline(
        const MPUReading &sensorReading
    );

    MPUModelSample normalizeSample(
        const MPUReading &sensorReading
    ) const;

    void addSample(
        const MPUModelSample &sample
    );

    void copyOrderedWindow(
        MPUModelSample output[
            MPU_ML_WINDOW_SAMPLES
        ]
    ) const;

    void runInference();
};
