#pragma once

#include <Arduino.h>

#include "FSR.h"
#include "FSRFeatureExtractor.h"
#include "FSRInference.h"

enum class FSRMLStatus
{
    WAITING_FOR_SAMPLE,
    WAITING_FOR_OCCUPANT,
    INVALID_SAMPLE,
    COLLECTING_WINDOW,
    READY_NORMAL,
    READY_WEAK_ANOMALY,
    READY_STRONG_ANOMALY,
    INFERENCE_ERROR
};

struct FSRMLReading
{
    bool modelAvailable = true;
    bool valid = false;

    FSRMLStatus status =
        FSRMLStatus::WAITING_FOR_SAMPLE;

    uint16_t windowSamplesCollected = 0;
    uint16_t windowSamplesRequired =
        FSR_ML_WINDOW_SAMPLES;

    uint16_t samplesUntilNextInference =
        FSR_ML_WINDOW_SAMPLES;

    uint32_t windowsEvaluated = 0;

    unsigned long lastProcessedSampleMillis = 0;
    unsigned long lastInferenceMillis = 0;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;
    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class FSRML
{
public:
    FSRML() = default;

    void begin();

    // occupantPresent is independent C1001 occupancy context.
    // The FSR model itself still receives only FSR pressures.
    void update(
        const FSRReading &sensorReading,
        bool occupantPresent
    );

    const FSRMLReading&
    getReading() const;

    const char*
    getStatusText() const;

private:
    FSRFeatureExtractor featureExtractor;
    FSRInference inference;

    FSRMLReading reading;

    float pressureWindow[
        FSR_ML_WINDOW_SAMPLES
    ][
        NUM_FSR
    ] = {{0.0f}};

    uint16_t windowCount = 0;
    uint16_t writeIndex = 0;
    uint16_t newSamplesSinceInference = 0;

    bool firstInferenceCompleted = false;

    void resetWindow(
        FSRMLStatus status
    );

    void addSample(
        const FSRReading &sensorReading
    );

    void copyOrderedWindow(
        float output[
            FSR_ML_WINDOW_SAMPLES
        ][
            NUM_FSR
        ]
    ) const;

    bool sampleIsUsable(
        const FSRReading &sensorReading,
        bool occupantPresent
    ) const;

    void runInference();
};
