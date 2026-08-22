#pragma once

#include <Arduino.h>

#include "C1001.h"
#include "C1001FeatureExtractor.h"
#include "C1001Inference.h"

enum class C1001MLStatus
{
    WAITING_FOR_SAMPLE,
    WAITING_FOR_OCCUPANT,
    WAITING_FOR_WARMUP,
    INVALID_SAMPLE,
    MOTION_HOLD,
    COLLECTING_WINDOW,
    READY_NORMAL,
    READY_WEAK_ANOMALY,
    READY_STRONG_ANOMALY,
    INFERENCE_ERROR,

    // Appended to preserve existing status numeric values.
    REACQUISITION_HOLD
};

struct C1001MLReading
{
    bool modelAvailable = true;
    bool valid = false;

    C1001MLStatus status =
        C1001MLStatus::WAITING_FOR_SAMPLE;

    uint16_t windowSamplesCollected = 0;
    uint16_t windowSamplesRequired =
        C1001_ML_WINDOW_SAMPLES;

    uint16_t samplesUntilNextInference =
        C1001_ML_WINDOW_SAMPLES;

    uint32_t windowsEvaluated = 0;

    // Number of 1 Hz HR/RR samples held because C1001 radar
    // motion context indicated a likely artifact. Holding a
    // sample does NOT erase the existing ML window.
    uint32_t motionSamplesHeld = 0;

    uint32_t reacquisitionSamplesHeld = 0;

    uint32_t lastProcessedSampleSequence = 0;

    unsigned long lastInferenceMillis = 0;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class C1001ML
{
public:
    C1001ML() = default;

    void begin();

    // Call frequently from the main loop.
    //
    // A new value is consumed only when
    // C1001Reading.sampleSequence changes.
    void update(
        const C1001Reading &sensorReading
    );

    const C1001MLReading&
    getReading() const;

    const char*
    getStatusText() const;

private:
    C1001FeatureExtractor featureExtractor;
    C1001Inference inference;

    C1001MLReading reading;

    float hrWindow[
        C1001_ML_WINDOW_SAMPLES
    ] = {0.0f};

    float rrWindow[
        C1001_ML_WINDOW_SAMPLES
    ] = {0.0f};

    uint16_t windowCount = 0;

    uint16_t writeIndex = 0;

    uint16_t newSamplesSinceInference = 0;

    bool firstInferenceCompleted = false;

    void resetWindow(
        C1001MLStatus status
    );

    void addSample(
        float heartRate,
        float respiration
    );

    void copyOrderedWindow(
        float heartRate[C1001_ML_WINDOW_SAMPLES],
        float respiration[C1001_ML_WINDOW_SAMPLES]
    ) const;

    bool shouldHoldForMotion(
        const C1001Reading &sensorReading
    ) const;

    void runInference();
};
