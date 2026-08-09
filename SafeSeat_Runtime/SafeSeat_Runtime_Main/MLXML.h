#pragma once

#include <Arduino.h>

#include "MLX.h"
#include "MLXFeatureExtractor.h"
#include "MLXInference.h"

enum class MLXMLStatus
{
    WAITING_FOR_SAMPLE,
    SENSOR_UNAVAILABLE,
    WAITING_FOR_WARM_TARGET,
    COLLECTING_WINDOW,
    INSUFFICIENT_VALID_DATA,
    READY_NORMAL,
    READY_WEAK_ANOMALY,
    READY_STRONG_ANOMALY,
    INFERENCE_ERROR
};

struct MLXMLReading
{
    bool modelAvailable = true;
    bool valid = false;

    // True only when the MLX sees an object clearly warmer than
    // ambient. This is a deployment/target qualification gate,
    // not a medical threshold.
    bool warmTargetQualified = false;

    float targetDeltaC = NAN;

    MLXMLStatus status =
        MLXMLStatus::WAITING_FOR_SAMPLE;

    uint16_t windowSamplesCollected = 0;
    uint16_t windowSamplesRequired =
        MLX_ML_WINDOW_SAMPLES;

    uint16_t samplesUntilNextInference =
        MLX_ML_WINDOW_SAMPLES;

    uint16_t lastFiniteSampleCount = 0;

    uint32_t windowsEvaluated = 0;

    unsigned long lastProcessedPhysicalSampleCount = 0;

    unsigned long lastInferenceMillis = 0;

    float isolationForestDecision = NAN;
    float oneClassSVMDecision = NAN;

    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;

    bool bothModelsAnomaly = false;
    bool eitherModelAnomaly = false;
};

class MLXML
{
public:
    MLXML() = default;

    void begin();

    // Call frequently from the main loop.
    //
    // A value is consumed only when the MLX physical sample
    // count (accepted + rejected) changes.
    void update(
        const MLXReading &sensorReading
    );

    const MLXMLReading&
    getReading() const;

    const char*
    getStatusText() const;

private:
    static constexpr uint16_t
        MIN_FINITE_SAMPLES_FOR_INFERENCE = 60;

    // MLX90614 sensor-spec physical output range. This is only
    // used to reject impossible hardware values before feature
    // extraction. The model's own 20..45 C data-validity
    // features remain inside MLXFeatureExtractor.
    static constexpr float
        PHYSICAL_OBJECT_MIN_C = -40.0f;

    static constexpr float
        PHYSICAL_OBJECT_MAX_C = 125.0f;

    // Provisional bench/deployment gate. The current hardware log
    // showed background near -0.7..0 C delta and a forehead target
    // around +5..+8 C, so +2 C cleanly rejects obvious room/background
    // windows. This MUST be revalidated with the final seat/headrest
    // geometry. It is not a medical temperature threshold.
    static constexpr float
        WARM_TARGET_MIN_DELTA_C = 2.0f;

    MLXFeatureExtractor featureExtractor;
    MLXInference inference;

    MLXMLReading reading;

    float objectWindow[
        MLX_ML_WINDOW_SAMPLES
    ] = {0.0f};

    uint16_t windowCount = 0;

    uint16_t writeIndex = 0;

    uint16_t newSamplesSinceInference = 0;

    bool firstInferenceCompleted = false;

    void resetWindow(
        MLXMLStatus status
    );

    void addSample(
        float objectTemperature
    );

    void copyOrderedWindow(
        float objectTemperature[MLX_ML_WINDOW_SAMPLES]
    ) const;

    void runInference();
};
