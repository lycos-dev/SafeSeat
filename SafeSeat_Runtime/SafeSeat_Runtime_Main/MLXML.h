#pragma once

#include <Arduino.h>

#include "MLX.h"
#include "MLXNativeInference.h"

// ============================================================
// SAFESEAT MLX90614 NATIVE EXTERNAL MODEL - FINAL RUNTIME
// 2026-08-23
//
// Replaces the retired WESAD / Empatica E4 surrogate.
//
// Training source:
//   CorrectionForeheadTemperature human MLX90614 dataset.
//
// Deployment design:
//   - physical MLX acquisition remains 4 Hz
//   - 4 accepted samples -> one 1-second measurement block
//   - block must be thermally qualified and stable
//   - 30 stable one-second blocks establish a session baseline
//   - IF + OCSVM then evaluate every stable one-second block
//
// MODEL FEATURES ONLY:
//   1) object temperature - personal/session baseline
//   2) absolute value of feature 1
//
// NOT MODEL FEATURES:
//   - MLX ambient temperature (Ta)
//   - Object - ambient
//   - 1-second measurement standard deviation
//
// Those signals remain runtime quality/fusion context only.
// ============================================================

enum class MLXMLStatus
{
    WAITING_FOR_SAMPLE,
    SENSOR_UNAVAILABLE,
    WAITING_FOR_WARM_TARGET,
    UNSTABLE_TARGET,
    BUILDING_BASELINE,
    BASELINE_READY,
    READY_NORMAL,
    READY_WEAK_ANOMALY,
    READY_STRONG_ANOMALY,
    INFERENCE_ERROR
};

struct MLXMLReading
{
    bool modelAvailable = true;
    bool valid = false;

    bool warmTargetQualified = false;
    bool stabilityQualified = false;
    bool baselineReady = false;

    float targetDeltaC = NAN;
    float oneSecondObjectMeanC = NAN;
    float oneSecondAmbientMeanC = NAN;
    float oneSecondObjectStdC = NAN;

    float baselineObjectC = NAN;
    float deviationFromBaselineC = NAN;

    MLXMLStatus status = MLXMLStatus::WAITING_FOR_SAMPLE;

    uint8_t baselineBlocksCollected = 0;
    uint8_t baselineBlocksRequired = 30;

    uint32_t evaluatedBlocks = 0;
    uint32_t unstableBlocksHeld = 0;
    uint32_t targetLosses = 0;

    unsigned long lastProcessedAcceptedSampleCount = 0;
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
    void update(const MLXReading &sensorReading);

    const MLXMLReading& getReading() const;
    const char* getStatusText() const;

private:
    static constexpr uint8_t BLOCK_SAMPLES = 4;
    static constexpr uint8_t BASELINE_BLOCK_COUNT = 30;

    // Engineering target/quality gates; non-medical.
    static constexpr float WARM_TARGET_MIN_DELTA_C = 2.0f;
    static constexpr float MAX_ONE_SECOND_OBJECT_STD_C = 0.50f;

    MLXNativeInference inference;
    MLXMLReading reading;

    float blockObject[BLOCK_SAMPLES] = {0.0f};
    float blockAmbient[BLOCK_SAMPLES] = {0.0f};
    uint8_t blockCount = 0;

    float baselineBlocks[BASELINE_BLOCK_COUNT] = {0.0f};
    uint8_t baselineCount = 0;

    void clearDecision();
    void resetSessionBaseline(MLXMLStatus status);
    void consumeAcceptedSample(float objectC, float ambientC);
    void processOneSecondBlock();

    static float mean4(const float values[BLOCK_SAMPLES]);
    static float std4(const float values[BLOCK_SAMPLES], float mean);
    static float median30(const float values[BASELINE_BLOCK_COUNT]);
};
