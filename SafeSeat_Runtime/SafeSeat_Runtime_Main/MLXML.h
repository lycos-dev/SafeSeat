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
    TARGET_CONTRAST_DEGRADED,
    UNSTABLE_TARGET,
    BUILDING_BASELINE,
    BASELINE_READY,
    READY_NORMAL,
    READY_WEAK_ANOMALY,
    READY_STRONG_ANOMALY,
    TARGET_GEOMETRY_DEGRADED,
    REACQUIRING_TARGET,
    ANOMALY_CANDIDATE,
    INFERENCE_ERROR
};

struct MLXMLReading
{
    bool modelAvailable = true;
    bool valid = false;

    bool seatOccupied = false;

    // Compatibility fields used by existing dashboard/API.
    // warmTargetQualified now means the OCCUPANT SESSION is
    // qualified by occupancy, not by Object-Ta.
    // targetContrastDegraded is LOW-CONTRAST CONTEXT ONLY.
    bool warmTargetQualified = false;
    bool targetContrastDegraded = false;
    bool stabilityQualified = false;
    bool baselineReady = false;

    // Post-baseline target/FOV quality state. A rapid step away
    // from the trusted nape baseline is treated as geometry loss,
    // not as immediate physiological anomaly evidence.
    bool geometryDegraded = false;
    bool reacquiring = false;

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
    uint32_t geometryEvents = 0;

    uint8_t geometryReacquireStableBlocks = 0;
    uint8_t geometryReacquireRequiredBlocks = 3;

    uint8_t anomalyCandidateBlocks = 0;
    uint8_t anomalyPersistenceRequiredBlocks = 3;

    uint8_t lowContrastBlocks = 0;
    uint8_t targetLossGraceBlocks = 3;

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
    void update(const MLXReading &sensorReading, bool seatOccupied);

    const MLXMLReading& getReading() const;
    const char* getStatusText() const;

private:
    static constexpr uint8_t BLOCK_SAMPLES = 4;
    static constexpr uint8_t BASELINE_BLOCK_COUNT = 30;

    // Combined-runtime quality policy, 2026-08-23:
    //
    // Object-Ambient is NOT a hard target gate anymore.
    // FSR/C1001 occupancy establishes the occupant session.
    // Ambient and Object-Ta remain diagnostic/context only.
    //
    // The filtered signal still has a conservative transition
    // guard so abrupt FOV changes are not learned into baseline.
    static constexpr float HIGH_CONTRAST_CONTEXT_C = 2.0f;
    static constexpr float MAX_ONE_SECOND_OBJECT_STD_C = 1.00f;

    // FOV/geometry artifact guard. Physiological surface
    // temperature should not step by ~1 C in a single second or
    // ~1.4 C in two seconds. Such rapid shifts are quarantined
    // until the original nape geometry is reacquired.
    static constexpr float GEOMETRY_ONE_SEC_STEP_C = 0.90f;
    static constexpr float GEOMETRY_TWO_SEC_STEP_C = 1.40f;
    static constexpr float GEOMETRY_MIN_BASELINE_DEVIATION_C = 1.25f;
    static constexpr float GEOMETRY_REACQUIRE_BAND_C = 1.00f;
    static constexpr uint8_t GEOMETRY_REACQUIRE_BLOCKS = 3;

    // MLX is supporting evidence. A single anomalous 1-s block
    // must never become a Fusion vote.
    static constexpr uint8_t ANOMALY_PERSISTENCE_BLOCKS = 3;

    MLXNativeInference inference;
    MLXMLReading reading;

    float blockObject[BLOCK_SAMPLES] = {0.0f};
    float blockAmbient[BLOCK_SAMPLES] = {0.0f};
    uint8_t blockCount = 0;

    float baselineBlocks[BASELINE_BLOCK_COUNT] = {0.0f};
    uint8_t baselineCount = 0;

    bool targetLatched = false; // occupancy-session latch
    uint8_t lowContrastBlockCount = 0; // diagnostic only

    bool geometryHold = false;
    uint8_t geometryReacquireCount = 0;
    uint8_t consecutiveAnomalyBlocks = 0;

    float previousStableObjectC = NAN;
    float twoBlocksAgoStableObjectC = NAN;

    void clearDecision();
    void resetSessionBaseline(MLXMLStatus status);
    void consumeAcceptedSample(float objectC, float ambientC);
    void processOneSecondBlock();

    static float mean4(const float values[BLOCK_SAMPLES]);
    static float std4(const float values[BLOCK_SAMPLES], float mean);
    static float median30(const float values[BASELINE_BLOCK_COUNT]);
};
