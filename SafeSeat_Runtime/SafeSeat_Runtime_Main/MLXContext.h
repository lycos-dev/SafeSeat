#pragma once

#include <Arduino.h>

#include "MLX.h"


// ============================================================
// SAFESEAT MLX CONTEXT EVIDENCE - STEP 5.4.5
//
// Purpose:
// - Use the MLX90614 filtered OBJECT temperature as the primary
//   temperature signal.
// - Use MLX Ta (readAmbientTempC) only as thermal/sensor context.
// - Do NOT interpret Object-Ta as body temperature.
// - Build an occupant/session-specific object-temperature
//   baseline after a qualified warm optical target is present.
// - Compare current filtered object temperature against that
//   baseline as CONTEXT only.
//
// The old WESAD IF/OCSVM model is retained elsewhere for
// diagnostics, but Step 5.4.5 intentionally does NOT use its
// anomaly vote in Fusion because Step 5.4.3 demonstrated a
// severe contact-E4 vs non-contact-MLX domain mismatch.
// ============================================================


enum class MLXContextStatus
{
    UNAVAILABLE,
    WAITING_FOR_THERMAL_TARGET,
    BUILDING_BASELINE,
    STABLE,
    CONTEXT_CHANGE
};


struct MLXContextReading
{
    bool available = false;
    bool valid = false;

    // Optical/thermal quality gate only. This is NOT a body-
    // temperature calculation and is NOT a medical threshold.
    bool thermalContrastQualified = false;

    bool baselineReady = false;
    bool contextChange = false;

    float filteredObjectC = NAN;
    float sensorTaC = NAN;
    float thermalContrastC = NAN;

    float baselineObjectC = NAN;
    float deviationFromBaselineC = NAN;

    uint16_t baselineSamplesCollected = 0;
    uint16_t baselineSamplesRequired = 120;

    unsigned long lastProcessedAcceptedSampleCount = 0;
    unsigned long lastUpdateMillis = 0;

    MLXContextStatus status =
        MLXContextStatus::UNAVAILABLE;
};


class MLXContext
{
public:
    MLXContext();

    void begin();

    void update(
        const MLXReading& sensorReading
    );

    const MLXContextReading&
    getReading() const;

    const char*
    getStatusText() const;

private:
    // Existing engineering qualification gate. It is retained
    // only to prevent building a skin-surface baseline while the
    // MLX is pointed at a room/background target.
    static constexpr float
        MIN_THERMAL_CONTRAST_C =
            2.0f;

    // MLX filtered acquisition is 4 Hz. 120 accepted samples =
    // 30 s, giving a robust session baseline before Fusion uses
    // temperature context.
    static constexpr uint16_t
        BASELINE_SAMPLE_COUNT =
            120;

    // FDA non-contact IR dataset, Step 5.4.4:
    // p99 of within-subject repeated-round surface-temperature
    // RANGE = 1.8258 C across FLIR/ICI records.
    // Rounded to 1.85 C and used ONLY as a broad context-change
    // marker, not as a medical abnormal-temperature threshold.
    static constexpr float
        FDA_CONTEXT_CHANGE_DELTA_C =
            1.85f;

    MLXContextReading reading;

    float baselineBuffer[
        BASELINE_SAMPLE_COUNT
    ] = {0};

    uint16_t baselineCount = 0;

    void resetBaseline();

    static float median(
        float *values,
        uint16_t count
    );
};
