#pragma once

#include <Arduino.h>
#include "Config.h"

enum class PiezoStatus
{
    INITIALIZING,
    READY,
    INVALID_READING
};

struct PiezoReading
{
    bool valid = false;
    bool signalUsable = false;

    int rawADC = 0;

    float filteredSignal = 0.0f;
    float baseline = 0.0f;
    float respirationWave = 0.0f;

    bool breathDetectedThisSample = false;
    bool breathTrackingReady = false;
    bool breathDetectedRecently = false;

    unsigned long totalBreaths = 0;
    unsigned long lastBreathMillis = 0;
    unsigned long noBreathDurationMs = 0;

    // Conservative support flag only. Fusion never treats this as
    // a standalone emergency vote.
    bool noBreathTimerExceeded = false;

    float estimatedRespirationBPM = NAN;

    unsigned long sampleCount = 0;
    float actualSamplingRateHz = 0.0f;

    PiezoStatus status = PiezoStatus::INITIALIZING;
};

class PiezoSensor
{
public:
    PiezoSensor();

    bool begin();
    void update();

    const PiezoReading& getReading() const
    {
        return reading;
    }

private:
    PiezoReading reading{};

    bool filterInitialized = false;
    float filteredSignal = 0.0f;
    float baseline = 0.0f;
    float previousAbsoluteWave = 0.0f;

    unsigned long acquisitionStartMillis = 0;
    unsigned long lastSampleMillis = 0;
    unsigned long lastBreathTime = 0;

    static constexpr uint8_t BREATH_HISTORY_SIZE = 8;
    unsigned long breathTimes[BREATH_HISTORY_SIZE]{};
    uint8_t breathHistoryCount = 0;
    uint8_t breathHistoryWriteIndex = 0;

    void processSample(
        int rawADC,
        unsigned long now
    );

    void updateBreathingSupport(
        float wave,
        unsigned long now
    );

    void recordBreath(
        unsigned long now
    );

    float calculateRollingRespirationBPM() const;

    void updateSamplingDiagnostics(
        unsigned long now
    );
};
