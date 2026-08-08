#pragma once
#include <Arduino.h>
#include "Config.h"

enum class PiezoStatus { INITIALIZING, READY, INVALID_READING };

struct PiezoReading {
    bool valid=false;
    int rawADC=0;
    float filteredSignal=0.0f;
    float baseline=0.0f;
    float respirationWave=0.0f;
    bool breathDetectedThisSample=false;
    unsigned long totalBreaths=0;
    unsigned long lastBreathMillis=0;
    unsigned long noBreathDurationMs=0;
    bool noBreathTimerExceeded=false;
    float estimatedRespirationBPM=NAN;
    uint16_t windowSamplesAvailable=0;
    bool fullWindowReady=false;
    bool newFeatureWindowDue=false;
    unsigned long sampleCount=0;
    float actualSamplingRateHz=0.0f;
    PiezoStatus status=PiezoStatus::INITIALIZING;
};

class PiezoSensor {
public:
    PiezoSensor();
    bool begin();
    void update();
    const PiezoReading& getReading() const { return reading; }
    bool copyRespirationWindow(float destination[], uint16_t destinationSize) const;
    bool copyModelSourceWindow(float destination[], uint16_t destinationSize) const;
    void acknowledgeFeatureWindow() { reading.newFeatureWindowDue=false; }
private:
    PiezoReading reading;
    bool filterInitialized=false;
    float filteredSignal=0.0f;
    float baseline=0.0f;
    float previousWave=0.0f;
    unsigned long lastBreathTime=0;
    static constexpr uint8_t BREATH_HISTORY_SIZE=8;
    unsigned long breathTimes[BREATH_HISTORY_SIZE]{};
    uint8_t breathHistoryCount=0;
    uint8_t breathHistoryWriteIndex=0;
    float respirationBuffer[PIEZO_WINDOW_SAMPLES]{};
    float modelSourceBuffer[PIEZO_WINDOW_SAMPLES]{};
    uint16_t bufferWriteIndex=0;
    uint16_t bufferCount=0;
    uint16_t samplesSinceFeatureWindow=0;
    unsigned long lastSampleMillis=0;
    unsigned long acquisitionStartMillis=0;
    void processSample(int rawADC, unsigned long now);
    void updateAuxiliaryBreathing(float wave, unsigned long now);
    void recordBreath(unsigned long now);
    float calculateRollingRespirationBPM() const;
    void storeSample(float respirationWave, float rawModelSource);
    void updateSamplingDiagnostics(unsigned long now);
    bool copyWindow(const float source[], float destination[], uint16_t destinationSize) const;
};
