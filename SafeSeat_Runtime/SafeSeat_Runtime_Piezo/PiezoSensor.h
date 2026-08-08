#pragma once

#include <Arduino.h>

#include "Config.h"


// ============================================================
// PIEZO STATUS
// ============================================================

enum class PiezoStatus
{
    INITIALIZING,

    READY,

    INVALID_READING
};


// ============================================================
// PIEZO READING
// ============================================================

struct PiezoReading
{
    bool valid = false;


    // --------------------------------------------------------
    // ADC / filtered waveform
    // --------------------------------------------------------

    int rawADC = 0;

    float filteredSignal = 0.0f;

    float baseline = 0.0f;

    float respirationWave = 0.0f;


    // --------------------------------------------------------
    // Auxiliary breath detection
    //
    // Not ML classification.
    // --------------------------------------------------------

    bool breathDetectedThisSample = false;

    unsigned long totalBreaths = 0;

    unsigned long lastBreathMillis = 0;

    unsigned long noBreathDurationMs = 0;

    bool noBreathTimerExceeded = false;


    // --------------------------------------------------------
    // Rolling respiratory-rate estimate
    //
    // This is for diagnostics / fusion context only.
    // --------------------------------------------------------

    float estimatedRespirationBPM = NAN;


    // --------------------------------------------------------
    // Buffer state
    // --------------------------------------------------------

    uint16_t windowSamplesAvailable = 0;

    bool fullWindowReady = false;

    bool newFeatureWindowDue = false;


    // --------------------------------------------------------
    // Sampling diagnostics
    // --------------------------------------------------------

    unsigned long sampleCount = 0;

    float actualSamplingRateHz = 0.0f;


    PiezoStatus status =
        PiezoStatus::INITIALIZING;
};


// ============================================================
// PIEZO SENSOR
// ============================================================

class PiezoSensor
{
public:

    PiezoSensor();


    bool begin();


    /*
     * Call continuously.
     *
     * Internally sampled at exactly approximately 25 Hz.
     */
    void update();


    const PiezoReading&
    getReading() const;


    /*
     * Copy the chronological 30-second respiration window.
     *
     * Returns false until 750 valid samples exist.
     */
    bool copyRespirationWindow(
        float destination[],
        uint16_t destinationSize
    ) const;


    /*
     * Marks the current 5-second feature interval as consumed.
     *
     * Later PiezoFeatureExtractor will call this after
     * generating one feature vector.
     */
    void acknowledgeFeatureWindow();


private:

    PiezoReading reading;


    // ========================================================
    // FILTER STATE
    // ========================================================

    bool filterInitialized = false;

    float filteredSignal = 0.0f;

    float baseline = 0.0f;

    float previousWave = 0.0f;


    // ========================================================
    // PEAK DETECTION
    // ========================================================

    bool rising = false;

    unsigned long lastBreathTime = 0;


    /*
     * Breath timestamps allow a more useful rolling BPM
     * estimate than the old once-per-minute breath counter.
     */
    static constexpr uint8_t
        MAX_BREATH_TIMESTAMPS = 20;


    unsigned long breathTimes[
        MAX_BREATH_TIMESTAMPS
    ] = {0};


    uint8_t breathTimeCount = 0;

    uint8_t breathTimeWriteIndex = 0;


    // ========================================================
    // 30-SECOND RING BUFFER
    // ========================================================

    float respirationBuffer[
        PIEZO_WINDOW_SAMPLES
    ] = {0};


    uint16_t bufferWriteIndex = 0;

    uint16_t bufferCount = 0;


    /*
     * Number of samples collected since the last model feature
     * window was issued.
     */
    uint16_t samplesSinceFeatureWindow = 0;


    // ========================================================
    // TIMING
    // ========================================================

    unsigned long lastSampleMillis = 0;

    unsigned long previousCompletedSampleMillis = 0;


    // ========================================================
    // HELPERS
    // ========================================================

    bool isADCValid(
        int raw
    ) const;


    void initializeFilter(
        int raw
    );


    void processSignal(
        int raw
    );


    void processBreathDetection();


    void storeRespirationSample(
        float value
    );


    void storeBreathTimestamp(
        unsigned long timestamp
    );


    void updateRespirationRateEstimate();


    void updateNoBreathState();
};