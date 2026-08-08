#pragma once

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>


// ============================================================
// SAFESEAT FSR PHYSICAL LAYOUT
//
// BACKREST
//
// FSR1        FSR4
// FSR2        FSR5
// FSR3        FSR6
//
// CUSHION
//
// FSR1        FSR2        FSR3
//
// Internally:
//
// 0 = Backrest FSR1
// 1 = Backrest FSR2
// 2 = Backrest FSR3
// 3 = Backrest FSR4
// 4 = Backrest FSR5
// 5 = Backrest FSR6
// 6 = Cushion FSR1
// 7 = Cushion FSR2
// 8 = Cushion FSR3
// ============================================================

enum FSRIndex
{
    BACKREST_FSR1 = 0,
    BACKREST_FSR2 = 1,
    BACKREST_FSR3 = 2,

    BACKREST_FSR4 = 3,
    BACKREST_FSR5 = 4,
    BACKREST_FSR6 = 5,

    CUSHION_FSR1 = 6,
    CUSHION_FSR2 = 7,
    CUSHION_FSR3 = 8
};


constexpr int NUM_FSR = 9;


// ============================================================
// FSR STATUS
// ============================================================

enum class FSRStatus
{
    DISCONNECTED,

    CALIBRATING,

    READY,

    READING,

    INVALID_READING
};


// ============================================================
// FSR READING
// ============================================================

struct FSRReading
{
    bool connected = false;

    bool calibrated = false;

    bool valid = false;


    // --------------------------------------------------------
    // Raw ADC values
    // --------------------------------------------------------

    int16_t raw[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Adaptive filtered ADC values
    // --------------------------------------------------------

    float filtered[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Empty-seat calibration baseline
    // --------------------------------------------------------

    float baseline[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Baseline-subtracted pressure response
    //
    // This is the primary runtime pressure representation.
    // --------------------------------------------------------

    float pressure[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Group features
    // --------------------------------------------------------

    float backrestLeftTotal = 0.0f;

    float backrestRightTotal = 0.0f;


    float backrestUpperTotal = 0.0f;

    float backrestMiddleTotal = 0.0f;

    float backrestLowerTotal = 0.0f;


    float cushionLeft = 0.0f;

    float cushionCenter = 0.0f;

    float cushionRight = 0.0f;


    float backrestTotal = 0.0f;

    float cushionTotal = 0.0f;

    float wholeSeatTotal = 0.0f;


    // --------------------------------------------------------
    // Distribution features
    //
    // Same concepts used during FSR feature engineering.
    // --------------------------------------------------------

    float backrestLRBalance = 0.0f;

    float cushionLRBalance = 0.0f;

    float cushionCenterRatio = 0.0f;

    float backrestToCushionRatio = 0.0f;


    // --------------------------------------------------------
    // Per-channel pressure shares
    // --------------------------------------------------------

    float pressureShare[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Timing
    // --------------------------------------------------------

    unsigned long lastSampleMillis = 0;

    float actualSamplingRateHz = 0.0f;


    FSRStatus status =
        FSRStatus::DISCONNECTED;
};


// ============================================================
// FSR SENSOR CLASS
// ============================================================

class FSRSensor
{
public:

    FSRSensor();


    bool begin();


    /*
     * occupantPresent comes from the C1001.
     *
     * It is used ONLY to determine whether empty-seat baseline
     * drift correction is safe.
     *
     * It is NOT used as an ML label.
     */
    void update(
        bool occupantPresent
    );


    const FSRReading&
    getReading() const;


    bool hasValidReading() const;


    const char*
    getStatusText() const;


    const char*
    getSensorLabel(
        int index
    ) const;


private:

    Adafruit_ADS1115 ads1;

    Adafruit_ADS1115 ads2;


    FSRReading reading;


    // ========================================================
    // SAMPLE TIMING
    //
    // ChairPose-derived FSR feature engineering used
    // approximately 12.65 Hz.
    //
    // 80 ms corresponds to ~12.5 Hz.
    //
    // NOTE:
    // Actual rate is measured because ADS1115 reads are
    // blocking and can affect the true sample period.
    // ========================================================

    static constexpr unsigned long
        TARGET_SAMPLE_INTERVAL_MS = 80UL;


    unsigned long previousCompletedSample =
        0;


    // ========================================================
    // MEDIAN FILTER
    //
    // Preserved from your tested FSR implementation.
    // ========================================================

    static constexpr int
        MEDIAN_SAMPLES = 5;


    // ========================================================
    // EMPTY-SEAT CALIBRATION
    // ========================================================

    static constexpr int
        CALIBRATION_ROUNDS = 20;


    static constexpr unsigned long
        CALIBRATION_DELAY_MS = 3000UL;


    // ========================================================
    // BASELINE / NOISE
    // ========================================================

    static constexpr float
        NOISE_MARGIN = 20.0f;


    /*
     * Slow baseline adaptation.
     *
     * Only allowed when:
     *     C1001 says no occupant
     * AND
     *     current pressure response remains small.
     */
    static constexpr float
        BASELINE_ADAPT_ALPHA = 0.003f;


    static constexpr float
        BASELINE_ADAPT_MAX_TOTAL = 500.0f;


    // ========================================================
    // SENSOR LABELS
    // ========================================================

    static const char*
        SENSOR_LABELS[NUM_FSR];


    // ========================================================
    // LOW-LEVEL READS
    // ========================================================

    int16_t readMedianADS(
        Adafruit_ADS1115& module,
        uint8_t channel
    );


    int16_t readMedianNative(
        uint8_t pin
    );


    bool readAllSensors(
        int16_t destination[]
    );


    // ========================================================
    // FILTER
    // ========================================================

    float applyAdaptiveFilter(
        int16_t raw,
        float previousFiltered
    );


    // ========================================================
    // BASELINE
    // ========================================================

    bool calibrateEmptySeat();


    float calculatePressureDelta(
        float filtered,
        float baseline
    );


    void updateBaselineWhenEmpty(
        bool occupantPresent
    );


    // ========================================================
    // FEATURE CALCULATION
    // ========================================================

    void calculatePressureFeatures();


    void calculatePressureShares();


    // ========================================================
    // VALIDATION
    // ========================================================

    bool readingsAreValid(
        const int16_t values[]
    ) const;
};