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
// Mapping to the proven combined sketch:
//
// BACKREST_FSR1 = BackLeftTop
// BACKREST_FSR2 = BackLeftMiddle
// BACKREST_FSR3 = BackLeftBottom
// BACKREST_FSR4 = BackRightTop
// BACKREST_FSR5 = BackRightMiddle
// BACKREST_FSR6 = BackRightBottom
// CUSHION_FSR1  = CushionLeft
// CUSHION_FSR2  = CushionCenter
// CUSHION_FSR3  = CushionRight
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
// STATUS
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
// READING
// ============================================================

struct FSRReading
{
    bool connected = false;
    bool calibrated = false;
    bool valid = false;

    int16_t raw[NUM_FSR] = {0};

    float filtered[NUM_FSR] = {0};

    float baseline[NUM_FSR] = {0};

    float pressure[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Group / distribution features
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

    float backrestLRBalance = 0.0f;
    float cushionLRBalance = 0.0f;
    float cushionCenterRatio = 0.0f;
    float backrestToCushionRatio = 0.0f;

    float pressureShare[NUM_FSR] = {0};


    // --------------------------------------------------------
    // Old-prototype posture context
    //
    // These are retained as runtime diagnostics only.
    // ML inference later uses the trained FSR feature pipeline.
    // --------------------------------------------------------

    bool occupied = false;
    bool backContact = false;

    float horizontalCOP = 0.0f;
    float sideAsymmetry = 0.0f;

    char datasetOrient = 'a';
    char datasetLean = '-';


    // --------------------------------------------------------
    // Timing
    // --------------------------------------------------------

    unsigned long lastSampleMillis = 0;
    float actualSamplingRateHz = 0.0f;

    FSRStatus status = FSRStatus::DISCONNECTED;
};


class FSRSensor
{
public:
    FSRSensor();

    bool begin();

    /*
     * occupantPresent is kept in the interface for Fusion-ready
     * compatibility with Runtime_Main.
     *
     * Step 3 deliberately restores the old proven FSR baseline
     * behavior internally; it does not use C1001 as a label.
     */
    void update(
        bool occupantPresent
    );

    const FSRReading& getReading() const;

    bool hasValidReading() const;

    const char* getStatusText() const;

    const char* getSensorLabel(
        int index
    ) const;


private:
    Adafruit_ADS1115 ads1;
    Adafruit_ADS1115 ads2;

    FSRReading reading;


    // ========================================================
    // PROVEN HARDWARE / FILTER SETTINGS
    // ========================================================

    static constexpr int
        MEDIAN_SAMPLES = 5;

    static constexpr int
        CALIBRATION_ROUNDS = 20;

    static constexpr unsigned long
        CALIBRATION_DELAY_MS = 3000UL;

    static constexpr float
        NOISE_MARGIN = 20.0f;

    static constexpr float
        BASELINE_ADAPT_ALPHA = 0.003f;


    // Old prototype posture thresholds.
    static constexpr float
        CUSHION_OCCUPANCY_THRESHOLD = 900.0f;

    static constexpr float
        BACK_CONTACT_THRESHOLD = 800.0f;

    static constexpr float
        FORWARD_BACK_RATIO = 0.30f;

    static constexpr float
        BACKWARD_BACK_RATIO = 1.25f;

    static constexpr float
        LEAN_ASYMMETRY_THRESHOLD = 0.16f;


    // ========================================================
    // RUNTIME SAMPLE TIMING
    //
    // The frame scheduler becomes eligible to start a new frame
    // after this interval, but one completed frame requires nine
    // cooperative channel reads. On the proven combined runtime
    // the effective completed-frame cadence is ~4.5 Hz.
    //
    // Step 5.5 retrains the FSR model to that observed cadence.
    // ========================================================

    static constexpr unsigned long
        TARGET_SAMPLE_INTERVAL_MS = 80UL;

    unsigned long previousCompletedSample = 0;


    // ========================================================
    // NON-BLOCKING FRAME SCHEDULER
    //
    // A full FSR frame contains 9 channels. Reading all nine
    // channels in one update() call starved the high-rate MPU.
    //
    // Step 4.5 reads ONE FSR channel per update() invocation,
    // allowing Main to run MPU updates between channels.
    // ========================================================

    bool frameInProgress = false;

    uint8_t frameChannelIndex = 0;

    int16_t pendingRaw[NUM_FSR] = {0};


    static const char*
        SENSOR_LABELS[NUM_FSR];


    int16_t readMedianADS(
        Adafruit_ADS1115& module,
        uint8_t channel
    );

    int16_t readMedianNative(
        uint8_t pin
    );

    void readAllSensors(
        int16_t destination[]
    );

    int16_t readOneSensor(
        uint8_t index
    );

    void completeScheduledFrame();

    float applyAdaptiveFilter(
        int16_t raw,
        float previousFiltered
    );

    bool calibrateEmptySeat();

    float calculatePressureDelta(
        float filtered,
        float baseline
    );

    void calculatePressureFeatures();

    void calculatePressureShares();

    void calculatePrototypePostureContext();

    void updateEmptySeatBaseline();
};
