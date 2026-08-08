#pragma once

#include <Arduino.h>
#include "DFRobot_HumanDetection.h"

enum class C1001Status
{
    DISCONNECTED,

    NO_OCCUPANT,

    WAITING_FOR_VITALS,

    WARMING_UP,

    INVALID_VITALS,

    STRONG_MOTION,

    MOTION_RECOVERY,

    MODERATE_MOTION,

    COLLECTING_FILTER_SAMPLES,

    SPIKE_REJECTED,

    FILTER_INITIALIZED,

    TRUSTED,

    SUSTAINED_CHANGE,

    HOLDING_LAST_VALUE
};


struct C1001Reading
{
    bool connected = false;

    bool present = false;

    int motion = -1;

    int moveRange = -1;


    // -------------------------
    // Raw sensor output
    // -------------------------

    int rawRespiration = 0;

    int rawHeartRate = 0;


    // -------------------------
    // Median values
    // -------------------------

    int medianRespiration = 0;

    int medianHeartRate = 0;


    // -------------------------
    // Final trusted output
    // -------------------------

    float filteredRespiration = NAN;

    float filteredHeartRate = NAN;


    // -------------------------
    // Runtime state
    // -------------------------

    bool validRespiration = false;

    bool validHeartRate = false;

    bool validPair = false;

    bool warmedUp = false;

    bool trustedVitalsAvailable = false;

    bool motionArtifactActive = false;


    int recoveryStableCount = 0;

    int cleanSampleCount = 0;


    unsigned long warmupRemainingSeconds = 0;


    C1001Status status =
        C1001Status::DISCONNECTED;
};


class C1001Sensor
{
public:

    C1001Sensor();


    bool begin();


    /*
     * Call update() frequently.
     *
     * The sensor itself is only sampled once per second.
     */
    void update();


    const C1001Reading& getReading() const;


    bool hasTrustedVitals() const;


    const char* getStatusText() const;


private:

    DFRobot_HumanDetection human;


    C1001Reading reading;


    // ========================================================
    // TIMING
    // ========================================================

    static constexpr unsigned long
        WARMUP_MS = 60000UL;

    static constexpr unsigned long
        SAMPLE_INTERVAL_MS = 1000UL;


    unsigned long lastSampleTime = 0;

    unsigned long warmupStartTime = 0;


    bool timerStarted = false;

    bool wasPresent = false;


    // ========================================================
    // VALID PHYSIOLOGICAL INPUT RANGES
    //
    // These are broad sensor-artifact rejection ranges.
    // They are NOT medical emergency thresholds.
    // ========================================================

    static constexpr int MIN_RR = 5;

    static constexpr int MAX_RR = 45;


    static constexpr int MIN_HR = 35;

    static constexpr int MAX_HR = 200;


    // ========================================================
    // MOTION ARTIFACT SETTINGS
    // ========================================================

    static constexpr int
        MODERATE_MOTION_RANGE = 15;

    static constexpr int
        STRONG_MOTION_RANGE = 30;

    static constexpr int
        RECOVERY_STABLE_SAMPLES = 5;


    // ========================================================
    // FILTER SETTINGS
    // ========================================================

    static constexpr int
        MEDIAN_WINDOW = 5;


    static constexpr int
        MAX_RR_JUMP = 8;

    static constexpr int
        MAX_HR_JUMP = 20;


    static constexpr int
        CHANGE_CONFIRMATION_COUNT = 3;


    static constexpr int
        RR_CANDIDATE_TOLERANCE = 3;

    static constexpr int
        HR_CANDIDATE_TOLERANCE = 8;


    static constexpr float
        RR_EMA_ALPHA = 0.30f;

    static constexpr float
        HR_EMA_ALPHA = 0.25f;


    // ========================================================
    // MOTION STATE
    // ========================================================

    bool motionArtifactActive = false;

    int stableRecoveryCount = 0;


    // ========================================================
    // MEDIAN BUFFER
    // ========================================================

    int rrBuffer[MEDIAN_WINDOW] = {0};

    int hrBuffer[MEDIAN_WINDOW] = {0};


    int bufferIndex = 0;

    int bufferCount = 0;


    // ========================================================
    // TRUSTED FILTER STATE
    // ========================================================

    bool filterInitialized = false;


    int acceptedRR = 0;

    int acceptedHR = 0;


    float filteredRR = 0.0f;

    float filteredHR = 0.0f;


    // ========================================================
    // LARGE CHANGE CONFIRMATION
    // ========================================================

    int rrCandidate = 0;

    int rrCandidateCount = 0;


    int hrCandidate = 0;

    int hrCandidateCount = 0;


    // ========================================================
    // HELPERS
    // ========================================================

    bool isValidRespiration(
        int value
    ) const;


    bool isValidHeartRate(
        int value
    ) const;


    void clearMedianBuffers();


    void resetAllFilters();


    void resetOccupantSession();


    int calculateMedian(
        const int source[],
        int count
    );


    void addToMedianBuffers(
        int respiration,
        int heartRate
    );


    bool confirmLargeChange(
        int newValue,
        int tolerance,
        int& candidate,
        int& candidateCount
    );


    void processVitalSigns(
        int rawRR,
        int rawHR
    );


    void updateStatus();
};