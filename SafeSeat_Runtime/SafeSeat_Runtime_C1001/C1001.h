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

    HOLDING_LAST_VALUE,

    // Appended to preserve the numeric values of all existing
    // wire/status codes used by the Main Hub.
    REACQUIRING_TARGET,

    REACQUIRING_REBASELINE
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
    // 1 Hz sample identity
    //
    // Incremented only after a complete C1001 sensor poll has
    // finished. C1001ML uses this to avoid ingesting the same
    // sample multiple times from the fast main loop.
    // -------------------------

    uint32_t sampleSequence = 0;

    unsigned long sampleTimestampMillis = 0;


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

    // True while the radar is being quarantined after a strong
    // movement/obstruction. During this state, the last trusted
    // filtered vitals remain available, but new raw HR/RR samples
    // must NOT enter the ML window.
    bool reacquisitionActive = false;

    bool reacquisitionRebaseline = false;

    int reacquisitionStableCount = 0;

    int reacquisitionElapsedSamples = 0;


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

    // Live validation 2026-08-22 showed isolated MoveRange spikes
    // (30..100) while the participant was effectively stationary.
    // Do not classify one radar spike as sustained strong motion.
    static constexpr int
        STRONG_MOTION_CONFIRM_SAMPLES = 2;

    static constexpr int
        RECOVERY_STABLE_SAMPLES = 5;


    // ========================================================
    // POST-MOTION / TARGET REACQUISITION SETTINGS
    //
    // Live disturbance testing on 2026-08-22 showed that after
    // an obstruction, body shift, or leaving/re-entering the
    // radar field, C1001 HR/RR can remain falsely elevated even
    // after MoveRange falls back to 1..10. A simple 3-sample
    // sustained-change rule is therefore unsafe immediately after
    // strong radar disturbance.
    //
    // Policy:
    // - ANY MoveRange >=30 starts a non-destructive quarantine.
    // - Preserve the ML window and hold the last trusted vitals.
    // - Fast exit: require 5 consecutive low-motion samples whose
    //   raw HR/RR have returned near the pre-motion trusted level.
    // - If that does not happen within 30 samples, collect a fresh
    //   5-sample low-motion median baseline while ML remains held.
    // ========================================================

    static constexpr int
        REACQ_STABLE_SAMPLES = 5;

    static constexpr int
        REACQ_MAX_HOLD_SAMPLES = 30;

    static constexpr int
        REACQ_RR_BASELINE_TOLERANCE = 5;

    static constexpr int
        REACQ_HR_BASELINE_TOLERANCE = 15;


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

    // Consecutive >= STRONG_MOTION_RANGE samples.
    // One isolated spike does not activate the full motion-artifact
    // state; two consecutive strong samples are required.
    int strongMotionConfirmCount = 0;

    int stableRecoveryCount = 0;


    // ========================================================
    // REACQUISITION STATE
    // ========================================================

    bool reacquisitionActive = false;

    bool reacquisitionRebaseline = false;

    int reacquisitionStableCount = 0;

    int reacquisitionElapsedSamples = 0;

    int reacquisitionBaselineRR = 0;

    int reacquisitionBaselineHR = 0;


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


    void beginReacquisition();


    bool processReacquisitionSample(
        int rawRR,
        int rawHR,
        int moveRange
    );


    void finishReacquisition();


    void updateStatus();
};