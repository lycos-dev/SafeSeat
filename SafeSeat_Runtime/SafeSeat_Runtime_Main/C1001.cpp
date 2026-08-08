#include "C1001.h"
#include "Config.h"


// ============================================================
// CONSTRUCTOR
// ============================================================

C1001Sensor::C1001Sensor()
    : human(&Serial1)
{
}


// ============================================================
// INITIALIZATION
// ============================================================

bool C1001Sensor::begin()
{
    Serial1.begin(
        115200,
        SERIAL_8N1,
        C1001_RX_PIN,
        C1001_TX_PIN
    );


    Serial.println();
    Serial.println(
        "[C1001] Initializing..."
    );


    if (human.begin() != 0)
    {
        reading.connected = false;

        reading.status =
            C1001Status::DISCONNECTED;

        Serial.println(
            "[C1001] ERROR: initialization failed."
        );

        return false;
    }


    Serial.println(
        "[C1001] Initialization successful."
    );


    Serial.println(
        "[C1001] Switching to Sleep Detection mode..."
    );


    if (
        human.configWorkMode(
            human.eSleepMode
        )
        != 0
    )
    {
        reading.connected = false;

        reading.status =
            C1001Status::DISCONNECTED;

        Serial.println(
            "[C1001] ERROR: work-mode switch failed."
        );

        return false;
    }


    Serial.println(
        "[C1001] Sleep Detection mode active."
    );


    // Enable HP LED.
    human.configLEDLight(
        human.eHPLed,
        1
    );


    // Required by the tested standalone sketch
    // after sensor configuration.
    human.sensorRet();


    delay(500);


    reading.connected = true;


    resetOccupantSession();


    lastSampleTime =
        millis();


    Serial.println(
        "[C1001] Runtime module ready."
    );


    return true;
}


// ============================================================
// VALIDATION
// ============================================================

bool C1001Sensor::isValidRespiration(
    int value
) const
{
    return (
        value != 0
        &&
        value != 255
        &&
        value >= MIN_RR
        &&
        value <= MAX_RR
    );
}


bool C1001Sensor::isValidHeartRate(
    int value
) const
{
    return (
        value != 0
        &&
        value != 255
        &&
        value >= MIN_HR
        &&
        value <= MAX_HR
    );
}


// ============================================================
// RESET
// ============================================================

void C1001Sensor::clearMedianBuffers()
{
    bufferIndex = 0;

    bufferCount = 0;


    for (
        int i = 0;
        i < MEDIAN_WINDOW;
        i++
    )
    {
        rrBuffer[i] = 0;

        hrBuffer[i] = 0;
    }


    rrCandidate = 0;

    rrCandidateCount = 0;


    hrCandidate = 0;

    hrCandidateCount = 0;


    reading.cleanSampleCount = 0;
}


void C1001Sensor::resetAllFilters()
{
    clearMedianBuffers();


    filterInitialized = false;


    acceptedRR = 0;

    acceptedHR = 0;


    filteredRR = 0.0f;

    filteredHR = 0.0f;


    motionArtifactActive = false;

    stableRecoveryCount = 0;


    reading.filteredRespiration =
        NAN;

    reading.filteredHeartRate =
        NAN;


    reading.medianRespiration = 0;

    reading.medianHeartRate = 0;


    reading.trustedVitalsAvailable =
        false;


    reading.motionArtifactActive =
        false;


    reading.recoveryStableCount =
        0;
}


void C1001Sensor::resetOccupantSession()
{
    timerStarted = false;

    wasPresent = false;


    warmupStartTime = 0;


    reading.present = false;

    reading.warmedUp = false;


    reading.warmupRemainingSeconds =
        0;


    resetAllFilters();
}


// ============================================================
// MEDIAN
// ============================================================

int C1001Sensor::calculateMedian(
    const int source[],
    int count
)
{
    int temporary[
        MEDIAN_WINDOW
    ];


    for (
        int i = 0;
        i < count;
        i++
    )
    {
        temporary[i] =
            source[i];
    }


    // Insertion sort
    for (
        int i = 1;
        i < count;
        i++
    )
    {
        int current =
            temporary[i];


        int j =
            i - 1;


        while (
            j >= 0
            &&
            temporary[j] > current
        )
        {
            temporary[j + 1] =
                temporary[j];

            j--;
        }


        temporary[j + 1] =
            current;
    }


    if (
        count % 2 == 1
    )
    {
        return temporary[
            count / 2
        ];
    }


    return (
        temporary[
            (count / 2) - 1
        ]
        +
        temporary[
            count / 2
        ]
    ) / 2;
}


void C1001Sensor::addToMedianBuffers(
    int respiration,
    int heartRate
)
{
    rrBuffer[
        bufferIndex
    ] = respiration;


    hrBuffer[
        bufferIndex
    ] = heartRate;


    bufferIndex++;


    if (
        bufferIndex >=
        MEDIAN_WINDOW
    )
    {
        bufferIndex = 0;
    }


    if (
        bufferCount <
        MEDIAN_WINDOW
    )
    {
        bufferCount++;
    }


    reading.cleanSampleCount =
        bufferCount;
}


// ============================================================
// LARGE CHANGE CONFIRMATION
// ============================================================

bool C1001Sensor::confirmLargeChange(
    int newValue,
    int tolerance,
    int& candidate,
    int& candidateCount
)
{
    if (
        candidateCount == 0
        ||
        abs(
            newValue
            -
            candidate
        ) > tolerance
    )
    {
        candidate =
            newValue;

        candidateCount =
            1;
    }
    else
    {
        candidateCount++;
    }


    return (
        candidateCount >=
        CHANGE_CONFIRMATION_COUNT
    );
}


// ============================================================
// VITAL FILTER
// ============================================================

void C1001Sensor::processVitalSigns(
    int rawRR,
    int rawHR
)
{
    addToMedianBuffers(
        rawRR,
        rawHR
    );


    // Need five clean samples first.
    if (
        bufferCount <
        MEDIAN_WINDOW
    )
    {
        reading.status =
            C1001Status::
                COLLECTING_FILTER_SAMPLES;

        return;
    }


    int medianRR =
        calculateMedian(
            rrBuffer,
            bufferCount
        );


    int medianHR =
        calculateMedian(
            hrBuffer,
            bufferCount
        );


    reading.medianRespiration =
        medianRR;


    reading.medianHeartRate =
        medianHR;


    // --------------------------------------------------------
    // First trusted values
    // --------------------------------------------------------

    if (!filterInitialized)
    {
        acceptedRR =
            medianRR;


        acceptedHR =
            medianHR;


        filteredRR =
            static_cast<float>(
                acceptedRR
            );


        filteredHR =
            static_cast<float>(
                acceptedHR
            );


        filterInitialized =
            true;


        rrCandidateCount = 0;

        hrCandidateCount = 0;


        reading.filteredRespiration =
            filteredRR;


        reading.filteredHeartRate =
            filteredHR;


        reading.trustedVitalsAvailable =
            true;


        reading.status =
            C1001Status::
                FILTER_INITIALIZED;


        return;
    }


    // --------------------------------------------------------
    // Compare with trusted values
    // --------------------------------------------------------

    bool rrNormalChange =
        abs(
            medianRR
            -
            acceptedRR
        )
        <=
        MAX_RR_JUMP;


    bool hrNormalChange =
        abs(
            medianHR
            -
            acceptedHR
        )
        <=
        MAX_HR_JUMP;


    bool rrCanBeAccepted =
        rrNormalChange;


    bool hrCanBeAccepted =
        hrNormalChange;


    // --------------------------------------------------------
    // RR candidate
    // --------------------------------------------------------

    if (rrNormalChange)
    {
        rrCandidateCount = 0;
    }
    else
    {
        rrCanBeAccepted =
            confirmLargeChange(
                medianRR,
                RR_CANDIDATE_TOLERANCE,
                rrCandidate,
                rrCandidateCount
            );
    }


    // --------------------------------------------------------
    // HR candidate
    // --------------------------------------------------------

    if (hrNormalChange)
    {
        hrCandidateCount = 0;
    }
    else
    {
        hrCanBeAccepted =
            confirmLargeChange(
                medianHR,
                HR_CANDIDATE_TOLERANCE,
                hrCandidate,
                hrCandidateCount
            );
    }


    // Both signals must be trustworthy.
    if (
        !rrCanBeAccepted
        ||
        !hrCanBeAccepted
    )
    {
        reading.status =
            C1001Status::
                SPIKE_REJECTED;


        reading.trustedVitalsAvailable =
            true;


        return;
    }


    bool sustainedChange =
        !rrNormalChange
        ||
        !hrNormalChange;


    acceptedRR =
        medianRR;


    acceptedHR =
        medianHR;


    // --------------------------------------------------------
    // EMA
    // --------------------------------------------------------

    filteredRR =
        RR_EMA_ALPHA
        *
        acceptedRR
        +
        (
            1.0f
            -
            RR_EMA_ALPHA
        )
        *
        filteredRR;


    filteredHR =
        HR_EMA_ALPHA
        *
        acceptedHR
        +
        (
            1.0f
            -
            HR_EMA_ALPHA
        )
        *
        filteredHR;


    rrCandidateCount = 0;

    hrCandidateCount = 0;


    reading.filteredRespiration =
        filteredRR;


    reading.filteredHeartRate =
        filteredHR;


    reading.trustedVitalsAvailable =
        true;


    if (sustainedChange)
    {
        reading.status =
            C1001Status::
                SUSTAINED_CHANGE;
    }
    else
    {
        reading.status =
            C1001Status::
                TRUSTED;
    }
}


// ============================================================
// UPDATE
// ============================================================

void C1001Sensor::update()
{
    if (!reading.connected)
    {
        reading.status =
            C1001Status::DISCONNECTED;

        return;
    }


    unsigned long now =
        millis();


    if (
        now
        -
        lastSampleTime
        <
        SAMPLE_INTERVAL_MS
    )
    {
        return;
    }


    lastSampleTime =
        now;


    // --------------------------------------------------------
    // SENSOR READ
    // --------------------------------------------------------

    int presence =
        human.smHumanData(
            human.eHumanPresence
        );


    int motion =
        human.smHumanData(
            human.eHumanMovement
        );


    int moveRange =
        human.smHumanData(
            human.eHumanMovingRange
        );


    int rawRR =
        human.getBreatheValue();


    int rawHR =
        human.getHeartRate();


    reading.present =
        (
            presence == 1
        );


    reading.motion =
        motion;


    reading.moveRange =
        moveRange;


    reading.rawRespiration =
        rawRR;


    reading.rawHeartRate =
        rawHR;


    reading.validRespiration =
        isValidRespiration(
            rawRR
        );


    reading.validHeartRate =
        isValidHeartRate(
            rawHR
        );


    reading.validPair =
        reading.validRespiration
        &&
        reading.validHeartRate;


    // --------------------------------------------------------
    // OCCUPANT LEFT
    // --------------------------------------------------------

    if (
        presence == 0
    )
    {
        if (wasPresent)
        {
            resetOccupantSession();
        }


        reading.status =
            C1001Status::
                NO_OCCUPANT;


        return;
    }


    if (
        presence != 1
    )
    {
        reading.status =
            C1001Status::
                INVALID_VITALS;

        return;
    }


    wasPresent =
        true;


    // --------------------------------------------------------
    // START WARM-UP ONLY AFTER REAL RR + HR
    // --------------------------------------------------------

    if (
        !timerStarted
        &&
        reading.validPair
    )
    {
        warmupStartTime =
            now;


        timerStarted =
            true;


        reading.warmedUp =
            false;
    }


    if (!timerStarted)
    {
        reading.status =
            C1001Status::
                WAITING_FOR_VITALS;


        return;
    }


    unsigned long elapsed =
        now
        -
        warmupStartTime;


    if (
        !reading.warmedUp
        &&
        elapsed <
        WARMUP_MS
    )
    {
        reading.warmupRemainingSeconds =
            (
                WARMUP_MS
                -
                elapsed
                +
                999UL
            )
            /
            1000UL;


        reading.status =
            C1001Status::
                WARMING_UP;


        return;
    }


    // --------------------------------------------------------
    // WARM-UP COMPLETE
    // --------------------------------------------------------

    if (!reading.warmedUp)
    {
        reading.warmedUp =
            true;


        reading.warmupRemainingSeconds =
            0;


        clearMedianBuffers();
    }


    // --------------------------------------------------------
    // Validate RR / HR
    // --------------------------------------------------------

    if (!reading.validPair)
    {
        reading.status =
            C1001Status::
                INVALID_VITALS;


        return;
    }


    // --------------------------------------------------------
    // Motion level
    // --------------------------------------------------------

    bool strongMotion =
        moveRange >=
        STRONG_MOTION_RANGE;


    bool moderateMotion =
        moveRange >=
        MODERATE_MOTION_RANGE
        &&
        moveRange <
        STRONG_MOTION_RANGE;


    bool lowMotion =
        moveRange <
        MODERATE_MOTION_RANGE;


    // --------------------------------------------------------
    // STRONG MOTION
    // --------------------------------------------------------

    if (strongMotion)
    {
        motionArtifactActive =
            true;


        stableRecoveryCount =
            0;


        rrCandidateCount = 0;

        hrCandidateCount = 0;


        reading.motionArtifactActive =
            true;


        reading.recoveryStableCount =
            0;


        reading.status =
            C1001Status::
                STRONG_MOTION;


        return;
    }


    // --------------------------------------------------------
    // RECOVERY
    // --------------------------------------------------------

    if (motionArtifactActive)
    {
        if (lowMotion)
        {
            stableRecoveryCount++;


            reading.recoveryStableCount =
                stableRecoveryCount;


            if (
                stableRecoveryCount >=
                RECOVERY_STABLE_SAMPLES
            )
            {
                motionArtifactActive =
                    false;


                stableRecoveryCount =
                    0;


                reading.motionArtifactActive =
                    false;


                reading.recoveryStableCount =
                    0;


                clearMedianBuffers();


                reading.status =
                    C1001Status::
                        COLLECTING_FILTER_SAMPLES;


                return;
            }
        }
        else
        {
            stableRecoveryCount =
                0;


            reading.recoveryStableCount =
                0;
        }


        reading.motionArtifactActive =
            true;


        reading.status =
            C1001Status::
                MOTION_RECOVERY;


        return;
    }


    // --------------------------------------------------------
    // MODERATE MOTION
    //
    // Do not contaminate median buffers.
    // Hold previous trusted values.
    // --------------------------------------------------------

    if (moderateMotion)
    {
        reading.status =
            C1001Status::
                MODERATE_MOTION;


        return;
    }


    // --------------------------------------------------------
    // CLEAN SAMPLE
    // --------------------------------------------------------

    processVitalSigns(
        rawRR,
        rawHR
    );


    reading.motionArtifactActive =
        false;


    reading.recoveryStableCount =
        0;
}


// ============================================================
// GETTERS
// ============================================================

const C1001Reading&
C1001Sensor::getReading() const
{
    return reading;
}


bool C1001Sensor::hasTrustedVitals() const
{
    return (
        reading.connected
        &&
        reading.present
        &&
        reading.warmedUp
        &&
        reading.trustedVitalsAvailable
        &&
        isfinite(
            reading.filteredRespiration
        )
        &&
        isfinite(
            reading.filteredHeartRate
        )
    );
}


// ============================================================
// STATUS TEXT
// ============================================================

const char*
C1001Sensor::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case C1001Status::DISCONNECTED:
            return "DISCONNECTED";

        case C1001Status::NO_OCCUPANT:
            return "NO OCCUPANT";

        case C1001Status::WAITING_FOR_VITALS:
            return "WAITING FOR VALID RR + HR";

        case C1001Status::WARMING_UP:
            return "WARMING UP";

        case C1001Status::INVALID_VITALS:
            return "INVALID VITALS";

        case C1001Status::STRONG_MOTION:
            return "STRONG MOTION ARTIFACT";

        case C1001Status::MOTION_RECOVERY:
            return "MOTION RECOVERY";

        case C1001Status::MODERATE_MOTION:
            return "MODERATE MOTION - HOLD";

        case C1001Status::COLLECTING_FILTER_SAMPLES:
            return "COLLECTING FILTER SAMPLES";

        case C1001Status::SPIKE_REJECTED:
            return "SPIKE REJECTED - HOLD";

        case C1001Status::FILTER_INITIALIZED:
            return "FILTER INITIALIZED";

        case C1001Status::TRUSTED:
            return "FILTERED AND TRUSTED";

        case C1001Status::SUSTAINED_CHANGE:
            return "SUSTAINED CHANGE CONFIRMED";

        case C1001Status::HOLDING_LAST_VALUE:
            return "HOLDING LAST TRUSTED VALUE";

        default:
            return "UNKNOWN";
    }
}