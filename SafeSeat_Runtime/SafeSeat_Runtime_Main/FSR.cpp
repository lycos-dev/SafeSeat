#include "FSR.h"
#include "Config.h"

#include <Wire.h>
#include <math.h>


// ============================================================
// SENSOR LABELS
//
// Match the proven combined sketch terminology.
// ============================================================

const char*
FSRSensor::SENSOR_LABELS[NUM_FSR] =
{
    "BackLeftTop",
    "BackLeftMiddle",
    "BackLeftBottom",

    "BackRightTop",
    "BackRightMiddle",
    "BackRightBottom",

    "CushionLeft",
    "CushionCenter",
    "CushionRight"
};


// ============================================================
// CONSTRUCTOR
// ============================================================

FSRSensor::FSRSensor()
{
}


// ============================================================
// INITIALIZATION
//
// Step 3 restores the exact successful hardware pattern:
//
// ads1.begin(0x48)
// ads1.setGain(GAIN_ONE)
//
// ads2.begin(0x49)
// ads2.setGain(GAIN_ONE)
//
// No forced 860 SPS data rate.
// No extra negative-value rejection.
//
// Wire and ESP32 ADC are already initialized ONCE by Main.
// ============================================================

bool FSRSensor::begin()
{
    reading =
        FSRReading{};


    reading.status =
        FSRStatus::CALIBRATING;


    Serial.println();
    Serial.println(
        "[FSR] Initializing from proven combined sketch..."
    );


    // --------------------------------------------------------
    // ADS1115 #1
    //
    // A0 = Backrest FSR1 / BackLeftTop
    // A1 = Backrest FSR2 / BackLeftMiddle
    // A2 = Backrest FSR3 / BackLeftBottom
    // A3 = Backrest FSR4 / BackRightTop
    // --------------------------------------------------------

    if (
        !ads1.begin(
            ADS1115_1_ADDRESS
        )
    )
    {
        reading.connected =
            false;


        reading.status =
            FSRStatus::DISCONNECTED;


        Serial.println(
            "ADS1115 #1 (0x48) NOT FOUND!"
        );


        return false;
    }


    ads1.setGain(
        GAIN_ONE
    );


    Serial.println(
        "ADS1115 #1 Ready - Backrest FSR1 to FSR4"
    );


    // --------------------------------------------------------
    // ADS1115 #2
    //
    // A0 = Backrest FSR5 / BackRightMiddle
    // A1 = Backrest FSR6 / BackRightBottom
    // A2 = Cushion FSR1 / CushionLeft
    // A3 = Cushion FSR2 / CushionCenter
    // --------------------------------------------------------

    if (
        !ads2.begin(
            ADS1115_2_ADDRESS
        )
    )
    {
        reading.connected =
            false;


        reading.status =
            FSRStatus::DISCONNECTED;


        Serial.println(
            "ADS1115 #2 (0x49) NOT FOUND!"
        );


        return false;
    }


    ads2.setGain(
        GAIN_ONE
    );


    Serial.println(
        "ADS1115 #2 Ready - Backrest FSR5/6 + Cushion Left/Center"
    );


    Serial.println(
        "GPIO34 Ready - Cushion Right"
    );


    reading.connected =
        true;


    if (
        !calibrateEmptySeat()
    )
    {
        reading.calibrated =
            false;


        reading.valid =
            false;


        reading.status =
            FSRStatus::INVALID_READING;


        Serial.println(
            "[FSR] ERROR: empty-seat calibration failed."
        );


        return false;
    }


    reading.status =
        FSRStatus::READY;


    Serial.println(
        "[FSR] Runtime module ready."
    );


    return true;
}


// ============================================================
// MEDIAN ADS1115 READ
//
// Exact proven 5-read median behavior.
// ============================================================

int16_t FSRSensor::readMedianADS(
    Adafruit_ADS1115& module,
    uint8_t channel
)
{
    int16_t samples[
        MEDIAN_SAMPLES
    ];


    for (
        int i = 0;
        i < MEDIAN_SAMPLES;
        i++
    )
    {
        samples[i] =
            module.readADC_SingleEnded(
                channel
            );


        delayMicroseconds(
            250
        );
    }


    for (
        int i = 0;
        i < MEDIAN_SAMPLES - 1;
        i++
    )
    {
        for (
            int j = i + 1;
            j < MEDIAN_SAMPLES;
            j++
        )
        {
            if (
                samples[j]
                <
                samples[i]
            )
            {
                int16_t temporary =
                    samples[i];


                samples[i] =
                    samples[j];


                samples[j] =
                    temporary;
            }
        }
    }


    return samples[
        MEDIAN_SAMPLES / 2
    ];
}


// ============================================================
// MEDIAN ESP32 ADC READ
// ============================================================

int16_t FSRSensor::readMedianNative(
    uint8_t pin
)
{
    int16_t samples[
        MEDIAN_SAMPLES
    ];


    for (
        int i = 0;
        i < MEDIAN_SAMPLES;
        i++
    )
    {
        samples[i] =
            analogRead(
                pin
            );


        delayMicroseconds(
            250
        );
    }


    for (
        int i = 0;
        i < MEDIAN_SAMPLES - 1;
        i++
    )
    {
        for (
            int j = i + 1;
            j < MEDIAN_SAMPLES;
            j++
        )
        {
            if (
                samples[j]
                <
                samples[i]
            )
            {
                int16_t temporary =
                    samples[i];


                samples[i] =
                    samples[j];


                samples[j] =
                    temporary;
            }
        }
    }


    return samples[
        MEDIAN_SAMPLES / 2
    ];
}


// ============================================================
// READ ALL NINE FSRs
//
// IMPORTANT FIX:
//
// The failed Runtime_Main version rejected any negative ADS1115
// sample. The proven combined sketch did NOT do that.
//
// ADS1115 single-ended channels near zero can still return a
// tiny negative code from ADC offset/noise. That must not make
// the entire 9-sensor calibration fail.
//
// Therefore this routine intentionally mirrors the proven code
// and simply returns the median readings.
// ============================================================

void FSRSensor::readAllSensors(
    int16_t destination[]
)
{
    destination[
        BACKREST_FSR1
    ] =
        readMedianADS(
            ads1,
            0
        );


    destination[
        BACKREST_FSR2
    ] =
        readMedianADS(
            ads1,
            1
        );


    destination[
        BACKREST_FSR3
    ] =
        readMedianADS(
            ads1,
            2
        );


    destination[
        BACKREST_FSR4
    ] =
        readMedianADS(
            ads1,
            3
        );


    destination[
        BACKREST_FSR5
    ] =
        readMedianADS(
            ads2,
            0
        );


    destination[
        BACKREST_FSR6
    ] =
        readMedianADS(
            ads2,
            1
        );


    destination[
        CUSHION_FSR1
    ] =
        readMedianADS(
            ads2,
            2
        );


    destination[
        CUSHION_FSR2
    ] =
        readMedianADS(
            ads2,
            3
        );


    destination[
        CUSHION_FSR3
    ] =
        readMedianNative(
            CUSHION_FSR3_PIN
        );
}


// ============================================================
// ADAPTIVE FILTER
//
// Exact proven response:
// - ignore <15 ADC jitter
// - faster release than application
// ============================================================

float FSRSensor::applyAdaptiveFilter(
    int16_t raw,
    float previousFiltered
)
{
    float difference =
        fabs(
            static_cast<float>(
                raw
            )
            -
            previousFiltered
        );


    if (
        difference
        <
        15.0f
    )
    {
        return previousFiltered;
    }


    float alpha;


    if (
        raw
        <
        previousFiltered
    )
    {
        if (
            difference
            >
            5000.0f
        )
        {
            alpha =
                0.95f;
        }
        else if (
            difference
            >
            1000.0f
        )
        {
            alpha =
                0.85f;
        }
        else
        {
            alpha =
                0.65f;
        }
    }
    else
    {
        if (
            difference
            >
            5000.0f
        )
        {
            alpha =
                0.90f;
        }
        else if (
            difference
            >
            1000.0f
        )
        {
            alpha =
                0.70f;
        }
        else
        {
            alpha =
                0.40f;
        }
    }


    return (
        alpha
        *
        static_cast<float>(
            raw
        )
    )
    +
    (
        1.0f
        -
        alpha
    )
    *
    previousFiltered;
}


// ============================================================
// EMPTY-SEAT CALIBRATION
//
// Exact proven pattern:
// - wait 3 seconds
// - 20 rounds
// - each round uses 5-read median per sensor
// - 50 ms between rounds
// ============================================================

bool FSRSensor::calibrateEmptySeat()
{
    reading.status =
        FSRStatus::CALIBRATING;


    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        " FSR EMPTY-SEAT CALIBRATION"
    );

    Serial.println(
        "========================================"
    );

    Serial.println(
        "Keep the backrest and cushion EMPTY."
    );

    Serial.println(
        "Calibration begins in 3 seconds..."
    );


    delay(
        CALIBRATION_DELAY_MS
    );


    float totals[
        NUM_FSR
    ] = {0};


    for (
        int round = 0;
        round < CALIBRATION_ROUNDS;
        round++
    )
    {
        int16_t values[
            NUM_FSR
        ];


        readAllSensors(
            values
        );


        for (
            int i = 0;
            i < NUM_FSR;
            i++
        )
        {
            totals[i] +=
                static_cast<float>(
                    values[i]
                );
        }


        Serial.print(
            "."
        );


        delay(
            50
        );
    }


    Serial.println();


    for (
        int i = 0;
        i < NUM_FSR;
        i++
    )
    {
        reading.baseline[i] =
            totals[i]
            /
            static_cast<float>(
                CALIBRATION_ROUNDS
            );


        reading.filtered[i] =
            reading.baseline[i];


        reading.pressure[i] =
            0.0f;


        reading.pressureShare[i] =
            0.0f;
    }


    reading.calibrated =
        true;


    reading.valid =
        true;


    Serial.println(
        "FSR baseline calibration complete."
    );


    for (
        int i = 0;
        i < NUM_FSR;
        i++
    )
    {
        Serial.print(
            "  "
        );

        Serial.print(
            SENSOR_LABELS[i]
        );

        Serial.print(
            " baseline: "
        );

        Serial.println(
            reading.baseline[i],
            1
        );
    }


    Serial.println();


    previousCompletedSample =
        millis();


    reading.lastSampleMillis =
        previousCompletedSample;


    return true;
}


// ============================================================
// PRESSURE DELTA
// ============================================================

float FSRSensor::calculatePressureDelta(
    float filtered,
    float baseline
)
{
    float difference =
        filtered
        -
        baseline
        -
        NOISE_MARGIN;


    if (
        difference
        <
        0.0f
    )
    {
        difference =
            0.0f;
    }


    return difference;
}


// ============================================================
// FEATURE CALCULATION
// ============================================================

void FSRSensor::calculatePressureFeatures()
{
    reading.backrestLeftTotal =
        reading.pressure[
            BACKREST_FSR1
        ]
        +
        reading.pressure[
            BACKREST_FSR2
        ]
        +
        reading.pressure[
            BACKREST_FSR3
        ];


    reading.backrestRightTotal =
        reading.pressure[
            BACKREST_FSR4
        ]
        +
        reading.pressure[
            BACKREST_FSR5
        ]
        +
        reading.pressure[
            BACKREST_FSR6
        ];


    reading.backrestUpperTotal =
        reading.pressure[
            BACKREST_FSR1
        ]
        +
        reading.pressure[
            BACKREST_FSR4
        ];


    reading.backrestMiddleTotal =
        reading.pressure[
            BACKREST_FSR2
        ]
        +
        reading.pressure[
            BACKREST_FSR5
        ];


    reading.backrestLowerTotal =
        reading.pressure[
            BACKREST_FSR3
        ]
        +
        reading.pressure[
            BACKREST_FSR6
        ];


    reading.cushionLeft =
        reading.pressure[
            CUSHION_FSR1
        ];


    reading.cushionCenter =
        reading.pressure[
            CUSHION_FSR2
        ];


    reading.cushionRight =
        reading.pressure[
            CUSHION_FSR3
        ];


    reading.backrestTotal =
        reading.backrestLeftTotal
        +
        reading.backrestRightTotal;


    reading.cushionTotal =
        reading.cushionLeft
        +
        reading.cushionCenter
        +
        reading.cushionRight;


    reading.wholeSeatTotal =
        reading.backrestTotal
        +
        reading.cushionTotal;


    constexpr float EPSILON =
        0.000001f;


    // Keep the current runtime balance convention.
    // Negative means right-dominant with this field definition.
    reading.backrestLRBalance =
        (
            reading.backrestLeftTotal
            -
            reading.backrestRightTotal
        )
        /
        (
            reading.backrestTotal
            +
            EPSILON
        );


    reading.cushionLRBalance =
        (
            reading.cushionLeft
            -
            reading.cushionRight
        )
        /
        (
            reading.cushionLeft
            +
            reading.cushionRight
            +
            EPSILON
        );


    reading.cushionCenterRatio =
        reading.cushionCenter
        /
        (
            reading.cushionTotal
            +
            EPSILON
        );


    reading.backrestToCushionRatio =
        reading.backrestTotal
        /
        (
            reading.cushionTotal
            +
            EPSILON
        );


    calculatePressureShares();


    calculatePrototypePostureContext();
}


// ============================================================
// PRESSURE SHARES
// ============================================================

void FSRSensor::calculatePressureShares()
{
    constexpr float EPSILON =
        0.000001f;


    float denominator =
        reading.wholeSeatTotal
        +
        EPSILON;


    for (
        int i = 0;
        i < NUM_FSR;
        i++
    )
    {
        reading.pressureShare[i] =
            reading.pressure[i]
            /
            denominator;
    }
}


// ============================================================
// PROTOTYPE POSTURE CONTEXT
//
// Reproduces the diagnostic behavior that was already working
// in the combined sketch.
// ============================================================

void FSRSensor::calculatePrototypePostureContext()
{
    reading.occupied =
        reading.cushionTotal
        >=
        CUSHION_OCCUPANCY_THRESHOLD;


    reading.backContact =
        reading.backrestTotal
        >=
        BACK_CONTACT_THRESHOLD;


    if (
        reading.cushionTotal
        >
        0.0f
    )
    {
        reading.horizontalCOP =
            (
                reading.cushionRight
                -
                reading.cushionLeft
            )
            /
            reading.cushionTotal;
    }
    else
    {
        reading.horizontalCOP =
            0.0f;
    }


    float combinedLeft =
        reading.backrestLeftTotal
        +
        reading.cushionLeft;


    float combinedRight =
        reading.backrestRightTotal
        +
        reading.cushionRight;


    float sideTotal =
        combinedLeft
        +
        combinedRight;


    if (
        sideTotal
        >
        0.0f
    )
    {
        reading.sideAsymmetry =
            (
                combinedRight
                -
                combinedLeft
            )
            /
            sideTotal;
    }
    else
    {
        reading.sideAsymmetry =
            0.0f;
    }


    if (
        !reading.occupied
    )
    {
        reading.datasetOrient =
            'a';


        reading.datasetLean =
            '-';


        return;
    }


    if (
        reading.sideAsymmetry
        <=
        -LEAN_ASYMMETRY_THRESHOLD
    )
    {
        reading.datasetLean =
            'l';
    }
    else if (
        reading.sideAsymmetry
        >=
        LEAN_ASYMMETRY_THRESHOLD
    )
    {
        reading.datasetLean =
            'r';
    }
    else
    {
        reading.datasetLean =
            'c';
    }


    if (
        !reading.backContact
        ||
        reading.backrestToCushionRatio
        <
        FORWARD_BACK_RATIO
    )
    {
        reading.datasetOrient =
            'f';
    }
    else if (
        reading.backrestToCushionRatio
        >
        BACKWARD_BACK_RATIO
    )
    {
        reading.datasetOrient =
            'b';
    }
    else
    {
        reading.datasetOrient =
            's';
    }
}


// ============================================================
// EMPTY-SEAT BASELINE DRIFT CORRECTION
//
// This restores the old proven behavior:
//
// do not adapt if cushion declares occupied
// do not adapt if meaningful backrest contact exists
// ============================================================

void FSRSensor::updateEmptySeatBaseline()
{
    if (
        reading.occupied
    )
    {
        return;
    }


    if (
        reading.backrestTotal
        >=
        BACK_CONTACT_THRESHOLD
    )
    {
        return;
    }


    for (
        int i = 0;
        i < NUM_FSR;
        i++
    )
    {
        reading.baseline[i] =
            BASELINE_ADAPT_ALPHA
            *
            reading.filtered[i]
            +
            (
                1.0f
                -
                BASELINE_ADAPT_ALPHA
            )
            *
            reading.baseline[i];


        reading.pressure[i] =
            calculatePressureDelta(
                reading.filtered[i],
                reading.baseline[i]
            );
    }
}


// ============================================================
// UPDATE
// ============================================================

void FSRSensor::update(
    bool occupantPresent
)
{
    /*
     * Kept only for interface compatibility in Step 3.
     *
     * The restored FSR acquisition does not use C1001 to alter
     * the pressure measurement itself.
     */
    (void) occupantPresent;


    if (
        !reading.connected
        ||
        !reading.calibrated
    )
    {
        return;
    }


    unsigned long now =
        millis();


    if (
        now
        -
        reading.lastSampleMillis
        <
        TARGET_SAMPLE_INTERVAL_MS
    )
    {
        return;
    }


    reading.status =
        FSRStatus::READING;


    int16_t values[
        NUM_FSR
    ];


    readAllSensors(
        values
    );


    for (
        int i = 0;
        i < NUM_FSR;
        i++
    )
    {
        reading.raw[i] =
            values[i];


        reading.filtered[i] =
            applyAdaptiveFilter(
                values[i],
                reading.filtered[i]
            );


        reading.pressure[i] =
            calculatePressureDelta(
                reading.filtered[i],
                reading.baseline[i]
            );
    }


    calculatePressureFeatures();


    updateEmptySeatBaseline();


    // Recalculate after baseline drift correction.
    calculatePressureFeatures();


    unsigned long completedNow =
        millis();


    if (
        previousCompletedSample
        >
        0
    )
    {
        unsigned long delta =
            completedNow
            -
            previousCompletedSample;


        if (
            delta
            >
            0
        )
        {
            reading.actualSamplingRateHz =
                1000.0f
                /
                static_cast<float>(
                    delta
                );
        }
    }


    previousCompletedSample =
        completedNow;


    reading.lastSampleMillis =
        completedNow;


    reading.valid =
        true;


    reading.status =
        FSRStatus::READY;
}


// ============================================================
// GETTERS
// ============================================================

const FSRReading&
FSRSensor::getReading() const
{
    return reading;
}


bool FSRSensor::hasValidReading() const
{
    return (
        reading.connected
        &&
        reading.calibrated
        &&
        reading.valid
    );
}


const char*
FSRSensor::getSensorLabel(
    int index
) const
{
    if (
        index < 0
        ||
        index >= NUM_FSR
    )
    {
        return "Unknown FSR";
    }


    return SENSOR_LABELS[
        index
    ];
}


// ============================================================
// STATUS TEXT
// ============================================================

const char*
FSRSensor::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case FSRStatus::DISCONNECTED:
            return "DISCONNECTED";


        case FSRStatus::CALIBRATING:
            return "CALIBRATING";


        case FSRStatus::READY:
            return "READY";


        case FSRStatus::READING:
            return "READING";


        case FSRStatus::INVALID_READING:
            return "INVALID READING";


        default:
            return "UNKNOWN";
    }
}
