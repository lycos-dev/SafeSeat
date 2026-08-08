#include "FSR.h"
#include "Config.h"

#include <math.h>


// ============================================================
// SENSOR LABELS
// ============================================================

const char*
FSRSensor::SENSOR_LABELS[NUM_FSR] =
{
    "Backrest FSR1",
    "Backrest FSR2",
    "Backrest FSR3",

    "Backrest FSR4",
    "Backrest FSR5",
    "Backrest FSR6",

    "Cushion FSR1",
    "Cushion FSR2",
    "Cushion FSR3"
};


// ============================================================
// CONSTRUCTOR
// ============================================================

FSRSensor::FSRSensor()
{
}


// ============================================================
// INITIALIZATION
// ============================================================

bool FSRSensor::begin()
{
    reading.status =
        FSRStatus::CALIBRATING;


    Serial.println();
    Serial.println(
        "[FSR] Initializing..."
    );


    // --------------------------------------------------------
    // ESP32 ADC
    // --------------------------------------------------------

    analogReadResolution(
        12
    );


    analogSetPinAttenuation(
        CUSHION_FSR3_PIN,
        ADC_11db
    );


    // --------------------------------------------------------
    // ADS1115 #1
    //
    // 0x48
    //
    // A0 = Backrest FSR1
    // A1 = Backrest FSR2
    // A2 = Backrest FSR3
    // A3 = Backrest FSR4
    // --------------------------------------------------------

    if (
        !ads1.begin(
            ADS1115_1_ADDRESS,
            &Wire
        )
    )
    {
        reading.connected =
            false;


        reading.status =
            FSRStatus::DISCONNECTED;


        Serial.println(
            "[FSR] ERROR: ADS1115 #1 (0x48) not detected."
        );


        return false;
    }


    ads1.setGain(
        GAIN_ONE
    );


    Serial.println(
        "[FSR] ADS1115 #1 (0x48) ready."
    );


    // --------------------------------------------------------
    // ADS1115 #2
    //
    // 0x49
    //
    // A0 = Backrest FSR5
    // A1 = Backrest FSR6
    // A2 = Cushion FSR1
    // A3 = Cushion FSR2
    // --------------------------------------------------------

    if (
        !ads2.begin(
            ADS1115_2_ADDRESS,
            &Wire
        )
    )
    {
        reading.connected =
            false;


        reading.status =
            FSRStatus::DISCONNECTED;


        Serial.println(
            "[FSR] ERROR: ADS1115 #2 (0x49) not detected."
        );


        return false;
    }


    ads2.setGain(
        GAIN_ONE
    );


    Serial.println(
        "[FSR] ADS1115 #2 (0x49) ready."
    );


    reading.connected =
        true;


    // --------------------------------------------------------
    // EMPTY-SEAT CALIBRATION
    // --------------------------------------------------------

    if (
        !calibrateEmptySeat()
    )
    {
        reading.connected =
            false;


        reading.calibrated =
            false;


        reading.status =
            FSRStatus::
                INVALID_READING;


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
// ============================================================

bool FSRSensor::readAllSensors(
    int16_t destination[]
)
{
    // --------------------------------------------------------
    // ADS1115 #1
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // ADS1115 #2
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // ESP32 native ADC
    // --------------------------------------------------------

    destination[
        CUSHION_FSR3
    ] =
        readMedianNative(
            CUSHION_FSR3_PIN
        );


    return readingsAreValid(
        destination
    );
}


// ============================================================
// BASIC VALIDATION
// ============================================================

bool FSRSensor::readingsAreValid(
    const int16_t values[]
) const
{
    for (
        int i = 0;
        i < NUM_FSR;
        i++
    )
    {
        /*
         * ADS1115 single-ended values should never be negative.
         *
         * ESP32 ADC also cannot return a negative value.
         */

        if (
            values[i] < 0
        )
        {
            return false;
        }
    }


    return true;
}


// ============================================================
// ADAPTIVE FILTER
//
// Preserved from your tested FSR implementation.
//
// Key property:
// fast release after pressure removal to avoid lingering values.
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


    // --------------------------------------------------------
    // Tiny noise
    // --------------------------------------------------------

    if (
        difference < 15.0f
    )
    {
        return previousFiltered;
    }


    float alpha;


    // --------------------------------------------------------
    // PRESSURE RELEASE
    // --------------------------------------------------------

    if (
        raw <
        previousFiltered
    )
    {
        if (
            difference >
            5000.0f
        )
        {
            alpha =
                0.95f;
        }
        else if (
            difference >
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


    // --------------------------------------------------------
    // PRESSURE APPLICATION
    // --------------------------------------------------------

    else
    {
        if (
            difference >
            5000.0f
        )
        {
            alpha =
                0.90f;
        }
        else if (
            difference >
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
        raw
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
// ============================================================

bool FSRSensor::calibrateEmptySeat()
{
    reading.status =
        FSRStatus::CALIBRATING;


    Serial.println();
    Serial.println(
        "=========================================="
    );

    Serial.println(
        " FSR EMPTY-SEAT CALIBRATION"
    );

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "Keep BACKREST and CUSHION completely empty."
    );


    Serial.print(
        "Calibration begins in "
    );

    Serial.print(
        CALIBRATION_DELAY_MS / 1000
    );

    Serial.println(
        " seconds..."
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


        if (
            !readAllSensors(
                values
            )
        )
        {
            Serial.println();
            Serial.println(
                "[FSR] ERROR during baseline calibration."
            );

            return false;
        }


        for (
            int i = 0;
            i < NUM_FSR;
            i++
        )
        {
            totals[i] +=
                values[i];
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
            CALIBRATION_ROUNDS;


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
        "[FSR] Empty-seat calibration complete."
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
            " baseline = "
        );

        Serial.println(
            reading.baseline[i],
            1
        );
    }


    Serial.println();


    previousCompletedSample =
        millis();


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
        difference <
        0.0f
    )
    {
        difference =
            0.0f;
    }


    return difference;
}


// ============================================================
// GROUP / DISTRIBUTION FEATURES
// ============================================================

void FSRSensor::calculatePressureFeatures()
{
    // ========================================================
    // BACKREST LEFT
    // ========================================================

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


    // ========================================================
    // BACKREST RIGHT
    // ========================================================

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


    // ========================================================
    // BACKREST VERTICAL LEVELS
    // ========================================================

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


    // ========================================================
    // CUSHION
    // ========================================================

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


    // ========================================================
    // TOTALS
    // ========================================================

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


    // ========================================================
    // BACKREST LEFT/RIGHT BALANCE
    //
    // -1 approximately = left dominant
    //  0               = balanced
    // +1 approximately = right dominant
    // ========================================================

    reading.backrestLRBalance =
        (
            reading.backrestLeftTotal
            -
            reading.backrestRightTotal
        )
        /
        (
            reading.backrestLeftTotal
            +
            reading.backrestRightTotal
            +
            EPSILON
        );


    // ========================================================
    // CUSHION LEFT/RIGHT BALANCE
    // ========================================================

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


    // ========================================================
    // CUSHION CENTER SHARE
    // ========================================================

    reading.cushionCenterRatio =
        reading.cushionCenter
        /
        (
            reading.cushionTotal
            +
            EPSILON
        );


    // ========================================================
    // BACKREST / CUSHION RELATIONSHIP
    // ========================================================

    reading.backrestToCushionRatio =
        reading.backrestTotal
        /
        (
            reading.cushionTotal
            +
            EPSILON
        );


    calculatePressureShares();
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
// EMPTY-SEAT BASELINE DRIFT CORRECTION
// ============================================================

void FSRSensor::updateBaselineWhenEmpty(
    bool occupantPresent
)
{
    /*
     * NEVER adapt baseline while the C1001 sees an occupant.
     */

    if (
        occupantPresent
    )
    {
        return;
    }


    /*
     * Extra safety:
     *
     * If meaningful pressure is present despite C1001 saying
     * no occupant, do not modify calibration.
     *
     * This prevents a bag/object/person missed by C1001 from
     * being absorbed into the empty-seat baseline.
     */

    if (
        reading.wholeSeatTotal
        >
        BASELINE_ADAPT_MAX_TOTAL
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
            (
                BASELINE_ADAPT_ALPHA
                *
                reading.filtered[i]
            )
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


    if (
        !readAllSensors(
            values
        )
    )
    {
        reading.valid =
            false;


        reading.status =
            FSRStatus::
                INVALID_READING;


        return;
    }


    // ========================================================
    // RAW + FILTER
    // ========================================================

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


    // ========================================================
    // FEATURES
    // ========================================================

    calculatePressureFeatures();


    // ========================================================
    // EMPTY-SEAT BASELINE ADAPTATION
    // ========================================================

    updateBaselineWhenEmpty(
        occupantPresent
    );


    /*
     * Recalculate after baseline movement.
     */

    calculatePressureFeatures();


    // ========================================================
    // SAMPLE RATE MONITOR
    // ========================================================

    unsigned long completedNow =
        millis();


    if (
        previousCompletedSample > 0
    )
    {
        unsigned long delta =
            completedNow
            -
            previousCompletedSample;


        if (
            delta > 0
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