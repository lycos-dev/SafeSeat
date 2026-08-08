#include "C1001.h"
#include "Config.h"

#include <algorithm>

C1001Sensor::C1001Sensor()
    : human(&Serial1) {
}

bool C1001Sensor::begin() {

    Serial1.begin(
        C1001_BAUD,
        SERIAL_8N1,
        C1001_RX_PIN,
        C1001_TX_PIN
    );

    if (human.begin() != 0) {
        reading.connected = false;
        return false;
    }

    reading.connected = true;

    return true;
}

bool C1001Sensor::isValidVital(
    float value
) const {

    return (
        !isnan(value)
        &&
        value > 0
        &&
        value != 255
    );
}

void C1001Sensor::resetWarmup() {

    timerStarted = false;
    reading.warmedUp = false;

    validVitalsStart = 0;

    bufferCount = 0;
    bufferIndex = 0;

    emaInitialized = false;

    rrEMA = 0;
    hrEMA = 0;
}

void C1001Sensor::pushVitalSample(
    float rr,
    float hr
) {

    rrBuffer[
        bufferIndex
    ] = rr;

    hrBuffer[
        bufferIndex
    ] = hr;

    bufferIndex = (
        bufferIndex + 1
    ) % MEDIAN_SIZE;

    if (
        bufferCount
        < MEDIAN_SIZE
    ) {
        bufferCount++;
    }
}

float C1001Sensor::medianOf(
    const float* values,
    int count
) {

    if (count <= 0) {
        return NAN;
    }

    float temp[
        MEDIAN_SIZE
    ];

    for (
        int i = 0;
        i < count;
        i++
    ) {
        temp[i] = values[i];
    }

    std::sort(
        temp,
        temp + count
    );

    if (
        count % 2 == 1
    ) {
        return temp[
            count / 2
        ];
    }

    return (
        temp[
            count / 2 - 1
        ]
        +
        temp[
            count / 2
        ]
    ) / 2.0f;
}

void C1001Sensor::update() {

    if (!reading.connected) {
        return;
    }

    // --------------------------------------------------------
    // Presence / motion
    // --------------------------------------------------------

    int presence =
    human.smHumanData(
        human.eHumanPresence
    );

reading.present =
    (presence == 1);

reading.motionStatus =
    human.smHumanData(
        human.eHumanMovement
    );

reading.moveRange =
    human.smHumanData(
        human.eHumanMovingRange
    );

    // --------------------------------------------------------
    // Vital signs
    // --------------------------------------------------------

    float rr =
        human.getBreatheValue();

    float hr =
        human.getHeartRate();

    reading.respirationRaw = rr;
    reading.heartRateRaw = hr;

    bool validRR =
        isValidVital(rr);

    bool validHR =
        isValidVital(hr);

    reading.validVitals =
        validRR
        &&
        validHR;

    // --------------------------------------------------------
    // Warm-up starts only after real RR + HR values exist
    // --------------------------------------------------------

    if (
        !reading.present
        ||
        !reading.validVitals
    ) {

        resetWarmup();

        reading.respirationFiltered =
            NAN;

        reading.heartRateFiltered =
            NAN;

        return;
    }

    if (!timerStarted) {

        timerStarted = true;

        validVitalsStart =
            millis();

        reading.warmedUp = false;
    }

    if (
        !reading.warmedUp
        &&
        millis()
            - validVitalsStart
            >= C1001_WARMUP_MS
    ) {

        reading.warmedUp = true;

        bufferCount = 0;
        bufferIndex = 0;

        emaInitialized = false;
    }

    if (!reading.warmedUp) {

        reading.respirationFiltered =
            NAN;

        reading.heartRateFiltered =
            NAN;

        return;
    }

    // --------------------------------------------------------
    // Median + EMA
    // --------------------------------------------------------

    pushVitalSample(
        rr,
        hr
    );

    if (
        bufferCount
        < MEDIAN_SIZE
    ) {
        return;
    }

    float rrMedian =
        medianOf(
            rrBuffer,
            bufferCount
        );

    float hrMedian =
        medianOf(
            hrBuffer,
            bufferCount
        );

    if (!emaInitialized) {

        rrEMA = rrMedian;
        hrEMA = hrMedian;

        emaInitialized = true;

    } else {

        rrEMA =
            EMA_ALPHA
            * rrMedian
            +
            (
                1.0f
                - EMA_ALPHA
            )
            * rrEMA;

        hrEMA =
            EMA_ALPHA
            * hrMedian
            +
            (
                1.0f
                - EMA_ALPHA
            )
            * hrEMA;
    }

    reading.respirationFiltered =
        rrEMA;

    reading.heartRateFiltered =
        hrEMA;
}

const C1001Reading&
C1001Sensor::getReading() const {

    return reading;
}