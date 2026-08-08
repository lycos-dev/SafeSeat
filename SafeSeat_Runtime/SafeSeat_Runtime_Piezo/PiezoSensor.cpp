#include "PiezoSensor.h"

#include <math.h>


// ============================================================
// CONSTRUCTOR
// ============================================================

PiezoSensor::PiezoSensor()
{
}


// ============================================================
// INITIALIZATION
// ============================================================

bool PiezoSensor::begin()
{
    reading.status =
        PiezoStatus::INITIALIZING;


    Serial.println();
    Serial.println(
        "[PIEZO] Initializing seatbelt respiration sensor..."
    );


    analogReadResolution(
        PIEZO_ADC_RESOLUTION_BITS
    );


    /*
     * GPIO34 is ADC1 and is suitable for the PVDF signal.
     */
    analogSetPinAttenuation(
        PIEZO_PIN,
        ADC_11db
    );


    delay(
        100
    );


    int initialRaw =
        analogRead(
            PIEZO_PIN
        );


    if (
        !isADCValid(
            initialRaw
        )
    )
    {
        reading.status =
            PiezoStatus::INVALID_READING;


        Serial.println(
            "[PIEZO] ERROR: invalid initial ADC value."
        );


        return false;
    }


    initializeFilter(
        initialRaw
    );


    unsigned long now =
        millis();


    lastSampleMillis =
        now;


    previousCompletedSampleMillis =
        now;


    lastBreathTime =
        now;


    reading.lastBreathMillis =
        now;


    reading.valid =
        true;


    reading.status =
        PiezoStatus::READY;


    Serial.println(
        "[PIEZO] Sensor ready."
    );


    Serial.print(
        "[PIEZO] Runtime sampling target: "
    );

    Serial.print(
        PIEZO_SAMPLE_RATE_HZ,
        1
    );

    Serial.println(
        " Hz"
    );


    Serial.print(
        "[PIEZO] ML window: "
    );

    Serial.print(
        PIEZO_WINDOW_SECONDS
    );

    Serial.print(
        " sec / "
    );

    Serial.print(
        PIEZO_WINDOW_SAMPLES
    );

    Serial.println(
        " samples"
    );


    Serial.print(
        "[PIEZO] ML stride: "
    );

    Serial.print(
        PIEZO_STRIDE_SECONDS
    );

    Serial.println(
        " sec"
    );


    return true;
}


// ============================================================
// ADC VALIDATION
// ============================================================

bool PiezoSensor::isADCValid(
    int raw
) const
{
    return (
        raw >= 0
        &&
        raw <= 4095
    );
}


// ============================================================
// FILTER INITIALIZATION
// ============================================================

void PiezoSensor::initializeFilter(
    int raw
)
{
    filteredSignal =
        static_cast<float>(
            raw
        );


    baseline =
        static_cast<float>(
            raw
        );


    previousWave =
        0.0f;


    filterInitialized =
        true;


    reading.rawADC =
        raw;


    reading.filteredSignal =
        filteredSignal;


    reading.baseline =
        baseline;


    reading.respirationWave =
        0.0f;
}


// ============================================================
// SIGNAL FILTERING
//
// Preserved from your tested seatbelt Piezo code:
//
// raw ADC
// -> EMA
// -> slow moving baseline
// -> baseline-subtracted respiration waveform
// ============================================================

void PiezoSensor::processSignal(
    int raw
)
{
    if (
        !filterInitialized
    )
    {
        initializeFilter(
            raw
        );

        return;
    }


    // --------------------------------------------------------
    // Fast EMA smoothing
    // --------------------------------------------------------

    filteredSignal =
        (
            PIEZO_EMA_ALPHA
            *
            static_cast<float>(
                raw
            )
        )
        +
        (
            1.0f
            -
            PIEZO_EMA_ALPHA
        )
        *
        filteredSignal;


    // --------------------------------------------------------
    // Slow baseline tracking
    // --------------------------------------------------------

    baseline =
        (
            PIEZO_BASELINE_ALPHA
            *
            filteredSignal
        )
        +
        (
            1.0f
            -
            PIEZO_BASELINE_ALPHA
        )
        *
        baseline;


    // --------------------------------------------------------
    // Respiration waveform
    // --------------------------------------------------------

    float respirationWave =
        filteredSignal
        -
        baseline;


    reading.rawADC =
        raw;


    reading.filteredSignal =
        filteredSignal;


    reading.baseline =
        baseline;


    reading.respirationWave =
        respirationWave;
}


// ============================================================
// BREATH TIMESTAMP
// ============================================================

void PiezoSensor::storeBreathTimestamp(
    unsigned long timestamp
)
{
    breathTimes[
        breathTimeWriteIndex
    ] =
        timestamp;


    breathTimeWriteIndex++;


    if (
        breathTimeWriteIndex
        >=
        MAX_BREATH_TIMESTAMPS
    )
    {
        breathTimeWriteIndex =
            0;
    }


    if (
        breathTimeCount
        <
        MAX_BREATH_TIMESTAMPS
    )
    {
        breathTimeCount++;
    }
}


// ============================================================
// AUXILIARY BREATH DETECTION
// ============================================================

void PiezoSensor::processBreathDetection()
{
    reading.breathDetectedThisSample =
        false;


    float currentWave =
        reading.respirationWave;


    if (
        currentWave
        >
        previousWave
    )
    {
        rising =
            true;
    }


    if (
        rising
        &&
        currentWave
        <
        previousWave
    )
    {
        unsigned long now =
            millis();


        if (
            previousWave
            >
            PIEZO_PEAK_THRESHOLD

            &&

            now
            -
            lastBreathTime
            >
            PIEZO_BREATH_COOLDOWN_MS
        )
        {
            lastBreathTime =
                now;


            reading.lastBreathMillis =
                now;


            reading.totalBreaths++;


            reading.breathDetectedThisSample =
                true;


            storeBreathTimestamp(
                now
            );


            updateRespirationRateEstimate();
        }


        rising =
            false;
    }


    previousWave =
        currentWave;
}


// ============================================================
// RESPIRATION RATE ESTIMATE
// ============================================================

void PiezoSensor::updateRespirationRateEstimate()
{
    if (
        breathTimeCount
        <
        2
    )
    {
        reading.estimatedRespirationBPM =
            NAN;


        return;
    }


    /*
     * Reconstruct chronological order of timestamps.
     */

    unsigned long ordered[
        MAX_BREATH_TIMESTAMPS
    ];


    uint8_t oldestIndex;


    if (
        breathTimeCount
        <
        MAX_BREATH_TIMESTAMPS
    )
    {
        oldestIndex =
            0;
    }
    else
    {
        oldestIndex =
            breathTimeWriteIndex;
    }


    for (
        uint8_t i = 0;
        i < breathTimeCount;
        i++
    )
    {
        uint8_t index =
            (
                oldestIndex
                +
                i
            )
            %
            MAX_BREATH_TIMESTAMPS;


        ordered[i] =
            breathTimes[
                index
            ];
    }


    float intervalSumSeconds =
        0.0f;


    uint8_t intervalCount =
        0;


    for (
        uint8_t i = 1;
        i < breathTimeCount;
        i++
    )
    {
        unsigned long intervalMs =
            ordered[i]
            -
            ordered[i - 1];


        /*
         * Ignore obviously impossible/duplicate intervals.
         */
        if (
            intervalMs
            >=
            PIEZO_BREATH_COOLDOWN_MS
        )
        {
            intervalSumSeconds +=
                static_cast<float>(
                    intervalMs
                )
                /
                1000.0f;


            intervalCount++;
        }
    }


    if (
        intervalCount == 0
    )
    {
        reading.estimatedRespirationBPM =
            NAN;


        return;
    }


    float meanInterval =
        intervalSumSeconds
        /
        static_cast<float>(
            intervalCount
        );


    if (
        meanInterval <= 0.0f
    )
    {
        reading.estimatedRespirationBPM =
            NAN;


        return;
    }


    reading.estimatedRespirationBPM =
        60.0f
        /
        meanInterval;
}


// ============================================================
// NO-BREATH STATE
// ============================================================

void PiezoSensor::updateNoBreathState()
{
    unsigned long now =
        millis();


    reading.noBreathDurationMs =
        now
        -
        lastBreathTime;


    /*
     * Unlike the old test code, DO NOT reset lastBreathTime
     * after the timer expires.
     *
     * Otherwise a 40-second absence of breathing could appear
     * as repeated independent 15-second events.
     *
     * We preserve the real duration continuously.
     */

    reading.noBreathTimerExceeded =
        (
            reading.noBreathDurationMs
            >=
            PIEZO_APNEA_TIME_MS
        );
}


// ============================================================
// RING BUFFER
// ============================================================

void PiezoSensor::storeRespirationSample(
    float value
)
{
    respirationBuffer[
        bufferWriteIndex
    ] =
        value;


    bufferWriteIndex++;


    if (
        bufferWriteIndex
        >=
        PIEZO_WINDOW_SAMPLES
    )
    {
        bufferWriteIndex =
            0;
    }


    if (
        bufferCount
        <
        PIEZO_WINDOW_SAMPLES
    )
    {
        bufferCount++;
    }


    reading.windowSamplesAvailable =
        bufferCount;


    reading.fullWindowReady =
        (
            bufferCount
            >=
            PIEZO_WINDOW_SAMPLES
        );


    if (
        !reading.fullWindowReady
    )
    {
        /*
         * Do not count model strides until the first complete
         * 30-second window exists.
         */

        samplesSinceFeatureWindow =
            0;


        return;
    }


    samplesSinceFeatureWindow++;


    /*
     * First full window should become available immediately.
     */
    if (
        bufferCount
        ==
        PIEZO_WINDOW_SAMPLES

        &&

        reading.sampleCount
        ==
        PIEZO_WINDOW_SAMPLES
    )
    {
        reading.newFeatureWindowDue =
            true;


        samplesSinceFeatureWindow =
            0;


        return;
    }


    if (
        samplesSinceFeatureWindow
        >=
        PIEZO_STRIDE_SAMPLES
    )
    {
        reading.newFeatureWindowDue =
            true;


        samplesSinceFeatureWindow =
            0;
    }
}


// ============================================================
// COPY WINDOW IN CHRONOLOGICAL ORDER
// ============================================================

bool PiezoSensor::copyRespirationWindow(
    float destination[],
    uint16_t destinationSize
) const
{
    if (
        destination == nullptr
        ||
        destinationSize
            <
            PIEZO_WINDOW_SAMPLES
        ||
        bufferCount
            <
            PIEZO_WINDOW_SAMPLES
    )
    {
        return false;
    }


    /*
     * With a full ring buffer, bufferWriteIndex points to
     * the oldest sample (the next value to be overwritten).
     */

    uint16_t readIndex =
        bufferWriteIndex;


    for (
        uint16_t i = 0;
        i < PIEZO_WINDOW_SAMPLES;
        i++
    )
    {
        destination[i] =
            respirationBuffer[
                readIndex
            ];


        readIndex++;


        if (
            readIndex
            >=
            PIEZO_WINDOW_SAMPLES
        )
        {
            readIndex =
                0;
        }
    }


    return true;
}


// ============================================================
// FEATURE WINDOW ACKNOWLEDGEMENT
// ============================================================

void PiezoSensor::acknowledgeFeatureWindow()
{
    reading.newFeatureWindowDue =
        false;
}


// ============================================================
// UPDATE
// ============================================================

void PiezoSensor::update()
{
    unsigned long now =
        millis();


    if (
        now
        -
        lastSampleMillis
        <
        PIEZO_SAMPLE_INTERVAL_MS
    )
    {
        /*
         * No-breath duration should still update even when an
         * ADC sample is not due yet.
         */

        updateNoBreathState();

        return;
    }


    unsigned long elapsed =
        now
        -
        lastSampleMillis;


    lastSampleMillis =
        now;


    // ========================================================
    // SAMPLING DIAGNOSTIC
    // ========================================================

    if (
        elapsed > 0
    )
    {
        reading.actualSamplingRateHz =
            1000.0f
            /
            static_cast<float>(
                elapsed
            );
    }


    // ========================================================
    // ADC
    // ========================================================

    int raw =
        analogRead(
            PIEZO_PIN
        );


    if (
        !isADCValid(
            raw
        )
    )
    {
        reading.valid =
            false;


        reading.status =
            PiezoStatus::
                INVALID_READING;


        return;
    }


    // ========================================================
    // SIGNAL PROCESSING
    // ========================================================

    processSignal(
        raw
    );


    processBreathDetection();


    updateNoBreathState();


    // ========================================================
    // STORE MODEL SOURCE SIGNAL
    // ========================================================

    storeRespirationSample(
        reading.respirationWave
    );


    reading.sampleCount++;


    reading.valid =
        true;


    reading.status =
        PiezoStatus::READY;
}


// ============================================================
// GETTERS
// ============================================================

const PiezoReading&
PiezoSensor::getReading() const
{
    return reading;
}