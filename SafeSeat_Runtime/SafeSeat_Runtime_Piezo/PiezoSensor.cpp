#include "PiezoSensor.h"

#include <math.h>
#include <string.h>

PiezoSensor::PiezoSensor()
{
}

bool PiezoSensor::begin()
{
    Serial.println(
        "[PIEZO] Initializing deterministic seatbelt respiration support..."
    );

    analogReadResolution(
        PIEZO_ADC_RESOLUTION_BITS
    );

    pinMode(
        PIEZO_PIN,
        INPUT
    );

    reading = PiezoReading{};

    filterInitialized = false;
    filteredSignal = 0.0f;
    baseline = 0.0f;
    previousAbsoluteWave = 0.0f;

    lastBreathTime = 0;
    breathHistoryCount = 0;
    breathHistoryWriteIndex = 0;

    memset(
        breathTimes,
        0,
        sizeof(breathTimes)
    );

    acquisitionStartMillis = millis();
    lastSampleMillis = acquisitionStartMillis;

    reading.status = PiezoStatus::INITIALIZING;
    reading.valid = true;

    Serial.println(
        "[PIEZO] Sensor acquisition ready."
    );

    Serial.printf(
        "[PIEZO] Sampling target: %.1f Hz\n",
        PIEZO_SAMPLE_RATE_HZ
    );

    Serial.println(
        "[PIEZO] Deployment policy: deterministic respiration support; NO Piezo ML model."
    );

    return true;
}

void PiezoSensor::update()
{
    const unsigned long now =
        millis();

    if (
        now - lastSampleMillis
        <
        PIEZO_SAMPLE_INTERVAL_MS
    )
    {
        return;
    }

    lastSampleMillis +=
        PIEZO_SAMPLE_INTERVAL_MS;

    if (
        now - lastSampleMillis
        >
        PIEZO_SAMPLE_INTERVAL_MS * 4UL
    )
    {
        lastSampleMillis = now;
    }

    const int rawADC =
        analogRead(
            PIEZO_PIN
        );

    processSample(
        rawADC,
        now
    );

    reading.sampleCount++;

    updateSamplingDiagnostics(
        now
    );
}

void PiezoSensor::processSample(
    int rawADC,
    unsigned long now
)
{
    reading.rawADC =
        rawADC;

    if (
        rawADC < PIEZO_ADC_MIN
        ||
        rawADC > PIEZO_ADC_MAX
    )
    {
        reading.valid = false;
        reading.signalUsable = false;
        reading.status =
            PiezoStatus::INVALID_READING;
        return;
    }

    reading.valid = true;

    const float raw =
        static_cast<float>(
            rawADC
        );

    if (!filterInitialized)
    {
        filteredSignal =
            raw;

        baseline =
            raw;

        previousAbsoluteWave =
            0.0f;

        filterInitialized =
            true;
    }
    else
    {
        filteredSignal +=
            PIEZO_EMA_ALPHA
            *
            (
                raw
                -
                filteredSignal
            );

        baseline +=
            PIEZO_BASELINE_ALPHA
            *
            (
                filteredSignal
                -
                baseline
            );
    }

    const float wave =
        filteredSignal
        -
        baseline;

    reading.filteredSignal =
        filteredSignal;

    reading.baseline =
        baseline;

    reading.respirationWave =
        wave;

    // "signalUsable" here only means the ADC/filter path has
    // initialized and completed the startup settle period.
    // It does NOT mean breathing has been medically validated.
    reading.signalUsable =
        filterInitialized
        &&
        (
            now - acquisitionStartMillis
            >=
            PIEZO_STARTUP_SETTLE_MS
        );

    reading.status =
        reading.signalUsable
            ? PiezoStatus::READY
            : PiezoStatus::INITIALIZING;

    updateBreathingSupport(
        wave,
        now
    );

    previousAbsoluteWave =
        fabsf(
            wave
        );
}

void PiezoSensor::updateBreathingSupport(
    float wave,
    unsigned long now
)
{
    reading.breathDetectedThisSample =
        false;

    const float absoluteWave =
        fabsf(
            wave
        );

    // Detect an excursion crossing the engineering event threshold.
    // Absolute magnitude makes the detector insensitive to PVDF
    // polarity; cooldown suppresses double counting of one cycle.
    const bool currentAbove =
        absoluteWave
        >
        PIEZO_EVENT_THRESHOLD;

    const bool previousAbove =
        previousAbsoluteWave
        >
        PIEZO_EVENT_THRESHOLD;

    if (
        reading.signalUsable
        &&
        currentAbove
        &&
        !previousAbove
        &&
        (
            lastBreathTime == 0
            ||
            now - lastBreathTime
            >=
            PIEZO_BREATH_COOLDOWN_MS
        )
    )
    {
        recordBreath(
            now
        );
    }

    reading.breathTrackingReady =
        breathHistoryCount
        >=
        PIEZO_MIN_BREATHS_FOR_TRACKING;

    if (lastBreathTime == 0)
    {
        reading.noBreathDurationMs =
            now - acquisitionStartMillis;
    }
    else
    {
        reading.noBreathDurationMs =
            now - lastBreathTime;
    }

    reading.breathDetectedRecently =
        reading.breathTrackingReady
        &&
        reading.noBreathDurationMs
        <
        PIEZO_NO_BREATH_SUPPORT_MS;

    // Conservative gate: the timer cannot become a support concern
    // until repeatable breath events were seen first.
    reading.noBreathTimerExceeded =
        reading.signalUsable
        &&
        reading.breathTrackingReady
        &&
        reading.noBreathDurationMs
        >=
        PIEZO_NO_BREATH_SUPPORT_MS;

    reading.estimatedRespirationBPM =
        calculateRollingRespirationBPM();
}

void PiezoSensor::recordBreath(
    unsigned long now
)
{
    reading.breathDetectedThisSample =
        true;

    reading.totalBreaths++;

    reading.lastBreathMillis =
        now;

    lastBreathTime =
        now;

    breathTimes[
        breathHistoryWriteIndex
    ] = now;

    breathHistoryWriteIndex =
        (
            breathHistoryWriteIndex
            +
            1
        )
        %
        BREATH_HISTORY_SIZE;

    if (
        breathHistoryCount
        <
        BREATH_HISTORY_SIZE
    )
    {
        breathHistoryCount++;
    }
}

float PiezoSensor::calculateRollingRespirationBPM() const
{
    if (
        breathHistoryCount
        <
        2
    )
    {
        return NAN;
    }

    const uint8_t oldestIndex =
        breathHistoryCount
        <
        BREATH_HISTORY_SIZE
            ? 0
            : breathHistoryWriteIndex;

    const uint8_t newestIndex =
        (
            breathHistoryWriteIndex
            +
            BREATH_HISTORY_SIZE
            -
            1
        )
        %
        BREATH_HISTORY_SIZE;

    const unsigned long oldest =
        breathTimes[
            oldestIndex
        ];

    const unsigned long newest =
        breathTimes[
            newestIndex
        ];

    if (
        newest
        <=
        oldest
    )
    {
        return NAN;
    }

    const float minutes =
        (
            newest
            -
            oldest
        )
        /
        60000.0f;

    if (
        minutes
        <=
        0.0f
    )
    {
        return NAN;
    }

    return
        static_cast<float>(
            breathHistoryCount - 1
        )
        /
        minutes;
}

void PiezoSensor::updateSamplingDiagnostics(
    unsigned long now
)
{
    const unsigned long elapsed =
        now - acquisitionStartMillis;

    reading.actualSamplingRateHz =
        elapsed
            ?
            (
                reading.sampleCount
                *
                1000.0f
            )
            /
            static_cast<float>(
                elapsed
            )
            :
            0.0f;
}
