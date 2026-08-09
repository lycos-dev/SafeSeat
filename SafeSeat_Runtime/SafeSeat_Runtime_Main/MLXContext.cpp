#include "MLXContext.h"

#include <math.h>


MLXContext::MLXContext()
{
}


void MLXContext::begin()
{
    reading =
        MLXContextReading{};

    reading.available =
        true;

    reading.baselineSamplesRequired =
        BASELINE_SAMPLE_COUNT;

    resetBaseline();

    Serial.println();
    Serial.println(
        "[MLX-CONTEXT] Step 5.4.5 initialized."
    );
    Serial.println(
        "[MLX-CONTEXT] Primary signal: filtered MLX OBJECT temperature."
    );
    Serial.println(
        "[MLX-CONTEXT] MLX Ta is context only; Object-Ta is NOT body temperature."
    );
    Serial.println(
        "[MLX-CONTEXT] 30-s filtered-object baseline required before Fusion context is valid."
    );
    Serial.println(
        "[MLX-CONTEXT] Context-change reference: +/-1.85 C from baseline (FDA p99 repeated-round range; non-medical)."
    );
    Serial.println(
        "[MLX-CONTEXT] WESAD IF/OCSVM remains diagnostic-only and is NOT fused."
    );
}


void MLXContext::resetBaseline()
{
    baselineCount =
        0;

    reading.valid =
        false;

    reading.baselineReady =
        false;

    reading.contextChange =
        false;

    reading.baselineObjectC =
        NAN;

    reading.deviationFromBaselineC =
        NAN;

    reading.baselineSamplesCollected =
        0;
}


float MLXContext::median(
    float *values,
    uint16_t count
)
{
    if (
        values == nullptr
        ||
        count == 0
    )
    {
        return NAN;
    }

    // BASELINE_SAMPLE_COUNT is only 120, so an in-place
    // insertion sort is simple and deterministic on ESP32.
    for (
        uint16_t i = 1;
        i < count;
        i++
    )
    {
        float key =
            values[i];

        int j =
            static_cast<int>(i)
            -
            1;

        while (
            j >= 0
            &&
            values[j] > key
        )
        {
            values[j + 1] =
                values[j];

            j--;
        }

        values[j + 1] =
            key;
    }

    if (
        count % 2U
        ==
        1U
    )
    {
        return values[
            count / 2U
        ];
    }

    return 0.5f
        *
        (
            values[
                count / 2U - 1U
            ]
            +
            values[
                count / 2U
            ]
        );
}


void MLXContext::update(
    const MLXReading& sensorReading
)
{
    reading.lastUpdateMillis =
        millis();

    if (
        !sensorReading.connected
        ||
        !sensorReading.valid
        ||
        !isfinite(
            sensorReading.filteredObjectC
        )
        ||
        !isfinite(
            sensorReading.filteredAmbientC
        )
        ||
        !isfinite(
            sensorReading.objectMinusAmbientC
        )
    )
    {
        reading.valid =
            false;

        reading.thermalContrastQualified =
            false;

        reading.status =
            MLXContextStatus::UNAVAILABLE;

        return;
    }

    reading.filteredObjectC =
        sensorReading.filteredObjectC;

    reading.sensorTaC =
        sensorReading.filteredAmbientC;

    reading.thermalContrastC =
        sensorReading.objectMinusAmbientC;

    reading.thermalContrastQualified =
        reading.thermalContrastC
        >=
        MIN_THERMAL_CONTRAST_C;

    // Do not repeatedly consume the same 4-Hz physical sample
    // when loop() runs faster than MLX acquisition.
    if (
        sensorReading.acceptedSampleCount
        ==
        reading.lastProcessedAcceptedSampleCount
    )
    {
        return;
    }

    reading.lastProcessedAcceptedSampleCount =
        sensorReading.acceptedSampleCount;

    if (
        !reading.thermalContrastQualified
    )
    {
        // Conservative behavior: a lost/non-warm target clears
        // the session baseline so a later target starts cleanly.
        resetBaseline();

        reading.status =
            MLXContextStatus::WAITING_FOR_THERMAL_TARGET;

        return;
    }

    if (
        !reading.baselineReady
    )
    {
        if (
            baselineCount
            <
            BASELINE_SAMPLE_COUNT
        )
        {
            baselineBuffer[
                baselineCount
            ] =
                reading.filteredObjectC;

            baselineCount++;
        }

        reading.baselineSamplesCollected =
            baselineCount;

        reading.status =
            MLXContextStatus::BUILDING_BASELINE;

        if (
            baselineCount
            >=
            BASELINE_SAMPLE_COUNT
        )
        {
            float working[
                BASELINE_SAMPLE_COUNT
            ];

            for (
                uint16_t i = 0;
                i < BASELINE_SAMPLE_COUNT;
                i++
            )
            {
                working[i] =
                    baselineBuffer[i];
            }

            reading.baselineObjectC =
                median(
                    working,
                    BASELINE_SAMPLE_COUNT
                );

            reading.baselineReady =
                isfinite(
                    reading.baselineObjectC
                );

            reading.valid =
                reading.baselineReady;

            reading.deviationFromBaselineC =
                reading.filteredObjectC
                -
                reading.baselineObjectC;

            reading.contextChange =
                reading.valid
                &&
                fabsf(
                    reading.deviationFromBaselineC
                )
                >
                FDA_CONTEXT_CHANGE_DELTA_C;

            reading.status =
                reading.contextChange
                    ? MLXContextStatus::CONTEXT_CHANGE
                    : MLXContextStatus::STABLE;
        }

        return;
    }

    reading.valid =
        true;

    reading.deviationFromBaselineC =
        reading.filteredObjectC
        -
        reading.baselineObjectC;

    reading.contextChange =
        fabsf(
            reading.deviationFromBaselineC
        )
        >
        FDA_CONTEXT_CHANGE_DELTA_C;

    reading.status =
        reading.contextChange
            ? MLXContextStatus::CONTEXT_CHANGE
            : MLXContextStatus::STABLE;
}


const MLXContextReading&
MLXContext::getReading() const
{
    return reading;
}


const char*
MLXContext::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case MLXContextStatus::UNAVAILABLE:
            return "UNAVAILABLE";

        case MLXContextStatus::WAITING_FOR_THERMAL_TARGET:
            return "WAITING FOR THERMAL TARGET";

        case MLXContextStatus::BUILDING_BASELINE:
            return "BUILDING 30 s BASELINE";

        case MLXContextStatus::STABLE:
            return "STABLE VS SESSION BASELINE";

        case MLXContextStatus::CONTEXT_CHANGE:
            return "CONTEXT CHANGE VS SESSION BASELINE";

        default:
            return "UNKNOWN";
    }
}
