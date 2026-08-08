#include "MLXML.h"

#include <math.h>

// ============================================================
// INITIALIZATION
// ============================================================

void MLXML::begin()
{
    reading =
        MLXMLReading{};

    resetWindow(
        MLXMLStatus::WAITING_FOR_SAMPLE
    );

    Serial.println();
    Serial.println(
        "[MLX-ML] Embedded model initialized."
    );

    Serial.println(
        "[MLX-ML] Window: 30 s @ 4 Hz, stride: 15 s."
    );

    Serial.println(
        "[MLX-ML] Source: raw MLX90614 object temperature."
    );

    Serial.println(
        "[MLX-ML] Ambient temperature remains Fusion context only."
    );

    Serial.println(
        "[MLX-ML] Isolation Forest + One-Class SVM ready."
    );
}


// ============================================================
// RESET
// ============================================================

void MLXML::resetWindow(
    MLXMLStatus status
)
{
    windowCount =
        0;

    writeIndex =
        0;

    newSamplesSinceInference =
        0;

    firstInferenceCompleted =
        false;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        objectWindow[i] =
            NAN;
    }

    reading.valid =
        false;

    reading.status =
        status;

    reading.windowSamplesCollected =
        0;

    reading.samplesUntilNextInference =
        MLX_ML_WINDOW_SAMPLES;

    reading.lastFiniteSampleCount =
        0;

    reading.isolationForestDecision =
        NAN;

    reading.oneClassSVMDecision =
        NAN;

    reading.isolationForestAnomaly =
        false;

    reading.oneClassSVMAnomaly =
        false;

    reading.bothModelsAnomaly =
        false;

    reading.eitherModelAnomaly =
        false;
}


// ============================================================
// ADD SAMPLE
// ============================================================

void MLXML::addSample(
    float objectTemperature
)
{
    objectWindow[
        writeIndex
    ] =
        objectTemperature;

    writeIndex++;

    if (
        writeIndex
        >=
        MLX_ML_WINDOW_SAMPLES
    )
    {
        writeIndex =
            0;
    }

    if (
        windowCount
        <
        MLX_ML_WINDOW_SAMPLES
    )
    {
        windowCount++;
    }

    newSamplesSinceInference++;

    reading.windowSamplesCollected =
        windowCount;

    if (
        !firstInferenceCompleted
    )
    {
        reading.samplesUntilNextInference =
            MLX_ML_WINDOW_SAMPLES
            -
            windowCount;
    }
    else
    {
        uint16_t progress =
            newSamplesSinceInference;

        if (
            progress
            >
            MLX_ML_STRIDE_SAMPLES
        )
        {
            progress =
                MLX_ML_STRIDE_SAMPLES;
        }

        reading.samplesUntilNextInference =
            MLX_ML_STRIDE_SAMPLES
            -
            progress;
    }
}


// ============================================================
// ORDERED WINDOW
// ============================================================

void MLXML::copyOrderedWindow(
    float objectTemperature[MLX_ML_WINDOW_SAMPLES]
) const
{
    if (
        windowCount
        <
        MLX_ML_WINDOW_SAMPLES
    )
    {
        return;
    }

    // Once full, writeIndex points to the oldest sample.
    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        uint16_t sourceIndex =
            (
                writeIndex
                +
                i
            )
            %
            MLX_ML_WINDOW_SAMPLES;

        objectTemperature[i] =
            objectWindow[
                sourceIndex
            ];
    }
}


// ============================================================
// RUN INFERENCE
// ============================================================

void MLXML::runInference()
{
    float orderedObjectTemperature[
        MLX_ML_WINDOW_SAMPLES
    ];

    copyOrderedWindow(
        orderedObjectTemperature
    );

    MLXFeatureVector
        features;

    if (
        !featureExtractor.extract(
            orderedObjectTemperature,
            features
        )
        ||
        !features.valid
    )
    {
        reading.valid =
            false;

        reading.status =
            MLXMLStatus::INFERENCE_ERROR;

        return;
    }

    reading.lastFiniteSampleCount =
        features.finiteSampleCount;

    // Runtime signal-quality gate. The training configuration
    // accepted windows with at least 60 samples; do not create
    // model evidence from a severely missing 30-second window.
    if (
        features.finiteSampleCount
        <
        MIN_FINITE_SAMPLES_FOR_INFERENCE
    )
    {
        reading.valid =
            false;

        reading.status =
            MLXMLStatus::INSUFFICIENT_VALID_DATA;

        firstInferenceCompleted =
            true;

        newSamplesSinceInference =
            0;

        reading.samplesUntilNextInference =
            MLX_ML_STRIDE_SAMPLES;

        return;
    }

    MLXInferenceResult
        result;

    if (
        !inference.predict(
            features.values,
            result
        )
        ||
        !result.valid
    )
    {
        reading.valid =
            false;

        reading.status =
            MLXMLStatus::INFERENCE_ERROR;

        return;
    }

    reading.valid =
        true;

    reading.isolationForestDecision =
        result.isolationForestDecision;

    reading.oneClassSVMDecision =
        result.oneClassSVMDecision;

    reading.isolationForestAnomaly =
        result.isolationForestAnomaly;

    reading.oneClassSVMAnomaly =
        result.oneClassSVMAnomaly;

    reading.bothModelsAnomaly =
        result.bothModelsAnomaly;

    reading.eitherModelAnomaly =
        result.eitherModelAnomaly;

    reading.windowsEvaluated++;

    reading.lastInferenceMillis =
        millis();

    reading.status =
        result.bothModelsAnomaly
            ? MLXMLStatus::READY_STRONG_ANOMALY
            : (
                result.eitherModelAnomaly
                    ? MLXMLStatus::READY_WEAK_ANOMALY
                    : MLXMLStatus::READY_NORMAL
            );

    firstInferenceCompleted =
        true;

    newSamplesSinceInference =
        0;

    reading.samplesUntilNextInference =
        MLX_ML_STRIDE_SAMPLES;
}


// ============================================================
// UPDATE
// ============================================================

void MLXML::update(
    const MLXReading &sensorReading
)
{
    unsigned long physicalSampleCount =
        sensorReading.acceptedSampleCount
        +
        sensorReading.rejectedSampleCount;

    if (
        physicalSampleCount
        ==
        reading.lastProcessedPhysicalSampleCount
    )
    {
        return;
    }

    reading.lastProcessedPhysicalSampleCount =
        physicalSampleCount;

    if (
        !sensorReading.connected
    )
    {
        resetWindow(
            MLXMLStatus::SENSOR_UNAVAILABLE
        );

        reading.lastProcessedPhysicalSampleCount =
            physicalSampleCount;

        return;
    }

    float objectTemperature =
        sensorReading.rawObjectC;

    // Only reject impossible/non-finite hardware values here.
    // Values outside the WESAD 20..45 C data range are retained
    // so the trained validity features can represent them.
    if (
        !isfinite(
            objectTemperature
        )
        ||
        objectTemperature
        <
        PHYSICAL_OBJECT_MIN_C
        ||
        objectTemperature
        >
        PHYSICAL_OBJECT_MAX_C
    )
    {
        objectTemperature =
            NAN;
    }

    addSample(
        objectTemperature
    );

    reading.status =
        MLXMLStatus::COLLECTING_WINDOW;

    bool shouldInfer =
        windowCount
        >=
        MLX_ML_WINDOW_SAMPLES
        &&
        (
            !firstInferenceCompleted
            ||
            newSamplesSinceInference
            >=
            MLX_ML_STRIDE_SAMPLES
        );

    if (
        shouldInfer
    )
    {
        runInference();
    }
}


// ============================================================
// READING ACCESS
// ============================================================

const MLXMLReading&
MLXML::getReading() const
{
    return reading;
}


// ============================================================
// STATUS TEXT
// ============================================================

const char*
MLXML::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case MLXMLStatus::WAITING_FOR_SAMPLE:
            return "WAITING FOR SAMPLE";

        case MLXMLStatus::SENSOR_UNAVAILABLE:
            return "SENSOR UNAVAILABLE";

        case MLXMLStatus::COLLECTING_WINDOW:
            return "COLLECTING 30 s WINDOW";

        case MLXMLStatus::INSUFFICIENT_VALID_DATA:
            return "INSUFFICIENT VALID DATA";

        case MLXMLStatus::READY_NORMAL:
            return "READY - NORMAL";

        case MLXMLStatus::READY_WEAK_ANOMALY:
            return "READY - WEAK ANOMALY";

        case MLXMLStatus::READY_STRONG_ANOMALY:
            return "READY - STRONG ANOMALY";

        case MLXMLStatus::INFERENCE_ERROR:
            return "INFERENCE ERROR";

        default:
            return "UNKNOWN";
    }
}
