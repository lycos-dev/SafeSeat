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
        "[MLX-ML-DIAG] WESAD surrogate model initialized (diagnostic only; NOT fused)."
    );

    Serial.println(
        "[MLX-ML-DIAG] Window: 30 s @ 4 Hz, stride: 15 s."
    );

    Serial.println(
        "[MLX-ML-DIAG] Source: qualified MLX90614 object temperature."
    );

    Serial.println(
        "[MLX-ML-DIAG] 30-s window is median-centered before 38-feature inference.\n[MLX-ML-DIAG] MLX Ta is not model input."
    );

    Serial.println(
        "[MLX-ML-DIAG] Thermal-contrast gate: Object-Ta >= 2.0 C (engineering gate only)."
    );

    Serial.println(
        "[MLX-ML-DIAG] Empty/background windows are NOT sent to the diagnostic model."
    );

    Serial.println(
        "[MLX-ML-DIAG] Gate is provisional/non-medical; final seat calibration still required."
    );

    Serial.println(
        "[MLX-ML-DIAG] Isolation Forest + One-Class SVM ready; Fusion ignores these votes."
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

    reading.warmTargetQualified =
        false;

    reading.targetDeltaC =
        NAN;

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

    // Step 5.4.3 feature dumps are intentionally disabled in
    // Step 5.4.5 so later all-sensor dashboard testing remains
    // compact. The diagnostic model itself still runs.

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

    // ========================================================
    // WARM-TARGET QUALIFICATION
    //
    // The trained WESAD model represents skin temperature. Do
    // not ask it to classify an obvious room/background window.
    // Ambient and Object-Ambient remain runtime/Fusion context;
    // only raw object temperature is still fed into the model.
    //
    // +2 C is a provisional engineering gate based on the current
    // bench separation. It is NOT a medical threshold and must be
    // recalibrated on the final seat/headrest geometry.
    // ========================================================

    float targetDeltaC =
        sensorReading.objectMinusAmbientC;

    bool warmTargetQualified =
        sensorReading.valid
        &&
        isfinite(
            targetDeltaC
        )
        &&
        targetDeltaC
        >=
        WARM_TARGET_MIN_DELTA_C;

    if (
        !warmTargetQualified
    )
    {
        resetWindow(
            MLXMLStatus::WAITING_FOR_WARM_TARGET
        );

        reading.lastProcessedPhysicalSampleCount =
            physicalSampleCount;

        reading.warmTargetQualified =
            false;

        reading.targetDeltaC =
            targetDeltaC;

        return;
    }

    reading.warmTargetQualified =
        true;

    reading.targetDeltaC =
        targetDeltaC;

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

        case MLXMLStatus::WAITING_FOR_WARM_TARGET:
            return "WAITING FOR WARM TARGET";

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
