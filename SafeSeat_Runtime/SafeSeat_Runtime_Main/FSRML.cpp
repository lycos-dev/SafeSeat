#include "FSRML.h"

#include <math.h>

// ============================================================
// INITIALIZATION
// ============================================================

void FSRML::begin()
{
    reading =
        FSRMLReading{};

    resetWindow(
        FSRMLStatus::WAITING_FOR_SAMPLE
    );

    Serial.println();
    Serial.println(
        "[FSR-ML] Embedded Step 5.5 model initialized."
    );

    Serial.println(
        "[FSR-ML] Runtime alignment: 23 completed FSR frames @ ~4.5 Hz."
    );

    Serial.println(
        "[FSR-ML] Stride: 5 completed frames (~1.1 s)."
    );

    Serial.println(
        "[FSR-ML] Representation: per-frame 9-sensor pressure shares."
    );

    Serial.println(
        "[FSR-ML] Absolute pressure magnitude is NOT model input."
    );

    Serial.println(
        "[FSR-ML] Isolation Forest + One-Class SVM ready."
    );
}


// ============================================================
// RESET
//
// Stale FSR anomaly evidence must not survive an occupant loss,
// invalid frame, or lack of usable seat pressure.
// ============================================================

void FSRML::resetWindow(
    FSRMLStatus status
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
        uint16_t frame = 0;
        frame < FSR_ML_WINDOW_SAMPLES;
        frame++
    )
    {
        for (
            uint8_t sensor = 0;
            sensor < NUM_FSR;
            sensor++
        )
        {
            pressureWindow[
                frame
            ][
                sensor
            ] =
                0.0f;
        }
    }

    reading.valid =
        false;

    reading.status =
        status;

    reading.windowSamplesCollected =
        0;

    reading.samplesUntilNextInference =
        FSR_ML_WINDOW_SAMPLES;

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
// SAMPLE QUALITY / OCCUPANCY GATE
//
// ChairPose training windows contain seated pressure patterns.
// We therefore do not classify an empty/zero-pressure frame.
//
// This gate is NOT a medical/anomaly threshold. C1001 presence,
// calibrated FSR occupancy, or calibrated FSR back contact only
// establishes that a seated-pressure window is meaningful.
// ============================================================

bool FSRML::sampleIsUsable(
    const FSRReading &sensorReading,
    bool occupantPresent
) const
{
    if (
        !sensorReading.connected
        ||
        !sensorReading.calibrated
        ||
        !sensorReading.valid
    )
    {
        return false;
    }

    const bool occupancyQualified =
        occupantPresent
        ||
        sensorReading.occupied
        ||
        sensorReading.backContact;

    if (
        !occupancyQualified
    )
    {
        return false;
    }

    double total =
        0.0;

    for (
        uint8_t sensor = 0;
        sensor < NUM_FSR;
        sensor++
    )
    {
        float value =
            sensorReading.pressure[
                sensor
            ];

        if (
            !isfinite(
                value
            )
            ||
            value
            <
            0.0f
        )
        {
            return false;
        }

        total +=
            static_cast<double>(
                value
            );
    }

    return
        total
        >
        1.0e-6;
}


// ============================================================
// ADD COMPLETED FSR FRAME
// ============================================================

void FSRML::addSample(
    const FSRReading &sensorReading
)
{
    for (
        uint8_t sensor = 0;
        sensor < NUM_FSR;
        sensor++
    )
    {
        pressureWindow[
            writeIndex
        ][
            sensor
        ] =
            sensorReading.pressure[
                sensor
            ];
    }

    writeIndex++;

    if (
        writeIndex
        >=
        FSR_ML_WINDOW_SAMPLES
    )
    {
        writeIndex =
            0;
    }

    if (
        windowCount
        <
        FSR_ML_WINDOW_SAMPLES
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
            FSR_ML_WINDOW_SAMPLES
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
            FSR_ML_STRIDE_SAMPLES
        )
        {
            progress =
                FSR_ML_STRIDE_SAMPLES;
        }

        reading.samplesUntilNextInference =
            FSR_ML_STRIDE_SAMPLES
            -
            progress;
    }
}


// ============================================================
// ORDERED 23-FRAME WINDOW
// ============================================================

void FSRML::copyOrderedWindow(
    float output[
        FSR_ML_WINDOW_SAMPLES
    ][
        NUM_FSR
    ]
) const
{
    if (
        windowCount
        <
        FSR_ML_WINDOW_SAMPLES
    )
    {
        return;
    }

    for (
        uint16_t frame = 0;
        frame < FSR_ML_WINDOW_SAMPLES;
        frame++
    )
    {
        uint16_t sourceIndex =
            (
                writeIndex
                +
                frame
            )
            %
            FSR_ML_WINDOW_SAMPLES;

        for (
            uint8_t sensor = 0;
            sensor < NUM_FSR;
            sensor++
        )
        {
            output[
                frame
            ][
                sensor
            ] =
                pressureWindow[
                    sourceIndex
                ][
                    sensor
                ];
        }
    }
}


// ============================================================
// INFERENCE
// ============================================================

void FSRML::runInference()
{
    float orderedPressure[
        FSR_ML_WINDOW_SAMPLES
    ][
        NUM_FSR
    ];

    copyOrderedWindow(
        orderedPressure
    );

    FSRFeatureVector
        features;

    if (
        !featureExtractor.extract(
            orderedPressure,
            features
        )
    )
    {
        reading.valid =
            false;

        reading.status =
            FSRMLStatus::INFERENCE_ERROR;

        return;
    }

    FSRInferenceResult
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
            FSRMLStatus::INFERENCE_ERROR;

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
            ? FSRMLStatus::READY_STRONG_ANOMALY
            : (
                result.eitherModelAnomaly
                    ? FSRMLStatus::READY_WEAK_ANOMALY
                    : FSRMLStatus::READY_NORMAL
            );

    firstInferenceCompleted =
        true;

    newSamplesSinceInference =
        0;

    reading.samplesUntilNextInference =
        FSR_ML_STRIDE_SAMPLES;
}


// ============================================================
// UPDATE
//
// Process each completed physical FSR frame exactly once by
// tracking FSRReading.lastSampleMillis.
// ============================================================

void FSRML::update(
    const FSRReading &sensorReading,
    bool occupantPresent
)
{
    if (
        sensorReading.lastSampleMillis
        ==
        0UL
        ||
        sensorReading.lastSampleMillis
        ==
        reading.lastProcessedSampleMillis
    )
    {
        return;
    }

    reading.lastProcessedSampleMillis =
        sensorReading.lastSampleMillis;

    if (
        !sensorReading.connected
        ||
        !sensorReading.calibrated
        ||
        !sensorReading.valid
    )
    {
        resetWindow(
            FSRMLStatus::WAITING_FOR_SAMPLE
        );

        reading.lastProcessedSampleMillis =
            sensorReading.lastSampleMillis;

        return;
    }

    const bool occupancyQualified =
        occupantPresent
        ||
        sensorReading.occupied
        ||
        sensorReading.backContact;

    if (
        !occupancyQualified
    )
    {
        resetWindow(
            FSRMLStatus::WAITING_FOR_OCCUPANT
        );

        reading.lastProcessedSampleMillis =
            sensorReading.lastSampleMillis;

        return;
    }

    if (
        !sampleIsUsable(
            sensorReading,
            occupantPresent
        )
    )
    {
        resetWindow(
            FSRMLStatus::INVALID_SAMPLE
        );

        reading.lastProcessedSampleMillis =
            sensorReading.lastSampleMillis;

        return;
    }

    addSample(
        sensorReading
    );

    reading.status =
        FSRMLStatus::COLLECTING_WINDOW;

    const bool shouldInfer =
        windowCount
        >=
        FSR_ML_WINDOW_SAMPLES
        &&
        (
            !firstInferenceCompleted
            ||
            newSamplesSinceInference
            >=
            FSR_ML_STRIDE_SAMPLES
        );

    if (
        shouldInfer
    )
    {
        runInference();
    }
}


// ============================================================
// GETTERS
// ============================================================

const FSRMLReading&
FSRML::getReading() const
{
    return reading;
}


const char*
FSRML::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case FSRMLStatus::WAITING_FOR_SAMPLE:
            return "WAITING FOR VALID FSR FRAME";

        case FSRMLStatus::WAITING_FOR_OCCUPANT:
            return "WAITING FOR OCCUPANT / PRESSURE";

        case FSRMLStatus::INVALID_SAMPLE:
            return "RESET - INVALID / ZERO PRESSURE FRAME";

        case FSRMLStatus::COLLECTING_WINDOW:
            return "COLLECTING ~5 s WINDOW";

        case FSRMLStatus::READY_NORMAL:
            return "READY - NORMAL";

        case FSRMLStatus::READY_WEAK_ANOMALY:
            return "READY - WEAK ANOMALY";

        case FSRMLStatus::READY_STRONG_ANOMALY:
            return "READY - STRONG ANOMALY";

        case FSRMLStatus::INFERENCE_ERROR:
            return "INFERENCE ERROR";

        default:
            return "UNKNOWN";
    }
}
