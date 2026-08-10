#include "C1001ML.h"

#include <math.h>

// ============================================================
// INITIALIZATION
// ============================================================

void C1001ML::begin()
{
    reading =
        C1001MLReading{};

    resetWindow(
        C1001MLStatus::WAITING_FOR_SAMPLE
    );

    Serial.println();
    Serial.println(
        "[C1001-ML] Embedded model initialized."
    );

    Serial.println(
        "[C1001-ML] Window: 30 s @ 1 Hz, stride: 15 s."
    );

    Serial.println(
        "[C1001-ML] Isolation Forest + One-Class SVM ready."
    );
}


// ============================================================
// RESET
//
// Resetting the window also invalidates the previous model
// result. This prevents stale C1001 evidence from surviving an
// occupant change, invalid sensor sample, or motion-artifact
// interval.
// ============================================================

void C1001ML::resetWindow(
    C1001MLStatus status
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
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        hrWindow[i] =
            0.0f;

        rrWindow[i] =
            0.0f;
    }

    reading.valid =
        false;

    reading.status =
        status;

    reading.windowSamplesCollected =
        0;

    reading.samplesUntilNextInference =
        C1001_ML_WINDOW_SAMPLES;

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
// MOTION REJECTION
//
// The C1001 model was trained on physiological HR/RR windows,
// not on radar movement artifacts.
//
// Strong/moderate motion and recovery periods therefore reset
// the model window instead of feeding contaminated samples into
// the anomaly models.
// ============================================================

bool C1001ML::shouldRejectForMotion(
    const C1001Reading &sensorReading
) const
{
    return
        sensorReading.motionArtifactActive
        ||
        sensorReading.status
        ==
        C1001Status::STRONG_MOTION
        ||
        sensorReading.status
        ==
        C1001Status::MODERATE_MOTION
        ||
        sensorReading.status
        ==
        C1001Status::MOTION_RECOVERY;
}


// ============================================================
// ADD SAMPLE
// ============================================================

void C1001ML::addSample(
    float heartRate,
    float respiration
)
{
    hrWindow[
        writeIndex
    ] =
        heartRate;

    rrWindow[
        writeIndex
    ] =
        respiration;

    writeIndex++;

    if (
        writeIndex
        >=
        C1001_ML_WINDOW_SAMPLES
    )
    {
        writeIndex =
            0;
    }

    if (
        windowCount
        <
        C1001_ML_WINDOW_SAMPLES
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
            C1001_ML_WINDOW_SAMPLES
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
            C1001_ML_STRIDE_SAMPLES
        )
        {
            progress =
                C1001_ML_STRIDE_SAMPLES;
        }

        reading.samplesUntilNextInference =
            C1001_ML_STRIDE_SAMPLES
            -
            progress;
    }
}


// ============================================================
// ORDERED WINDOW
//
// Ring-buffer output is oldest -> newest, matching the training
// feature-engineering window order.
// ============================================================

void C1001ML::copyOrderedWindow(
    float heartRate[C1001_ML_WINDOW_SAMPLES],
    float respiration[C1001_ML_WINDOW_SAMPLES]
) const
{
    if (
        windowCount
        <
        C1001_ML_WINDOW_SAMPLES
    )
    {
        return;
    }

    // Once full, writeIndex points to the oldest sample.
    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
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
            C1001_ML_WINDOW_SAMPLES;

        heartRate[i] =
            hrWindow[
                sourceIndex
            ];

        respiration[i] =
            rrWindow[
                sourceIndex
            ];
    }
}


// ============================================================
// RUN INFERENCE
// ============================================================

void C1001ML::runInference()
{
    float orderedHR[
        C1001_ML_WINDOW_SAMPLES
    ];

    float orderedRR[
        C1001_ML_WINDOW_SAMPLES
    ];

    copyOrderedWindow(
        orderedHR,
        orderedRR
    );

    C1001FeatureVector
        features;

    if (
        !featureExtractor.extract(
            orderedHR,
            orderedRR,
            features
        )
    )
    {
        reading.valid =
            false;

        reading.status =
            C1001MLStatus::INFERENCE_ERROR;

        return;
    }

    C1001InferenceResult
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
            C1001MLStatus::INFERENCE_ERROR;

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
            ? C1001MLStatus::READY_STRONG_ANOMALY
            : (
                result.eitherModelAnomaly
                    ? C1001MLStatus::READY_WEAK_ANOMALY
                    : C1001MLStatus::READY_NORMAL
            );

    firstInferenceCompleted =
        true;

    newSamplesSinceInference =
        0;

    reading.samplesUntilNextInference =
        C1001_ML_STRIDE_SAMPLES;
}


// ============================================================
// UPDATE
// ============================================================

void C1001ML::update(
    const C1001Reading &sensorReading
)
{
    // Fast main loop: process each physical 1 Hz C1001 poll only
    // once.
    if (
        sensorReading.sampleSequence
        ==
        reading.lastProcessedSampleSequence
    )
    {
        return;
    }

    reading.lastProcessedSampleSequence =
        sensorReading.sampleSequence;

    if (
        !sensorReading.connected
    )
    {
        resetWindow(
            C1001MLStatus::WAITING_FOR_SAMPLE
        );

        reading.lastProcessedSampleSequence =
            sensorReading.sampleSequence;

        return;
    }

    if (
        !sensorReading.present
    )
    {
        resetWindow(
            C1001MLStatus::WAITING_FOR_OCCUPANT
        );

        reading.lastProcessedSampleSequence =
            sensorReading.sampleSequence;

        return;
    }

    if (
        !sensorReading.warmedUp
    )
    {
        resetWindow(
            C1001MLStatus::WAITING_FOR_WARMUP
        );

        reading.lastProcessedSampleSequence =
            sensorReading.sampleSequence;

        return;
    }

    if (
        shouldRejectForMotion(
            sensorReading
        )
    )
    {
        resetWindow(
            C1001MLStatus::MOTION_RESET
        );

        reading.lastProcessedSampleSequence =
            sensorReading.sampleSequence;

        return;
    }

    // Match the broad validity gate used to build the BIDMC
    // C1001 training dataset. These limits are data-cleaning
    // bounds, NOT medical emergency thresholds.
    const bool trainingValidHeartRate =
        sensorReading.rawHeartRate
        !=
        0
        &&
        sensorReading.rawHeartRate
        !=
        255
        &&
        sensorReading.rawHeartRate
        >=
        30
        &&
        sensorReading.rawHeartRate
        <=
        220;

    const bool trainingValidRespiration =
        sensorReading.rawRespiration
        !=
        0
        &&
        sensorReading.rawRespiration
        !=
        255
        &&
        sensorReading.rawRespiration
        >=
        4
        &&
        sensorReading.rawRespiration
        <=
        60;

    if (
        !trainingValidHeartRate
        ||
        !trainingValidRespiration
    )
    {
        resetWindow(
            C1001MLStatus::INVALID_SAMPLE
        );

        reading.lastProcessedSampleSequence =
            sensorReading.sampleSequence;

        return;
    }

    addSample(
        static_cast<float>(
            sensorReading.rawHeartRate
        ),
        static_cast<float>(
            sensorReading.rawRespiration
        )
    );

    reading.status =
        C1001MLStatus::COLLECTING_WINDOW;

    bool shouldInfer =
        windowCount
        >=
        C1001_ML_WINDOW_SAMPLES
        &&
        (
            !firstInferenceCompleted
            ||
            newSamplesSinceInference
            >=
            C1001_ML_STRIDE_SAMPLES
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

const C1001MLReading&
C1001ML::getReading() const
{
    return reading;
}


const char*
C1001ML::getStatusText() const
{
    switch (
        reading.status
    )
    {
        case C1001MLStatus::WAITING_FOR_SAMPLE:
            return "WAITING FOR SAMPLE";

        case C1001MLStatus::WAITING_FOR_OCCUPANT:
            return "WAITING FOR OCCUPANT";

        case C1001MLStatus::WAITING_FOR_WARMUP:
            return "WAITING FOR C1001 WARM-UP";

        case C1001MLStatus::INVALID_SAMPLE:
            return "RESET - INVALID SENSOR SAMPLE";

        case C1001MLStatus::MOTION_RESET:
            return "RESET - MOTION ARTIFACT";

        case C1001MLStatus::COLLECTING_WINDOW:
            return "COLLECTING 30 s WINDOW";

        case C1001MLStatus::READY_NORMAL:
            return "READY - NORMAL";

        case C1001MLStatus::READY_WEAK_ANOMALY:
            return "READY - WEAK ANOMALY";

        case C1001MLStatus::READY_STRONG_ANOMALY:
            return "READY - STRONG ANOMALY";

        case C1001MLStatus::INFERENCE_ERROR:
            return "INFERENCE ERROR";

        default:
            return "UNKNOWN";
    }
}
