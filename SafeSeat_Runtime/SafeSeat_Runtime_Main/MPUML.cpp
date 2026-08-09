#include "MPUML.h"

#include <math.h>

// ============================================================
// INITIALIZATION
// ============================================================

void MPUML::begin()
{
    reading =
        MPUMLReading{};

    lastProcessedSampleCount = 0;

    resetBaseline();

    resetWindow(
        MPUMLStatus::CALIBRATING_STATIONARY_BASELINE
    );

    Serial.println();
    Serial.println(
        "[MPU-ML] Embedded Step 5.6.1 model initialized."
    );

    Serial.println(
        "[MPU-ML] Model contract: 80 samples / 1.0 s, stride 40 samples."
    );

    Serial.println(
        "[MPU-ML] Features: 198, RobustScaler + IF + One-Class SVM."
    );

    Serial.println(
        "[MPU-ML] Startup calibration: first 80 runtime samples estimate stationary accel/gyro offsets."
    );

    Serial.println(
        "[MPU-ML] Model-domain accel: offset-removed m/s^2; gyro: offset-removed rad/s."
    );

    Serial.println(
        "[MPU-ML] Fusion role: vehicle/road-motion artifact context only."
    );
}


// ============================================================
// RESET MODEL WINDOW
// ============================================================

void MPUML::resetWindow(
    MPUMLStatus status
)
{
    windowCount = 0;
    writeIndex = 0;
    newSamplesSinceInference = 0;
    firstInferenceCompleted = false;

    for (
        uint16_t i = 0;
        i < MPU_ML_WINDOW_SAMPLES;
        i++
    )
    {
        sampleWindow[i] =
            MPUModelSample{};
    }

    reading.valid = false;
    reading.status = status;

    reading.windowSamplesCollected = 0;
    reading.samplesUntilNextInference =
        MPU_ML_WINDOW_SAMPLES;

    reading.isolationForestDecision = NAN;
    reading.oneClassSVMDecision = NAN;

    reading.isolationForestAnomaly = false;
    reading.oneClassSVMAnomaly = false;
    reading.bothModelsAnomaly = false;
    reading.eitherModelAnomaly = false;
}


// ============================================================
// RESET / ACCUMULATE STATIONARY OFFSET BASELINE
//
// The Road Data model was trained from calibrated smartphone
// accelerometer/gyroscope channels whose stationary offsets were
// removed. SafeSeat therefore estimates the fixed MPU6050
// stationary gravity/bias vector at startup before model windows
// are collected.
//
// This is an engineering calibration, not an anomaly threshold.
// ============================================================

void MPUML::resetBaseline()
{
    baselineAccelXSum = 0.0;
    baselineAccelYSum = 0.0;
    baselineAccelZSum = 0.0;

    baselineGyroXSum = 0.0;
    baselineGyroYSum = 0.0;
    baselineGyroZSum = 0.0;

    baselineAccelXG = 0.0f;
    baselineAccelYG = 0.0f;
    baselineAccelZG = 0.0f;

    baselineGyroXDps = 0.0f;
    baselineGyroYDps = 0.0f;
    baselineGyroZDps = 0.0f;

    reading.stationaryBaselineReady = false;
    reading.baselineSamplesCollected = 0;
}


void MPUML::accumulateBaseline(
    const MPUReading &sensorReading
)
{
    baselineAccelXSum +=
        static_cast<double>(sensorReading.accelX);

    baselineAccelYSum +=
        static_cast<double>(sensorReading.accelY);

    baselineAccelZSum +=
        static_cast<double>(sensorReading.accelZ);

    baselineGyroXSum +=
        static_cast<double>(sensorReading.gyroX);

    baselineGyroYSum +=
        static_cast<double>(sensorReading.gyroY);

    baselineGyroZSum +=
        static_cast<double>(sensorReading.gyroZ);

    reading.baselineSamplesCollected++;

    if (
        reading.baselineSamplesCollected
        <
        MPU_ML_BASELINE_SAMPLES
    )
    {
        reading.status =
            MPUMLStatus::CALIBRATING_STATIONARY_BASELINE;

        return;
    }

    const double divisor =
        static_cast<double>(
            MPU_ML_BASELINE_SAMPLES
        );

    baselineAccelXG =
        static_cast<float>(
            baselineAccelXSum / divisor
        );

    baselineAccelYG =
        static_cast<float>(
            baselineAccelYSum / divisor
        );

    baselineAccelZG =
        static_cast<float>(
            baselineAccelZSum / divisor
        );

    baselineGyroXDps =
        static_cast<float>(
            baselineGyroXSum / divisor
        );

    baselineGyroYDps =
        static_cast<float>(
            baselineGyroYSum / divisor
        );

    baselineGyroZDps =
        static_cast<float>(
            baselineGyroZSum / divisor
        );

    reading.stationaryBaselineReady = true;

    resetWindow(
        MPUMLStatus::COLLECTING_WINDOW
    );

    // resetWindow intentionally clears model readiness only.
    reading.stationaryBaselineReady = true;
    reading.baselineSamplesCollected =
        MPU_ML_BASELINE_SAMPLES;
}


// ============================================================
// CONVERT MPU6050 RUNTIME VALUES INTO TRAINING-DOMAIN UNITS
//
// Runtime acquisition:
//     accel = g including gravity
//     gyro  = deg/s
//
// Model domain:
//     accel = m/s^2 after startup stationary offset removal
//     gyro  = rad/s after startup stationary bias removal
// ============================================================

MPUModelSample MPUML::normalizeSample(
    const MPUReading &sensorReading
) const
{
    MPUModelSample output;

    output.accelX =
        (
            sensorReading.accelX -
            baselineAccelXG
        )
        *
        STANDARD_GRAVITY_MPS2;

    output.accelY =
        (
            sensorReading.accelY -
            baselineAccelYG
        )
        *
        STANDARD_GRAVITY_MPS2;

    output.accelZ =
        (
            sensorReading.accelZ -
            baselineAccelZG
        )
        *
        STANDARD_GRAVITY_MPS2;

    output.gyroX =
        (
            sensorReading.gyroX -
            baselineGyroXDps
        )
        *
        DEG_TO_RAD_FACTOR;

    output.gyroY =
        (
            sensorReading.gyroY -
            baselineGyroYDps
        )
        *
        DEG_TO_RAD_FACTOR;

    output.gyroZ =
        (
            sensorReading.gyroZ -
            baselineGyroZDps
        )
        *
        DEG_TO_RAD_FACTOR;

    return output;
}


// ============================================================
// ADD MODEL-DOMAIN SAMPLE
// ============================================================

void MPUML::addSample(
    const MPUModelSample &sample
)
{
    sampleWindow[writeIndex] =
        sample;

    writeIndex++;

    if (
        writeIndex >=
        MPU_ML_WINDOW_SAMPLES
    )
    {
        writeIndex = 0;
    }

    if (
        windowCount <
        MPU_ML_WINDOW_SAMPLES
    )
    {
        windowCount++;
    }

    newSamplesSinceInference++;

    reading.windowSamplesCollected =
        windowCount;

    if (!firstInferenceCompleted)
    {
        reading.samplesUntilNextInference =
            MPU_ML_WINDOW_SAMPLES -
            windowCount;
    }
    else
    {
        uint16_t progress =
            newSamplesSinceInference;

        if (
            progress >
            MPU_ML_STRIDE_SAMPLES
        )
        {
            progress =
                MPU_ML_STRIDE_SAMPLES;
        }

        reading.samplesUntilNextInference =
            MPU_ML_STRIDE_SAMPLES -
            progress;
    }
}


// ============================================================
// ORDERED WINDOW
// ============================================================

void MPUML::copyOrderedWindow(
    MPUModelSample output[
        MPU_ML_WINDOW_SAMPLES
    ]
) const
{
    if (
        windowCount <
        MPU_ML_WINDOW_SAMPLES
    )
    {
        return;
    }

    for (
        uint16_t i = 0;
        i < MPU_ML_WINDOW_SAMPLES;
        i++
    )
    {
        const uint16_t sourceIndex =
            (
                writeIndex +
                i
            )
            %
            MPU_ML_WINDOW_SAMPLES;

        output[i] =
            sampleWindow[sourceIndex];
    }
}


// ============================================================
// INFERENCE
// ============================================================

void MPUML::runInference()
{
    MPUModelSample ordered[
        MPU_ML_WINDOW_SAMPLES
    ];

    copyOrderedWindow(ordered);

    MPUFeatureVector features;

    if (
        !featureExtractor.extract(
            ordered,
            features
        )
    )
    {
        reading.valid = false;
        reading.status =
            MPUMLStatus::INFERENCE_ERROR;

        return;
    }

    MPUInferenceResult result;

    if (
        !inference.predict(
            features.values,
            result
        )
        ||
        !result.valid
    )
    {
        reading.valid = false;
        reading.status =
            MPUMLStatus::INFERENCE_ERROR;

        return;
    }

    reading.valid = true;

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

    if (
        result.bothModelsAnomaly
    )
    {
        reading.status =
            MPUMLStatus::READY_STRONG_ROAD_MOTION;
    }
    else if (
        result.eitherModelAnomaly
    )
    {
        reading.status =
            MPUMLStatus::READY_WEAK_ROAD_MOTION;
    }
    else
    {
        reading.status =
            MPUMLStatus::READY_NORMAL;
    }

    firstInferenceCompleted = true;
    newSamplesSinceInference = 0;
    reading.samplesUntilNextInference =
        MPU_ML_STRIDE_SAMPLES;
}


// ============================================================
// UPDATE
// ============================================================

void MPUML::update(
    const MPUReading &sensorReading
)
{
    if (
        !sensorReading.connected ||
        !sensorReading.valid
    )
    {
        reading.valid = false;
        reading.status =
            MPUMLStatus::INVALID_SAMPLE;

        return;
    }

    // Process every physical MPU acquisition exactly once.
    if (
        sensorReading.sampleCount ==
        lastProcessedSampleCount
    )
    {
        return;
    }

    lastProcessedSampleCount =
        sensorReading.sampleCount;

    if (
        !reading.stationaryBaselineReady
    )
    {
        accumulateBaseline(
            sensorReading
        );

        return;
    }

    const MPUModelSample sample =
        normalizeSample(
            sensorReading
        );

    if (
        !isfinite(sample.accelX) ||
        !isfinite(sample.accelY) ||
        !isfinite(sample.accelZ) ||
        !isfinite(sample.gyroX) ||
        !isfinite(sample.gyroY) ||
        !isfinite(sample.gyroZ)
    )
    {
        reading.valid = false;
        reading.status =
            MPUMLStatus::INVALID_SAMPLE;

        return;
    }

    addSample(sample);

    if (
        windowCount <
        MPU_ML_WINDOW_SAMPLES
    )
    {
        reading.valid = false;
        reading.status =
            MPUMLStatus::COLLECTING_WINDOW;

        return;
    }

    if (
        !firstInferenceCompleted ||
        newSamplesSinceInference >=
            MPU_ML_STRIDE_SAMPLES
    )
    {
        runInference();
    }
}


// ============================================================
// GETTERS
// ============================================================

const MPUMLReading&
MPUML::getReading() const
{
    return reading;
}


const char*
MPUML::getStatusText() const
{
    switch (reading.status)
    {
        case MPUMLStatus::WAITING_FOR_SAMPLE:
            return "WAITING FOR SAMPLE";

        case MPUMLStatus::CALIBRATING_STATIONARY_BASELINE:
            return "CALIBRATING STATIONARY BASELINE";

        case MPUMLStatus::INVALID_SAMPLE:
            return "INVALID SAMPLE";

        case MPUMLStatus::COLLECTING_WINDOW:
            return "COLLECTING 1 s WINDOW";

        case MPUMLStatus::READY_NORMAL:
            return "READY - NORMAL ROAD MOTION";

        case MPUMLStatus::READY_WEAK_ROAD_MOTION:
            return "READY - WEAK ROAD/MOTION ANOMALY";

        case MPUMLStatus::READY_STRONG_ROAD_MOTION:
            return "READY - STRONG ROAD/MOTION ANOMALY";

        case MPUMLStatus::INFERENCE_ERROR:
            return "INFERENCE ERROR";

        default:
            return "UNKNOWN";
    }
}
