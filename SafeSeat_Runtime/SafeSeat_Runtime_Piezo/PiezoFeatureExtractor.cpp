#include "PiezoFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <cstring>



// ============================================================
// CONSTRUCTOR
// ============================================================

PiezoFeatureExtractor::PiezoFeatureExtractor()
{
    reset();
}



// ============================================================
// RESET
// ============================================================

void PiezoFeatureExtractor::reset()
{
    writeIndex = 0;

    full = false;

    memset(
        buffer,
        0,
        sizeof(buffer)
    );
}



// ============================================================
// ADD SAMPLE
// ============================================================
//
// Samples arrive from PiezoSensor at exactly 25 Hz.
//
// Buffer acts as a rolling
// 30-second circular window.
//
// Once full,
// oldest samples are overwritten.
//

void PiezoFeatureExtractor::addSample(float sample)
{
    buffer[writeIndex] = sample;

    writeIndex++;

    if (writeIndex >= PIEZO_WINDOW_SAMPLES)
    {
        writeIndex = 0;

        full = true;
    }
}



// ============================================================
// WINDOW READY?
// ============================================================

bool PiezoFeatureExtractor::isWindowReady() const
{
    return full;
}



// ============================================================
// SAMPLE COUNT
// ============================================================

uint16_t PiezoFeatureExtractor::sampleCount() const
{
    if (full)
    {
        return PIEZO_WINDOW_SAMPLES;
    }

    return writeIndex;
}



// ============================================================
// COPY WINDOW
// ============================================================
//
// Training always sees chronological windows.
//
// Circular buffer:
//
//      newest
//         |
//         V
//
// 480 .... 749 0 .... 479
//
// becomes
//
// 0
// 1
// 2
// ...
// 749
//

void PiezoFeatureExtractor::copyOrdered(float *destination)
{
    if (!full)
    {
        memcpy(
            destination,
            buffer,
            writeIndex * sizeof(float)
        );

        return;
    }

    uint16_t source = writeIndex;

    for (uint16_t i = 0;
         i < PIEZO_WINDOW_SAMPLES;
         i++)
    {
        destination[i] =
            buffer[source];

        source++;

        if (source >= PIEZO_WINDOW_SAMPLES)
        {
            source = 0;
        }
    }
}



// ============================================================
// COMPUTE FEATURES
// ============================================================
//
// Temporary stub.
//
// Full implementation added gradually.
//

void PiezoFeatureExtractor::computeFeatures(
    PiezoFeatures &features)
{
    memset(
        &features,
        0,
        sizeof(PiezoFeatures)
    );

    if (!full)
    {
        return;
    }

    float signal[PIEZO_WINDOW_SAMPLES];

    copyOrdered(signal);

    float sortedSignal[
    PIEZO_WINDOW_SAMPLES
];

memcpy(
    sortedSignal,
    signal,
    sizeof(signal)
);


std::sort(
    sortedSignal,
    sortedSignal
        +
        PIEZO_WINDOW_SAMPLES
);

    features.mean =
    computeMean(signal,
                PIEZO_WINDOW_SAMPLES);

features.std =
    computeStd(signal,
               PIEZO_WINDOW_SAMPLES,
               features.mean);

features.minimum =
    computeMin(signal,
               PIEZO_WINDOW_SAMPLES);

features.maximum =
    computeMax(signal,
               PIEZO_WINDOW_SAMPLES);

features.range =
    features.maximum -
    features.minimum;
  
features.median =
    computeMedian(
        sortedSignal,
        PIEZO_WINDOW_SAMPLES
    );

features.iqr =
    computeIQR(
        sortedSignal,
        PIEZO_WINDOW_SAMPLES
    );

features.rms =
    computeRMS(signal,
               PIEZO_WINDOW_SAMPLES);

features.energy =
    computeEnergy(signal,
                  PIEZO_WINDOW_SAMPLES);
}

float PiezoFeatureExtractor::computeMean(
    const float *x,
    uint16_t n)
{
    float sum = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        sum += x[i];
    }

    return sum / n;
}

float PiezoFeatureExtractor::computeStd(
    const float *x,
    uint16_t n,
    float mean)
{
    float variance = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        float d = x[i] - mean;

        variance += d * d;
    }

    variance /= n;

    return sqrtf(variance);
}

float PiezoFeatureExtractor::computeMin(
    const float *x,
    uint16_t n)
{
    float m = x[0];

    for (uint16_t i = 1; i < n; i++)
    {
        if (x[i] < m)
            m = x[i];
    }

    return m;
}

float PiezoFeatureExtractor::computeMax(
    const float *x,
    uint16_t n)
{
    float m = x[0];

    for (uint16_t i = 1; i < n; i++)
    {
        if (x[i] > m)
            m = x[i];
    }

    return m;
}

float PiezoFeatureExtractor::computeRMS(
    const float *x,
    uint16_t n)
{
    float sum = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        sum += x[i] * x[i];
    }

    return sqrtf(sum / n);
}

float PiezoFeatureExtractor::computeEnergy(
    const float *x,
    uint16_t n)
{
    float sum = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        sum += x[i] * x[i];
    }

    return sum / static_cast<float>(n);
}

// ============================================================
// PERCENTILE
//
// Matches NumPy's default linear percentile interpolation:
//
// position = (N - 1) * percentile
//
// Example:
// percentile = 0.25 -> 25th percentile
// percentile = 0.75 -> 75th percentile
//
// IMPORTANT:
// Input array MUST already be sorted ascending.
// ============================================================

float PiezoFeatureExtractor::computePercentile(
    const float *sorted,
    uint16_t n,
    float percentile)
{
    if (
        sorted == nullptr
        ||
        n == 0
    )
    {
        return 0.0f;
    }

    if (n == 1)
    {
        return sorted[0];
    }

    if (percentile <= 0.0f)
    {
        return sorted[0];
    }

    if (percentile >= 1.0f)
    {
        return sorted[n - 1];
    }


    float position =
        percentile
        *
        static_cast<float>(
            n - 1
        );


    uint16_t lowerIndex =
        static_cast<uint16_t>(
            floorf(position)
        );


    uint16_t upperIndex =
        static_cast<uint16_t>(
            ceilf(position)
        );


    if (
        lowerIndex ==
        upperIndex
    )
    {
        return sorted[
            lowerIndex
        ];
    }


    float fraction =
        position
        -
        static_cast<float>(
            lowerIndex
        );


    float lowerValue =
        sorted[
            lowerIndex
        ];


    float upperValue =
        sorted[
            upperIndex
        ];


    return (
        lowerValue
        +
        fraction
        *
        (
            upperValue
            -
            lowerValue
        )
    );
}


// ============================================================
// MEDIAN
// ============================================================

float PiezoFeatureExtractor::computeMedian(
    const float *sorted,
    uint16_t n)
{
    return computePercentile(
        sorted,
        n,
        0.50f
    );
}


// ============================================================
// IQR
//
// Python:
//
// q75 = np.percentile(signal, 75)
// q25 = np.percentile(signal, 25)
// IQR = q75 - q25
// ============================================================

float PiezoFeatureExtractor::computeIQR(
    const float *sorted,
    uint16_t n)
{
    float q25 =
        computePercentile(
            sorted,
            n,
            0.25f
        );


    float q75 =
        computePercentile(
            sorted,
            n,
            0.75f
        );


    return (
        q75
        -
        q25
    );
}