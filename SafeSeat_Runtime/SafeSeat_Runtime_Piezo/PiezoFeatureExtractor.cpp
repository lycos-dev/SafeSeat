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
    float energy = 0.0f;

    for (uint16_t i = 0; i < n; i++)
    {
        energy += x[i] * x[i];
    }

    return energy;
}