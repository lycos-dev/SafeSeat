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

    // Feature calculations
    // added in the following sections.
}