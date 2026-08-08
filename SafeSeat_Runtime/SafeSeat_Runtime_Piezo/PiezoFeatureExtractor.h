#ifndef PIEZO_FEATURE_EXTRACTOR_H
#define PIEZO_FEATURE_EXTRACTOR_H

#include <Arduino.h>

constexpr uint16_t PIEZO_SAMPLE_RATE = 25;
constexpr uint16_t PIEZO_WINDOW_SECONDS = 30;

constexpr uint16_t PIEZO_WINDOW_SAMPLES =
    PIEZO_SAMPLE_RATE * PIEZO_WINDOW_SECONDS;

struct PiezoFeatures
{
    float mean;

    float std;

    float minimum;
    float maximum;
    float range;

    float median;
    float iqr;

    float rms;

    float energy;

    float meanAbsDiff;
    float stdDiff;

    float zeroCrossingRate;

    float dominantFrequencyHz;
    float respirationBPM;

    float spectralEntropy;

    float autocorrelationPeak;
};

class PiezoFeatureExtractor
{
public:

    PiezoFeatureExtractor();

    void reset();

    void addSample(float sample);

    bool isWindowReady() const;

    uint16_t sampleCount() const;

    void computeFeatures(PiezoFeatures &features);

private:

    float buffer[PIEZO_WINDOW_SAMPLES];

    uint16_t writeIndex;

    bool full;

    void copyOrdered(float *destination);

    float computeMean(const float *x, uint16_t n);

float computeStd(const float *x,
                 uint16_t n,
                 float mean);

float computeMin(const float *x,
                 uint16_t n);

float computeMax(const float *x,
                 uint16_t n);

float computeRMS(const float *x,
                 uint16_t n);

float computeEnergy(const float *x,
                    uint16_t n);

                    float computePercentile(
    const float *sorted,
    uint16_t n,
    float percentile
);

float computeMedian(
    const float *sorted,
    uint16_t n
);

float computeIQR(
    const float *sorted,
    uint16_t n
);

float computeMeanAbsDiff(
    const float *x,
    uint16_t n
);

float computeStdDiff(
    const float *x,
    uint16_t n
);

float computeZeroCrossingRate(
    const float *x,
    uint16_t n,
    float mean
);

};

#endif