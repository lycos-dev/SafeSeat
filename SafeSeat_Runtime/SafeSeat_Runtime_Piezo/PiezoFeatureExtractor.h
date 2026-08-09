#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================
// SAFESEAT PIEZO FEATURE VECTOR
//
// Order matches the trained Python pipeline exactly:
//
//  1 mean
//  2 std
//  3 min
//  4 max
//  5 range
//  6 median
//  7 iqr
//  8 rms
//  9 energy
// 10 mean_abs_diff
// 11 std_diff
// 12 zero_crossing_rate
// 13 dominant_frequency_hz
// 14 estimated_respiration_bpm
// 15 spectral_entropy
// 16 autocorrelation_peak
// ============================================================

struct PiezoFeatures
{
    float mean = 0.0f;
    float std = 0.0f;
    float minimum = 0.0f;
    float maximum = 0.0f;
    float range = 0.0f;
    float median = 0.0f;
    float iqr = 0.0f;
    float rms = 0.0f;
    float energy = 0.0f;
    float meanAbsDiff = 0.0f;
    float stdDiff = 0.0f;
    float zeroCrossingRate = 0.0f;
    float dominantFrequencyHz = 0.0f;
    float respirationBPM = 0.0f;
    float spectralEntropy = 0.0f;
    float autocorrelationPeak = 0.0f;
};

class PiezoFeatureExtractor
{
public:
    PiezoFeatureExtractor() = default;

    // --------------------------------------------------------
    // Runtime normalization bridge
    //
    // Training normalization:
    //     (x - median) / (1.4826 * MAD)
    //
    // Step 5.7 retraining now applies this normalization to
    // each complete 30-second window in BOTH Python training
    // and live firmware. This is therefore runtime-aligned.
    //
    // WESAD RespiBAN -> PVDF remains a sensor-domain surrogate
    // and still requires physical deployment validation.
    // --------------------------------------------------------
    bool robustNormalizeWindow(
        const float *input,
        float *output,
        uint16_t n
    ) const;

    // Extract all 16 model features from a normalized window.
    bool computeFeatures(
        const float *signal,
        uint16_t n,
        PiezoFeatures &features
    ) const;

private:
    float computeMean(
        const float *x,
        uint16_t n
    ) const;

    float computeStd(
        const float *x,
        uint16_t n,
        float mean
    ) const;

    float computeMin(
        const float *x,
        uint16_t n
    ) const;

    float computeMax(
        const float *x,
        uint16_t n
    ) const;

    float computeRMS(
        const float *x,
        uint16_t n
    ) const;

    float computeEnergy(
        const float *x,
        uint16_t n
    ) const;

    float computePercentile(
        const float *sorted,
        uint16_t n,
        float percentile
    ) const;

    float computeMedian(
        const float *sorted,
        uint16_t n
    ) const;

    float computeIQR(
        const float *sorted,
        uint16_t n
    ) const;

    float computeMeanAbsDiff(
        const float *x,
        uint16_t n
    ) const;

    float computeStdDiff(
        const float *x,
        uint16_t n
    ) const;

    float computeZeroCrossingRate(
        const float *x,
        uint16_t n,
        float mean
    ) const;

    void computeFrequencyFeatures(
        const float *x,
        uint16_t n,
        float samplingRate,
        float mean,
        float &dominantFrequencyHz,
        float &respirationBPM,
        float &spectralEntropy
    ) const;

    float computeAutocorrelationPeak(
        const float *x,
        uint16_t n,
        float samplingRate,
        float mean
    ) const;
};
