#include "PiezoFeatureExtractor.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// ============================================================
// RUNTIME ROBUST NORMALIZATION
//
// Mirrors the training formula:
//
// median = np.median(signal)
// mad = np.median(abs(signal - median))
// scale = 1.4826 * mad
//
// if scale ~ 0:
//     scale = std(signal)
//
// if still ~ 0:
//     scale = 1
//
// normalized = (signal - median) / scale
//
// NOTE:
// Training calculated this over a complete participant
// recording. Live firmware cannot see the future, so this
// implementation applies the same formula to the current
// 30-second window. This deployment adaptation requires
// real-Piezo validation before the trained model is trusted.
// ============================================================

bool PiezoFeatureExtractor::robustNormalizeWindow(
    const float *input,
    float *output,
    uint16_t n
) const
{
    if (
        input == nullptr ||
        output == nullptr ||
        n == 0
    )
    {
        return false;
    }

    // Current model window is fixed at 750 samples.
    if (n > PIEZO_WINDOW_SAMPLES)
    {
        return false;
    }

    float sorted[PIEZO_WINDOW_SAMPLES];

    memcpy(
        sorted,
        input,
        n * sizeof(float)
    );

    std::sort(
        sorted,
        sorted + n
    );

    float median =
        computeMedian(
            sorted,
            n
        );

    float absoluteDeviation[
        PIEZO_WINDOW_SAMPLES
    ];

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        absoluteDeviation[i] =
            fabsf(
                input[i] - median
            );
    }

    std::sort(
        absoluteDeviation,
        absoluteDeviation + n
    );

    float mad =
        computeMedian(
            absoluteDeviation,
            n
        );

    float robustScale =
        1.4826f * mad;

    if (
        robustScale < 1.0e-12f
    )
    {
        float mean =
            computeMean(
                input,
                n
            );

        robustScale =
            computeStd(
                input,
                n,
                mean
            );
    }

    if (
        robustScale < 1.0e-12f
    )
    {
        robustScale =
            1.0f;
    }

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        output[i] =
            (
                input[i] - median
            )
            /
            robustScale;
    }

    return true;
}

// ============================================================
// FEATURE EXTRACTION
// ============================================================

bool PiezoFeatureExtractor::computeFeatures(
    const float *signal,
    uint16_t n,
    PiezoFeatures &features
) const
{
    features =
        PiezoFeatures{};

    if (
        signal == nullptr ||
        n != PIEZO_WINDOW_SAMPLES
    )
    {
        return false;
    }

    float sortedSignal[
        PIEZO_WINDOW_SAMPLES
    ];

    memcpy(
        sortedSignal,
        signal,
        n * sizeof(float)
    );

    std::sort(
        sortedSignal,
        sortedSignal + n
    );

    features.mean =
        computeMean(
            signal,
            n
        );

    features.std =
        computeStd(
            signal,
            n,
            features.mean
        );

    features.minimum =
        computeMin(
            signal,
            n
        );

    features.maximum =
        computeMax(
            signal,
            n
        );

    features.range =
        features.maximum -
        features.minimum;

    features.median =
        computeMedian(
            sortedSignal,
            n
        );

    features.iqr =
        computeIQR(
            sortedSignal,
            n
        );

    features.rms =
        computeRMS(
            signal,
            n
        );

    features.energy =
        computeEnergy(
            signal,
            n
        );

    features.meanAbsDiff =
        computeMeanAbsDiff(
            signal,
            n
        );

    features.stdDiff =
        computeStdDiff(
            signal,
            n
        );

    features.zeroCrossingRate =
        computeZeroCrossingRate(
            signal,
            n,
            features.mean
        );

    computeFrequencyFeatures(
        signal,
        n,
        PIEZO_SAMPLE_RATE_HZ,
        features.mean,
        features.dominantFrequencyHz,
        features.respirationBPM,
        features.spectralEntropy
    );

    features.autocorrelationPeak =
        computeAutocorrelationPeak(
            signal,
            n,
            PIEZO_SAMPLE_RATE_HZ,
            features.mean
        );

    return true;
}

// ============================================================
// BASIC STATISTICS
// ============================================================

float PiezoFeatureExtractor::computeMean(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    double sum = 0.0;

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        sum +=
            static_cast<double>(
                x[i]
            );
    }

    return static_cast<float>(
        sum /
        static_cast<double>(
            n
        )
    );
}

float PiezoFeatureExtractor::computeStd(
    const float *x,
    uint16_t n,
    float mean
) const
{
    if (
        x == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    double variance = 0.0;

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        double d =
            static_cast<double>(
                x[i]
            )
            -
            static_cast<double>(
                mean
            );

        variance +=
            d * d;
    }

    variance /=
        static_cast<double>(
            n
        );

    return static_cast<float>(
        sqrt(
            variance
        )
    );
}

float PiezoFeatureExtractor::computeMin(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    float result =
        x[0];

    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        if (
            x[i] < result
        )
        {
            result =
                x[i];
        }
    }

    return result;
}

float PiezoFeatureExtractor::computeMax(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    float result =
        x[0];

    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        if (
            x[i] > result
        )
        {
            result =
                x[i];
        }
    }

    return result;
}

float PiezoFeatureExtractor::computeRMS(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    double sum =
        0.0;

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        double value =
            static_cast<double>(
                x[i]
            );

        sum +=
            value * value;
    }

    return static_cast<float>(
        sqrt(
            sum /
            static_cast<double>(
                n
            )
        )
    );
}

// Python:
// np.mean(np.square(signal))
float PiezoFeatureExtractor::computeEnergy(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    double sum =
        0.0;

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        double value =
            static_cast<double>(
                x[i]
            );

        sum +=
            value * value;
    }

    return static_cast<float>(
        sum /
        static_cast<double>(
            n
        )
    );
}

// ============================================================
// PERCENTILES / MEDIAN / IQR
//
// Matches NumPy default linear percentile interpolation:
//
// position = percentile * (N - 1)
// ============================================================

float PiezoFeatureExtractor::computePercentile(
    const float *sorted,
    uint16_t n,
    float percentile
) const
{
    if (
        sorted == nullptr ||
        n == 0
    )
    {
        return 0.0f;
    }

    if (
        n == 1
    )
    {
        return sorted[0];
    }

    if (
        percentile <= 0.0f
    )
    {
        return sorted[0];
    }

    if (
        percentile >= 1.0f
    )
    {
        return sorted[
            n - 1
        ];
    }

    float position =
        percentile *
        static_cast<float>(
            n - 1
        );

    uint16_t lowerIndex =
        static_cast<uint16_t>(
            floorf(
                position
            )
        );

    uint16_t upperIndex =
        static_cast<uint16_t>(
            ceilf(
                position
            )
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
        position -
        static_cast<float>(
            lowerIndex
        );

    return
        sorted[
            lowerIndex
        ]
        +
        fraction *
        (
            sorted[
                upperIndex
            ]
            -
            sorted[
                lowerIndex
            ]
        );
}

float PiezoFeatureExtractor::computeMedian(
    const float *sorted,
    uint16_t n
) const
{
    return computePercentile(
        sorted,
        n,
        0.50f
    );
}

float PiezoFeatureExtractor::computeIQR(
    const float *sorted,
    uint16_t n
) const
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

    return q75 - q25;
}

// ============================================================
// FIRST-DIFFERENCE FEATURES
// ============================================================

float PiezoFeatureExtractor::computeMeanAbsDiff(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n <= 1
    )
    {
        return 0.0f;
    }

    double sum =
        0.0;

    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        sum +=
            fabs(
                static_cast<double>(
                    x[i]
                )
                -
                static_cast<double>(
                    x[i - 1]
                )
            );
    }

    return static_cast<float>(
        sum /
        static_cast<double>(
            n - 1
        )
    );
}

float PiezoFeatureExtractor::computeStdDiff(
    const float *x,
    uint16_t n
) const
{
    if (
        x == nullptr ||
        n <= 1
    )
    {
        return 0.0f;
    }

    uint16_t diffCount =
        n - 1;

    double diffMean =
        0.0;

    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        diffMean +=
            static_cast<double>(
                x[i]
            )
            -
            static_cast<double>(
                x[i - 1]
            );
    }

    diffMean /=
        static_cast<double>(
            diffCount
        );

    double variance =
        0.0;

    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        double difference =
            static_cast<double>(
                x[i]
            )
            -
            static_cast<double>(
                x[i - 1]
            );

        double deviation =
            difference -
            diffMean;

        variance +=
            deviation *
            deviation;
    }

    variance /=
        static_cast<double>(
            diffCount
        );

    return static_cast<float>(
        sqrt(
            variance
        )
    );
}

// ============================================================
// ZERO-CROSSING RATE
//
// Python:
// centered = signal - np.mean(signal)
// signs = np.sign(centered)
// crossings = sum(sign[i] * sign[i+1] < 0)
// ============================================================

float PiezoFeatureExtractor::computeZeroCrossingRate(
    const float *x,
    uint16_t n,
    float mean
) const
{
    if (
        x == nullptr ||
        n <= 1
    )
    {
        return 0.0f;
    }

    uint16_t crossings =
        0;

    for (
        uint16_t i = 0;
        i < n - 1;
        i++
    )
    {
        float current =
            x[i] - mean;

        float next =
            x[
                i + 1
            ] - mean;

        if (
            (
                current > 0.0f &&
                next < 0.0f
            )
            ||
            (
                current < 0.0f &&
                next > 0.0f
            )
        )
        {
            crossings++;
        }
    }

    return
        static_cast<float>(
            crossings
        )
        /
        static_cast<float>(
            n - 1
        );
}

// ============================================================
// FREQUENCY FEATURES
//
// Mirrors the bins used by scipy.signal.periodogram for
// N=750 and Fs=25 Hz, while calculating only 0.05-1.00 Hz.
//
// The constant periodogram scale cancels for:
// - argmax dominant frequency
// - normalized spectral entropy
// ============================================================

void PiezoFeatureExtractor::computeFrequencyFeatures(
    const float *x,
    uint16_t n,
    float samplingRate,
    float mean,
    float &dominantFrequencyHz,
    float &respirationBPM,
    float &spectralEntropy
) const
{
    dominantFrequencyHz =
        0.0f;

    respirationBPM =
        0.0f;

    spectralEntropy =
        0.0f;

    if (
        x == nullptr ||
        n == 0 ||
        samplingRate <= 0.0f
    )
    {
        return;
    }

    const double frequencyResolution =
        static_cast<double>(
            samplingRate
        )
        /
        static_cast<double>(
            n
        );

    int firstBin =
        static_cast<int>(
            ceil(
                0.05 /
                frequencyResolution
            )
        );

    int lastBin =
        static_cast<int>(
            floor(
                1.00 /
                frequencyResolution
            )
        );

    int maximumOneSidedBin =
        n / 2;

    if (
        lastBin >
        maximumOneSidedBin
    )
    {
        lastBin =
            maximumOneSidedBin;
    }

    if (
        firstBin < 0
    )
    {
        firstBin =
            0;
    }

    if (
        lastBin <
        firstBin
    )
    {
        return;
    }

    const int bandBinCount =
        lastBin -
        firstBin +
        1;

    constexpr int
        MAX_RESPIRATION_BINS =
            64;

    if (
        bandBinCount >
        MAX_RESPIRATION_BINS
    )
    {
        return;
    }

    double power[
        MAX_RESPIRATION_BINS
    ] = {0.0};

    double totalPower =
        0.0;

    double maximumPower =
        -1.0;

    int dominantBin =
        firstBin;

    constexpr double TWO_PI_LOCAL =
        6.28318530717958647692;

    int powerIndex =
        0;

    for (
        int k = firstBin;
        k <= lastBin;
        k++
    )
    {
        double realPart =
            0.0;

        double imaginaryPart =
            0.0;

        for (
            uint16_t sampleIndex = 0;
            sampleIndex < n;
            sampleIndex++
        )
        {
            double centered =
                static_cast<double>(
                    x[
                        sampleIndex
                    ]
                )
                -
                static_cast<double>(
                    mean
                );

            double angle =
                TWO_PI_LOCAL *
                static_cast<double>(
                    k
                ) *
                static_cast<double>(
                    sampleIndex
                ) /
                static_cast<double>(
                    n
                );

            realPart +=
                centered *
                cos(
                    angle
                );

            imaginaryPart -=
                centered *
                sin(
                    angle
                );
        }

        double binPower =
            realPart *
            realPart
            +
            imaginaryPart *
            imaginaryPart;

        power[
            powerIndex
        ] =
            binPower;

        totalPower +=
            binPower;

        if (
            binPower >
            maximumPower
        )
        {
            maximumPower =
                binPower;

            dominantBin =
                k;
        }

        powerIndex++;
    }

    if (
        totalPower <= 0.0 ||
        maximumPower <= 0.0
    )
    {
        return;
    }

    dominantFrequencyHz =
        static_cast<float>(
            static_cast<double>(
                dominantBin
            )
            *
            frequencyResolution
        );

    respirationBPM =
        dominantFrequencyHz *
        60.0f;

    double entropy =
        0.0;

    for (
        int i = 0;
        i < bandBinCount;
        i++
    )
    {
        if (
            power[i] <= 0.0
        )
        {
            continue;
        }

        double probability =
            power[i] /
            totalPower;

        entropy -=
            probability *
            (
                log(
                    probability
                )
                /
                log(
                    2.0
                )
            );
    }

    double maximumEntropy =
        log(
            static_cast<double>(
                bandBinCount
            )
        )
        /
        log(
            2.0
        );

    if (
        maximumEntropy > 0.0
    )
    {
        spectralEntropy =
            static_cast<float>(
                entropy /
                maximumEntropy
            );
    }
}

// ============================================================
// AUTOCORRELATION PEAK
//
// Matches Python's normalized positive-lag correlation:
//
// min lag = 1 sec
// max lag = 20 sec
// normalization = correlation(lag) / correlation(0)
// ============================================================

float PiezoFeatureExtractor::computeAutocorrelationPeak(
    const float *x,
    uint16_t n,
    float samplingRate,
    float mean
) const
{
    if (
        x == nullptr ||
        n == 0 ||
        samplingRate <= 0.0f
    )
    {
        return 0.0f;
    }

    double zeroLagCorrelation =
        0.0;

    for (
        uint16_t i = 0;
        i < n;
        i++
    )
    {
        double centered =
            static_cast<double>(
                x[i]
            )
            -
            static_cast<double>(
                mean
            );

        zeroLagCorrelation +=
            centered *
            centered;
    }

    double variance =
        zeroLagCorrelation /
        static_cast<double>(
            n
        );

    if (
        variance < 1.0e-12 ||
        zeroLagCorrelation <= 0.0
    )
    {
        return 0.0f;
    }

    int minimumLag =
        static_cast<int>(
            samplingRate *
            1.0f
        );

    int maximumLag =
        static_cast<int>(
            samplingRate *
            20.0f
        );

    int largestPossibleLag =
        static_cast<int>(
            n
        ) - 1;

    if (
        maximumLag >
        largestPossibleLag
    )
    {
        maximumLag =
            largestPossibleLag;
    }

    if (
        maximumLag <=
        minimumLag
    )
    {
        return 0.0f;
    }

    double maximumNormalizedCorrelation =
        -1.0;

    for (
        int lag = minimumLag;
        lag <= maximumLag;
        lag++
    )
    {
        double correlation =
            0.0;

        int overlappingSamples =
            static_cast<int>(
                n
            )
            -
            lag;

        for (
            int i = 0;
            i < overlappingSamples;
            i++
        )
        {
            double first =
                static_cast<double>(
                    x[i]
                )
                -
                static_cast<double>(
                    mean
                );

            double second =
                static_cast<double>(
                    x[
                        i + lag
                    ]
                )
                -
                static_cast<double>(
                    mean
                );

            correlation +=
                first *
                second;
        }

        double normalizedCorrelation =
            correlation /
            zeroLagCorrelation;

        if (
            normalizedCorrelation >
            maximumNormalizedCorrelation
        )
        {
            maximumNormalizedCorrelation =
                normalizedCorrelation;
        }
    }

    if (
        maximumNormalizedCorrelation <
        -0.999999
    )
    {
        return 0.0f;
    }

    return static_cast<float>(
        maximumNormalizedCorrelation
    );
}
