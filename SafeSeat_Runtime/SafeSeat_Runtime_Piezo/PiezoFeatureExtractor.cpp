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

features.meanAbsDiff =
    computeMeanAbsDiff(
        signal,
        PIEZO_WINDOW_SAMPLES
    );

features.stdDiff =
    computeStdDiff(
        signal,
        PIEZO_WINDOW_SAMPLES
    );


features.zeroCrossingRate =
    computeZeroCrossingRate(
        signal,
        PIEZO_WINDOW_SAMPLES,
        features.mean
    );

// ============================================================
// FREQUENCY FEATURES
// ============================================================

computeFrequencyFeatures(
    signal,
    PIEZO_WINDOW_SAMPLES,
    static_cast<float>(
        PIEZO_SAMPLE_RATE
    ),
    features.mean,
    features.dominantFrequencyHz,
    features.respirationBPM,
    features.spectralEntropy
);


// ============================================================
// AUTOCORRELATION
// ============================================================

features.autocorrelationPeak =
    computeAutocorrelationPeak(
        signal,
        PIEZO_WINDOW_SAMPLES,
        static_cast<float>(
            PIEZO_SAMPLE_RATE
        ),
        features.mean
    );
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

// ============================================================
// MEAN ABSOLUTE FIRST DIFFERENCE
//
// Python:
//
// diff = np.diff(signal)
// np.mean(np.abs(diff))
// ============================================================

float PiezoFeatureExtractor::computeMeanAbsDiff(
    const float *x,
    uint16_t n)
{
    if (
        x == nullptr
        ||
        n <= 1
    )
    {
        return 0.0f;
    }


    float sum =
        0.0f;


    uint16_t diffCount =
        n - 1;


    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        float difference =
            x[i]
            -
            x[i - 1];


        sum +=
            fabsf(
                difference
            );
    }


    return (
        sum
        /
        static_cast<float>(
            diffCount
        )
    );
}



// ============================================================
// STANDARD DEVIATION OF FIRST DIFFERENCE
//
// Python:
//
// diff = np.diff(signal)
// np.std(diff)
//
// NumPy default:
// ddof = 0
// ============================================================

float PiezoFeatureExtractor::computeStdDiff(
    const float *x,
    uint16_t n)
{
    if (
        x == nullptr
        ||
        n <= 1
    )
    {
        return 0.0f;
    }


    uint16_t diffCount =
        n - 1;


    // --------------------------------------------------------
    // Mean of differences
    // --------------------------------------------------------

    float diffMean =
        0.0f;


    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        diffMean +=
            (
                x[i]
                -
                x[i - 1]
            );
    }


    diffMean /=
        static_cast<float>(
            diffCount
        );


    // --------------------------------------------------------
    // Population variance
    // --------------------------------------------------------

    float variance =
        0.0f;


    for (
        uint16_t i = 1;
        i < n;
        i++
    )
    {
        float difference =
            x[i]
            -
            x[i - 1];


        float deviation =
            difference
            -
            diffMean;


        variance +=
            deviation
            *
            deviation;
    }


    variance /=
        static_cast<float>(
            diffCount
        );


    return sqrtf(
        variance
    );
}



// ============================================================
// ZERO CROSSING RATE
//
// Python:
//
// centered = signal - np.mean(signal)
// signs = np.sign(centered)
//
// crossings = np.sum(
//     signs[:-1] * signs[1:] < 0
// )
//
// ZCR = crossings / (N - 1)
//
// IMPORTANT:
// Exact zeros do NOT count as crossings because:
//     sign(0) = 0
//
// and:
//
//     0 * +/-1
//
// is never < 0.
// ============================================================

float PiezoFeatureExtractor::computeZeroCrossingRate(
    const float *x,
    uint16_t n,
    float mean)
{
    if (
        x == nullptr
        ||
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
            x[i]
            -
            mean;


        float next =
            x[i + 1]
            -
            mean;


        bool currentPositive =
            current > 0.0f;


        bool currentNegative =
            current < 0.0f;


        bool nextPositive =
            next > 0.0f;


        bool nextNegative =
            next < 0.0f;


        if (
            (
                currentPositive
                &&
                nextNegative
            )
            ||
            (
                currentNegative
                &&
                nextPositive
            )
        )
        {
            crossings++;
        }
    }


    return (
        static_cast<float>(
            crossings
        )
        /
        static_cast<float>(
            n - 1
        )
    );
}

// ============================================================
// FREQUENCY-DOMAIN FEATURES
//
// Mirrors the relevant behavior of:
//
// scipy.signal.periodogram(
//     signal,
//     fs=25,
//     detrend="constant"
// )
//
// We only calculate DFT bins inside:
//
//     0.05 Hz <= f <= 1.00 Hz
//
// because those are the only bins used by the Python model.
//
// For N = 750 and Fs = 25 Hz:
//
//     frequency resolution = 25 / 750
//                          = 0.033333... Hz
//
// Therefore the respiration band corresponds to:
//
//     k = 2 ... 30
//
// The exact periodogram scaling factor is unnecessary here,
// because:
//
// 1. dominant-frequency selection only depends on relative power
// 2. spectral-entropy probabilities normalize by total power
//
// so a constant scale multiplier cancels.
// ============================================================

void PiezoFeatureExtractor::computeFrequencyFeatures(
    const float *x,
    uint16_t n,
    float samplingRate,
    float mean,
    float &dominantFrequencyHz,
    float &respirationBPM,
    float &spectralEntropy)
{
    dominantFrequencyHz = 0.0f;
    respirationBPM = 0.0f;
    spectralEntropy = 0.0f;


    if (
        x == nullptr
        ||
        n == 0
        ||
        samplingRate <= 0.0f
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Frequency-bin spacing
    // --------------------------------------------------------

    const double frequencyResolution =
        static_cast<double>(
            samplingRate
        )
        /
        static_cast<double>(
            n
        );


    // --------------------------------------------------------
    // Match Python mask:
    //
    // frequencies >= 0.05
    // frequencies <= 1.00
    // --------------------------------------------------------

    int firstBin =
        static_cast<int>(
            ceil(
                0.05
                /
                frequencyResolution
            )
        );


    int lastBin =
        static_cast<int>(
            floor(
                1.00
                /
                frequencyResolution
            )
        );


    // One-sided DFT cannot exceed Nyquist bin.
    int maximumOneSidedBin =
        n / 2;


    if (
        lastBin
        >
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
        firstBin = 0;
    }


    if (
        lastBin < firstBin
    )
    {
        return;
    }


    const int bandBinCount =
        lastBin
        -
        firstBin
        +
        1;


    /*
     * For the current SafeSeat configuration this is 29.
     *
     * Use a fixed-size array safely larger than needed.
     */
    constexpr int MAX_RESPIRATION_BINS = 64;


    if (
        bandBinCount
        >
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


    const double TWO_PI_LOCAL =
        6.28318530717958647692;


    // ========================================================
    // DIRECT DFT — RESPIRATION BINS ONLY
    // ========================================================

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
            /*
             * scipy periodogram detrend="constant"
             *
             * -> subtract the window mean.
             */

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
                TWO_PI_LOCAL
                *
                static_cast<double>(
                    k
                )
                *
                static_cast<double>(
                    sampleIndex
                )
                /
                static_cast<double>(
                    n
                );


            realPart +=
                centered
                *
                cos(
                    angle
                );


            imaginaryPart -=
                centered
                *
                sin(
                    angle
                );
        }


        // Magnitude-squared spectral power.
        double binPower =
            (
                realPart
                *
                realPart
            )
            +
            (
                imaginaryPart
                *
                imaginaryPart
            );


        power[
            powerIndex
        ] =
            binPower;


        totalPower +=
            binPower;


        if (
            binPower
            >
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


    // ========================================================
    // NO SPECTRAL POWER
    //
    // Matches:
    //
    // if np.all(respiration_power <= 0):
    //     return zeros
    // ========================================================

    if (
        totalPower <= 0.0
        ||
        maximumPower <= 0.0
    )
    {
        dominantFrequencyHz =
            0.0f;


        respirationBPM =
            0.0f;


        spectralEntropy =
            0.0f;


        return;
    }


    // ========================================================
    // DOMINANT FREQUENCY
    // ========================================================

    dominantFrequencyHz =
        static_cast<float>(
            static_cast<double>(
                dominantBin
            )
            *
            frequencyResolution
        );


    // ========================================================
    // ESTIMATED RESPIRATION BPM
    //
    // Python:
    //
    // dominant_frequency * 60
    // ========================================================

    respirationBPM =
        dominantFrequencyHz
        *
        60.0f;


    // ========================================================
    // SPECTRAL ENTROPY
    //
    // Python:
    //
    // probabilities = power / total_power
    //
    // entropy =
    //     -sum(p * log2(p))
    //
    // max_entropy =
    //     log2(len(respiration_power))
    //
    // spectral_entropy =
    //     entropy / max_entropy
    // ========================================================

    double entropy =
        0.0;


    for (
        int i = 0;
        i < bandBinCount;
        i++
    )
    {
        if (
            power[i]
            <= 0.0
        )
        {
            continue;
        }


        double probability =
            power[i]
            /
            totalPower;


        entropy -=
            probability
            *
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
                entropy
                /
                maximumEntropy
            );
    }
    else
    {
        spectralEntropy =
            0.0f;
    }
}



// ============================================================
// AUTOCORRELATION PEAK
//
// Matches Python:
//
// centered = signal - mean
// variance = np.var(centered)
//
// correlation = np.correlate(
//     centered,
//     centered,
//     mode="full"
// )
//
// keep positive lags
//
// correlation /= correlation[0]
//
// search:
//
//     1 second <= lag <= 20 seconds
//
// Python does NOT divide every lag by its overlap count.
// Therefore this implementation also uses the raw overlapping
// dot product before dividing by lag-zero correlation.
// ============================================================

float PiezoFeatureExtractor::computeAutocorrelationPeak(
    const float *x,
    uint16_t n,
    float samplingRate,
    float mean)
{
    if (
        x == nullptr
        ||
        n == 0
        ||
        samplingRate <= 0.0f
    )
    {
        return 0.0f;
    }


    // ========================================================
    // ZERO-LAG CORRELATION
    //
    // correlation[0] =
    // sum(centered[i]^2)
    // ========================================================

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
            centered
            *
            centered;
    }


    /*
     * Equivalent to Python's very-small-variance guard.
     *
     * np.var(centered) < 1e-12
     *
     * Since:
     *
     * zeroLagCorrelation = variance * N
     */
    double variance =
        zeroLagCorrelation
        /
        static_cast<double>(
            n
        );


    if (
        variance < 1e-12
        ||
        zeroLagCorrelation <= 0.0
    )
    {
        return 0.0f;
    }


    // ========================================================
    // LAG RANGE
    //
    // Python:
    //
    // min_lag = int(fs * 1.0)
    // max_lag = int(fs * 20.0)
    // ========================================================

    int minimumLag =
        static_cast<int>(
            samplingRate
            *
            1.0f
        );


    int maximumLag =
        static_cast<int>(
            samplingRate
            *
            20.0f
        );


    int largestPossibleLag =
        static_cast<int>(
            n
        )
        -
        1;


    if (
        maximumLag
        >
        largestPossibleLag
    )
    {
        maximumLag =
            largestPossibleLag;
    }


    if (
        maximumLag
        <=
        minimumLag
    )
    {
        return 0.0f;
    }


    double maximumNormalizedCorrelation =
        -1.0;


    // ========================================================
    // AUTOCORRELATION
    // ========================================================

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
                        i
                        +
                        lag
                    ]
                )
                -
                static_cast<double>(
                    mean
                );


            correlation +=
                first
                *
                second;
        }


        double normalizedCorrelation =
            correlation
            /
            zeroLagCorrelation;


        if (
            normalizedCorrelation
            >
            maximumNormalizedCorrelation
        )
        {
            maximumNormalizedCorrelation =
                normalizedCorrelation;
        }
    }


    if (
        maximumNormalizedCorrelation
        ==
        -1.0
    )
    {
        return 0.0f;
    }


    return static_cast<float>(
        maximumNormalizedCorrelation
    );
}