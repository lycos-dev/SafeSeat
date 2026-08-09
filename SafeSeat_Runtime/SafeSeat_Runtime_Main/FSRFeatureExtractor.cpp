#include "FSRFeatureExtractor.h"

#include <math.h>

// ============================================================
// NUMPY-COMPATIBLE PERCENTILE FOR FIXED 23-SAMPLE WINDOW
//
// np.percentile default method is linear interpolation:
// position = q * (n - 1)
// ============================================================

double FSRFeatureExtractor::percentile(
    const double values[FSR_ML_WINDOW_SAMPLES],
    double quantile
) const
{
    double sorted[FSR_ML_WINDOW_SAMPLES];

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        sorted[i] =
            values[i];
    }

    // Small fixed array: insertion sort is deterministic and
    // avoids dynamic allocation.
    for (
        uint16_t i = 1;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        double key =
            sorted[i];

        int j =
            static_cast<int>(
                i
            )
            -
            1;

        while (
            j >= 0
            &&
            sorted[j]
            >
            key
        )
        {
            sorted[j + 1] =
                sorted[j];

            j--;
        }

        sorted[j + 1] =
            key;
    }

    double position =
        quantile
        *
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
            -
            1
        );

    uint16_t lower =
        static_cast<uint16_t>(
            floor(
                position
            )
        );

    uint16_t upper =
        static_cast<uint16_t>(
            ceil(
                position
            )
        );

    if (
        lower
        ==
        upper
    )
    {
        return sorted[
            lower
        ];
    }

    double fraction =
        position
        -
        static_cast<double>(
            lower
        );

    return
        sorted[lower]
        *
        (
            1.0
            -
            fraction
        )
        +
        sorted[upper]
        *
        fraction;
}


// ============================================================
// MEAN ABSOLUTE FIRST DIFFERENCE
// ============================================================

double FSRFeatureExtractor::meanAbsDiff(
    const double values[FSR_ML_WINDOW_SAMPLES]
) const
{
    double total =
        0.0;

    for (
        uint16_t i = 1;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        total +=
            fabs(
                values[i]
                -
                values[i - 1]
            );
    }

    return
        total
        /
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
            -
            1
        );
}


// ============================================================
// SUMMARY:
// mean, population std, range, median, IQR, mean abs diff
// ============================================================

void FSRFeatureExtractor::summarize(
    const double values[FSR_ML_WINDOW_SAMPLES],
    bool includeMeanAbsDiff,
    float output[],
    uint16_t &index
) const
{
    double sum =
        0.0;

    double minimum =
        values[0];

    double maximum =
        values[0];

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        sum +=
            values[i];

        if (
            values[i]
            <
            minimum
        )
        {
            minimum =
                values[i];
        }

        if (
            values[i]
            >
            maximum
        )
        {
            maximum =
                values[i];
        }
    }

    double mean =
        sum
        /
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
        );

    double variance =
        0.0;

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        double d =
            values[i]
            -
            mean;

        variance +=
            d
            *
            d;
    }

    variance /=
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
        );

    double median =
        percentile(
            values,
            0.5
        );

    double q25 =
        percentile(
            values,
            0.25
        );

    double q75 =
        percentile(
            values,
            0.75
        );

    output[index++] =
        static_cast<float>(
            mean
        );

    output[index++] =
        static_cast<float>(
            sqrt(
                variance
            )
        );

    output[index++] =
        static_cast<float>(
            maximum
            -
            minimum
        );

    output[index++] =
        static_cast<float>(
            median
        );

    output[index++] =
        static_cast<float>(
            q75
            -
            q25
        );

    if (
        includeMeanAbsDiff
    )
    {
        output[index++] =
            static_cast<float>(
                meanAbsDiff(
                    values
                )
            );
    }
}


// ============================================================
// SUMMARY:
// mean, population std, range
// ============================================================

void FSRFeatureExtractor::summarizeMeanStdRange(
    const double values[FSR_ML_WINDOW_SAMPLES],
    float output[],
    uint16_t &index
) const
{
    double sum =
        0.0;

    double minimum =
        values[0];

    double maximum =
        values[0];

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        sum +=
            values[i];

        if (
            values[i]
            <
            minimum
        )
        {
            minimum =
                values[i];
        }

        if (
            values[i]
            >
            maximum
        )
        {
            maximum =
                values[i];
        }
    }

    double mean =
        sum
        /
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
        );

    double variance =
        0.0;

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        double d =
            values[i]
            -
            mean;

        variance +=
            d
            *
            d;
    }

    variance /=
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
        );

    output[index++] =
        static_cast<float>(
            mean
        );

    output[index++] =
        static_cast<float>(
            sqrt(
                variance
            )
        );

    output[index++] =
        static_cast<float>(
            maximum
            -
            minimum
        );
}


// ============================================================
// SUMMARY:
// mean, population std
// ============================================================

void FSRFeatureExtractor::summarizeMeanStd(
    const double values[FSR_ML_WINDOW_SAMPLES],
    float output[],
    uint16_t &index
) const
{
    double sum =
        0.0;

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        sum +=
            values[i];
    }

    double mean =
        sum
        /
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
        );

    double variance =
        0.0;

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES;
        i++
    )
    {
        double d =
            values[i]
            -
            mean;

        variance +=
            d
            *
            d;
    }

    variance /=
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
        );

    output[index++] =
        static_cast<float>(
            mean
        );

    output[index++] =
        static_cast<float>(
            sqrt(
                variance
            )
        );
}


// ============================================================
// FEATURE EXTRACTION
//
// Exact Step 5.5 feature order:
// - 9 sensors x 6 share statistics = 54
// - spatial/group distribution features
// - center-of-pressure-like features
// - frame-to-frame spatial redistribution
//
// Absolute pressure magnitude is NOT a model feature.
// ============================================================

bool FSRFeatureExtractor::extract(
    const float pressureWindow[FSR_ML_WINDOW_SAMPLES][NUM_FSR],
    FSRFeatureVector &output
) const
{
    output =
        FSRFeatureVector{};

    double shares[
        FSR_ML_WINDOW_SAMPLES
    ][
        NUM_FSR
    ];

    for (
        uint16_t frame = 0;
        frame < FSR_ML_WINDOW_SAMPLES;
        frame++
    )
    {
        double total =
            0.0;

        for (
            uint8_t sensor = 0;
            sensor < NUM_FSR;
            sensor++
        )
        {
            double value =
                static_cast<double>(
                    pressureWindow[
                        frame
                    ][
                        sensor
                    ]
                );

            if (
                !isfinite(
                    value
                )
            )
            {
                return false;
            }

            if (
                value
                <
                0.0
            )
            {
                value =
                    0.0;
            }

            shares[frame][sensor] =
                value;

            total +=
                value;
        }

        if (
            total
            <=
            EPSILON
        )
        {
            return false;
        }

        for (
            uint8_t sensor = 0;
            sensor < NUM_FSR;
            sensor++
        )
        {
            shares[frame][sensor] /=
                total;
        }
    }

    uint16_t featureIndex =
        0;

    // --------------------------------------------------------
    // 54 per-sensor share features
    // --------------------------------------------------------

    double series[
        FSR_ML_WINDOW_SAMPLES
    ];

    for (
        uint8_t sensor = 0;
        sensor < NUM_FSR;
        sensor++
    )
    {
        for (
            uint16_t frame = 0;
            frame < FSR_ML_WINDOW_SAMPLES;
            frame++
        )
        {
            series[frame] =
                shares[frame][sensor];
        }

        summarize(
            series,
            true,
            output.values,
            featureIndex
        );
    }


    // --------------------------------------------------------
    // Derived per-frame spatial series
    // --------------------------------------------------------

    double back[
        FSR_ML_WINDOW_SAMPLES
    ];

    double cushion[
        FSR_ML_WINDOW_SAMPLES
    ];

    double backLR[
        FSR_ML_WINDOW_SAMPLES
    ];

    double cushionLR[
        FSR_ML_WINDOW_SAMPLES
    ];

    double cushionCenter[
        FSR_ML_WINDOW_SAMPLES
    ];

    double upperBack[
        FSR_ML_WINDOW_SAMPLES
    ];

    double middleBack[
        FSR_ML_WINDOW_SAMPLES
    ];

    double lowerBack[
        FSR_ML_WINDOW_SAMPLES
    ];

    double globalLeft[
        FSR_ML_WINDOW_SAMPLES
    ];

    double entropy[
        FSR_ML_WINDOW_SAMPLES
    ];

    double copX[
        FSR_ML_WINDOW_SAMPLES
    ];

    double copY[
        FSR_ML_WINDOW_SAMPLES
    ];

    constexpr double SENSOR_X[
        NUM_FSR
    ] =
    {
        -1.0, -1.0, -1.0,
         1.0,  1.0,  1.0,
        -1.0,  0.0,  1.0
    };

    constexpr double SENSOR_Y[
        NUM_FSR
    ] =
    {
         1.0,  0.0, -1.0,
         1.0,  0.0, -1.0,
        -2.0, -2.0, -2.0
    };

    const double entropyDenominator =
        log(
            9.0
        );

    for (
        uint16_t frame = 0;
        frame < FSR_ML_WINDOW_SAMPLES;
        frame++
    )
    {
        double backLeft =
            shares[frame][0]
            +
            shares[frame][1]
            +
            shares[frame][2];

        double backRight =
            shares[frame][3]
            +
            shares[frame][4]
            +
            shares[frame][5];

        back[frame] =
            backLeft
            +
            backRight;

        cushion[frame] =
            shares[frame][6]
            +
            shares[frame][7]
            +
            shares[frame][8];

        double backDenominator =
            back[frame]
            >
            EPSILON
                ? back[frame]
                : EPSILON;

        backLR[frame] =
            (
                backLeft
                -
                backRight
            )
            /
            backDenominator;

        double cushionSide =
            shares[frame][6]
            +
            shares[frame][8];

        double cushionSideDenominator =
            cushionSide
            >
            EPSILON
                ? cushionSide
                : EPSILON;

        cushionLR[frame] =
            (
                shares[frame][6]
                -
                shares[frame][8]
            )
            /
            cushionSideDenominator;

        double cushionDenominator =
            cushion[frame]
            >
            EPSILON
                ? cushion[frame]
                : EPSILON;

        cushionCenter[frame] =
            shares[frame][7]
            /
            cushionDenominator;

        upperBack[frame] =
            (
                shares[frame][0]
                +
                shares[frame][3]
            )
            /
            backDenominator;

        middleBack[frame] =
            (
                shares[frame][1]
                +
                shares[frame][4]
            )
            /
            backDenominator;

        lowerBack[frame] =
            (
                shares[frame][2]
                +
                shares[frame][5]
            )
            /
            backDenominator;

        globalLeft[frame] =
            shares[frame][0]
            +
            shares[frame][1]
            +
            shares[frame][2]
            +
            shares[frame][6];

        double frameEntropy =
            0.0;

        double x =
            0.0;

        double y =
            0.0;

        for (
            uint8_t sensor = 0;
            sensor < NUM_FSR;
            sensor++
        )
        {
            double p =
                shares[frame][sensor];

            double safeP =
                p
                >
                EPSILON
                    ? p
                    : EPSILON;

            frameEntropy -=
                p
                *
                log(
                    safeP
                );

            x +=
                p
                *
                SENSOR_X[
                    sensor
                ];

            y +=
                p
                *
                SENSOR_Y[
                    sensor
                ];
        }

        entropy[frame] =
            frameEntropy
            /
            entropyDenominator;

        copX[frame] =
            x;

        copY[frame] =
            y;
    }


    // --------------------------------------------------------
    // Feature order from feature_manifest.json
    // --------------------------------------------------------

    summarizeMeanStdRange(
        back,
        output.values,
        featureIndex
    );

    for (
        uint8_t which = 0;
        which < 2;
        which++
    )
    {
        const double *values =
            which == 0
                ? backLR
                : cushionLR;

        summarizeMeanStdRange(
            values,
            output.values,
            featureIndex
        );

        output.values[
            featureIndex++
        ] =
            static_cast<float>(
                meanAbsDiff(
                    values
                )
            );
    }

    summarizeMeanStdRange(
        cushionCenter,
        output.values,
        featureIndex
    );

    summarizeMeanStdRange(
        upperBack,
        output.values,
        featureIndex
    );

    summarizeMeanStdRange(
        middleBack,
        output.values,
        featureIndex
    );

    summarizeMeanStdRange(
        lowerBack,
        output.values,
        featureIndex
    );

    summarizeMeanStd(
        globalLeft,
        output.values,
        featureIndex
    );

    summarizeMeanStdRange(
        entropy,
        output.values,
        featureIndex
    );

    for (
        uint8_t which = 0;
        which < 2;
        which++
    )
    {
        const double *values =
            which == 0
                ? copX
                : copY;

        summarizeMeanStdRange(
            values,
            output.values,
            featureIndex
        );

        output.values[
            featureIndex++
        ] =
            static_cast<float>(
                meanAbsDiff(
                    values
                )
            );
    }


    // --------------------------------------------------------
    // Spatial L1 change (22 frame transitions)
    // --------------------------------------------------------

    double spatialL1[
        FSR_ML_WINDOW_SAMPLES
        -
        1
    ];

    for (
        uint16_t frame = 1;
        frame < FSR_ML_WINDOW_SAMPLES;
        frame++
    )
    {
        double totalChange =
            0.0;

        for (
            uint8_t sensor = 0;
            sensor < NUM_FSR;
            sensor++
        )
        {
            totalChange +=
                fabs(
                    shares[frame][sensor]
                    -
                    shares[frame - 1][sensor]
                );
        }

        spatialL1[
            frame - 1
        ] =
            totalChange;
    }

    double l1Sum =
        0.0;

    double l1Min =
        spatialL1[0];

    double l1Max =
        spatialL1[0];

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES - 1;
        i++
    )
    {
        l1Sum +=
            spatialL1[i];

        if (
            spatialL1[i]
            <
            l1Min
        )
        {
            l1Min =
                spatialL1[i];
        }

        if (
            spatialL1[i]
            >
            l1Max
        )
        {
            l1Max =
                spatialL1[i];
        }
    }

    double l1Mean =
        l1Sum
        /
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
            -
            1
        );

    double l1Variance =
        0.0;

    for (
        uint16_t i = 0;
        i < FSR_ML_WINDOW_SAMPLES - 1;
        i++
    )
    {
        double d =
            spatialL1[i]
            -
            l1Mean;

        l1Variance +=
            d
            *
            d;
    }

    l1Variance /=
        static_cast<double>(
            FSR_ML_WINDOW_SAMPLES
            -
            1
        );

    output.values[
        featureIndex++
    ] =
        static_cast<float>(
            l1Mean
        );

    output.values[
        featureIndex++
    ] =
        static_cast<float>(
            sqrt(
                l1Variance
            )
        );

    output.values[
        featureIndex++
    ] =
        static_cast<float>(
            l1Max
        );


    if (
        featureIndex
        !=
        FSR_MODEL_FEATURE_COUNT
    )
    {
        return false;
    }

    for (
        uint16_t i = 0;
        i < FSR_MODEL_FEATURE_COUNT;
        i++
    )
    {
        if (
            !isfinite(
                output.values[i]
            )
        )
        {
            return false;
        }
    }

    return true;
}
