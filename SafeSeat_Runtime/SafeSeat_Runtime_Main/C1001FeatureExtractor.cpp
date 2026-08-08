#include "C1001FeatureExtractor.h"

#include <math.h>

// ============================================================
// FEATURE EXTRACTION
//
// This reproduces preprocessing/feature_engineer_c1001.py.
//
// Feature order is the exact 64-column order saved in:
// models/C1001/feature_columns.json
// ============================================================

namespace
{
    void insertionSort(
        float values[],
        uint16_t count
    )
    {
        for (
            uint16_t i = 1;
            i < count;
            i++
        )
        {
            float current =
                values[i];

            int j =
                static_cast<int>(
                    i
                )
                -
                1;

            while (
                j >= 0
                &&
                values[j] > current
            )
            {
                values[j + 1] =
                    values[j];

                j--;
            }

            values[j + 1] =
                current;
        }
    }
}


// ============================================================
// INVALID CODE
// ============================================================

bool C1001FeatureExtractor::isInvalidCode(
    float value
) const
{
    return
        isfinite(
            value
        )
        &&
        (
            fabsf(
                value
            )
            <
            1.0e-6f
            ||
            fabsf(
                value
                -
                255.0f
            )
            <
            1.0e-6f
        );
}


// ============================================================
// PANDAS-LIKE INTERPOLATION
//
// Python training used:
//
// Series.interpolate(
//     method="linear",
//     limit=5,
//     limit_direction="both"
// )
//
// For an internal missing run, up to five samples are filled
// from each direction. Leading/trailing missing runs receive
// up to five nearest-value fills.
// ============================================================

void C1001FeatureExtractor::prepareSignal(
    const float raw[C1001_ML_WINDOW_SAMPLES],
    float prepared[C1001_ML_WINDOW_SAMPLES]
) const
{
    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float value =
            raw[i];

        if (
            !isfinite(
                value
            )
            ||
            isInvalidCode(
                value
            )
        )
        {
            prepared[i] =
                NAN;
        }
        else
        {
            prepared[i] =
                value;
        }
    }

    uint16_t index =
        0;

    while (
        index
        <
        C1001_ML_WINDOW_SAMPLES
    )
    {
        if (
            isfinite(
                prepared[index]
            )
        )
        {
            index++;
            continue;
        }

        uint16_t start =
            index;

        while (
            index
            <
            C1001_ML_WINDOW_SAMPLES
            &&
            !isfinite(
                prepared[index]
            )
        )
        {
            index++;
        }

        uint16_t end =
            index - 1;

        bool hasLeft =
            start > 0
            &&
            isfinite(
                prepared[
                    start - 1
                ]
            );

        bool hasRight =
            index
            <
            C1001_ML_WINDOW_SAMPLES
            &&
            isfinite(
                prepared[index]
            );

        if (
            hasLeft
            &&
            hasRight
        )
        {
            float leftValue =
                prepared[
                    start - 1
                ];

            float rightValue =
                prepared[index];

            uint16_t missingCount =
                end
                -
                start
                +
                1;

            float denominator =
                static_cast<float>(
                    missingCount + 1
                );

            for (
                uint16_t position = start;
                position <= end;
                position++
            )
            {
                uint16_t distanceFromLeft =
                    position
                    -
                    start
                    +
                    1;

                uint16_t distanceFromRight =
                    end
                    -
                    position
                    +
                    1;

                if (
                    distanceFromLeft
                    <=
                    INTERPOLATION_LIMIT
                    ||
                    distanceFromRight
                    <=
                    INTERPOLATION_LIMIT
                )
                {
                    float fraction =
                        static_cast<float>(
                            distanceFromLeft
                        )
                        /
                        denominator;

                    prepared[position] =
                        leftValue
                        +
                        fraction
                        *
                        (
                            rightValue
                            -
                            leftValue
                        );
                }
            }
        }
        else if (
            hasRight
        )
        {
            // Leading run: pandas limit_direction="both"
            // fills the five values nearest the first known value.
            uint16_t firstFill =
                start;

            uint16_t runLength =
                end
                -
                start
                +
                1;

            if (
                runLength
                >
                INTERPOLATION_LIMIT
            )
            {
                firstFill =
                    end
                    -
                    INTERPOLATION_LIMIT
                    +
                    1;
            }

            for (
                uint16_t position = firstFill;
                position <= end;
                position++
            )
            {
                prepared[position] =
                    prepared[index];
            }
        }
        else if (
            hasLeft
        )
        {
            // Trailing run: fill the five values nearest the
            // last known value.
            uint16_t lastFill =
                end;

            uint16_t runLength =
                end
                -
                start
                +
                1;

            if (
                runLength
                >
                INTERPOLATION_LIMIT
            )
            {
                lastFill =
                    start
                    +
                    INTERPOLATION_LIMIT
                    -
                    1;
            }

            for (
                uint16_t position = start;
                position <= lastFill;
                position++
            )
            {
                prepared[position] =
                    prepared[
                        start - 1
                    ];
            }
        }
    }
}


// ============================================================
// QUANTILE
//
// numpy.quantile default linear interpolation.
// ============================================================

float C1001FeatureExtractor::quantile(
    const float sortedValues[C1001_ML_WINDOW_SAMPLES],
    uint16_t count,
    float q
) const
{
    if (
        count == 0
    )
    {
        return NAN;
    }

    if (
        count == 1
    )
    {
        return sortedValues[0];
    }

    float position =
        (
            static_cast<float>(
                count
                -
                1
            )
        )
        *
        q;

    uint16_t lower =
        static_cast<uint16_t>(
            floorf(
                position
            )
        );

    uint16_t upper =
        static_cast<uint16_t>(
            ceilf(
                position
            )
        );

    if (
        lower == upper
    )
    {
        return sortedValues[lower];
    }

    float fraction =
        position
        -
        static_cast<float>(
            lower
        );

    return
        sortedValues[lower]
        +
        fraction
        *
        (
            sortedValues[upper]
            -
            sortedValues[lower]
        );
}


// ============================================================
// MEDIAN
// ============================================================

float C1001FeatureExtractor::medianOfValues(
    const float values[C1001_ML_WINDOW_SAMPLES],
    uint16_t count
) const
{
    if (
        count == 0
    )
    {
        return NAN;
    }

    float sorted[
        C1001_ML_WINDOW_SAMPLES
    ];

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        sorted[i] =
            values[i];
    }

    insertionSort(
        sorted,
        count
    );

    if (
        count % 2U
        ==
        1U
    )
    {
        return sorted[
            count / 2U
        ];
    }

    return
        (
            sorted[
                count / 2U - 1U
            ]
            +
            sorted[
                count / 2U
            ]
        )
        /
        2.0f;
}


// ============================================================
// SLOPE
//
// Equivalent to the first-order np.polyfit slope using only
// finite samples and their ORIGINAL positions.
// ============================================================

float C1001FeatureExtractor::slope(
    const float values[C1001_ML_WINDOW_SAMPLES]
) const
{
    double sumX =
        0.0;

    double sumY =
        0.0;

    double sumXX =
        0.0;

    double sumXY =
        0.0;

    uint16_t count =
        0;

    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        if (
            !isfinite(
                values[i]
            )
        )
        {
            continue;
        }

        double x =
            static_cast<double>(
                i
            );

        double y =
            static_cast<double>(
                values[i]
            );

        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
        count++;
    }

    if (
        count < 2
    )
    {
        return NAN;
    }

    double denominator =
        static_cast<double>(
            count
        )
        *
        sumXX
        -
        sumX
        *
        sumX;

    if (
        fabs(
            denominator
        )
        <
        1.0e-12
    )
    {
        return 0.0f;
    }

    double result =
        (
            static_cast<double>(
                count
            )
            *
            sumXY
            -
            sumX
            *
            sumY
        )
        /
        denominator;

    return static_cast<float>(
        result
    );
}


// ============================================================
// LAG-1 AUTOCORRELATION
//
// Matches np.corrcoef(left, right)[0, 1] after pairwise finite
// masking. Constant signals return 0.
// ============================================================

float C1001FeatureExtractor::lag1Autocorrelation(
    const float values[C1001_ML_WINDOW_SAMPLES]
) const
{
    double sumLeft =
        0.0;

    double sumRight =
        0.0;

    uint16_t count =
        0;

    for (
        uint16_t i = 0;
        i + 1 < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float left =
            values[i];

        float right =
            values[
                i + 1
            ];

        if (
            !isfinite(
                left
            )
            ||
            !isfinite(
                right
            )
        )
        {
            continue;
        }

        sumLeft +=
            left;

        sumRight +=
            right;

        count++;
    }

    if (
        count < 3
    )
    {
        return NAN;
    }

    double meanLeft =
        sumLeft
        /
        static_cast<double>(
            count
        );

    double meanRight =
        sumRight
        /
        static_cast<double>(
            count
        );

    double covarianceNumerator =
        0.0;

    double leftSquares =
        0.0;

    double rightSquares =
        0.0;

    for (
        uint16_t i = 0;
        i + 1 < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float left =
            values[i];

        float right =
            values[
                i + 1
            ];

        if (
            !isfinite(
                left
            )
            ||
            !isfinite(
                right
            )
        )
        {
            continue;
        }

        double leftCentered =
            static_cast<double>(
                left
            )
            -
            meanLeft;

        double rightCentered =
            static_cast<double>(
                right
            )
            -
            meanRight;

        covarianceNumerator +=
            leftCentered
            *
            rightCentered;

        leftSquares +=
            leftCentered
            *
            leftCentered;

        rightSquares +=
            rightCentered
            *
            rightCentered;
    }

    if (
        leftSquares
        <
        1.0e-24
        ||
        rightSquares
        <
        1.0e-24
    )
    {
        return 0.0f;
    }

    return static_cast<float>(
        covarianceNumerator
        /
        sqrt(
            leftSquares
            *
            rightSquares
        )
    );
}


// ============================================================
// STATISTICAL FEATURES
//
// Output indices:
//
//  0 count
//  1 missing_fraction
//  2 mean
//  3 median
//  4 std (sample, ddof=1)
//  5 min
//  6 max
//  7 range
//  8 q05
//  9 q25
// 10 q75
// 11 q95
// 12 iqr
// 13 mad
// 14 rms
// 15 slope
// 16 first
// 17 last
// 18 delta
// 19 mean_abs_change
// 20 max_abs_change
// 21 lag1_autocorrelation
// ============================================================

void C1001FeatureExtractor::statisticalFeatures(
    const float prepared[C1001_ML_WINDOW_SAMPLES],
    float output[22]
) const
{
    float valid[
        C1001_ML_WINDOW_SAMPLES
    ];

    uint16_t count =
        0;

    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        if (
            isfinite(
                prepared[i]
            )
        )
        {
            valid[count++] =
                prepared[i];
        }
    }

    output[0] =
        static_cast<float>(
            count
        );

    output[1] =
        1.0f
        -
        static_cast<float>(
            count
        )
        /
        static_cast<float>(
            C1001_ML_WINDOW_SAMPLES
        );

    if (
        count == 0
    )
    {
        for (
            uint8_t i = 2;
            i < 22;
            i++
        )
        {
            output[i] =
                NAN;
        }

        return;
    }

    double sum =
        0.0;

    double sumSquares =
        0.0;

    float minimum =
        valid[0];

    float maximum =
        valid[0];

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        float value =
            valid[i];

        sum +=
            value;

        sumSquares +=
            static_cast<double>(
                value
            )
            *
            value;

        if (
            value < minimum
        )
        {
            minimum =
                value;
        }

        if (
            value > maximum
        )
        {
            maximum =
                value;
        }
    }

    float mean =
        static_cast<float>(
            sum
            /
            static_cast<double>(
                count
            )
        );

    float sorted[
        C1001_ML_WINDOW_SAMPLES
    ];

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        sorted[i] =
            valid[i];
    }

    insertionSort(
        sorted,
        count
    );

    float median =
        quantile(
            sorted,
            count,
            0.50f
        );

    double varianceNumerator =
        0.0;

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        double difference =
            static_cast<double>(
                valid[i]
            )
            -
            mean;

        varianceNumerator +=
            difference
            *
            difference;
    }

    float sampleStd =
        count > 1
            ? static_cast<float>(
                sqrt(
                    varianceNumerator
                    /
                    static_cast<double>(
                        count - 1
                    )
                )
            )
            : 0.0f;

    float q05 =
        quantile(
            sorted,
            count,
            0.05f
        );

    float q25 =
        quantile(
            sorted,
            count,
            0.25f
        );

    float q75 =
        quantile(
            sorted,
            count,
            0.75f
        );

    float q95 =
        quantile(
            sorted,
            count,
            0.95f
        );

    float deviations[
        C1001_ML_WINDOW_SAMPLES
    ];

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        deviations[i] =
            fabsf(
                valid[i]
                -
                median
            );
    }

    float mad =
        medianOfValues(
            deviations,
            count
        );

    float meanAbsChange =
        0.0f;

    float maxAbsChange =
        0.0f;

    if (
        count > 1
    )
    {
        double changeSum =
            0.0;

        for (
            uint16_t i = 1;
            i < count;
            i++
        )
        {
            float change =
                fabsf(
                    valid[i]
                    -
                    valid[
                        i - 1
                    ]
                );

            changeSum +=
                change;

            if (
                change
                >
                maxAbsChange
            )
            {
                maxAbsChange =
                    change;
            }
        }

        meanAbsChange =
            static_cast<float>(
                changeSum
                /
                static_cast<double>(
                    count - 1
                )
            );
    }

    output[2] =
        mean;

    output[3] =
        median;

    output[4] =
        sampleStd;

    output[5] =
        minimum;

    output[6] =
        maximum;

    output[7] =
        maximum
        -
        minimum;

    output[8] =
        q05;

    output[9] =
        q25;

    output[10] =
        q75;

    output[11] =
        q95;

    output[12] =
        q75
        -
        q25;

    output[13] =
        mad;

    output[14] =
        static_cast<float>(
            sqrt(
                sumSquares
                /
                static_cast<double>(
                    count
                )
            )
        );

    output[15] =
        slope(
            prepared
        );

    output[16] =
        valid[0];

    output[17] =
        valid[
            count - 1
        ];

    output[18] =
        valid[
            count - 1
        ]
        -
        valid[0];

    output[19] =
        meanAbsChange;

    output[20] =
        maxAbsChange;

    output[21] =
        lag1Autocorrelation(
            prepared
        );
}


// ============================================================
// LONGEST TRUE RUN
// ============================================================

uint16_t C1001FeatureExtractor::longestTrueRun(
    const bool mask[C1001_ML_WINDOW_SAMPLES]
) const
{
    uint16_t longest =
        0;

    uint16_t current =
        0;

    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        if (
            mask[i]
        )
        {
            current++;

            if (
                current > longest
            )
            {
                longest =
                    current;
            }
        }
        else
        {
            current =
                0;
        }
    }

    return longest;
}


// ============================================================
// RANGE FEATURES
//
// Output indices:
// 0 valid_fraction
// 1 invalid_fraction
// 2 normal_fraction
// 3 below_normal_fraction
// 4 above_normal_fraction
// 5 longest_invalid_run
// 6 longest_below_normal_run
// 7 longest_above_normal_run
// ============================================================

void C1001FeatureExtractor::rangeFeatures(
    const float raw[C1001_ML_WINDOW_SAMPLES],
    float validMinimum,
    float validMaximum,
    float normalMinimum,
    float normalMaximum,
    float output[8]
) const
{
    bool invalidMask[
        C1001_ML_WINDOW_SAMPLES
    ];

    bool belowMask[
        C1001_ML_WINDOW_SAMPLES
    ];

    bool aboveMask[
        C1001_ML_WINDOW_SAMPLES
    ];

    uint16_t validCount =
        0;

    uint16_t invalidCount =
        0;

    uint16_t normalCount =
        0;

    uint16_t belowCount =
        0;

    uint16_t aboveCount =
        0;

    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float value =
            raw[i];

        bool valid =
            isfinite(
                value
            )
            &&
            !isInvalidCode(
                value
            )
            &&
            value
            >=
            validMinimum
            &&
            value
            <=
            validMaximum;

        bool below =
            valid
            &&
            value
            <
            normalMinimum;

        bool normal =
            valid
            &&
            value
            >=
            normalMinimum
            &&
            value
            <=
            normalMaximum;

        bool above =
            valid
            &&
            value
            >
            normalMaximum;

        bool invalid =
            !valid;

        invalidMask[i] =
            invalid;

        belowMask[i] =
            below;

        aboveMask[i] =
            above;

        validCount +=
            valid
                ? 1
                : 0;

        invalidCount +=
            invalid
                ? 1
                : 0;

        normalCount +=
            normal
                ? 1
                : 0;

        belowCount +=
            below
                ? 1
                : 0;

        aboveCount +=
            above
                ? 1
                : 0;
    }

    float denominator =
        static_cast<float>(
            C1001_ML_WINDOW_SAMPLES
        );

    output[0] =
        static_cast<float>(
            validCount
        )
        /
        denominator;

    output[1] =
        static_cast<float>(
            invalidCount
        )
        /
        denominator;

    output[2] =
        static_cast<float>(
            normalCount
        )
        /
        denominator;

    output[3] =
        static_cast<float>(
            belowCount
        )
        /
        denominator;

    output[4] =
        static_cast<float>(
            aboveCount
        )
        /
        denominator;

    output[5] =
        static_cast<float>(
            longestTrueRun(
                invalidMask
            )
        );

    output[6] =
        static_cast<float>(
            longestTrueRun(
                belowMask
            )
        );

    output[7] =
        static_cast<float>(
            longestTrueRun(
                aboveMask
            )
        );
}


// ============================================================
// CROSS FEATURES
//
// Output:
// 0 rr_hr_pair_count
// 1 rr_hr_correlation
// 2 hr_rr_ratio_mean
// 3 rr_hr_ratio_mean
// ============================================================

void C1001FeatureExtractor::crossFeatures(
    const float rrPrepared[C1001_ML_WINDOW_SAMPLES],
    const float hrPrepared[C1001_ML_WINDOW_SAMPLES],
    float output[4]
) const
{
    uint16_t count =
        0;

    double sumRR =
        0.0;

    double sumHR =
        0.0;

    double hrRrRatioSum =
        0.0;

    double rrHrRatioSum =
        0.0;

    uint16_t hrRrRatioCount =
        0;

    uint16_t rrHrRatioCount =
        0;

    for (
        uint16_t i = 0;
        i < C1001_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float rr =
            rrPrepared[i];

        float hr =
            hrPrepared[i];

        if (
            !isfinite(
                rr
            )
            ||
            !isfinite(
                hr
            )
        )
        {
            continue;
        }

        sumRR +=
            rr;

        sumHR +=
            hr;

        count++;

        if (
            fabsf(
                rr
            )
            >
            1.0e-12f
        )
        {
            hrRrRatioSum +=
                static_cast<double>(
                    hr
                )
                /
                rr;

            hrRrRatioCount++;
        }

        if (
            fabsf(
                hr
            )
            >
            1.0e-12f
        )
        {
            rrHrRatioSum +=
                static_cast<double>(
                    rr
                )
                /
                hr;

            rrHrRatioCount++;
        }
    }

    output[0] =
        static_cast<float>(
            count
        );

    if (
        count == 0
    )
    {
        output[1] =
            NAN;

        output[2] =
            NAN;

        output[3] =
            NAN;

        return;
    }

    float correlation =
        0.0f;

    if (
        count >= 3
    )
    {
        double meanRR =
            sumRR
            /
            static_cast<double>(
                count
            );

        double meanHR =
            sumHR
            /
            static_cast<double>(
                count
            );

        double covarianceNumerator =
            0.0;

        double rrSquares =
            0.0;

        double hrSquares =
            0.0;

        for (
            uint16_t i = 0;
            i < C1001_ML_WINDOW_SAMPLES;
            i++
        )
        {
            float rr =
                rrPrepared[i];

            float hr =
                hrPrepared[i];

            if (
                !isfinite(
                    rr
                )
                ||
                !isfinite(
                    hr
                )
            )
            {
                continue;
            }

            double rrCentered =
                static_cast<double>(
                    rr
                )
                -
                meanRR;

            double hrCentered =
                static_cast<double>(
                    hr
                )
                -
                meanHR;

            covarianceNumerator +=
                rrCentered
                *
                hrCentered;

            rrSquares +=
                rrCentered
                *
                rrCentered;

            hrSquares +=
                hrCentered
                *
                hrCentered;
        }

        if (
            rrSquares
            >
            1.0e-24
            &&
            hrSquares
            >
            1.0e-24
        )
        {
            correlation =
                static_cast<float>(
                    covarianceNumerator
                    /
                    sqrt(
                        rrSquares
                        *
                        hrSquares
                    )
                );
        }
    }

    output[1] =
        correlation;

    output[2] =
        hrRrRatioCount > 0
            ? static_cast<float>(
                hrRrRatioSum
                /
                static_cast<double>(
                    hrRrRatioCount
                )
            )
            : NAN;

    output[3] =
        rrHrRatioCount > 0
            ? static_cast<float>(
                rrHrRatioSum
                /
                static_cast<double>(
                    rrHrRatioCount
                )
            )
            : NAN;
}


// ============================================================
// EXTRACT
// ============================================================

bool C1001FeatureExtractor::extract(
    const float rawHeartRate[C1001_ML_WINDOW_SAMPLES],
    const float rawRespiration[C1001_ML_WINDOW_SAMPLES],
    C1001FeatureVector &output
) const
{
    output =
        C1001FeatureVector{};

    float hrPrepared[
        C1001_ML_WINDOW_SAMPLES
    ];

    float rrPrepared[
        C1001_ML_WINDOW_SAMPLES
    ];

    prepareSignal(
        rawHeartRate,
        hrPrepared
    );

    prepareSignal(
        rawRespiration,
        rrPrepared
    );

    float hrStats[22];
    float rrStats[22];

    float hrRange[8];
    float rrRange[8];

    float cross[4];

    statisticalFeatures(
        hrPrepared,
        hrStats
    );

    statisticalFeatures(
        rrPrepared,
        rrStats
    );

    rangeFeatures(
        rawHeartRate,
        30.0f,
        220.0f,
        60.0f,
        100.0f,
        hrRange
    );

    rangeFeatures(
        rawRespiration,
        4.0f,
        60.0f,
        12.0f,
        20.0f,
        rrRange
    );

    crossFeatures(
        rrPrepared,
        hrPrepared,
        cross
    );

    // --------------------------------------------------------
    // EXACT feature_columns.json ORDER
    // --------------------------------------------------------

    output.values[0]  = hrRange[4];   // hr_above_normal_fraction
    output.values[1]  = hrRange[3];   // hr_below_normal_fraction
    output.values[2]  = hrStats[0];   // hr_count
    output.values[3]  = hrStats[18];  // hr_delta
    output.values[4]  = hrStats[16];  // hr_first
    output.values[5]  = hrRange[1];   // hr_invalid_fraction
    output.values[6]  = hrStats[12];  // hr_iqr
    output.values[7]  = hrStats[21];  // hr_lag1_autocorrelation
    output.values[8]  = hrStats[17];  // hr_last
    output.values[9]  = hrRange[7];   // hr_longest_above_normal_run
    output.values[10] = hrRange[6];   // hr_longest_below_normal_run
    output.values[11] = hrRange[5];   // hr_longest_invalid_run
    output.values[12] = hrStats[13];  // hr_mad
    output.values[13] = hrStats[6];   // hr_max
    output.values[14] = hrStats[20];  // hr_max_abs_change
    output.values[15] = hrStats[2];   // hr_mean
    output.values[16] = hrStats[19];  // hr_mean_abs_change
    output.values[17] = hrStats[3];   // hr_median
    output.values[18] = hrStats[5];   // hr_min
    output.values[19] = hrStats[1];   // hr_missing_fraction
    output.values[20] = hrRange[2];   // hr_normal_fraction
    output.values[21] = hrStats[8];   // hr_q05
    output.values[22] = hrStats[9];   // hr_q25
    output.values[23] = hrStats[10];  // hr_q75
    output.values[24] = hrStats[11];  // hr_q95
    output.values[25] = hrStats[7];   // hr_range
    output.values[26] = hrStats[14];  // hr_rms
    output.values[27] = cross[2];     // hr_rr_ratio_mean
    output.values[28] = hrStats[15];  // hr_slope
    output.values[29] = hrStats[4];   // hr_std
    output.values[30] = hrRange[0];   // hr_valid_fraction

    output.values[31] = rrRange[4];   // rr_above_normal_fraction
    output.values[32] = rrRange[3];   // rr_below_normal_fraction
    output.values[33] = rrStats[0];   // rr_count
    output.values[34] = rrStats[18];  // rr_delta
    output.values[35] = rrStats[16];  // rr_first
    output.values[36] = cross[1];     // rr_hr_correlation
    output.values[37] = cross[0];     // rr_hr_pair_count
    output.values[38] = cross[3];     // rr_hr_ratio_mean
    output.values[39] = rrRange[1];   // rr_invalid_fraction
    output.values[40] = rrStats[12];  // rr_iqr
    output.values[41] = rrStats[21];  // rr_lag1_autocorrelation
    output.values[42] = rrStats[17];  // rr_last
    output.values[43] = rrRange[7];   // rr_longest_above_normal_run
    output.values[44] = rrRange[6];   // rr_longest_below_normal_run
    output.values[45] = rrRange[5];   // rr_longest_invalid_run
    output.values[46] = rrStats[13];  // rr_mad
    output.values[47] = rrStats[6];   // rr_max
    output.values[48] = rrStats[20];  // rr_max_abs_change
    output.values[49] = rrStats[2];   // rr_mean
    output.values[50] = rrStats[19];  // rr_mean_abs_change
    output.values[51] = rrStats[3];   // rr_median
    output.values[52] = rrStats[5];   // rr_min
    output.values[53] = rrStats[1];   // rr_missing_fraction
    output.values[54] = rrRange[2];   // rr_normal_fraction
    output.values[55] = rrStats[8];   // rr_q05
    output.values[56] = rrStats[9];   // rr_q25
    output.values[57] = rrStats[10];  // rr_q75
    output.values[58] = rrStats[11];  // rr_q95
    output.values[59] = rrStats[7];   // rr_range
    output.values[60] = rrStats[14];  // rr_rms
    output.values[61] = rrStats[15];  // rr_slope
    output.values[62] = rrStats[4];   // rr_std
    output.values[63] = rrRange[0];   // rr_valid_fraction

    // The model preprocessor can impute a small number of
    // non-finite feature values, exactly like sklearn.
    //
    // We only require that at least one finite HR and RR sample
    // survived signal preparation.
    output.valid =
        hrStats[0] > 0.0f
        &&
        rrStats[0] > 0.0f;

    return output.valid;
}
