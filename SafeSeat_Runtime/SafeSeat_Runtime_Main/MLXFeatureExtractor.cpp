#include "MLXFeatureExtractor.h"

#include <math.h>


// ============================================================
// PANDAS-LIKE LIMITED LINEAR INTERPOLATION
//
// Training:
// Series.interpolate(
//     method="linear",
//     limit=8,
//     limit_direction="both"
// )
//
// Internal gaps receive up to eight fills from each side.
// Leading/trailing gaps receive up to eight fills adjacent to
// the nearest finite value.
// ============================================================

void MLXFeatureExtractor::interpolateTemperature(
    const float raw[MLX_ML_WINDOW_SAMPLES],
    float cleaned[MLX_ML_WINDOW_SAMPLES]
) const
{
    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        cleaned[i] =
            raw[i];
    }

    uint16_t index =
        0;

    while (
        index
        <
        MLX_ML_WINDOW_SAMPLES
    )
    {
        if (
            isfinite(
                cleaned[index]
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
            MLX_ML_WINDOW_SAMPLES
            &&
            !isfinite(
                cleaned[index]
            )
        )
        {
            index++;
        }

        uint16_t end =
            index;

        uint16_t runLength =
            end
            -
            start;

        bool hasLeft =
            start
            >
            0
            &&
            isfinite(
                cleaned[
                    start
                    -
                    1
                ]
            );

        bool hasRight =
            end
            <
            MLX_ML_WINDOW_SAMPLES
            &&
            isfinite(
                cleaned[
                    end
                ]
            );

        if (
            hasLeft
            &&
            hasRight
        )
        {
            float leftValue =
                cleaned[
                    start
                    -
                    1
                ];

            float rightValue =
                cleaned[
                    end
                ];

            for (
                uint16_t k = 0;
                k < runLength;
                k++
            )
            {
                bool fillFromLeft =
                    k
                    <
                    INTERPOLATION_LIMIT;

                bool fillFromRight =
                    (
                        runLength
                        -
                        1
                        -
                        k
                    )
                    <
                    INTERPOLATION_LIMIT;

                if (
                    !fillFromLeft
                    &&
                    !fillFromRight
                )
                {
                    continue;
                }

                float fraction =
                    static_cast<float>(
                        k
                        +
                        1
                    )
                    /
                    static_cast<float>(
                        runLength
                        +
                        1
                    );

                cleaned[
                    start
                    +
                    k
                ] =
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
        else if (
            hasLeft
        )
        {
            uint16_t fillCount =
                runLength
                <
                INTERPOLATION_LIMIT
                    ? runLength
                    : INTERPOLATION_LIMIT;

            float leftValue =
                cleaned[
                    start
                    -
                    1
                ];

            for (
                uint16_t k = 0;
                k < fillCount;
                k++
            )
            {
                cleaned[
                    start
                    +
                    k
                ] =
                    leftValue;
            }
        }
        else if (
            hasRight
        )
        {
            uint16_t fillCount =
                runLength
                <
                INTERPOLATION_LIMIT
                    ? runLength
                    : INTERPOLATION_LIMIT;

            float rightValue =
                cleaned[
                    end
                ];

            for (
                uint16_t k = 0;
                k < fillCount;
                k++
            )
            {
                uint16_t destination =
                    end
                    -
                    1
                    -
                    k;

                cleaned[
                    destination
                ] =
                    rightValue;
            }
        }
    }
}


// ============================================================
// FINITE VALUES
// ============================================================

uint16_t MLXFeatureExtractor::collectFinite(
    const float values[MLX_ML_WINDOW_SAMPLES],
    float finiteValues[MLX_ML_WINDOW_SAMPLES]
) const
{
    uint16_t count =
        0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        if (
            isfinite(
                values[i]
            )
        )
        {
            finiteValues[
                count
            ] =
                values[i];

            count++;
        }
    }

    return count;
}


// ============================================================
// IN-PLACE INSERTION SORT
// ============================================================

void MLXFeatureExtractor::sortValues(
    float values[MLX_ML_WINDOW_SAMPLES],
    uint16_t count
) const
{
    for (
        uint16_t i = 1;
        i < count;
        i++
    )
    {
        float key =
            values[i];

        int16_t j =
            static_cast<int16_t>(
                i
            )
            -
            1;

        while (
            j
            >=
            0
            &&
            values[
                static_cast<uint16_t>(
                    j
                )
            ]
            >
            key
        )
        {
            values[
                static_cast<uint16_t>(
                    j
                    +
                    1
                )
            ] =
                values[
                    static_cast<uint16_t>(
                        j
                    )
                ];

            j--;
        }

        values[
            static_cast<uint16_t>(
                j
                +
                1
            )
        ] =
            key;
    }
}


// ============================================================
// NUMPY-LIKE LINEAR QUANTILE
//
// numpy.quantile default method="linear":
// index = (n - 1) * q
// ============================================================

float MLXFeatureExtractor::quantileSorted(
    const float sortedValues[MLX_ML_WINDOW_SAMPLES],
    uint16_t count,
    float q
) const
{
    if (
        count
        ==
        0
    )
    {
        return NAN;
    }

    if (
        count
        ==
        1
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
        upper
        >=
        count
    )
    {
        upper =
            count
            -
            1;
    }

    float fraction =
        position
        -
        static_cast<float>(
            lower
        );

    return
        sortedValues[
            lower
        ]
        +
        fraction
        *
        (
            sortedValues[
                upper
            ]
            -
            sortedValues[
                lower
            ]
        );
}


float MLXFeatureExtractor::medianSorted(
    const float sortedValues[MLX_ML_WINDOW_SAMPLES],
    uint16_t count
) const
{
    return quantileSorted(
        sortedValues,
        count,
        0.5f
    );
}


// ============================================================
// AUTOCORRELATION
//
// Equivalent to numpy.corrcoef(left, right)[0, 1] after
// dropping pairs with non-finite values.
// ============================================================

float MLXFeatureExtractor::autocorrelation(
    const float values[MLX_ML_WINDOW_SAMPLES],
    uint8_t lag
) const
{
    if (
        MLX_ML_WINDOW_SAMPLES
        <=
        lag
    )
    {
        return NAN;
    }

    uint16_t count =
        0;

    double sumLeft =
        0.0;

    double sumRight =
        0.0;

    for (
        uint16_t i = 0;
        i
        +
        lag
        <
        MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float left =
            values[i];

        float right =
            values[
                i
                +
                lag
            ];

        if (
            isfinite(
                left
            )
            &&
            isfinite(
                right
            )
        )
        {
            sumLeft +=
                left;

            sumRight +=
                right;

            count++;
        }
    }

    if (
        count
        <
        3
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

    double sumLeftSquared =
        0.0;

    double sumRightSquared =
        0.0;

    double sumCross =
        0.0;

    for (
        uint16_t i = 0;
        i
        +
        lag
        <
        MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float left =
            values[i];

        float right =
            values[
                i
                +
                lag
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

        double centeredLeft =
            static_cast<double>(
                left
            )
            -
            meanLeft;

        double centeredRight =
            static_cast<double>(
                right
            )
            -
            meanRight;

        sumLeftSquared +=
            centeredLeft
            *
            centeredLeft;

        sumRightSquared +=
            centeredRight
            *
            centeredRight;

        sumCross +=
            centeredLeft
            *
            centeredRight;
    }

    if (
        sumLeftSquared
        <
        1.0e-24
        ||
        sumRightSquared
        <
        1.0e-24
    )
    {
        return 0.0f;
    }

    return static_cast<float>(
        sumCross
        /
        sqrt(
            sumLeftSquared
            *
            sumRightSquared
        )
    );
}


// ============================================================
// LINEAR SLOPE
//
// x = sample index / 4 Hz.
// ============================================================

float MLXFeatureExtractor::linearSlope(
    const float values[MLX_ML_WINDOW_SAMPLES]
) const
{
    uint16_t count =
        0;

    double sumX =
        0.0;

    double sumY =
        0.0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
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
            )
            /
            static_cast<double>(
                SAMPLING_RATE_HZ
            );

        sumX +=
            x;

        sumY +=
            values[i];

        count++;
    }

    if (
        count
        <
        2
    )
    {
        return NAN;
    }

    double meanX =
        sumX
        /
        static_cast<double>(
            count
        );

    double meanY =
        sumY
        /
        static_cast<double>(
            count
        );

    double numerator =
        0.0;

    double denominator =
        0.0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
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
            )
            /
            static_cast<double>(
                SAMPLING_RATE_HZ
            );

        double dx =
            x
            -
            meanX;

        numerator +=
            dx
            *
            (
                static_cast<double>(
                    values[i]
                )
                -
                meanY
            );

        denominator +=
            dx
            *
            dx;
    }

    if (
        denominator
        <=
        0.0
    )
    {
        return 0.0f;
    }

    return static_cast<float>(
        numerator
        /
        denominator
    );
}


// ============================================================
// LINEAR RESIDUAL SAMPLE STANDARD DEVIATION
// ============================================================

float MLXFeatureExtractor::linearResidualStd(
    const float values[MLX_ML_WINDOW_SAMPLES]
) const
{
    uint16_t count =
        0;

    double sumX =
        0.0;

    double sumY =
        0.0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
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
            )
            /
            static_cast<double>(
                SAMPLING_RATE_HZ
            );

        sumX +=
            x;

        sumY +=
            values[i];

        count++;
    }

    if (
        count
        <
        3
    )
    {
        return NAN;
    }

    double meanX =
        sumX
        /
        static_cast<double>(
            count
        );

    double meanY =
        sumY
        /
        static_cast<double>(
            count
        );

    double numerator =
        0.0;

    double denominator =
        0.0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
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
            )
            /
            static_cast<double>(
                SAMPLING_RATE_HZ
            );

        double dx =
            x
            -
            meanX;

        numerator +=
            dx
            *
            (
                static_cast<double>(
                    values[i]
                )
                -
                meanY
            );

        denominator +=
            dx
            *
            dx;
    }

    double slope =
        denominator
        >
        0.0
            ? numerator
              /
              denominator
            : 0.0;

    double intercept =
        meanY
        -
        slope
        *
        meanX;

    // numpy.std(residuals, ddof=1) subtracts the residual mean.
    double residualSum =
        0.0;

    double residualSquaredSum =
        0.0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
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
            )
            /
            static_cast<double>(
                SAMPLING_RATE_HZ
            );

        double residual =
            static_cast<double>(
                values[i]
            )
            -
            (
                slope
                *
                x
                +
                intercept
            );

        residualSum +=
            residual;

        residualSquaredSum +=
            residual
            *
            residual;
    }

    double residualMean =
        residualSum
        /
        static_cast<double>(
            count
        );

    double centeredSquaredSum =
        residualSquaredSum
        -
        static_cast<double>(
            count
        )
        *
        residualMean
        *
        residualMean;

    if (
        centeredSquaredSum
        <
        0.0
        &&
        centeredSquaredSum
        >
        -1.0e-12
    )
    {
        centeredSquaredSum =
            0.0;
    }

    return static_cast<float>(
        sqrt(
            centeredSquaredSum
            /
            static_cast<double>(
                count
                -
                1
            )
        )
    );
}


// ============================================================
// LONGEST INVALID RUN
//
// Training validity definition:
// finite and 20 C <= value <= 45 C.
// ============================================================

uint16_t MLXFeatureExtractor::longestInvalidRun(
    const float raw[MLX_ML_WINDOW_SAMPLES]
) const
{
    uint16_t longest =
        0;

    uint16_t current =
        0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        bool inRange =
            isfinite(
                raw[i]
            )
            &&
            raw[i]
            >=
            TRAINING_VALID_MIN_C
            &&
            raw[i]
            <=
            TRAINING_VALID_MAX_C;

        if (
            !inRange
        )
        {
            current++;

            if (
                current
                >
                longest
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
// EXTRACT FEATURES
// ============================================================

bool MLXFeatureExtractor::extract(
    const float rawObjectTemperature[MLX_ML_WINDOW_SAMPLES],
    MLXFeatureVector &output
) const
{
    output =
        MLXFeatureVector{};

    float cleaned[
        MLX_ML_WINDOW_SAMPLES
    ];

    interpolateTemperature(
        rawObjectTemperature,
        cleaned
    );

    float valid[
        MLX_ML_WINDOW_SAMPLES
    ];

    uint16_t count =
        collectFinite(
            cleaned,
            valid
        );

    output.finiteSampleCount =
        count;

    if (
        count
        ==
        0
    )
    {
        // Keep features as NAN where statistics are undefined.
        for (
            uint16_t i = 0;
            i < MLX_MODEL_FEATURE_COUNT;
            i++
        )
        {
            output.values[i] =
                NAN;
        }

        return false;
    }

    // ========================================================
    // STEP 5.4.1 TRANSFER CORRECTION
    //
    // Match Python training exactly:
    // cleaned temperature -> finite median -> subtract that
    // median from every finite sample in this 30-second window.
    //
    // This makes the learned feature vector invariant to a
    // constant absolute temperature offset (e.g. WESAD wrist
    // level versus MLX non-contact palm/forehead level).
    // Ambient temperature is NOT subtracted here and remains
    // separate deployment context / warm-target qualification.
    // ========================================================

    float centerValues[
        MLX_ML_WINDOW_SAMPLES
    ];

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        centerValues[i] =
            valid[i];
    }

    sortValues(
        centerValues,
        count
    );

    float windowMedian =
        medianSorted(
            centerValues,
            count
        );

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        if (
            isfinite(
                cleaned[i]
            )
        )
        {
            cleaned[i] =
                cleaned[i]
                -
                windowMedian;
        }
    }

    count =
        collectFinite(
            cleaned,
            valid
        );

    output.finiteSampleCount =
        count;

    float sorted[
        MLX_ML_WINDOW_SAMPLES
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

    sortValues(
        sorted,
        count
    );

    float q05 =
        quantileSorted(
            sorted,
            count,
            0.05f
        );

    float q10 =
        quantileSorted(
            sorted,
            count,
            0.10f
        );

    float q25 =
        quantileSorted(
            sorted,
            count,
            0.25f
        );

    float q75 =
        quantileSorted(
            sorted,
            count,
            0.75f
        );

    float q90 =
        quantileSorted(
            sorted,
            count,
            0.90f
        );

    float q95 =
        quantileSorted(
            sorted,
            count,
            0.95f
        );

    float median =
        medianSorted(
            sorted,
            count
        );

    double sum =
        0.0;

    double squareSum =
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

        squareSum +=
            static_cast<double>(
                value
            )
            *
            static_cast<double>(
                value
            );

        if (
            value
            <
            minimum
        )
        {
            minimum =
                value;
        }

        if (
            value
            >
            maximum
        )
        {
            maximum =
                value;
        }
    }

    double mean =
        sum
        /
        static_cast<double>(
            count
        );

    double variance =
        0.0;

    if (
        count
        >
        1
    )
    {
        double squaredDeviationSum =
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

            squaredDeviationSum +=
                difference
                *
                difference;
        }

        variance =
            squaredDeviationSum
            /
            static_cast<double>(
                count
                -
                1
            );
    }

    float standardDeviation =
        static_cast<float>(
            sqrt(
                variance
            )
        );

    float absoluteDeviations[
        MLX_ML_WINDOW_SAMPLES
    ];

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        absoluteDeviations[i] =
            fabsf(
                valid[i]
                -
                median
            );
    }

    sortValues(
        absoluteDeviations,
        count
    );

    float mad =
        medianSorted(
            absoluteDeviations,
            count
        );

    float rms =
        static_cast<float>(
            sqrt(
                squareSum
                /
                static_cast<double>(
                    count
                )
            )
        );

    // --------------------------------------------------------
    // Changes over compressed finite values, matching
    // np.diff(valid).
    // --------------------------------------------------------

    uint16_t differenceCount =
        count
        >
        0
            ? count
              -
              1
            : 0;

    double differenceSum =
        0.0;

    double absoluteDifferenceSum =
        0.0;

    float maxAbsoluteDifference =
        0.0f;

    float largestWarmingStep =
        0.0f;

    float largestCoolingStep =
        0.0f;

    uint16_t warmingCount =
        0;

    uint16_t coolingCount =
        0;

    uint16_t unchangedCount =
        0;

    double differences[
        MLX_ML_WINDOW_SAMPLES
    ];

    constexpr double
        EPSILON =
            1.0e-9;

    if (
        differenceCount
        >
        0
    )
    {
        largestWarmingStep =
            valid[1]
            -
            valid[0];

        largestCoolingStep =
            largestWarmingStep;

        for (
            uint16_t i = 0;
            i < differenceCount;
            i++
        )
        {
            double difference =
                static_cast<double>(
                    valid[
                        i
                        +
                        1
                    ]
                )
                -
                static_cast<double>(
                    valid[i]
                );

            differences[i] =
                difference;

            differenceSum +=
                difference;

            double absoluteDifference =
                fabs(
                    difference
                );

            absoluteDifferenceSum +=
                absoluteDifference;

            if (
                absoluteDifference
                >
                maxAbsoluteDifference
            )
            {
                maxAbsoluteDifference =
                    static_cast<float>(
                        absoluteDifference
                    );
            }

            if (
                difference
                >
                largestWarmingStep
            )
            {
                largestWarmingStep =
                    static_cast<float>(
                        difference
                    );
            }

            if (
                difference
                <
                largestCoolingStep
            )
            {
                largestCoolingStep =
                    static_cast<float>(
                        difference
                    );
            }

            if (
                difference
                >
                EPSILON
            )
            {
                warmingCount++;
            }
            else if (
                difference
                <
                -EPSILON
            )
            {
                coolingCount++;
            }
            else
            {
                unchangedCount++;
            }
        }
    }

    float meanAbsoluteChange =
        differenceCount
        >
        0
            ? static_cast<float>(
                absoluteDifferenceSum
                /
                static_cast<double>(
                    differenceCount
                )
            )
            : 0.0f;

    float changeStd =
        0.0f;

    if (
        differenceCount
        >
        1
    )
    {
        double differenceMean =
            differenceSum
            /
            static_cast<double>(
                differenceCount
            );

        double differenceSquaredDeviationSum =
            0.0;

        for (
            uint16_t i = 0;
            i < differenceCount;
            i++
        )
        {
            double d =
                differences[i]
                -
                differenceMean;

            differenceSquaredDeviationSum +=
                d
                *
                d;
        }

        changeStd =
            static_cast<float>(
                sqrt(
                    differenceSquaredDeviationSum
                    /
                    static_cast<double>(
                        differenceCount
                        -
                        1
                    )
                )
            );
    }

    float meanChangePerSecond =
        differenceCount
        >
        0
            ? static_cast<float>(
                differenceSum
                /
                static_cast<double>(
                    differenceCount
                )
                *
                SAMPLING_RATE_HZ
            )
            : 0.0f;

    float maxChangePerSecond =
        maxAbsoluteDifference
        *
        SAMPLING_RATE_HZ;

    float warmingFraction =
        differenceCount
        >
        0
            ? static_cast<float>(
                warmingCount
            )
              /
              static_cast<float>(
                differenceCount
            )
            : 0.0f;

    float coolingFraction =
        differenceCount
        >
        0
            ? static_cast<float>(
                coolingCount
            )
              /
              static_cast<float>(
                differenceCount
            )
            : 0.0f;

    float unchangedFraction =
        differenceCount
        >
        0
            ? static_cast<float>(
                unchangedCount
            )
              /
              static_cast<float>(
                differenceCount
            )
            : 1.0f;

    // --------------------------------------------------------
    // Raw validity features
    // --------------------------------------------------------

    uint16_t validRawCount =
        0;

    uint16_t belowRawCount =
        0;

    uint16_t aboveRawCount =
        0;

    for (
        uint16_t i = 0;
        i < MLX_ML_WINDOW_SAMPLES;
        i++
    )
    {
        float value =
            rawObjectTemperature[i];

        if (
            !isfinite(
                value
            )
        )
        {
            continue;
        }

        if (
            value
            <
            TRAINING_VALID_MIN_C
        )
        {
            belowRawCount++;
        }
        else if (
            value
            >
            TRAINING_VALID_MAX_C
        )
        {
            aboveRawCount++;
        }
        else
        {
            validRawCount++;
        }
    }

    uint16_t invalidRawCount =
        MLX_ML_WINDOW_SAMPLES
        -
        validRawCount;

    float denominator =
        static_cast<float>(
            MLX_ML_WINDOW_SAMPLES
        );

    float validFraction =
        static_cast<float>(
            validRawCount
        )
        /
        denominator;

    float invalidFraction =
        static_cast<float>(
            invalidRawCount
        )
        /
        denominator;

    float belowFraction =
        static_cast<float>(
            belowRawCount
        )
        /
        denominator;

    float aboveFraction =
        static_cast<float>(
            aboveRawCount
        )
        /
        denominator;

    float missingFraction =
        1.0f
        -
        static_cast<float>(
            count
        )
        /
        denominator;

    uint16_t within02Count =
        0;

    uint16_t within05Count =
        0;

    for (
        uint16_t i = 0;
        i < count;
        i++
    )
    {
        float distance =
            fabsf(
                valid[i]
                -
                median
            );

        if (
            distance
            <=
            0.2f
            +
            1.0e-5f
        )
        {
            within02Count++;
        }

        if (
            distance
            <=
            0.5f
            +
            1.0e-5f
        )
        {
            within05Count++;
        }
    }

    float within02Fraction =
        static_cast<float>(
            within02Count
        )
        /
        static_cast<float>(
            count
        );

    float within05Fraction =
        static_cast<float>(
            within05Count
        )
        /
        static_cast<float>(
            count
        );

    // ========================================================
    // EXACT 38-FEATURE ORDER FROM STEP 5.4.1 feature_columns.json
    // All statistics below are derived from the median-centered
    // 30-second window. Raw absolute-temperature QA features are
    // intentionally excluded from the anomaly model.
    // ========================================================

    output.values[0] =
        changeStd;

    output.values[1] =
        coolingFraction;

    output.values[2] =
        static_cast<float>(
            count
        );

    output.values[3] =
        valid[
            count
            -
            1
        ]
        -
        valid[0];

    output.values[4] =
        valid[0];

    output.values[5] =
        q75
        -
        q25;

    output.values[6] =
        autocorrelation(
            cleaned,
            1
        );

    output.values[7] =
        autocorrelation(
            cleaned,
            4
        );

    output.values[8] =
        differenceCount
        >
        0
            ? largestCoolingStep
            : 0.0f;

    output.values[9] =
        differenceCount
        >
        0
            ? largestWarmingStep
            : 0.0f;

    output.values[10] =
        valid[
            count
            -
            1
        ];

    output.values[11] =
        linearResidualStd(
            cleaned
        );

    output.values[12] =
        mad;

    output.values[13] =
        maximum;

    output.values[14] =
        maxAbsoluteDifference;

    output.values[15] =
        maxChangePerSecond;

    output.values[16] =
        static_cast<float>(
            mean
        );

    output.values[17] =
        meanAbsoluteChange;

    output.values[18] =
        meanChangePerSecond;

    output.values[19] =
        minimum;

    output.values[20] =
        missingFraction;

    output.values[21] =
        static_cast<float>(
            coolingCount
        );

    output.values[22] =
        static_cast<float>(
            warmingCount
        );

    output.values[23] =
        q05;

    output.values[24] =
        q10;

    output.values[25] =
        q25;

    output.values[26] =
        q75;

    output.values[27] =
        q90;

    output.values[28] =
        q95;

    output.values[29] =
        maximum
        -
        minimum;

    output.values[30] =
        rms;

    output.values[31] =
        linearSlope(
            cleaned
        );

    output.values[32] =
        standardDeviation;

    output.values[33] =
        unchangedFraction;

    output.values[34] =
        static_cast<float>(
            variance
        );

    output.values[35] =
        warmingFraction;

    output.values[36] =
        within02Fraction;

    output.values[37] =
        within05Fraction;

    output.valid =
        true;

    return true;
}
