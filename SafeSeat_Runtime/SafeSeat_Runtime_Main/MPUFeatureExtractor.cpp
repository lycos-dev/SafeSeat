#include "MPUFeatureExtractor.h"

#include <math.h>

// ============================================================
// SIGNAL MAPPING
// 0 accel magnitude
// 1 accel X
// 2 accel Y
// 3 accel Z
// 4 gyro magnitude
// 5 gyro X
// 6 gyro Y
// 7 gyro Z
// ============================================================

float MPUFeatureExtractor::signalValue(
    const MPUModelSample &sample,
    uint8_t signalIndex
) const
{
    switch (signalIndex)
    {
        case 0:
            return sqrtf(
                sample.accelX * sample.accelX +
                sample.accelY * sample.accelY +
                sample.accelZ * sample.accelZ
            );

        case 1:
            return sample.accelX;

        case 2:
            return sample.accelY;

        case 3:
            return sample.accelZ;

        case 4:
            return sqrtf(
                sample.gyroX * sample.gyroX +
                sample.gyroY * sample.gyroY +
                sample.gyroZ * sample.gyroZ
            );

        case 5:
            return sample.gyroX;

        case 6:
            return sample.gyroY;

        case 7:
            return sample.gyroZ;

        default:
            return NAN;
    }
}


// ============================================================
// SMALL FIXED-SIZE SORT
//
// 80 values only. Insertion sort avoids dynamic allocation.
// ============================================================

void MPUFeatureExtractor::sortValues(
    float values[MPU_ML_WINDOW_SAMPLES]
) const
{
    for (uint16_t i = 1; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        float key = values[i];
        int16_t j = static_cast<int16_t>(i) - 1;

        while (
            j >= 0 &&
            values[j] > key
        )
        {
            values[j + 1] = values[j];
            j--;
        }

        values[j + 1] = key;
    }
}


// ============================================================
// NUMPY-COMPATIBLE LINEAR QUANTILE
//
// numpy.quantile default:
// position = (N - 1) * q
// linearly interpolate between neighboring ordered values.
// ============================================================

float MPUFeatureExtractor::quantileSorted(
    const float sorted[MPU_ML_WINDOW_SAMPLES],
    float q
) const
{
    if (q <= 0.0f)
    {
        return sorted[0];
    }

    if (q >= 1.0f)
    {
        return sorted[MPU_ML_WINDOW_SAMPLES - 1];
    }

    float position =
        static_cast<float>(MPU_ML_WINDOW_SAMPLES - 1) * q;

    uint16_t lower =
        static_cast<uint16_t>(floorf(position));

    uint16_t upper =
        static_cast<uint16_t>(ceilf(position));

    if (lower == upper)
    {
        return sorted[lower];
    }

    float fraction =
        position - static_cast<float>(lower);

    return
        sorted[lower] +
        fraction * (sorted[upper] - sorted[lower]);
}


// ============================================================
// PEARSON CORRELATION
//
// Matches numpy.corrcoef for finite vectors. The training
// extractor returns 0 when either vector is effectively constant.
// ============================================================

float MPUFeatureExtractor::correlation(
    const float *left,
    const float *right,
    uint16_t count
) const
{
    if (count < 3)
    {
        return NAN;
    }

    double sumLeft = 0.0;
    double sumRight = 0.0;

    for (uint16_t i = 0; i < count; i++)
    {
        if (!isfinite(left[i]) || !isfinite(right[i]))
        {
            return NAN;
        }

        sumLeft += static_cast<double>(left[i]);
        sumRight += static_cast<double>(right[i]);
    }

    const double meanLeft =
        sumLeft / static_cast<double>(count);

    const double meanRight =
        sumRight / static_cast<double>(count);

    double numerator = 0.0;
    double leftSquared = 0.0;
    double rightSquared = 0.0;

    for (uint16_t i = 0; i < count; i++)
    {
        double dl =
            static_cast<double>(left[i]) - meanLeft;

        double dr =
            static_cast<double>(right[i]) - meanRight;

        numerator += dl * dr;
        leftSquared += dl * dl;
        rightSquared += dr * dr;
    }

    if (
        leftSquared < 1.0e-24 ||
        rightSquared < 1.0e-24
    )
    {
        return 0.0f;
    }

    return static_cast<float>(
        numerator /
        sqrt(leftSquared * rightSquared)
    );
}


// ============================================================
// PER-SIGNAL FEATURES
//
// Matches Step 5.6.1 Python feature engineering:
// - sample std/variance (ddof=1)
// - mean square "energy"
// - numpy linear quantiles
// - median absolute deviation
// - least-squares slope on x=i/80 seconds
// - lag-1 Pearson autocorrelation
// - acceleration jerk = diff * 80 Hz
// ============================================================

bool MPUFeatureExtractor::computeStats(
    const MPUModelSample samples[MPU_ML_WINDOW_SAMPLES],
    uint8_t signalIndex,
    bool computeJerk,
    SignalStats &output
) const
{
    static float values[MPU_ML_WINDOW_SAMPLES];
    static float sorted[MPU_ML_WINDOW_SAMPLES];

    double sum = 0.0;
    double sumSquares = 0.0;

    for (uint16_t i = 0; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        float value =
            signalValue(samples[i], signalIndex);

        if (!isfinite(value))
        {
            return false;
        }

        values[i] = value;
        sorted[i] = value;

        sum += static_cast<double>(value);
        sumSquares +=
            static_cast<double>(value) *
            static_cast<double>(value);
    }

    sortValues(sorted);

    output.mean =
        static_cast<float>(
            sum / static_cast<double>(MPU_ML_WINDOW_SAMPLES)
        );

    output.energy =
        static_cast<float>(
            sumSquares /
            static_cast<double>(MPU_ML_WINDOW_SAMPLES)
        );

    output.rms =
        sqrtf(output.energy);

    output.first = values[0];
    output.last =
        values[MPU_ML_WINDOW_SAMPLES - 1];

    output.delta =
        output.last - output.first;

    output.min = sorted[0];
    output.max =
        sorted[MPU_ML_WINDOW_SAMPLES - 1];

    output.range =
        output.max - output.min;

    output.q05 =
        quantileSorted(sorted, 0.05f);

    output.q25 =
        quantileSorted(sorted, 0.25f);

    output.median =
        quantileSorted(sorted, 0.50f);

    output.q75 =
        quantileSorted(sorted, 0.75f);

    output.q95 =
        quantileSorted(sorted, 0.95f);

    output.iqr =
        output.q75 - output.q25;

    static float absoluteDeviations[MPU_ML_WINDOW_SAMPLES];

    for (uint16_t i = 0; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        absoluteDeviations[i] =
            fabsf(values[i] - output.median);
    }

    sortValues(absoluteDeviations);

    output.mad =
        quantileSorted(
            absoluteDeviations,
            0.50f
        );

    double varianceAccumulator = 0.0;

    for (uint16_t i = 0; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        double difference =
            static_cast<double>(values[i]) -
            static_cast<double>(output.mean);

        varianceAccumulator +=
            difference * difference;
    }

    output.variance =
        static_cast<float>(
            varianceAccumulator /
            static_cast<double>(
                MPU_ML_WINDOW_SAMPLES - 1
            )
        );

    output.std =
        sqrtf(output.variance);

    double sumAbsChange = 0.0;
    float maxAbsChange = 0.0f;

    for (uint16_t i = 1; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        float change =
            fabsf(values[i] - values[i - 1]);

        sumAbsChange +=
            static_cast<double>(change);

        if (change > maxAbsChange)
        {
            maxAbsChange = change;
        }
    }

    output.meanAbsChange =
        static_cast<float>(
            sumAbsChange /
            static_cast<double>(
                MPU_ML_WINDOW_SAMPLES - 1
            )
        );

    output.maxAbsChange =
        maxAbsChange;

    // Least-squares slope. x_i = i / 80 seconds.
    // Using double accumulation keeps parity with numpy.polyfit.
    double xMean = 0.0;
    double yMean = 0.0;

    for (uint16_t i = 0; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        xMean +=
            static_cast<double>(i) / 80.0;

        yMean +=
            static_cast<double>(values[i]);
    }

    xMean /= static_cast<double>(MPU_ML_WINDOW_SAMPLES);
    yMean /= static_cast<double>(MPU_ML_WINDOW_SAMPLES);

    double slopeNumerator = 0.0;
    double slopeDenominator = 0.0;

    for (uint16_t i = 0; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        double x =
            static_cast<double>(i) / 80.0;

        double dx = x - xMean;
        double dy =
            static_cast<double>(values[i]) - yMean;

        slopeNumerator += dx * dy;
        slopeDenominator += dx * dx;
    }

    output.slope =
        slopeDenominator > 0.0
            ? static_cast<float>(
                slopeNumerator / slopeDenominator
            )
            : 0.0f;

    output.lag1Autocorrelation =
        correlation(
            values,
            values + 1,
            MPU_ML_WINDOW_SAMPLES - 1
        );

    if (computeJerk)
    {
        double jerkAbsSum = 0.0;
        double jerkSquareSum = 0.0;
        double jerkSum = 0.0;
        float jerkMaxAbs = 0.0f;
        static float jerkValues[MPU_ML_WINDOW_SAMPLES - 1];

        for (uint16_t i = 1; i < MPU_ML_WINDOW_SAMPLES; i++)
        {
            float jerk =
                (values[i] - values[i - 1]) * 80.0f;

            jerkValues[i - 1] = jerk;

            float absoluteJerk =
                fabsf(jerk);

            jerkAbsSum +=
                static_cast<double>(absoluteJerk);

            jerkSquareSum +=
                static_cast<double>(jerk) *
                static_cast<double>(jerk);

            jerkSum +=
                static_cast<double>(jerk);

            if (absoluteJerk > jerkMaxAbs)
            {
                jerkMaxAbs = absoluteJerk;
            }
        }

        constexpr uint16_t JERK_COUNT =
            MPU_ML_WINDOW_SAMPLES - 1;

        output.jerkMeanAbs =
            static_cast<float>(
                jerkAbsSum /
                static_cast<double>(JERK_COUNT)
            );

        output.jerkMaxAbs =
            jerkMaxAbs;

        output.jerkRms =
            static_cast<float>(
                sqrt(
                    jerkSquareSum /
                    static_cast<double>(JERK_COUNT)
                )
            );

        double jerkMean =
            jerkSum /
            static_cast<double>(JERK_COUNT);

        double jerkVarianceAccumulator = 0.0;

        for (uint16_t i = 0; i < JERK_COUNT; i++)
        {
            double difference =
                static_cast<double>(jerkValues[i]) -
                jerkMean;

            jerkVarianceAccumulator +=
                difference * difference;
        }

        output.jerkStd =
            JERK_COUNT > 1
                ? static_cast<float>(
                    sqrt(
                        jerkVarianceAccumulator /
                        static_cast<double>(JERK_COUNT - 1)
                    )
                )
                : 0.0f;
    }

    return true;
}


// ============================================================
// COMPLETE 198-FEATURE VECTOR
// ============================================================

bool MPUFeatureExtractor::extract(
    const MPUModelSample samples[MPU_ML_WINDOW_SAMPLES],
    MPUFeatureVector &output
) const
{
    static SignalStats stats[8];

    for (uint8_t signal = 0; signal < 8; signal++)
    {
        bool accelerationSignal =
            signal <= 3;

        if (
            !computeStats(
                samples,
                signal,
                accelerationSignal,
                stats[signal]
            )
        )
        {
            return false;
        }
    }

    static float ax[MPU_ML_WINDOW_SAMPLES];
    static float ay[MPU_ML_WINDOW_SAMPLES];
    static float az[MPU_ML_WINDOW_SAMPLES];
    static float gx[MPU_ML_WINDOW_SAMPLES];
    static float gy[MPU_ML_WINDOW_SAMPLES];
    static float gz[MPU_ML_WINDOW_SAMPLES];

    for (uint16_t i = 0; i < MPU_ML_WINDOW_SAMPLES; i++)
    {
        ax[i] = samples[i].accelX;
        ay[i] = samples[i].accelY;
        az[i] = samples[i].accelZ;
        gx[i] = samples[i].gyroX;
        gy[i] = samples[i].gyroY;
        gz[i] = samples[i].gyroZ;
    }

    const float accelXYCorr =
        correlation(ax, ay, MPU_ML_WINDOW_SAMPLES);

    const float accelXZCorr =
        correlation(ax, az, MPU_ML_WINDOW_SAMPLES);

    const float accelYZCorr =
        correlation(ay, az, MPU_ML_WINDOW_SAMPLES);

    const float gyroXYCorr =
        correlation(gx, gy, MPU_ML_WINDOW_SAMPLES);

    const float gyroXZCorr =
        correlation(gx, gz, MPU_ML_WINDOW_SAMPLES);

    const float gyroYZCorr =
        correlation(gy, gz, MPU_ML_WINDOW_SAMPLES);

    output.values[0] = stats[0].delta; // accel_magnitude_delta
    output.values[1] = stats[0].energy; // accel_magnitude_energy
    output.values[2] = stats[0].first; // accel_magnitude_first
    output.values[3] = stats[0].iqr; // accel_magnitude_iqr
    output.values[4] = stats[0].jerkMaxAbs; // accel_magnitude_jerk_max_abs
    output.values[5] = stats[0].jerkMeanAbs; // accel_magnitude_jerk_mean_abs
    output.values[6] = stats[0].jerkRms; // accel_magnitude_jerk_rms
    output.values[7] = stats[0].jerkStd; // accel_magnitude_jerk_std
    output.values[8] = stats[0].lag1Autocorrelation; // accel_magnitude_lag1_autocorrelation
    output.values[9] = stats[0].last; // accel_magnitude_last
    output.values[10] = stats[0].mad; // accel_magnitude_mad
    output.values[11] = stats[0].max; // accel_magnitude_max
    output.values[12] = stats[0].maxAbsChange; // accel_magnitude_max_abs_change
    output.values[13] = stats[0].mean; // accel_magnitude_mean
    output.values[14] = stats[0].meanAbsChange; // accel_magnitude_mean_abs_change
    output.values[15] = stats[0].median; // accel_magnitude_median
    output.values[16] = stats[0].min; // accel_magnitude_min
    output.values[17] = stats[0].q05; // accel_magnitude_q05
    output.values[18] = stats[0].q25; // accel_magnitude_q25
    output.values[19] = stats[0].q75; // accel_magnitude_q75
    output.values[20] = stats[0].q95; // accel_magnitude_q95
    output.values[21] = stats[0].range; // accel_magnitude_range
    output.values[22] = stats[0].rms; // accel_magnitude_rms
    output.values[23] = stats[0].slope; // accel_magnitude_slope
    output.values[24] = stats[0].std; // accel_magnitude_std
    output.values[25] = stats[0].variance; // accel_magnitude_variance
    output.values[26] = stats[1].delta; // accel_x_delta
    output.values[27] = stats[1].energy; // accel_x_energy
    output.values[28] = stats[1].first; // accel_x_first
    output.values[29] = stats[1].iqr; // accel_x_iqr
    output.values[30] = stats[1].jerkMaxAbs; // accel_x_jerk_max_abs
    output.values[31] = stats[1].jerkMeanAbs; // accel_x_jerk_mean_abs
    output.values[32] = stats[1].jerkRms; // accel_x_jerk_rms
    output.values[33] = stats[1].jerkStd; // accel_x_jerk_std
    output.values[34] = stats[1].lag1Autocorrelation; // accel_x_lag1_autocorrelation
    output.values[35] = stats[1].last; // accel_x_last
    output.values[36] = stats[1].mad; // accel_x_mad
    output.values[37] = stats[1].max; // accel_x_max
    output.values[38] = stats[1].maxAbsChange; // accel_x_max_abs_change
    output.values[39] = stats[1].mean; // accel_x_mean
    output.values[40] = stats[1].meanAbsChange; // accel_x_mean_abs_change
    output.values[41] = stats[1].median; // accel_x_median
    output.values[42] = stats[1].min; // accel_x_min
    output.values[43] = stats[1].q05; // accel_x_q05
    output.values[44] = stats[1].q25; // accel_x_q25
    output.values[45] = stats[1].q75; // accel_x_q75
    output.values[46] = stats[1].q95; // accel_x_q95
    output.values[47] = stats[1].range; // accel_x_range
    output.values[48] = stats[1].rms; // accel_x_rms
    output.values[49] = stats[1].slope; // accel_x_slope
    output.values[50] = stats[1].std; // accel_x_std
    output.values[51] = stats[1].variance; // accel_x_variance
    output.values[52] = accelXYCorr; // accel_xy_corr
    output.values[53] = accelXZCorr; // accel_xz_corr
    output.values[54] = stats[2].delta; // accel_y_delta
    output.values[55] = stats[2].energy; // accel_y_energy
    output.values[56] = stats[2].first; // accel_y_first
    output.values[57] = stats[2].iqr; // accel_y_iqr
    output.values[58] = stats[2].jerkMaxAbs; // accel_y_jerk_max_abs
    output.values[59] = stats[2].jerkMeanAbs; // accel_y_jerk_mean_abs
    output.values[60] = stats[2].jerkRms; // accel_y_jerk_rms
    output.values[61] = stats[2].jerkStd; // accel_y_jerk_std
    output.values[62] = stats[2].lag1Autocorrelation; // accel_y_lag1_autocorrelation
    output.values[63] = stats[2].last; // accel_y_last
    output.values[64] = stats[2].mad; // accel_y_mad
    output.values[65] = stats[2].max; // accel_y_max
    output.values[66] = stats[2].maxAbsChange; // accel_y_max_abs_change
    output.values[67] = stats[2].mean; // accel_y_mean
    output.values[68] = stats[2].meanAbsChange; // accel_y_mean_abs_change
    output.values[69] = stats[2].median; // accel_y_median
    output.values[70] = stats[2].min; // accel_y_min
    output.values[71] = stats[2].q05; // accel_y_q05
    output.values[72] = stats[2].q25; // accel_y_q25
    output.values[73] = stats[2].q75; // accel_y_q75
    output.values[74] = stats[2].q95; // accel_y_q95
    output.values[75] = stats[2].range; // accel_y_range
    output.values[76] = stats[2].rms; // accel_y_rms
    output.values[77] = stats[2].slope; // accel_y_slope
    output.values[78] = stats[2].std; // accel_y_std
    output.values[79] = stats[2].variance; // accel_y_variance
    output.values[80] = accelYZCorr; // accel_yz_corr
    output.values[81] = stats[3].delta; // accel_z_delta
    output.values[82] = stats[3].energy; // accel_z_energy
    output.values[83] = stats[3].first; // accel_z_first
    output.values[84] = stats[3].iqr; // accel_z_iqr
    output.values[85] = stats[3].jerkMaxAbs; // accel_z_jerk_max_abs
    output.values[86] = stats[3].jerkMeanAbs; // accel_z_jerk_mean_abs
    output.values[87] = stats[3].jerkRms; // accel_z_jerk_rms
    output.values[88] = stats[3].jerkStd; // accel_z_jerk_std
    output.values[89] = stats[3].lag1Autocorrelation; // accel_z_lag1_autocorrelation
    output.values[90] = stats[3].last; // accel_z_last
    output.values[91] = stats[3].mad; // accel_z_mad
    output.values[92] = stats[3].max; // accel_z_max
    output.values[93] = stats[3].maxAbsChange; // accel_z_max_abs_change
    output.values[94] = stats[3].mean; // accel_z_mean
    output.values[95] = stats[3].meanAbsChange; // accel_z_mean_abs_change
    output.values[96] = stats[3].median; // accel_z_median
    output.values[97] = stats[3].min; // accel_z_min
    output.values[98] = stats[3].q05; // accel_z_q05
    output.values[99] = stats[3].q25; // accel_z_q25
    output.values[100] = stats[3].q75; // accel_z_q75
    output.values[101] = stats[3].q95; // accel_z_q95
    output.values[102] = stats[3].range; // accel_z_range
    output.values[103] = stats[3].rms; // accel_z_rms
    output.values[104] = stats[3].slope; // accel_z_slope
    output.values[105] = stats[3].std; // accel_z_std
    output.values[106] = stats[3].variance; // accel_z_variance
    output.values[107] = stats[4].delta; // gyro_magnitude_delta
    output.values[108] = stats[4].energy; // gyro_magnitude_energy
    output.values[109] = stats[4].first; // gyro_magnitude_first
    output.values[110] = stats[4].iqr; // gyro_magnitude_iqr
    output.values[111] = stats[4].lag1Autocorrelation; // gyro_magnitude_lag1_autocorrelation
    output.values[112] = stats[4].last; // gyro_magnitude_last
    output.values[113] = stats[4].mad; // gyro_magnitude_mad
    output.values[114] = stats[4].max; // gyro_magnitude_max
    output.values[115] = stats[4].maxAbsChange; // gyro_magnitude_max_abs_change
    output.values[116] = stats[4].mean; // gyro_magnitude_mean
    output.values[117] = stats[4].meanAbsChange; // gyro_magnitude_mean_abs_change
    output.values[118] = stats[4].median; // gyro_magnitude_median
    output.values[119] = stats[4].min; // gyro_magnitude_min
    output.values[120] = stats[4].q05; // gyro_magnitude_q05
    output.values[121] = stats[4].q25; // gyro_magnitude_q25
    output.values[122] = stats[4].q75; // gyro_magnitude_q75
    output.values[123] = stats[4].q95; // gyro_magnitude_q95
    output.values[124] = stats[4].range; // gyro_magnitude_range
    output.values[125] = stats[4].rms; // gyro_magnitude_rms
    output.values[126] = stats[4].slope; // gyro_magnitude_slope
    output.values[127] = stats[4].std; // gyro_magnitude_std
    output.values[128] = stats[4].variance; // gyro_magnitude_variance
    output.values[129] = stats[5].delta; // gyro_x_delta
    output.values[130] = stats[5].energy; // gyro_x_energy
    output.values[131] = stats[5].first; // gyro_x_first
    output.values[132] = stats[5].iqr; // gyro_x_iqr
    output.values[133] = stats[5].lag1Autocorrelation; // gyro_x_lag1_autocorrelation
    output.values[134] = stats[5].last; // gyro_x_last
    output.values[135] = stats[5].mad; // gyro_x_mad
    output.values[136] = stats[5].max; // gyro_x_max
    output.values[137] = stats[5].maxAbsChange; // gyro_x_max_abs_change
    output.values[138] = stats[5].mean; // gyro_x_mean
    output.values[139] = stats[5].meanAbsChange; // gyro_x_mean_abs_change
    output.values[140] = stats[5].median; // gyro_x_median
    output.values[141] = stats[5].min; // gyro_x_min
    output.values[142] = stats[5].q05; // gyro_x_q05
    output.values[143] = stats[5].q25; // gyro_x_q25
    output.values[144] = stats[5].q75; // gyro_x_q75
    output.values[145] = stats[5].q95; // gyro_x_q95
    output.values[146] = stats[5].range; // gyro_x_range
    output.values[147] = stats[5].rms; // gyro_x_rms
    output.values[148] = stats[5].slope; // gyro_x_slope
    output.values[149] = stats[5].std; // gyro_x_std
    output.values[150] = stats[5].variance; // gyro_x_variance
    output.values[151] = gyroXYCorr; // gyro_xy_corr
    output.values[152] = gyroXZCorr; // gyro_xz_corr
    output.values[153] = stats[6].delta; // gyro_y_delta
    output.values[154] = stats[6].energy; // gyro_y_energy
    output.values[155] = stats[6].first; // gyro_y_first
    output.values[156] = stats[6].iqr; // gyro_y_iqr
    output.values[157] = stats[6].lag1Autocorrelation; // gyro_y_lag1_autocorrelation
    output.values[158] = stats[6].last; // gyro_y_last
    output.values[159] = stats[6].mad; // gyro_y_mad
    output.values[160] = stats[6].max; // gyro_y_max
    output.values[161] = stats[6].maxAbsChange; // gyro_y_max_abs_change
    output.values[162] = stats[6].mean; // gyro_y_mean
    output.values[163] = stats[6].meanAbsChange; // gyro_y_mean_abs_change
    output.values[164] = stats[6].median; // gyro_y_median
    output.values[165] = stats[6].min; // gyro_y_min
    output.values[166] = stats[6].q05; // gyro_y_q05
    output.values[167] = stats[6].q25; // gyro_y_q25
    output.values[168] = stats[6].q75; // gyro_y_q75
    output.values[169] = stats[6].q95; // gyro_y_q95
    output.values[170] = stats[6].range; // gyro_y_range
    output.values[171] = stats[6].rms; // gyro_y_rms
    output.values[172] = stats[6].slope; // gyro_y_slope
    output.values[173] = stats[6].std; // gyro_y_std
    output.values[174] = stats[6].variance; // gyro_y_variance
    output.values[175] = gyroYZCorr; // gyro_yz_corr
    output.values[176] = stats[7].delta; // gyro_z_delta
    output.values[177] = stats[7].energy; // gyro_z_energy
    output.values[178] = stats[7].first; // gyro_z_first
    output.values[179] = stats[7].iqr; // gyro_z_iqr
    output.values[180] = stats[7].lag1Autocorrelation; // gyro_z_lag1_autocorrelation
    output.values[181] = stats[7].last; // gyro_z_last
    output.values[182] = stats[7].mad; // gyro_z_mad
    output.values[183] = stats[7].max; // gyro_z_max
    output.values[184] = stats[7].maxAbsChange; // gyro_z_max_abs_change
    output.values[185] = stats[7].mean; // gyro_z_mean
    output.values[186] = stats[7].meanAbsChange; // gyro_z_mean_abs_change
    output.values[187] = stats[7].median; // gyro_z_median
    output.values[188] = stats[7].min; // gyro_z_min
    output.values[189] = stats[7].q05; // gyro_z_q05
    output.values[190] = stats[7].q25; // gyro_z_q25
    output.values[191] = stats[7].q75; // gyro_z_q75
    output.values[192] = stats[7].q95; // gyro_z_q95
    output.values[193] = stats[7].range; // gyro_z_range
    output.values[194] = stats[7].rms; // gyro_z_rms
    output.values[195] = stats[7].slope; // gyro_z_slope
    output.values[196] = stats[7].std; // gyro_z_std
    output.values[197] = stats[7].variance; // gyro_z_variance

    for (uint16_t i = 0; i < MPU_MODEL_FEATURE_COUNT; i++)
    {
        if (!isfinite(output.values[i]))
        {
            return false;
        }
    }

    return true;
}
