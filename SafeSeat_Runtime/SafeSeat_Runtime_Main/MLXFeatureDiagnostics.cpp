#include "MLXFeatureDiagnostics.h"

#include <math.h>

namespace
{
static const char *const FEATURE_NAMES[MLX_MODEL_FEATURE_COUNT] = {
    "temp_change_std",
    "temp_cooling_fraction",
    "temp_count",
    "temp_delta",
    "temp_first",
    "temp_iqr",
    "temp_lag1_autocorrelation",
    "temp_lag4_autocorrelation",
    "temp_largest_cooling_step",
    "temp_largest_warming_step",
    "temp_last",
    "temp_linear_residual_std",
    "temp_mad",
    "temp_max",
    "temp_max_abs_change",
    "temp_max_change_per_second",
    "temp_mean",
    "temp_mean_abs_change",
    "temp_mean_change_per_second",
    "temp_min",
    "temp_missing_fraction",
    "temp_negative_change_count",
    "temp_positive_change_count",
    "temp_q05",
    "temp_q10",
    "temp_q25",
    "temp_q75",
    "temp_q90",
    "temp_q95",
    "temp_range",
    "temp_rms",
    "temp_slope_c_per_second",
    "temp_std",
    "temp_unchanged_fraction",
    "temp_variance",
    "temp_warming_fraction",
    "temp_within_0p2c_of_median_fraction",
    "temp_within_0p5c_of_median_fraction"
};

static const float WESAD_P01[MLX_MODEL_FEATURE_COUNT] = {
    0.005207556439f,
    0.02521008403f,
    120.0f,
    -0.24f,
    -0.16f,
    0.0f,
    0.6885413677f,
    -0.2813170299f,
    -0.07f,
    0.02f,
    -0.13f,
    0.008056660939f,
    0.0f,
    0.0f,
    0.02f,
    0.08f,
    -0.02266666667f,
    0.001428571429f,
    -0.008067226891f,
    -0.18115f,
    0.0f,
    3.0f,
    3.0f,
    -0.17f,
    -0.15f,
    -0.10615f,
    0.0f,
    0.0f,
    0.0f,
    0.03f,
    0.009288805114f,
    -0.008401605667f,
    0.008976027915f,
    0.8067226891f,
    8.056918768e-05f,
    0.02521008403f,
    0.9666666667f,
    1.0f
};

static const float WESAD_P05[MLX_MODEL_FEATURE_COUNT] = {
    0.006480882967f,
    0.04201680672f,
    120.0f,
    -0.12f,
    -0.08f,
    0.0f,
    0.7255956956f,
    -0.1277812486f,
    -0.06f,
    0.02f,
    -0.06f,
    0.009778714553f,
    0.0f,
    0.02f,
    0.02f,
    0.08f,
    -0.01133333333f,
    0.002016806723f,
    -0.004033613445f,
    -0.105f,
    0.0f,
    5.0f,
    5.0f,
    -0.1f,
    -0.08f,
    -0.05f,
    0.0f,
    0.0f,
    0.01f,
    0.04f,
    0.01154700538f,
    -0.004022834919f,
    0.01097999199f,
    0.8235294118f,
    0.0001205602241f,
    0.04201680672f,
    1.0f,
    1.0f
};

static const float WESAD_P50[MLX_MODEL_FEATURE_COUNT] = {
    0.009149878019f,
    0.06722689076f,
    120.0f,
    0.0f,
    0.0f,
    0.02f,
    0.8621249286f,
    0.4272050487f,
    -0.04f,
    0.04f,
    0.0f,
    0.01371992917f,
    0.02f,
    0.04f,
    0.04f,
    0.16f,
    0.0f,
    0.003193277311f,
    0.0f,
    -0.04f,
    0.0f,
    8.0f,
    8.0f,
    -0.03f,
    -0.02f,
    -0.02f,
    0.02f,
    0.02f,
    0.03f,
    0.07f,
    0.01879716291f,
    -0.0001377873463f,
    0.01772494484f,
    0.8571428571f,
    0.0003141736695f,
    0.06722689076f,
    1.0f,
    1.0f
};

static const float WESAD_P95[MLX_MODEL_FEATURE_COUNT] = {
    0.01234965129f,
    0.1008403361f,
    120.0f,
    0.14f,
    0.06f,
    0.1f,
    0.9867298025f,
    0.9482768492f,
    -0.02f,
    0.06f,
    0.08f,
    0.0230472243f,
    0.045f,
    0.1f,
    0.06f,
    0.24f,
    0.01167528736f,
    0.004705882353f,
    0.004705882353f,
    -0.02f,
    0.0f,
    12.0f,
    12.0f,
    -0.01f,
    0.0f,
    0.0f,
    0.05f,
    0.08f,
    0.1f,
    0.2f,
    0.06208729747f,
    0.005062907146f,
    0.0603418725f,
    0.9075630252f,
    0.003641142857f,
    0.1008403361f,
    1.0f,
    1.0f
};

static const float WESAD_P99[MLX_MODEL_FEATURE_COUNT] = {
    0.01414136011f,
    0.1176470588f,
    120.0f,
    0.28f,
    0.1223f,
    0.18f,
    0.9948184335f,
    0.9829535635f,
    -0.02f,
    0.07f,
    0.15f,
    0.0386320106f,
    0.08115f,
    0.18f,
    0.07f,
    0.28f,
    0.021f,
    0.005546218487f,
    0.009411764706f,
    0.0f,
    0.0f,
    14.0f,
    15.0f,
    0.0f,
    0.0f,
    0.0f,
    0.1f,
    0.142f,
    0.16115f,
    0.34f,
    0.1085946469f,
    0.01015823877f,
    0.1054194177f,
    0.9243697479f,
    0.0111132549f,
    0.1260504202f,
    1.0f,
    1.0f
};


bool outside(float value, float low, float high)
{
    if (!isfinite(value))
    {
        return true;
    }

    float scale = fmaxf(1.0f, fmaxf(fabsf(low), fabsf(high)));
    float eps = 1.0e-5f * scale;
    return value < low - eps || value > high + eps;
}
}

void MLXFeatureDiagnostics::print(
    uint32_t windowNumber,
    const float features[MLX_MODEL_FEATURE_COUNT],
    float isolationForestDecision,
    float oneClassSVMDecision
)
{
    uint8_t outside95Count = 0;
    uint8_t outside99Count = 0;

    for (uint8_t i = 0; i < MLX_MODEL_FEATURE_COUNT; i++)
    {
        if (outside(features[i], WESAD_P05[i], WESAD_P95[i])) outside95Count++;
        if (outside(features[i], WESAD_P01[i], WESAD_P99[i])) outside99Count++;
    }

    Serial.println();
    Serial.println("============================================================");
    Serial.print("[MLX-DIAG] LIVE FEATURE COMPARISON - WINDOW #");
    Serial.println(windowNumber);
    Serial.println("[MLX-DIAG] Model input: RAW MLX object temperature, then 30-s median-centering.");
    Serial.println("[MLX-DIAG] Dashboard 'Object' remains the separately filtered display/context value.");
    Serial.println("[MLX-DIAG] Reference: WESAD Step 5.4.1 TRAIN feature distribution.");
    Serial.print("[MLX-DIAG] Outside WESAD p05-p95: ");
    Serial.print(outside95Count);
    Serial.print(" / ");
    Serial.println(MLX_MODEL_FEATURE_COUNT);
    Serial.print("[MLX-DIAG] Outside WESAD p01-p99: ");
    Serial.print(outside99Count);
    Serial.print(" / ");
    Serial.println(MLX_MODEL_FEATURE_COUNT);
    Serial.print("[MLX-DIAG] IF decision  : ");
    Serial.println(isolationForestDecision, 6);
    Serial.print("[MLX-DIAG] SVM decision : ");
    Serial.println(oneClassSVMDecision, 6);
    Serial.println();
    Serial.println("# | feature | live | p01 | p50 | p99 | flag");

    for (uint8_t i = 0; i < MLX_MODEL_FEATURE_COUNT; i++)
    {
        bool out99 = outside(features[i], WESAD_P01[i], WESAD_P99[i]);
        bool out95 = outside(features[i], WESAD_P05[i], WESAD_P95[i]);

        Serial.print(i + 1);
        Serial.print(" | ");
        Serial.print(FEATURE_NAMES[i]);
        Serial.print(" | ");
        Serial.print(features[i], 7);
        Serial.print(" | ");
        Serial.print(WESAD_P01[i], 7);
        Serial.print(" | ");
        Serial.print(WESAD_P50[i], 7);
        Serial.print(" | ");
        Serial.print(WESAD_P99[i], 7);
        Serial.print(" | ");

        if (out99) Serial.println("OUTSIDE_P99");
        else if (out95) Serial.println("outside_p95");
        else Serial.println("inside");
    }

    Serial.println("============================================================");
    Serial.println();
}
