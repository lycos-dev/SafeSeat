#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED FROM SAFESEAT MPU6050 STEP 5.6.1 CANONICAL MODELS.
//
// Training/runtime contract:
// - Road Data calibrated accelerometer: m/s^2 with stationary offset removed
// - Road Data calibrated gyroscope: rad/s
// - runtime-aligned sampling: 80 Hz
// - 1.0 s window: 80 samples
// - stride: 40 samples
// - production features: 198
//
// Deployment note:
// The SafeSeat MPU6050 provides total acceleration in g and gyro in deg/s.
// MPUML performs a stationary startup offset calibration, converts accel to
// m/s^2 and gyro to rad/s, then feeds the exact feature/inference pipeline.

constexpr uint16_t MPU_MODEL_FEATURE_COUNT = 198;
constexpr uint16_t MPU_ML_WINDOW_SAMPLES = 80;
constexpr uint16_t MPU_ML_STRIDE_SAMPLES = 40;
constexpr uint16_t MPU_ML_BASELINE_SAMPLES = 80;

constexpr uint16_t MPU_IF_TREE_COUNT = 400;
constexpr uint32_t MPU_IF_NODE_COUNT = 31242UL;
constexpr uint16_t MPU_IF_MAX_SAMPLES = 256;

constexpr uint16_t MPU_OCSVM_SUPPORT_VECTOR_COUNT = 105;

extern const float MPU_IMPUTER_MEDIAN[MPU_MODEL_FEATURE_COUNT];
extern const float MPU_SCALER_CENTER[MPU_MODEL_FEATURE_COUNT];
extern const float MPU_SCALER_SCALE[MPU_MODEL_FEATURE_COUNT];

extern const uint32_t MPU_IF_TREE_OFFSETS[MPU_IF_TREE_COUNT + 1];
extern const int16_t MPU_IF_CHILDREN_LEFT[MPU_IF_NODE_COUNT];
extern const int16_t MPU_IF_CHILDREN_RIGHT[MPU_IF_NODE_COUNT];
extern const int16_t MPU_IF_FEATURE[MPU_IF_NODE_COUNT];
extern const float MPU_IF_THRESHOLD[MPU_IF_NODE_COUNT];
extern const uint16_t MPU_IF_N_NODE_SAMPLES[MPU_IF_NODE_COUNT];
extern const float MPU_IF_OFFSET;

extern const float MPU_OCSVM_SUPPORT_VECTORS[
    MPU_OCSVM_SUPPORT_VECTOR_COUNT * MPU_MODEL_FEATURE_COUNT
];
extern const float MPU_OCSVM_DUAL_COEF[MPU_OCSVM_SUPPORT_VECTOR_COUNT];
extern const float MPU_OCSVM_INTERCEPT;
extern const float MPU_OCSVM_GAMMA;
