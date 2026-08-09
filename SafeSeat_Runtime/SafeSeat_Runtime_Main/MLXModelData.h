#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED FROM THE RETRAINED SAFESEAT MLX90614 MODELS.
// Step 5.4.1 / 5.4.2
//
// Deployment feature reference:
//   qualified object-temperature window
//   -> limited interpolation
//   -> subtract 30-second window median
//   -> 38 relative/temporal features
//   -> imputer + RobustScaler
//   -> Isolation Forest + One-Class SVM
//
// Ambient temperature and Object-Ambient are NOT model features.
// They remain deployment context / warm-target qualification.
//
// Parameters are stored as float32 for embedded inference.

constexpr uint16_t MLX_MODEL_FEATURE_COUNT = 38;

constexpr uint16_t MLX_IF_TREE_COUNT = 300;
constexpr uint32_t MLX_IF_NODE_COUNT = 30502UL;
constexpr uint16_t MLX_IF_MAX_SAMPLES = 256;

constexpr uint16_t MLX_OCSVM_SUPPORT_VECTOR_COUNT =
    297;

extern const float MLX_IMPUTER_MEDIAN[MLX_MODEL_FEATURE_COUNT];
extern const float MLX_SCALER_CENTER[MLX_MODEL_FEATURE_COUNT];
extern const float MLX_SCALER_SCALE[MLX_MODEL_FEATURE_COUNT];

extern const uint32_t MLX_IF_TREE_OFFSETS[MLX_IF_TREE_COUNT + 1];
extern const int16_t MLX_IF_CHILDREN_LEFT[MLX_IF_NODE_COUNT];
extern const int16_t MLX_IF_CHILDREN_RIGHT[MLX_IF_NODE_COUNT];
extern const int8_t MLX_IF_FEATURE[MLX_IF_NODE_COUNT];
extern const float MLX_IF_THRESHOLD[MLX_IF_NODE_COUNT];
extern const uint16_t MLX_IF_N_NODE_SAMPLES[MLX_IF_NODE_COUNT];

extern const float MLX_IF_OFFSET;

extern const float MLX_OCSVM_SUPPORT_VECTORS[
    MLX_OCSVM_SUPPORT_VECTOR_COUNT * MLX_MODEL_FEATURE_COUNT
];

extern const float MLX_OCSVM_DUAL_COEF[
    MLX_OCSVM_SUPPORT_VECTOR_COUNT
];

extern const float MLX_OCSVM_INTERCEPT;
extern const float MLX_OCSVM_GAMMA;
