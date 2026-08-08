#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED FROM THE TRAINED SAFESEAT MLX90614 MODELS.
//
// Source artifacts:
// - models/MLX90614/mlx_preprocessor.joblib
// - models/MLX90614/isolation_forest.joblib
// - models/MLX90614/one_class_svm.joblib
// - models/MLX90614/feature_columns.json
//
// IMPORTANT:
// The WESAD model was trained on wearable skin-temperature
// patterns. Runtime ML uses the MLX90614 OBJECT-temperature
// stream only. Ambient temperature remains separate context
// for Fusion and is not fed into these models.
//
// Parameters are stored as float32 for embedded inference.

constexpr uint16_t MLX_MODEL_FEATURE_COUNT = 44;

constexpr uint16_t MLX_IF_TREE_COUNT = 300;
constexpr uint32_t MLX_IF_NODE_COUNT = 35818UL;
constexpr uint16_t MLX_IF_MAX_SAMPLES = 256;

constexpr uint16_t MLX_OCSVM_SUPPORT_VECTOR_COUNT =
    296;

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
