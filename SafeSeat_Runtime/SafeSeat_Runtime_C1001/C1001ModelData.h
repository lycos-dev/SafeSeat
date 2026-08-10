#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED FROM THE TRAINED SAFESEAT C1001 MODELS.
//
// Source artifacts:
// - models/C1001/c1001_preprocessor.joblib
// - models/C1001/isolation_forest.joblib
// - models/C1001/one_class_svm.joblib
// - models/C1001/feature_columns.json
//
// Parameters are stored as float32 for embedded inference.
// Validation against the original sklearn models is included
// in C1001_EMBEDDED_MODEL_VALIDATION.json.

constexpr uint16_t C1001_MODEL_FEATURE_COUNT = 64;

constexpr uint16_t C1001_IF_TREE_COUNT = 300;
constexpr uint32_t C1001_IF_NODE_COUNT = 33900;
constexpr uint16_t C1001_IF_MAX_SAMPLES = 256;

constexpr uint16_t C1001_OCSVM_SUPPORT_VECTOR_COUNT =
    85;

extern const float C1001_IMPUTER_MEDIAN[C1001_MODEL_FEATURE_COUNT];
extern const float C1001_SCALER_CENTER[C1001_MODEL_FEATURE_COUNT];
extern const float C1001_SCALER_SCALE[C1001_MODEL_FEATURE_COUNT];

extern const uint32_t C1001_IF_TREE_OFFSETS[C1001_IF_TREE_COUNT + 1];
extern const int16_t C1001_IF_CHILDREN_LEFT[C1001_IF_NODE_COUNT];
extern const int16_t C1001_IF_CHILDREN_RIGHT[C1001_IF_NODE_COUNT];
extern const int8_t C1001_IF_FEATURE[C1001_IF_NODE_COUNT];
extern const float C1001_IF_THRESHOLD[C1001_IF_NODE_COUNT];
extern const uint16_t C1001_IF_N_NODE_SAMPLES[C1001_IF_NODE_COUNT];

extern const float C1001_IF_OFFSET;

extern const float C1001_OCSVM_SUPPORT_VECTORS[
    C1001_OCSVM_SUPPORT_VECTOR_COUNT * C1001_MODEL_FEATURE_COUNT
];

extern const float C1001_OCSVM_DUAL_COEF[
    C1001_OCSVM_SUPPORT_VECTOR_COUNT
];

extern const float C1001_OCSVM_INTERCEPT;
extern const float C1001_OCSVM_GAMMA;
