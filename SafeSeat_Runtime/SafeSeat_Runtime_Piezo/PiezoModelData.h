#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED FROM SAFESEAT STEP 5.7 CANONICAL PIEZO MODELS.
// Source: WESAD Respiban surrogate, runtime-aligned at 25 Hz.
// Decision convention: >= 0 normal/inlier, < 0 anomaly.

constexpr uint16_t PIEZO_MODEL_FEATURE_COUNT = 16;

constexpr uint16_t PIEZO_IF_TREE_COUNT = 300;
constexpr uint32_t PIEZO_IF_NODE_COUNT = 34558;
constexpr uint16_t PIEZO_IF_MAX_SAMPLES = 256;

constexpr uint16_t PIEZO_OCSVM_SUPPORT_VECTOR_COUNT = 34;

extern const float PIEZO_SCALER_MEAN[PIEZO_MODEL_FEATURE_COUNT];
extern const float PIEZO_SCALER_SCALE[PIEZO_MODEL_FEATURE_COUNT];

extern const uint32_t PIEZO_IF_TREE_OFFSETS[PIEZO_IF_TREE_COUNT + 1];
extern const int16_t PIEZO_IF_CHILDREN_LEFT[PIEZO_IF_NODE_COUNT];
extern const int16_t PIEZO_IF_CHILDREN_RIGHT[PIEZO_IF_NODE_COUNT];
extern const int8_t PIEZO_IF_FEATURE[PIEZO_IF_NODE_COUNT];
extern const float PIEZO_IF_THRESHOLD[PIEZO_IF_NODE_COUNT];
extern const uint16_t PIEZO_IF_N_NODE_SAMPLES[PIEZO_IF_NODE_COUNT];

extern const float PIEZO_IF_OFFSET;

extern const float PIEZO_OCSVM_SUPPORT_VECTORS[
    PIEZO_OCSVM_SUPPORT_VECTOR_COUNT * PIEZO_MODEL_FEATURE_COUNT
];
extern const float PIEZO_OCSVM_DUAL_COEF[PIEZO_OCSVM_SUPPORT_VECTOR_COUNT];
extern const float PIEZO_OCSVM_INTERCEPT;
extern const float PIEZO_OCSVM_GAMMA;
