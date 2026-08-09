#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED FROM THE TRAINED SAFESEAT FSR STEP 5.5 MODELS.
//
// Source artifacts:
// - models/FSR/fsr_scaler.joblib
// - models/FSR/fsr_isolation_forest.joblib
// - models/FSR/fsr_one_class_svm.joblib
// - features/FSR/feature_manifest.json
//
// The model uses 23 completed FSR frames at approximately 4.5 Hz.
// Every frame is normalized to nine pressure shares before the
// 93 spatial/temporal features are calculated. Absolute ADC/force
// magnitude is intentionally not model input.

constexpr uint16_t FSR_MODEL_FEATURE_COUNT = 93;
constexpr uint16_t FSR_ML_WINDOW_SAMPLES = 23;
constexpr uint16_t FSR_ML_STRIDE_SAMPLES = 5;

constexpr uint16_t FSR_IF_TREE_COUNT = 200;
constexpr uint32_t FSR_IF_NODE_COUNT = 25614;
constexpr uint16_t FSR_IF_MAX_SAMPLES = 256;

constexpr uint16_t FSR_OCSVM_SUPPORT_VECTOR_COUNT = 66;

extern const float FSR_SCALER_MEAN[FSR_MODEL_FEATURE_COUNT];
extern const float FSR_SCALER_SCALE[FSR_MODEL_FEATURE_COUNT];

extern const uint32_t FSR_IF_TREE_OFFSETS[FSR_IF_TREE_COUNT + 1];
extern const int16_t FSR_IF_CHILDREN_LEFT[FSR_IF_NODE_COUNT];
extern const int16_t FSR_IF_CHILDREN_RIGHT[FSR_IF_NODE_COUNT];
extern const int8_t FSR_IF_FEATURE[FSR_IF_NODE_COUNT];
extern const float FSR_IF_THRESHOLD[FSR_IF_NODE_COUNT];
extern const uint16_t FSR_IF_N_NODE_SAMPLES[FSR_IF_NODE_COUNT];
extern const float FSR_IF_OFFSET;

extern const float FSR_OCSVM_SUPPORT_VECTORS[
    FSR_OCSVM_SUPPORT_VECTOR_COUNT * FSR_MODEL_FEATURE_COUNT
];
extern const float FSR_OCSVM_DUAL_COEF[FSR_OCSVM_SUPPORT_VECTOR_COUNT];
extern const float FSR_OCSVM_INTERCEPT;
extern const float FSR_OCSVM_GAMMA;
