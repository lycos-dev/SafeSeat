#pragma once

#include <Arduino.h>
#include <stdint.h>

// AUTO-GENERATED: SafeSeat C1001 2026 60 GHz mmWave candidate.
//
// Source:
//   models/C1001_MMwave_2026_candidate/
//
// Offline normal-reference validation:
//   IF:    19/19 held-out healthy windows NORMAL
//   OCSVM: 19/19 held-out healthy windows NORMAL
//
// Tuned OCSVM:
//   nu = 0.01
//   gamma = 0.0001
//
// IMPORTANT:
// This candidate is not promoted to canonical models/C1001 until
// physical C1001 live-runtime validation passes.

constexpr uint16_t C1001_MODEL_FEATURE_COUNT = 64;

constexpr uint16_t C1001_IF_TREE_COUNT = 300;
constexpr uint32_t C1001_IF_NODE_COUNT = 19118;
constexpr uint16_t C1001_IF_MAX_SAMPLES = 75;

constexpr uint16_t C1001_OCSVM_SUPPORT_VECTOR_COUNT =
    4;

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
