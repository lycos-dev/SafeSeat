#pragma once

#include <Arduino.h>
#include "MLXModelData.h"

// ============================================================
// STEP 5.4.3 - MLX LIVE FEATURE DIAGNOSTICS
//
// Diagnostic-only helper. It does NOT alter feature extraction,
// preprocessing, model inference, ModelEvidence, or Fusion.
//
// Each live 38-feature vector is compared against the empirical
// WESAD training distribution used for Step 5.4.1 retraining.
// ============================================================

class MLXFeatureDiagnostics
{
public:
    static constexpr uint8_t DETAILED_WINDOW_LIMIT = 4;

    static void print(
        uint32_t windowNumber,
        const float features[MLX_MODEL_FEATURE_COUNT],
        float isolationForestDecision,
        float oneClassSVMDecision
    );
};
