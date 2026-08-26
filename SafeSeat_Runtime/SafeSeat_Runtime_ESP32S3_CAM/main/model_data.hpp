#pragma once
#include <cstddef>
#include <cstdint>
namespace safeseat_model {
static constexpr int FEATURE_COUNT = 7;
extern const float SCALER_MEAN[FEATURE_COUNT];
extern const float SCALER_SCALE[FEATURE_COUNT];
extern const float OCSVM_SUPPORT_VECTORS[];
extern const float OCSVM_DUAL_COEF[];
extern const int OCSVM_SUPPORT_COUNT;
extern const float OCSVM_GAMMA;
extern const float OCSVM_INTERCEPT;
extern const float OCSVM_THRESHOLD;
struct IFTree { const int16_t *children_left; const int16_t *children_right; const int8_t *feature; const float *threshold; const uint16_t *n_node_samples; uint16_t node_count; };
extern const IFTree IF_TREES[];
extern const int IF_TREE_COUNT;
extern const int IF_MAX_SAMPLES;
extern const float IF_OFFSET;
extern const float IF_THRESHOLD;
}
