#pragma once
#include "dl_detect_define.hpp"
#include "model_data.hpp"

enum class SafeSeatPoseState {
    UNKNOWN = 0,
    NORMAL,
    DEVIATION_WEAK,
    DEVIATION_MODERATE,
    DEVIATION_STRONG,
};

struct SafeSeatPoseResult {
    SafeSeatPoseState state = SafeSeatPoseState::UNKNOWN;
    bool valid_pose = false;
    bool if_anomaly = false;
    bool ocsvm_anomaly = false;
    bool fallback_used = false;
    float if_score = 0.0f;
    float ocsvm_score = 0.0f;
    float fallback_shoulder_ratio = 0.0f;
    float fallback_box_ratio = 0.0f;
    float features[safeseat_model::FEATURE_COUNT] = {};
    float compensated_z[safeseat_model::FEATURE_COUNT] = {};
};

const char *safeseat_pose_state_name(SafeSeatPoseState state);

bool safeseat_extract_pose_features(
    const dl::detect::result_t &pose,
    int image_width,
    int image_height,
    float selected_out[safeseat_model::FEATURE_COUNT]);

SafeSeatPoseResult safeseat_evaluate_pose(
    const float selected[safeseat_model::FEATURE_COUNT],
    const float baseline[safeseat_model::FEATURE_COUNT]);

// V4.2 forward-lean fallback. It is deliberately narrow: it activates only
// when the nose is missing, both shoulders are present, and BOTH the shoulder
// span and detected-person box area grow by >=30% versus this passenger's
// calibrated upright baseline. A non-matching partial pose remains UNKNOWN.
SafeSeatPoseResult safeseat_evaluate_missing_nose_forward_fallback(
    const dl::detect::result_t &pose,
    int image_width,
    int image_height,
    const float baseline[safeseat_model::FEATURE_COUNT]);
