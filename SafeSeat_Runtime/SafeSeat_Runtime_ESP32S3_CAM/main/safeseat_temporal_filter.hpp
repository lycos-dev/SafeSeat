#pragma once
#include "safeseat_pose_anomaly.hpp"

enum class SafeSeatFilteredState {
    UNKNOWN = 0,
    NORMAL,
    DEVIATION_PENDING,
    DEVIATION_CONFIRMED,
};

struct SafeSeatTemporalStatus {
    SafeSeatFilteredState state = SafeSeatFilteredState::UNKNOWN;
    SafeSeatPoseState peak_raw_state = SafeSeatPoseState::UNKNOWN;
    int abnormal_streak = 0;
    int normal_streak = 0;
    int unknown_gap = 0;
    bool confirmed_deviation = false;
};

class SafeSeatTemporalFilter {
public:
    SafeSeatTemporalStatus update(bool valid, SafeSeatPoseState raw_state);
    void reset();
private:
    bool confirmed_deviation_ = false;
    bool has_trusted_normal_ = false;
    int abnormal_streak_ = 0;
    int normal_streak_ = 0;
    int unknown_gap_ = 0;
    int pending_normal_gap_ = 0;
    SafeSeatPoseState peak_raw_state_ = SafeSeatPoseState::UNKNOWN;
};

const char *safeseat_filtered_state_name(SafeSeatFilteredState state);
