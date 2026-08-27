#pragma once

#include "safeseat_pose_anomaly.hpp"
#include "safeseat_temporal_filter.hpp"

// V4.3.2 standalone verification contract.
// This layer is intentionally separate from the temporal filter: the filter may
// retain a recent NORMAL state across a short UNKNOWN gap for diagnostics, but
// an UNKNOWN current camera observation must NEVER clear a suspected emergency.
enum class SafeSeatVerificationState {
    IDLE = 0,
    CALIBRATING,
    CLEAR_UPRIGHT,
    HOLD_DEVIATION,
    HOLD_UNKNOWN,
};

const char *safeseat_verification_state_name(SafeSeatVerificationState state);

SafeSeatVerificationState safeseat_verification_decide(
    bool verification_active,
    bool baseline_ready,
    bool raw_valid,
    SafeSeatPoseState raw_state,
    SafeSeatFilteredState filtered_state);
