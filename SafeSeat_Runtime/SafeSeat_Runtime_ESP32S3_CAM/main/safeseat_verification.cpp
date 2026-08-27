#include "safeseat_verification.hpp"

const char *safeseat_verification_state_name(SafeSeatVerificationState state) {
    switch (state) {
        case SafeSeatVerificationState::IDLE: return "IDLE";
        case SafeSeatVerificationState::CALIBRATING: return "CALIBRATING";
        case SafeSeatVerificationState::CLEAR_UPRIGHT: return "CLEAR_UPRIGHT";
        case SafeSeatVerificationState::HOLD_DEVIATION: return "HOLD_DEVIATION";
        case SafeSeatVerificationState::HOLD_UNKNOWN: return "HOLD_UNKNOWN";
        default: return "HOLD_UNKNOWN";
    }
}

SafeSeatVerificationState safeseat_verification_decide(
    bool verification_active,
    bool baseline_ready,
    bool raw_valid,
    SafeSeatPoseState raw_state,
    SafeSeatFilteredState filtered_state) {

    if (!verification_active) {
        return SafeSeatVerificationState::IDLE;
    }

    if (!baseline_ready) {
        return SafeSeatVerificationState::CALIBRATING;
    }

    // SAFETY CONTRACT: current UNKNOWN is never equivalent to NORMAL even if
    // the diagnostic temporal filter is still holding a previously trusted
    // NORMAL state across a short gap.
    if (!raw_valid || raw_state == SafeSeatPoseState::UNKNOWN) {
        return SafeSeatVerificationState::HOLD_UNKNOWN;
    }

    if (raw_state != SafeSeatPoseState::NORMAL) {
        return SafeSeatVerificationState::HOLD_DEVIATION;
    }

    // Require the current observation to be explicitly NORMAL AND the temporal
    // filter to have cleared any prior confirmed/pending deviation. Therefore,
    // recovery after a confirmed deviation still requires the existing two
    // trusted normal observations.
    if (filtered_state == SafeSeatFilteredState::NORMAL) {
        return SafeSeatVerificationState::CLEAR_UPRIGHT;
    }

    if (filtered_state == SafeSeatFilteredState::DEVIATION_PENDING ||
        filtered_state == SafeSeatFilteredState::DEVIATION_CONFIRMED) {
        return SafeSeatVerificationState::HOLD_DEVIATION;
    }

    return SafeSeatVerificationState::HOLD_UNKNOWN;
}
