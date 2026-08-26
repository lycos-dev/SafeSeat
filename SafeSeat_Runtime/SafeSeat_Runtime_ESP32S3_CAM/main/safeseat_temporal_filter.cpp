#include "safeseat_temporal_filter.hpp"

namespace {
constexpr int ABNORMAL_REQUIRED = 2;
constexpr int NORMAL_REQUIRED_TO_CLEAR = 2;
constexpr int UNKNOWN_HOLD_MAX = 2;
constexpr int PENDING_NORMAL_GRACE = 1;

int severity_rank(SafeSeatPoseState s) {
    switch (s) {
        case SafeSeatPoseState::DEVIATION_STRONG: return 3;
        case SafeSeatPoseState::DEVIATION_MODERATE: return 2;
        case SafeSeatPoseState::DEVIATION_WEAK: return 1;
        default: return 0;
    }
}
}

const char *safeseat_filtered_state_name(SafeSeatFilteredState state) {
    switch (state) {
        case SafeSeatFilteredState::NORMAL: return "NORMAL";
        case SafeSeatFilteredState::DEVIATION_PENDING: return "DEVIATION_PENDING";
        case SafeSeatFilteredState::DEVIATION_CONFIRMED: return "DEVIATION_CONFIRMED";
        default: return "UNKNOWN";
    }
}

void SafeSeatTemporalFilter::reset() {
    confirmed_deviation_ = false;
    has_trusted_normal_ = false;
    abnormal_streak_ = 0;
    normal_streak_ = 0;
    unknown_gap_ = 0;
    pending_normal_gap_ = 0;
    peak_raw_state_ = SafeSeatPoseState::UNKNOWN;
}

SafeSeatTemporalStatus SafeSeatTemporalFilter::update(bool valid, SafeSeatPoseState raw_state) {
    SafeSeatFilteredState filtered = SafeSeatFilteredState::UNKNOWN;

    if (valid && raw_state == SafeSeatPoseState::NORMAL) {
        unknown_gap_ = 0;
        ++normal_streak_;
        has_trusted_normal_ = true;

        if (confirmed_deviation_) {
            abnormal_streak_ = 0;
            pending_normal_gap_ = 0;
            peak_raw_state_ = SafeSeatPoseState::UNKNOWN;
            if (normal_streak_ >= NORMAL_REQUIRED_TO_CLEAR) confirmed_deviation_ = false;
            filtered = confirmed_deviation_ ? SafeSeatFilteredState::DEVIATION_CONFIRMED
                                            : SafeSeatFilteredState::NORMAL;
        } else if (abnormal_streak_ > 0) {
            // V4.2: one intervening NORMAL no longer erases a pending
            // moderate/strong posture immediately. This addresses live
            // front/back sequences that alternate A-N-A due to 2-D pose jitter.
            ++pending_normal_gap_;
            if (pending_normal_gap_ <= PENDING_NORMAL_GRACE) {
                filtered = SafeSeatFilteredState::DEVIATION_PENDING;
            } else {
                abnormal_streak_ = 0;
                pending_normal_gap_ = 0;
                peak_raw_state_ = SafeSeatPoseState::UNKNOWN;
                filtered = SafeSeatFilteredState::NORMAL;
            }
        } else {
            pending_normal_gap_ = 0;
            peak_raw_state_ = SafeSeatPoseState::UNKNOWN;
            filtered = SafeSeatFilteredState::NORMAL;
        }
    } else if (valid && raw_state != SafeSeatPoseState::UNKNOWN) {
        unknown_gap_ = 0;
        normal_streak_ = 0;
        pending_normal_gap_ = 0;
        ++abnormal_streak_;
        if (severity_rank(raw_state) > severity_rank(peak_raw_state_)) peak_raw_state_ = raw_state;
        if (!confirmed_deviation_ && abnormal_streak_ >= ABNORMAL_REQUIRED) confirmed_deviation_ = true;
        filtered = confirmed_deviation_ ? SafeSeatFilteredState::DEVIATION_CONFIRMED
                                        : SafeSeatFilteredState::DEVIATION_PENDING;
    } else {
        ++unknown_gap_;
        if (confirmed_deviation_) {
            filtered = SafeSeatFilteredState::DEVIATION_CONFIRMED;
        } else if (abnormal_streak_ > 0 && unknown_gap_ <= UNKNOWN_HOLD_MAX) {
            filtered = SafeSeatFilteredState::DEVIATION_PENDING;
        } else if (has_trusted_normal_ && unknown_gap_ <= UNKNOWN_HOLD_MAX) {
            filtered = SafeSeatFilteredState::NORMAL;
        } else {
            if (unknown_gap_ > UNKNOWN_HOLD_MAX) {
                abnormal_streak_ = 0;
                normal_streak_ = 0;
                pending_normal_gap_ = 0;
                peak_raw_state_ = SafeSeatPoseState::UNKNOWN;
            }
            filtered = SafeSeatFilteredState::UNKNOWN;
        }
    }

    SafeSeatTemporalStatus out;
    out.state = filtered;
    out.peak_raw_state = peak_raw_state_;
    out.abnormal_streak = abnormal_streak_;
    out.normal_streak = normal_streak_;
    out.unknown_gap = unknown_gap_;
    out.confirmed_deviation = confirmed_deviation_;
    return out;
}
