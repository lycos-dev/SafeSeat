#include "safeseat_verification.hpp"
#include "safeseat_temporal_filter.hpp"
#include <cassert>
#include <cstdlib>
#include <iostream>

static void expect(SafeSeatVerificationState got, SafeSeatVerificationState want) {
    if (got != want) {
        std::cerr << "Expected " << safeseat_verification_state_name(want)
                  << " but got " << safeseat_verification_state_name(got) << "\n";
        std::abort();
    }
}

static bool integrated_upright_clear_allowed(
    SafeSeatVerificationState contract,
    const SafeSeatTemporalStatus &temporal) {
    return contract == SafeSeatVerificationState::CLEAR_UPRIGHT
        && temporal.normal_streak >= 2
        && temporal.abnormal_streak == 0;
}

int main() {
    using V = SafeSeatVerificationState;
    using P = SafeSeatPoseState;
    using F = SafeSeatFilteredState;

    // Critical safety contract inherited from standalone V4.3.2.
    expect(safeseat_verification_decide(true, true, false, P::UNKNOWN, F::NORMAL), V::HOLD_UNKNOWN);
    expect(safeseat_verification_decide(true, true, true, P::UNKNOWN, F::NORMAL), V::HOLD_UNKNOWN);

    // Integrated runtime is stricter than standalone display mode: one normal
    // frame is not enough to send UPRIGHT over ESP-NOW.
    SafeSeatTemporalFilter temporal;
    auto s = temporal.update(true, P::NORMAL);
    auto c = safeseat_verification_decide(true, true, true, P::NORMAL, s.state);
    assert(c == V::CLEAR_UPRIGHT);
    assert(s.normal_streak == 1);
    assert(!integrated_upright_clear_allowed(c, s));

    // Two consecutive clean normal observations permit UPRIGHT.
    s = temporal.update(true, P::NORMAL);
    c = safeseat_verification_decide(true, true, true, P::NORMAL, s.state);
    assert(s.normal_streak == 2);
    assert(s.abnormal_streak == 0);
    assert(integrated_upright_clear_allowed(c, s));

    // UNKNOWN between normals never itself clears and resets the path such that
    // a later explicit pair of clean normals is required before UPRIGHT.
    temporal.reset();
    s = temporal.update(false, P::UNKNOWN);
    c = safeseat_verification_decide(true, true, false, P::UNKNOWN, s.state);
    assert(c == V::HOLD_UNKNOWN);
    assert(!integrated_upright_clear_allowed(c, s));
    s = temporal.update(true, P::NORMAL);
    c = safeseat_verification_decide(true, true, true, P::NORMAL, s.state);
    assert(!integrated_upright_clear_allowed(c, s));
    s = temporal.update(true, P::NORMAL);
    c = safeseat_verification_decide(true, true, true, P::NORMAL, s.state);
    assert(integrated_upright_clear_allowed(c, s));

    // Two abnormalities confirm non-upright and must not clear.
    temporal.reset();
    s = temporal.update(true, P::DEVIATION_MODERATE);
    c = safeseat_verification_decide(true, true, true, P::DEVIATION_MODERATE, s.state);
    assert(c == V::HOLD_DEVIATION);
    assert(!integrated_upright_clear_allowed(c, s));
    s = temporal.update(true, P::DEVIATION_STRONG);
    c = safeseat_verification_decide(true, true, true, P::DEVIATION_STRONG, s.state);
    assert(s.state == F::DEVIATION_CONFIRMED);
    assert(c == V::HOLD_DEVIATION);
    assert(!integrated_upright_clear_allowed(c, s));

    std::cout << "PASS V5.3.2 integrated contract: UNKNOWN never clears; UPRIGHT requires two clean normals; confirmed deviation holds\n";
    return 0;
}
