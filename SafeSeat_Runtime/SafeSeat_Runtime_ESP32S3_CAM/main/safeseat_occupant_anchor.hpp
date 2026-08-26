#pragma once

#include "dl_detect_define.hpp"

#include <list>

constexpr int SAFESEAT_OCCUPANT_ANCHOR_COUNT = 6;

struct SafeSeatOccupantAnchor {
    // All values are image-normalized so the anchor is independent of resolution.
    float box_cx_norm = 0.0f;
    float box_cy_norm = 0.0f;
    float box_area_ratio = 0.0f;
    float shoulder_cx_norm = 0.0f;
    float shoulder_cy_norm = 0.0f;
    float shoulder_width_ratio = 0.0f;
};
static_assert(sizeof(SafeSeatOccupantAnchor) == SAFESEAT_OCCUPANT_ANCHOR_COUNT * sizeof(float), "SafeSeatOccupantAnchor must remain six contiguous floats for NVS serialization");

struct SafeSeatOccupantSelection {
    const dl::detect::result_t *pose = nullptr;
    bool selected_from_multiple = false;
    bool ambiguous = false;
    int rejected_background = 0;
    float match_cost = 0.0f;
    float area_scale = 0.0f;
};

bool safeseat_make_occupant_anchor_observation(
    const dl::detect::result_t &pose,
    int image_width,
    int image_height,
    SafeSeatOccupantAnchor &out);

// Calibration has no prior passenger anchor yet. Prefer the dominant foreground
// full pose; if two distinct people are similarly plausible, stay ambiguous.
SafeSeatOccupantSelection safeseat_select_calibration_occupant(
    const std::list<dl::detect::result_t> &poses,
    int image_width,
    int image_height,
    float min_person_score);

// Once calibration is complete, associate every later detection with the same
// seat occupant. Tiny/distant people and spatially unrelated people are rejected
// as background. If two distinct candidates fit the anchor almost equally well,
// return ambiguous rather than switching identities.
SafeSeatOccupantSelection safeseat_select_tracked_occupant(
    const std::list<dl::detect::result_t> &poses,
    int image_width,
    int image_height,
    const SafeSeatOccupantAnchor &anchor,
    float min_person_score);
