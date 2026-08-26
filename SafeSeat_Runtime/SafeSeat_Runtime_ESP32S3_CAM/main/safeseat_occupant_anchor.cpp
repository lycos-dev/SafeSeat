#include "safeseat_occupant_anchor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {
constexpr float EPS = 1e-6f;
constexpr int K_NOSE = 0;
constexpr int K_LEFT_SHOULDER = 5;
constexpr int K_RIGHT_SHOULDER = 6;

inline bool kpt_present(const std::vector<int> &k, int idx)
{
    if (static_cast<int>(k.size()) <= 2 * idx + 1) return false;
    return !(k[2 * idx] == 0 && k[2 * idx + 1] == 0);
}

float box_iou(const dl::detect::result_t &a, const dl::detect::result_t &b)
{
    if (a.box.size() < 4 || b.box.size() < 4) return 0.0f;
    const float ax1=a.box[0], ay1=a.box[1], ax2=a.box[2], ay2=a.box[3];
    const float bx1=b.box[0], by1=b.box[1], bx2=b.box[2], by2=b.box[3];
    const float ix1=std::max(ax1,bx1), iy1=std::max(ay1,by1);
    const float ix2=std::min(ax2,bx2), iy2=std::min(ay2,by2);
    const float iw=std::max(0.0f,ix2-ix1), ih=std::max(0.0f,iy2-iy1);
    const float inter=iw*ih;
    const float aa=std::max(0.0f,ax2-ax1)*std::max(0.0f,ay2-ay1);
    const float ba=std::max(0.0f,bx2-bx1)*std::max(0.0f,by2-by1);
    const float uni=aa+ba-inter;
    return uni>EPS ? inter/uni : 0.0f;
}

bool full_calibration_pose(const dl::detect::result_t &p)
{
    return p.keypoint.size() >= 34
        && kpt_present(p.keypoint, K_NOSE)
        && kpt_present(p.keypoint, K_LEFT_SHOULDER)
        && kpt_present(p.keypoint, K_RIGHT_SHOULDER);
}

struct Candidate {
    const dl::detect::result_t *pose = nullptr;
    SafeSeatOccupantAnchor obs;
    float cost = 0.0f;
    float area_scale = 0.0f;
};
}

bool safeseat_make_occupant_anchor_observation(
    const dl::detect::result_t &pose,
    int image_width,
    int image_height,
    SafeSeatOccupantAnchor &out)
{
    if (image_width <= 0 || image_height <= 0 || pose.box.size() < 4) return false;
    const float x1=pose.box[0], y1=pose.box[1], x2=pose.box[2], y2=pose.box[3];
    const float bw=x2-x1, bh=y2-y1;
    if (bw <= EPS || bh <= EPS) return false;

    out = {};
    out.box_cx_norm = ((x1+x2)*0.5f) / static_cast<float>(image_width);
    out.box_cy_norm = ((y1+y2)*0.5f) / static_cast<float>(image_height);
    out.box_area_ratio = (bw*bh) / static_cast<float>(image_width*image_height);

    if (pose.keypoint.size() >= 34
        && kpt_present(pose.keypoint, K_LEFT_SHOULDER)
        && kpt_present(pose.keypoint, K_RIGHT_SHOULDER))
    {
        const float ax=pose.keypoint[10], ay=pose.keypoint[11];
        const float bx=pose.keypoint[12], by=pose.keypoint[13];
        const float dx=bx-ax, dy=by-ay;
        out.shoulder_cx_norm = ((ax+bx)*0.5f) / static_cast<float>(image_width);
        out.shoulder_cy_norm = ((ay+by)*0.5f) / static_cast<float>(image_height);
        out.shoulder_width_ratio = std::sqrt(dx*dx+dy*dy) / static_cast<float>(image_width);
    } else {
        out.shoulder_cx_norm = out.box_cx_norm;
        out.shoulder_cy_norm = out.box_cy_norm;
        out.shoulder_width_ratio = 0.0f;
    }

    return std::isfinite(out.box_cx_norm) && std::isfinite(out.box_cy_norm)
        && std::isfinite(out.box_area_ratio) && out.box_area_ratio > EPS
        && std::isfinite(out.shoulder_cx_norm) && std::isfinite(out.shoulder_cy_norm)
        && std::isfinite(out.shoulder_width_ratio);
}

SafeSeatOccupantSelection safeseat_select_calibration_occupant(
    const std::list<dl::detect::result_t> &poses,
    int image_width,
    int image_height,
    float min_person_score)
{
    SafeSeatOccupantSelection out;
    std::vector<Candidate> cands;
    for (const auto &p : poses) {
        SafeSeatOccupantAnchor obs;
        if (p.score < min_person_score || !full_calibration_pose(p)
            || !safeseat_make_occupant_anchor_observation(p,image_width,image_height,obs)) {
            ++out.rejected_background;
            continue;
        }
        // Before a passenger anchor exists, enforce a coarse one-seat foreground
        // gate. The real seated occupant in SafeSeat's framing occupies a large
        // portion of QVGA; a small person several metres behind the seat must not
        // become a calibration sample if the foreground occupant is momentarily missed.
        if (obs.box_area_ratio < 0.08f || obs.shoulder_width_ratio < 0.08f) {
            ++out.rejected_background;
            continue;
        }
        Candidate c; c.pose=&p; c.obs=obs;
        // Camera is mounted for one seat: foreground size is the strongest
        // depth surrogate before an anchor exists. Slight score weighting only.
        c.cost = -(obs.box_area_ratio * (0.85f + 0.15f*std::max(0.0f,std::min(1.0f,p.score))));
        cands.push_back(c);
    }
    if (cands.empty()) return out;
    std::sort(cands.begin(),cands.end(),[](const Candidate&a,const Candidate&b){return a.cost<b.cost;});
    if (cands.size()==1) { out.pose=cands[0].pose; return out; }

    const Candidate &best=cands[0], &second=cands[1];
    const float area_ratio = best.obs.box_area_ratio / std::max(second.obs.box_area_ratio,EPS);
    const float best_center = std::fabs(best.obs.box_cx_norm-0.5f);
    const float second_center = std::fabs(second.obs.box_cx_norm-0.5f);
    const bool duplicate = box_iou(*best.pose,*second.pose) >= 0.15f;
    const bool clearly_foreground = area_ratio >= 1.35f;
    const bool clearly_centered = best_center + 0.12f < second_center
        && best.obs.box_area_ratio >= second.obs.box_area_ratio*0.85f;

    out.selected_from_multiple = true;
    if (duplicate || clearly_foreground || clearly_centered) {
        out.pose=best.pose;
        out.rejected_background += static_cast<int>(cands.size())-1;
        return out;
    }
    out.ambiguous=true;
    return out;
}

SafeSeatOccupantSelection safeseat_select_tracked_occupant(
    const std::list<dl::detect::result_t> &poses,
    int image_width,
    int image_height,
    const SafeSeatOccupantAnchor &anchor,
    float min_person_score)
{
    SafeSeatOccupantSelection out;
    if (anchor.box_area_ratio <= EPS) { out.ambiguous=true; return out; }

    std::vector<Candidate> cands;
    for (const auto &p : poses) {
        SafeSeatOccupantAnchor obs;
        if (p.score < min_person_score
            || !safeseat_make_occupant_anchor_observation(p,image_width,image_height,obs)) {
            ++out.rejected_background;
            continue;
        }

        const float area_scale = obs.box_area_ratio/std::max(anchor.box_area_ratio,EPS);
        const float dx = std::fabs(obs.box_cx_norm-anchor.box_cx_norm);
        const float dy = std::fabs(obs.box_cy_norm-anchor.box_cy_norm);
        if (area_scale < 0.28f || area_scale > 3.20f || dx > 0.38f || dy > 0.38f) {
            ++out.rejected_background;
            continue;
        }

        float shoulder_cost = 0.0f;
        if (obs.shoulder_width_ratio > EPS && anchor.shoulder_width_ratio > EPS) {
            const float shoulder_scale = obs.shoulder_width_ratio/anchor.shoulder_width_ratio;
            const float sdx = std::fabs(obs.shoulder_cx_norm-anchor.shoulder_cx_norm);
            const float sdy = std::fabs(obs.shoulder_cy_norm-anchor.shoulder_cy_norm);
            if (shoulder_scale < 0.35f || shoulder_scale > 2.50f || sdx > 0.40f || sdy > 0.40f) {
                ++out.rejected_background;
                continue;
            }
            shoulder_cost = 0.45f*(sdx/0.40f) + 0.25f*(sdy/0.40f)
                + 0.35f*std::fabs(std::log(std::max(shoulder_scale,EPS)));
        }

        Candidate c; c.pose=&p; c.obs=obs; c.area_scale=area_scale;
        c.cost = 1.20f*(dx/0.38f) + 0.80f*(dy/0.38f)
            + 0.60f*std::fabs(std::log(std::max(area_scale,EPS)))
            + shoulder_cost
            + 0.12f*(1.0f-std::max(0.0f,std::min(1.0f,p.score)));
        cands.push_back(c);
    }

    if (cands.empty()) return out;
    std::sort(cands.begin(),cands.end(),[](const Candidate&a,const Candidate&b){return a.cost<b.cost;});
    const Candidate &best=cands[0];
    out.selected_from_multiple = poses.size()>1;
    out.match_cost=best.cost;
    out.area_scale=best.area_scale;

    if (cands.size()>1) {
        const Candidate &second=cands[1];
        const bool duplicate = box_iou(*best.pose,*second.pose)>=0.15f;
        const bool clear_margin = (second.cost-best.cost)>=0.22f;
        if (!duplicate && !clear_margin) {
            out.ambiguous=true;
            return out;
        }
    }

    out.pose=best.pose;
    out.rejected_background += static_cast<int>(cands.size())-1;
    return out;
}
