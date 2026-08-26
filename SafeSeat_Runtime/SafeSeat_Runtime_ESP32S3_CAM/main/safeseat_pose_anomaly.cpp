#include "safeseat_pose_anomaly.hpp"
#include "model_data.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float SAFESEAT_EPS = 1e-6f;
constexpr int K_NOSE = 0;
constexpr int K_LEFT_SHOULDER = 5;
constexpr int K_RIGHT_SHOULDER = 6;

inline bool kpt_present(const std::vector<int> &k, int idx) {
    if (static_cast<int>(k.size()) <= 2 * idx + 1) return false;
    const int x = k[2 * idx], y = k[2 * idx + 1];
    return !(x == 0 && y == 0);
}
inline float dist2d(float x1,float y1,float x2,float y2){const float dx=x2-x1,dy=y2-y1;return std::sqrt(dx*dx+dy*dy);}
inline float rad_to_deg(float x){return x*(180.0f/PI_F);}
inline float wrap_180_to_90(float a){float x=std::fmod(a+90.0f,180.0f);if(x<0.0f)x+=180.0f;return x-90.0f;}
float average_path_length(uint16_t n){if(n<=1)return 0.0f;if(n==2)return 1.0f;constexpr float EULER_GAMMA=0.5772156649015329f;const float nf=static_cast<float>(n);return 2.0f*(std::log(nf-1.0f)+EULER_GAMMA)-2.0f*(nf-1.0f)/nf;}
float iforest_decision_function(const float *x){float total_depth=0.0f;for(int ti=0;ti<safeseat_model::IF_TREE_COUNT;++ti){const auto&t=safeseat_model::IF_TREES[ti];int node=0,edges=0;while(t.children_left[node]!=t.children_right[node]){const int feat=static_cast<int>(t.feature[node]);if(feat<0||feat>=safeseat_model::FEATURE_COUNT)break;node=(x[feat]<=t.threshold[node])?t.children_left[node]:t.children_right[node];++edges;if(node<0||node>=t.node_count)return NAN;}total_depth+=static_cast<float>(edges)+average_path_length(t.n_node_samples[node]);}const float denom=static_cast<float>(safeseat_model::IF_TREE_COUNT)*average_path_length(safeseat_model::IF_MAX_SAMPLES);if(denom<=SAFESEAT_EPS)return NAN;const float anomaly_score=std::pow(2.0f,-total_depth/denom);return -anomaly_score-safeseat_model::IF_OFFSET;}
float ocsvm_decision_function(const float *x){float sum=0.0f;for(int s=0;s<safeseat_model::OCSVM_SUPPORT_COUNT;++s){float d2=0.0f;const float*sv=&safeseat_model::OCSVM_SUPPORT_VECTORS[s*safeseat_model::FEATURE_COUNT];for(int j=0;j<safeseat_model::FEATURE_COUNT;++j){const float d=x[j]-sv[j];d2+=d*d;}sum+=safeseat_model::OCSVM_DUAL_COEF[s]*std::exp(-safeseat_model::OCSVM_GAMMA*d2);}return sum+safeseat_model::OCSVM_INTERCEPT;}
}

const char *safeseat_pose_state_name(SafeSeatPoseState state){switch(state){case SafeSeatPoseState::NORMAL:return "NORMAL";case SafeSeatPoseState::DEVIATION_WEAK:return "DEVIATION_WEAK";case SafeSeatPoseState::DEVIATION_MODERATE:return "DEVIATION_MODERATE";case SafeSeatPoseState::DEVIATION_STRONG:return "DEVIATION_STRONG";default:return "UNKNOWN";}}

bool safeseat_extract_pose_features(const dl::detect::result_t &pose,int width,int height,float f[7]){
    if(width<=0||height<=0||pose.keypoint.size()<34||pose.box.size()<4)return false;
    if(!kpt_present(pose.keypoint,K_NOSE)||!kpt_present(pose.keypoint,K_LEFT_SHOULDER)||!kpt_present(pose.keypoint,K_RIGHT_SHOULDER))return false;
    const float nx=pose.keypoint[0],ny=pose.keypoint[1];
    // COCO/YOLO anatomical left/right shoulder identities can flip between otherwise
    // similar frames. SafeSeat only needs posture-deviation magnitude, not anatomical
    // side naming, so canonicalize the pair by image X before computing geometry.
    const float ax=pose.keypoint[10],ay=pose.keypoint[11],bx=pose.keypoint[12],by=pose.keypoint[13];
    const bool a_is_image_left=ax<=bx;
    const float lsx=a_is_image_left?ax:bx, lsy=a_is_image_left?ay:by;
    const float rsx=a_is_image_left?bx:ax, rsy=a_is_image_left?by:ay;
    const float sw=std::max(dist2d(lsx,lsy,rsx,rsy),SAFESEAT_EPS),smx=(lsx+rsx)*0.5f,smy=(lsy+rsy)*0.5f;
    const float diag=std::sqrt(static_cast<float>(width*width+height*height));
    const float x1=pose.box[0],y1=pose.box[1],x2=pose.box[2],y2=pose.box[3],bw=std::max(0.0f,x2-x1),bh=std::max(0.0f,y2-y1);
    if(bw<=SAFESEAT_EPS||diag<=SAFESEAT_EPS)return false;
    f[0]=wrap_180_to_90(rad_to_deg(std::atan2(rsy-lsy,rsx-lsx)));
    f[1]=(ny-smy)/sw;
    f[2]=dist2d(nx,ny,lsx,lsy)/sw;
    f[3]=dist2d(nx,ny,rsx,rsy)/sw;
    f[4]=sw/static_cast<float>(width);
    f[5]=dist2d(nx,ny,smx,smy)/diag;
    f[6]=(bw*bh)/static_cast<float>(width*height);
    for(int j=0;j<safeseat_model::FEATURE_COUNT;++j)if(!std::isfinite(f[j]))return false;
    return true;
}

SafeSeatPoseResult safeseat_evaluate_pose(const float selected[7],const float baseline[7]){
    SafeSeatPoseResult r;std::memcpy(r.features,selected,sizeof(r.features));
    for(int j=0;j<safeseat_model::FEATURE_COUNT;++j){float delta=selected[j]-baseline[j];if(j==0)delta=wrap_180_to_90(delta);const float scale=std::max(std::fabs(safeseat_model::SCALER_SCALE[j]),SAFESEAT_EPS);r.compensated_z[j]=delta/scale;if(!std::isfinite(r.compensated_z[j]))return SafeSeatPoseResult{};}
    r.if_score=iforest_decision_function(r.compensated_z);r.ocsvm_score=ocsvm_decision_function(r.compensated_z);if(!std::isfinite(r.if_score)||!std::isfinite(r.ocsvm_score))return SafeSeatPoseResult{};r.valid_pose=true;r.if_anomaly=r.if_score<safeseat_model::IF_THRESHOLD;r.ocsvm_anomaly=r.ocsvm_score<safeseat_model::OCSVM_THRESHOLD;if(!r.if_anomaly&&!r.ocsvm_anomaly)r.state=SafeSeatPoseState::NORMAL;else if(r.if_anomaly&&r.ocsvm_anomaly)r.state=SafeSeatPoseState::DEVIATION_STRONG;else if(r.ocsvm_anomaly)r.state=SafeSeatPoseState::DEVIATION_MODERATE;else r.state=SafeSeatPoseState::DEVIATION_WEAK;return r;
}

SafeSeatPoseResult safeseat_evaluate_missing_nose_forward_fallback(
    const dl::detect::result_t &pose,
    int width,
    int height,
    const float baseline[7])
{
    SafeSeatPoseResult r;
    if (width <= 0 || height <= 0 || !baseline || pose.keypoint.size() < 34 || pose.box.size() < 4) return r;

    // This fallback exists specifically for the live failure mode where YOLO
    // keeps both shoulders but drops the face/nose during a forward lean.
    if (kpt_present(pose.keypoint, K_NOSE)) return r;
    if (!kpt_present(pose.keypoint, K_LEFT_SHOULDER) || !kpt_present(pose.keypoint, K_RIGHT_SHOULDER)) return r;

    const float ax = pose.keypoint[10], ay = pose.keypoint[11];
    const float bx = pose.keypoint[12], by = pose.keypoint[13];
    const float shoulder_width_img = dist2d(ax, ay, bx, by) / static_cast<float>(width);

    const float x1 = pose.box[0], y1 = pose.box[1], x2 = pose.box[2], y2 = pose.box[3];
    const float bw = std::max(0.0f, x2 - x1), bh = std::max(0.0f, y2 - y1);
    const float box_area_ratio = (bw * bh) / static_cast<float>(width * height);

    const float base_sw = std::max(std::fabs(baseline[4]), SAFESEAT_EPS);
    const float base_box = std::max(std::fabs(baseline[6]), SAFESEAT_EPS);
    const float shoulder_ratio = shoulder_width_img / base_sw;
    const float box_ratio = box_area_ratio / base_box;

    if (!std::isfinite(shoulder_ratio) || !std::isfinite(box_ratio)) return r;

    // Conservative paired growth gate derived from the live V4.1 sequence:
    // the two missing-nose forward frames were 1.36-1.40x wider at the
    // shoulders and 1.35-1.46x larger in box area than upright. 1.30x keeps
    // margin while requiring BOTH cues to move together.
    constexpr float MIN_GROWTH_RATIO = 1.30f;
    if (shoulder_ratio < MIN_GROWTH_RATIO || box_ratio < MIN_GROWTH_RATIO) return r;

    std::memcpy(r.features, baseline, sizeof(r.features));
    r.features[4] = shoulder_width_img;
    r.features[6] = box_area_ratio;
    r.compensated_z[4] = (shoulder_width_img - baseline[4]) /
        std::max(std::fabs(safeseat_model::SCALER_SCALE[4]), SAFESEAT_EPS);
    r.compensated_z[6] = (box_area_ratio - baseline[6]) /
        std::max(std::fabs(safeseat_model::SCALER_SCALE[6]), SAFESEAT_EPS);
    r.valid_pose = true;
    r.fallback_used = true;
    r.fallback_shoulder_ratio = shoulder_ratio;
    r.fallback_box_ratio = box_ratio;
    r.state = SafeSeatPoseState::DEVIATION_MODERATE;
    return r;
}

