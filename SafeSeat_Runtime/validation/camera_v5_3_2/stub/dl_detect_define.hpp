#pragma once
#include <vector>
namespace dl { namespace detect {
struct result_t {
    std::vector<int> box;
    std::vector<int> keypoint;
    float score = 0.0f;
};
}}
