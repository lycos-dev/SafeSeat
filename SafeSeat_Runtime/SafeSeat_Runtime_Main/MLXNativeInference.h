#pragma once
#include <stdint.h>

struct MLXNativeDecision
{
    float isolationForestDecision = 0.0f;
    float oneClassSVMDecision = 0.0f;
    bool isolationForestAnomaly = false;
    bool oneClassSVMAnomaly = false;
    bool eitherAnomaly = false;
    bool bothAnomaly = false;
};

class MLXNativeInference
{
public:
    MLXNativeDecision predict(const float features[2]) const;

private:
    void preprocess(const float input[2], float output[2]) const;
    float averagePathLength(uint16_t n) const;
    float isolationForestDecision(const float scaled[2]) const;
    float oneClassSVMDecision(const float scaled[2]) const;
};
