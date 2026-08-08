#pragma once
#include <Arduino.h>
#include "Config.h"

struct PiezoSignalQuality {
    bool valid=false;
    float railFraction=0.0f;
    float alignedMean=0.0f;
    float alignedStd=0.0f;
    float alignedMin=0.0f;
    float alignedMax=0.0f;
    bool excessiveRailContact=false;
    bool effectivelyFlat=false;
};

class PiezoSignalProcessor {
public:
    bool alignWindow(const float rawWindow[],float alignedWindow[],uint16_t n,PiezoSignalQuality &quality) const;
private:
    struct State { float z1=0.0f,z2=0.0f; };
    void linearDetrend(const float input[],float output[],uint16_t n) const;
    void filterForward(float signal[],uint16_t n) const;
    void reverse(float signal[],uint16_t n) const;
    void quality(const float raw[],const float aligned[],uint16_t n,PiezoSignalQuality &q) const;
};
