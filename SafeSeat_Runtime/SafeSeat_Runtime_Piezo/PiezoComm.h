#pragma once

#include <Arduino.h>

#include "PiezoProtocol.h"
#include "PiezoSensor.h"
#include "PiezoSignalProcessor.h"
#include "PiezoFeatureExtractor.h"
#include "PiezoInference.h"

class PiezoComm
{
public:
    PiezoComm() = default;

    bool begin();

    void update(
        const PiezoReading &reading,
        bool signalWindowAligned,
        const PiezoSignalQuality &signalQuality,
        bool featureVectorReady,
        bool inferenceReady,
        const PiezoFeatures &features,
        const PiezoInferenceResult &inference,
        unsigned long featureWindowCount
    );

    unsigned long getPacketsSent() const
    {
        return packetsSent;
    }

private:
    unsigned long lastSendMillis = 0;
    unsigned long packetsSent = 0;
    uint32_t sequence = 0;
    bool initialized = false;
};
