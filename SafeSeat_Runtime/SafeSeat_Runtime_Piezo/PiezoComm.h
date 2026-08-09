#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "PiezoProtocol.h"
#include "SafeSeatNowProtocol.h"
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

    unsigned long getSendErrors() const
    {
        return sendErrors;
    }

    unsigned long getHubBeaconsReceived() const
    {
        return hubBeaconsReceived;
    }

    bool isHubLocked() const
    {
        return hubLocked;
    }

    uint8_t getChannel() const
    {
        return currentChannel;
    }

    unsigned long getBeaconAgeMillis() const;

private:
    static PiezoComm *activeInstance;
    static const uint8_t BROADCAST_MAC[6];

    unsigned long lastSendMillis = 0;
    unsigned long packetsSent = 0;
    unsigned long sendErrors = 0;
    uint32_t sequence = 0;
    bool initialized = false;

    volatile bool hubLocked = false;
    volatile unsigned long lastHubBeaconMillis = 0;
    volatile unsigned long hubBeaconsReceived = 0;

    uint8_t currentChannel = 0;
    uint8_t nextScanChannel = 0;
    unsigned long lastChannelHopMillis = 0;

    bool ensureBroadcastPeer();
    void maintainHubChannel();
    void hopToNextChannel();

    static void onReceiveStatic(
        const esp_now_recv_info_t *info,
        const uint8_t *data,
        int len
    );

    void onReceive(
        const esp_now_recv_info_t *info,
        const uint8_t *data,
        int len
    );
};
