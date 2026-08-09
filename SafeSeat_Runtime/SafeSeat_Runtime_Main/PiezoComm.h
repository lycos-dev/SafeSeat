#pragma once

#include <Arduino.h>

#include "Fusion.h"
#include "PiezoProtocol.h"

struct PiezoRemoteStatus
{
    bool initialized = false;
    bool connected = false;

    uint32_t packetsReceived = 0;
    uint32_t badPackets = 0;
    uint32_t lastSequence = 0;

    unsigned long lastPacketMillis = 0;
    unsigned long packetAgeMillis = 0;

    float remoteSamplingRateHz = 0.0f;
    uint32_t remoteFeatureWindowCount = 0;
};

class PiezoComm
{
public:
    PiezoComm() = default;

    bool begin();
    void update();

    PiezoFusionEvidence getFusionEvidence() const;

    const PiezoRemoteStatus&
    getStatus() const
    {
        return status;
    }

private:
    uint8_t receiveBuffer[sizeof(PiezoWirePacket)]{};
    size_t receiveIndex = 0;

    PiezoWirePacket latestPacket{};
    PiezoRemoteStatus status{};

    bool packetReceived = false;

    void consumeByte(uint8_t value);
    void processBufferedPacket();
    bool packetIsValid(
        const PiezoWirePacket &packet
    ) const;
};
