#pragma once

#include <Arduino.h>

#include "Fusion.h"
#include "PiezoProtocol.h"
#include "SafeSeatNow.h"

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
    uint32_t remoteBreathCount = 0;
    uint32_t remoteSampleCount = 0;

    uint8_t linkChannel = 0;
    uint8_t sourceMac[6]{};
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

    const SafeSeatNowStatus&
    getWirelessStatus() const
    {
        return SafeSeatNow::instance().getStatus();
    }

private:
    PiezoWirePacket latestPacket{};
    PiezoRemoteStatus status{};
    bool packetReceived = false;

    void processPacket(
        const PiezoWirePacket &packet,
        const uint8_t sourceMac[6]
    );

    bool packetIsValid(
        const PiezoWirePacket &packet
    ) const;
};
