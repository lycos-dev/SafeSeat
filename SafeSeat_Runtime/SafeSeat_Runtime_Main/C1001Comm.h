#pragma once

#include <Arduino.h>

#include "C1001.h"
#include "C1001Protocol.h"
#include "Fusion.h"
#include "SafeSeatNow.h"

struct C1001RemoteStatus
{
    bool initialized = false;
    bool connected = false;
    bool stale = false;

    uint32_t packetsReceived = 0;
    uint32_t badPackets = 0;
    uint32_t lastSequence = 0;

    unsigned long lastPacketMillis = 0;
    unsigned long packetAgeMillis = 0;

    uint8_t linkChannel = 0;
    uint8_t sourceMac[6]{};

    uint8_t remoteMLStatus = 0;
    uint16_t remoteWindowSamplesCollected = 0;
    uint16_t remoteSamplesUntilNextInference = 0;
    uint16_t remoteWindowSamplesRequired = 0;
    uint32_t remoteWindowsEvaluated = 0;
};

class C1001Comm
{
public:
    C1001Comm() = default;

    bool begin();
    void update();

    C1001FusionInput getFusionInput() const;

    const C1001RemoteStatus& getStatus() const { return status; }
    const C1001Reading& getReading() const { return reading; }
    const ModelEvidence& getModelEvidence() const { return model; }

private:
    C1001WirePacket latestPacket{};
    C1001RemoteStatus status{};
    C1001Reading reading{};
    ModelEvidence model{};
    bool packetReceived = false;

    void processPacket(
        const C1001WirePacket &packet,
        const uint8_t sourceMac[6]
    );

    bool packetIsValid(const C1001WirePacket &packet) const;
    void invalidateStaleEvidence();
};
