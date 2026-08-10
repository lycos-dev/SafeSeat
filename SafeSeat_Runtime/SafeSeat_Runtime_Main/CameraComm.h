#pragma once

#include <Arduino.h>

#include "CameraProtocol.h"
#include "Fusion.h"
#include "SafeSeatNow.h"

struct CameraRemoteStatus
{
    bool initialized = false;
    bool connected = false;
    bool modelReady = false;
    bool cameraReady = false;
    bool psramReady = false;
    bool busy = false;

    uint8_t channel = 0;
    uint8_t sourceMac[6]{};

    uint32_t statusPacketsReceived = 0;
    uint32_t resultPacketsReceived = 0;
    uint32_t badPackets = 0;
    uint32_t ignoredResults = 0;
    uint32_t triggerPacketsQueued = 0;

    uint32_t activeRequestId = 0;
    bool requestActive = false;
    unsigned long requestAgeMillis = 0;
    unsigned long packetAgeMillis = 0;

    uint32_t lastResultRequestId = 0;
    CameraPostureClass lastPosture = CameraPostureClass::UNKNOWN;
    float lastConfidence = 0.0f;
    uint8_t lastValidFrames = 0;
    unsigned long lastRemoteInferenceMillis = 0;
};

class CameraComm
{
public:
    bool begin();
    void update();

    // Called after Fusion::update(). If Fusion requests camera
    // verification, this starts/resends one transaction until a
    // matching result is received or the request expires.
    void serviceVerificationRequest(bool requested);

    CameraFusionEvidence getFusionEvidence() const;
    const CameraRemoteStatus &getStatus() const { return status; }

private:
    SafeSeatNow *transport = nullptr;
    CameraRemoteStatus status{};

    unsigned long lastPacketMillis = 0;
    unsigned long requestStartMillis = 0;
    unsigned long lastTriggerSendMillis = 0;

    uint32_t nextRequestId = 1;
    uint32_t lastResultSequence = 0;

    bool hasAcceptedResult = false;
    CameraResultPacket acceptedResult{};
    unsigned long acceptedResultMillis = 0;

    void processStatusPacket(
        const CameraStatusPacket &packet,
        const uint8_t sourceMac[6]
    );

    void processResultPacket(
        const CameraResultPacket &packet,
        const uint8_t sourceMac[6]
    );

    void startRequest();
    void sendActiveRequest();
    void clearActiveRequest();
};
