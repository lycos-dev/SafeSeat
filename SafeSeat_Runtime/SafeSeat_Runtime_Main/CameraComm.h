#pragma once

#include <Arduino.h>

#include "CameraProtocol.h"
#include "Fusion.h"
#include "SafeSeatNow.h"

struct CameraRemoteStatus
{
    bool initialized = false;
    bool connected = false;
    bool stale = false;
    bool modelReady = false;
    bool cameraReady = false;
    bool psramReady = false;
    bool busy = false;
    bool baselineReady = false;
    bool baselineProvisional = false;
    bool calibrating = false;
    bool sessionActive = false;

    uint8_t channel = 0;
    uint8_t sourceMac[6]{};
    uint8_t calibrationCount = 0;
    uint8_t calibrationTarget = 5;

    uint32_t remoteSessionId = 0;
    uint32_t localSessionId = 0;
    uint32_t statusPacketsReceived = 0;
    uint32_t resultPacketsReceived = 0;
    uint32_t badPackets = 0;
    uint32_t ignoredResults = 0;
    uint32_t commandPacketsQueued = 0;
    uint32_t activeRequestId = 0;
    uint32_t lastHandledRequestId = 0;

    bool requestActive = false;
    bool localOccupancySessionActive = false;
    unsigned long requestAgeMillis = 0;
    unsigned long packetAgeMillis = 0;

    uint32_t lastResultRequestId = 0;
    CameraPostureClass lastPosture = CameraPostureClass::UNKNOWN;
    float lastConfidence = 0.0f;
    uint8_t lastValidFrames = 0;
    float lastIFScore = NAN;
    float lastOCSVMScore = NAN;
    unsigned long lastRemoteInferenceMillis = 0;
};

class CameraComm
{
public:
    bool begin();
    void update();

    // Occupancy lifecycle: stable entry creates one camera session and
    // requests upright calibration. Stable exit invalidates that baseline.
    void serviceOccupancySession(bool occupied);

    // Called after Fusion::update(). Starts one command-driven verification
    // transaction only when the current passenger baseline is ready.
    void serviceVerificationRequest(bool requested);

    CameraFusionEvidence getFusionEvidence() const;
    const CameraRemoteStatus &getStatus() const { return status; }

private:
    SafeSeatNow *transport = nullptr;
    CameraRemoteStatus status{};

    unsigned long lastPacketMillis = 0;
    unsigned long requestStartMillis = 0;
    unsigned long lastCommandSendMillis = 0;
    unsigned long lastRequestFinishedMillis = 0;

    uint32_t nextRequestId = 1;
    uint32_t sessionId = 0;

    bool occupancyCandidate = false;
    bool occupancyStable = false;
    unsigned long occupancyCandidateSince = 0;

    bool calibrationCommandPending = false;
    uint32_t calibrationRequestId = 0;
    bool resetCommandPending = false;
    uint32_t resetRequestId = 0;
    uint32_t closingSessionId = 0;

    bool hasAcceptedResult = false;
    CameraResultPacket acceptedResult{};
    unsigned long acceptedResultMillis = 0;

    void processStatusPacket(const CameraStatusPacket &packet, const uint8_t sourceMac[6]);
    void processResultPacket(const CameraResultPacket &packet, const uint8_t sourceMac[6]);

    uint32_t allocateRequestId();
    void startOccupancySession();
    void endOccupancySession();
    void serviceSessionCommands();
    bool sendCommand(CameraCommandType command, uint32_t requestId, uint32_t commandSessionId);

    void startVerificationRequest();
    void sendActiveVerificationRequest();
    void clearActiveRequest();
};
