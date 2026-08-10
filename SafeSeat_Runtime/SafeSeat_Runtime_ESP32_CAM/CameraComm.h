#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "CameraProtocol.h"

struct CameraNodeLinkStatus
{
    bool initialized = false;
    bool hubLocked = false;
    uint8_t channel = 0;
    uint32_t hubBeaconsReceived = 0;
    uint32_t statusPacketsSent = 0;
    uint32_t resultsSent = 0;
    uint32_t sendErrors = 0;
    uint32_t triggerPacketsReceived = 0;
    uint32_t duplicateTriggersIgnored = 0;
    unsigned long lastHubBeaconMillis = 0;
    uint8_t hubMac[6]{};
};

class CameraComm
{
public:
    bool begin();
    void update(
        bool modelReady,
        bool cameraReady,
        bool psramReady,
        bool busy,
        unsigned long lastInferenceMillis,
        uint32_t lastHandledRequestId
    );

    bool takeTrigger(CameraTriggerPacket &trigger);
    bool sendResult(CameraResultPacket packet);

    const CameraNodeLinkStatus &getStatus() const { return status; }

private:
    static CameraComm *activeInstance;
    static const uint8_t BROADCAST_MAC[6];

    CameraNodeLinkStatus status{};

    volatile bool pendingTriggerReady = false;
    CameraTriggerPacket pendingTrigger{};

    uint32_t statusSequence = 0;
    uint32_t lastAcceptedTriggerId = 0;
    unsigned long lastStatusMillis = 0;
    unsigned long lastScanStepMillis = 0;
    uint8_t nextScanChannel = 0;

    bool ensureBroadcastPeer();
    void setChannel(uint8_t channel);
    void serviceChannelDiscovery();
    void sendStatus(
        bool modelReady,
        bool cameraReady,
        bool psramReady,
        bool busy,
        unsigned long lastInferenceMillis,
        uint32_t lastHandledRequestId
    );

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
