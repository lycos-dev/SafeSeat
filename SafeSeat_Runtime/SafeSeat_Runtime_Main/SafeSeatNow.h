#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "C1001Protocol.h"
#include "CameraProtocol.h"
#include "SafeSeatNowProtocol.h"

struct SafeSeatNowStatus
{
    bool initialized = false;
    uint8_t channel = 0;
    uint32_t hubBeaconsSent = 0;
    uint32_t hubBeaconSendErrors = 0;
    uint32_t c1001PacketsQueued = 0;
    uint32_t cameraStatusPacketsQueued = 0;
    uint32_t cameraResultPacketsQueued = 0;
    uint32_t cameraTriggersSent = 0;
    uint32_t cameraTriggerSendErrors = 0;
    uint32_t unknownPacketsIgnored = 0;
};

class SafeSeatNow
{
public:
    static SafeSeatNow& instance();

    bool begin();
    void update();


    bool takeLatestC1001Packet(
        C1001WirePacket &packet,
        uint8_t sourceMac[6]
    );

    bool takeLatestCameraStatus(
        CameraStatusPacket &packet,
        uint8_t sourceMac[6]
    );

    bool takeLatestCameraResult(
        CameraResultPacket &packet,
        uint8_t sourceMac[6]
    );

    bool sendCameraTrigger(
        const CameraTriggerPacket &packet
    );

    const SafeSeatNowStatus& getStatus() const { return status; }

private:
    SafeSeatNow() = default;

    static SafeSeatNow *activeInstance;
    static const uint8_t BROADCAST_MAC[6];

    SafeSeatNowStatus status{};
    unsigned long lastBeaconMillis = 0;


    volatile bool pendingC1001Ready = false;
    C1001WirePacket pendingC1001Packet{};
    uint8_t pendingC1001Mac[6]{};

    volatile bool pendingCameraStatusReady = false;
    CameraStatusPacket pendingCameraStatus{};
    uint8_t pendingCameraStatusMac[6]{};

    volatile bool pendingCameraResultReady = false;
    CameraResultPacket pendingCameraResult{};
    uint8_t pendingCameraResultMac[6]{};

    bool ensureBroadcastPeer();
    void sendHubBeacon();

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
