#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "PiezoProtocol.h"
#include "SafeSeatNowProtocol.h"

struct SafeSeatNowStatus
{
    bool initialized = false;
    uint8_t channel = 0;
    uint32_t hubBeaconsSent = 0;
    uint32_t hubBeaconSendErrors = 0;
    uint32_t piezoPacketsQueued = 0;
};

class SafeSeatNow
{
public:
    static SafeSeatNow& instance();

    bool begin();
    void update();

    bool takeLatestPiezoPacket(
        PiezoWirePacket &packet,
        uint8_t sourceMac[6]
    );

    const SafeSeatNowStatus& getStatus() const
    {
        return status;
    }

private:
    SafeSeatNow() = default;

    static SafeSeatNow *activeInstance;
    static const uint8_t BROADCAST_MAC[6];

    SafeSeatNowStatus status{};
    unsigned long lastBeaconMillis = 0;

    volatile bool pendingPiezoReady = false;
    PiezoWirePacket pendingPiezoPacket{};
    uint8_t pendingPiezoMac[6]{};

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
