#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "C1001.h"
#include "C1001ML.h"
#include "C1001Protocol.h"
#include "SafeSeatNowProtocol.h"

class C1001Comm
{
public:
    C1001Comm() = default;

    bool begin();

    void update(
        const C1001Reading &sensorReading,
        const C1001MLReading &modelReading
    );

    unsigned long getPacketsSent() const { return packetsSent; }
    unsigned long getSendErrors() const { return sendErrors; }
    unsigned long getHubBeaconsReceived() const { return hubBeaconsReceived; }
    bool isHubLocked() const { return hubLocked; }
    uint8_t getChannel() const { return currentChannel; }
    unsigned long getBeaconAgeMillis() const;

private:
    static C1001Comm *activeInstance;
    static const uint8_t BROADCAST_MAC[6];

    bool initialized = false;
    unsigned long lastSendMillis = 0;
    unsigned long packetsSent = 0;
    unsigned long sendErrors = 0;
    uint32_t sequence = 0;

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
