#pragma once

#include <Arduino.h>
#include <WiFi.h>

struct SafeSeatWiFiStatus
{
    bool enabled = false;
    bool connecting = false;
    bool connected = false;
    uint32_t attempts = 0;
    uint32_t successfulConnections = 0;
    unsigned long lastAttemptMillis = 0;
    unsigned long connectedSinceMillis = 0;
};

class SafeSeatWiFi
{
public:
    void begin();
    void update();

    bool connected() const { return status.connected; }
    bool connecting() const { return status.connecting; }
    bool radioOwnsChannel() const { return status.connected || status.connecting; }
    const SafeSeatWiFiStatus &getStatus() const { return status; }

private:
    SafeSeatWiFiStatus status{};
    unsigned long attemptStartMillis = 0;
    bool firstAttemptPending = true;

    void startAttempt();
    void stopAttempt();
};
