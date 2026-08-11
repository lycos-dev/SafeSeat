#pragma once

#include <Arduino.h>
#include <WiFi.h>

struct SafeSeatAccessPointStatus
{
    bool initialized = false;
    bool running = false;
    uint8_t channel = 0;
    uint8_t connectedClients = 0;
    IPAddress ipAddress{};
};

class SafeSeatAccessPoint
{
public:
    bool begin();
    void update();

    const SafeSeatAccessPointStatus& getStatus() const
    {
        return status;
    }

private:
    SafeSeatAccessPointStatus status{};
};
