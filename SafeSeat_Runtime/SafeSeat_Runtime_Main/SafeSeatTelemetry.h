#pragma once

#include <Arduino.h>

#include "Fusion.h"
#include "C1001Comm.h"
#include "PiezoComm.h"
#include "CameraComm.h"
#include "SafeSeatAccessPoint.h"

// ============================================================
// SAFESEAT TELEMETRY SNAPSHOT - STEP 5.9.8
//
// This is a read-only copy of the latest Main Hub state for the
// local API/frontend layer. It does not change sensor, Fusion,
// ESP-NOW, or camera behavior.
// ============================================================

struct SafeSeatTelemetrySnapshot
{
    bool ready = false;
    unsigned long capturedMillis = 0;

    FusionInput input{};
    FusionReading fusion{};

    C1001RemoteStatus c1001Link{};
    PiezoRemoteStatus piezoLink{};
    CameraRemoteStatus cameraLink{};
    SafeSeatAccessPointStatus network{};
};

class SafeSeatTelemetry
{
public:
    void capture(
        const FusionInput &input,
        const FusionReading &fusionReading,
        const C1001RemoteStatus &c1001Status,
        const PiezoRemoteStatus &piezoStatus,
        const CameraRemoteStatus &cameraStatus,
        const SafeSeatAccessPointStatus &networkStatus
    );

    const SafeSeatTelemetrySnapshot &getSnapshot() const
    {
        return snapshot;
    }

private:
    SafeSeatTelemetrySnapshot snapshot{};
};
