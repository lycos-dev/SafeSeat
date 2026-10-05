#pragma once

#include <Arduino.h>

#include "Fusion.h"
#include "C1001Comm.h"
#include "CameraComm.h"
#include "SafeSeatAccessPoint.h"
#include "FSRML.h"
#include "MLXML.h"
#include "MPUML.h"

// ============================================================
// SAFESEAT TELEMETRY SNAPSHOT - STEP 5.9.9-R5
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
    FSRMLReading fsrMl{};
    MLXMLReading mlxMl{};
    MPUMLReading mpuMl{};
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
        const FSRMLReading &fsrMlReading,
        const MLXMLReading &mlxMlReading,
        const MPUMLReading &mpuMlReading,
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
