#include "SafeSeatTelemetry.h"

void SafeSeatTelemetry::capture(
    const FusionInput &input,
    const FusionReading &fusionReading,
    const C1001RemoteStatus &c1001Status,
    const FSRMLReading &fsrMlReading,
    const MLXMLReading &mlxMlReading,
    const MPUMLReading &mpuMlReading,
    const CameraRemoteStatus &cameraStatus,
    const SafeSeatAccessPointStatus &networkStatus
)
{
    snapshot.ready = true;
    snapshot.capturedMillis = millis();

    snapshot.input = input;
    snapshot.fusion = fusionReading;

    snapshot.c1001Link = c1001Status;
    snapshot.fsrMl = fsrMlReading;
    snapshot.mlxMl = mlxMlReading;
    snapshot.mpuMl = mpuMlReading;
    snapshot.cameraLink = cameraStatus;
    snapshot.network = networkStatus;
}
