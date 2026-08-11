#include "SafeSeatTelemetry.h"

void SafeSeatTelemetry::capture(
    const FusionInput &input,
    const FusionReading &fusionReading,
    const C1001RemoteStatus &c1001Status,
    const PiezoRemoteStatus &piezoStatus,
    const CameraRemoteStatus &cameraStatus,
    const SafeSeatAccessPointStatus &networkStatus
)
{
    snapshot.ready = true;
    snapshot.capturedMillis = millis();

    snapshot.input = input;
    snapshot.fusion = fusionReading;

    snapshot.c1001Link = c1001Status;
    snapshot.piezoLink = piezoStatus;
    snapshot.cameraLink = cameraStatus;
    snapshot.network = networkStatus;
}
