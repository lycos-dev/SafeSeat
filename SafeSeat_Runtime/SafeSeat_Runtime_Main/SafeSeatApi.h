#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "SafeSeatTelemetry.h"
#include "SafeSeatUatInjection.h"

// ============================================================
// SAFESEAT LOCAL TELEMETRY API + RESEARCHER UAT CONTROL — R5
//
// Main Hub address: http://192.168.4.1
//
// Canonical telemetry endpoints:
//   GET /api/v1/status
//   GET /api/v1/fusion
//   GET /api/v1/sensors
//   GET /api/v1/camera
//   GET /api/v1/network
//   GET /health
//
// Research / validation console:
//   GET  /uat
//   GET  /api/v1/uat/injection
//   POST /api/v1/uat/injection/start
//   POST /api/v1/uat/injection/update
//   POST /api/v1/uat/injection/stop
//
// Legacy direct-Warning simulation endpoints remain disabled (HTTP 410).
//
// IMPORTANT:
// - Fusion remains authoritative; no API endpoint writes WARNING/EMERGENCY.
// - Injected FSR/MLX/MPU values enter their deployed processing/model path.
// - C1001 injected HR/RR are sent to the remote C1001 node, whose deployed
//   30-second IF + OCSVM model still produces the evidence returned to Main.
// - Camera remains verification-only and event-triggered. A configured
//   synthetic UPRIGHT/NON_UPRIGHT result is held until Fusion itself requests
//   camera verification; it cannot create the original emergency candidate.
// - Stopping a test restores live sensor acquisition; reboot starts live.
// ============================================================

class SafeSeatApi
{
public:
    SafeSeatApi();

    bool begin(
        const SafeSeatTelemetry *telemetrySource
    );

    void update();

    bool isRunning() const
    {
        return running;
    }

private:
    WebServer server;
    const SafeSeatTelemetry *telemetry = nullptr;
    bool running = false;

    void registerRoutes();

    void handleRoot();
    void handleHealth();
    void handleUat();
    void handleUatStimulusStatus();
    void handleUatSimulateWarning();
    void handleUatClearSimulation();
    void handleUatInjectionStatus();
    void handleUatInjectionStart();
    void handleUatInjectionUpdate();
    void handleUatInjectionStop();
    void applyUatRequestArgs();
    void handleStatus();
    void handleFusion();
    void handleSensors();
    void handleCamera();
    void handleNetwork();
    void handleNotFound();

    void sendJson(
        int statusCode,
        const String &body
    );

    String buildHealthJson() const;
    String buildStatusJson() const;
    String buildFusionJson() const;
    String buildSensorsJson() const;
    String buildCameraJson() const;
    String buildNetworkJson() const;

    static void appendJsonBool(
        String &out,
        bool value
    );

    static void appendJsonFloat(
        String &out,
        float value,
        uint8_t decimals = 2
    );

    static void appendJsonString(
        String &out,
        const char *value
    );

    static void appendModelEvidence(
        String &out,
        const ModelEvidence &model
    );

    static void appendFloatArray(
        String &out,
        const float *values,
        size_t count,
        uint8_t decimals = 1
    );
};
