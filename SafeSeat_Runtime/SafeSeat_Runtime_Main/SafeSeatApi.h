#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "SafeSeatTelemetry.h"

// ============================================================
// SAFESEAT LOCAL TELEMETRY API + RESEARCHER UAT CONTROL
//
// Main Hub address: http://192.168.4.1
//
// Canonical endpoints:
//   GET /api/v1/status
//   GET /api/v1/fusion
//   GET /api/v1/sensors
//   GET /api/v1/camera
//   GET /api/v1/network
//   GET /health
//   GET /uat  (UAT evaluator browser monitor + controlled Warning stimulus)
//   GET  /api/v1/uat/stimulus
//   POST /api/v1/uat/simulate-warning
//   POST /api/v1/uat/clear-simulation
//
// Short aliases are retained for development convenience:
//   /status, /sensors, /camera, /network
//
// IMPORTANT:
// - Telemetry endpoints expose Main/Fusion state only.
// - Fusion remains authoritative; the API never writes a Fusion result directly.
// - /uat exposes one researcher-only CONTROLLED WARNING stimulus for UAT.
// - The UAT stimulus injects the already validated SYS02 one-strong-vote path.
// - No web endpoint can inject EMERGENCY or directly trigger/cancel camera/alerts.
// - No Firebase/backend technology is assumed here.
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
