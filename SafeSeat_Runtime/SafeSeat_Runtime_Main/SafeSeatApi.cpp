#include "SafeSeatApi.h"

#include <math.h>

#include "NetworkConfig.h"
#include "CameraProtocol.h"
#include "SafeSeatUatPage.h"

namespace
{
constexpr char API_SCHEMA_VERSION[] = "1.0.0";
constexpr char API_DEVICE_NAME[] = "SafeSeat Main Hub";
}

SafeSeatApi::SafeSeatApi()
    : server(80)
{
}

bool SafeSeatApi::begin(
    const SafeSeatTelemetry *telemetrySource
)
{
    if (telemetrySource == nullptr)
    {
        Serial.println("[API] ERROR: telemetry source is null.");
        return false;
    }

    telemetry = telemetrySource;

    registerRoutes();
    server.begin();
    running = true;

    Serial.println("[API] SafeSeat local telemetry API started.");
    Serial.println("[API] Base URL : http://192.168.4.1");
    Serial.println("[API] Status   : /api/v1/status");
    Serial.println("[API] Fusion   : /api/v1/fusion");
    Serial.println("[API] Sensors  : /api/v1/sensors");
    Serial.println("[API] Camera   : /api/v1/camera");
    Serial.println("[API] Network  : /api/v1/network");
    Serial.println("[API] Health   : /health");
    Serial.println("[API] UAT      : /uat (live monitor + sensor-value injection)");
    Serial.println("[API] UAT API  : /api/v1/uat/injection");

    return true;
}

void SafeSeatApi::update()
{
    if (!running)
    {
        return;
    }

    server.handleClient();
}

void SafeSeatApi::registerRoutes()
{
    server.on("/", HTTP_GET, [this]() { handleRoot(); });
    server.on("/health", HTTP_GET, [this]() { handleHealth(); });
    server.on("/uat", HTTP_GET, [this]() { handleUat(); });
    server.on("/maintenance", HTTP_GET, [this]() { handleUat(); });

    // Legacy UAT endpoints are retained only so old bookmarks/tools fail
    // explicitly. They never inject UAT state directly into Fusion.
    server.on("/api/v1/uat/stimulus", HTTP_GET, [this]() { handleUatStimulusStatus(); });
    server.on("/api/v1/uat/simulate-warning", HTTP_POST, [this]() { handleUatSimulateWarning(); });
    server.on("/api/v1/uat/clear-simulation", HTTP_POST, [this]() { handleUatClearSimulation(); });

    server.on("/api/v1/uat/injection", HTTP_GET, [this]() { handleUatInjectionStatus(); });
    server.on("/api/v1/uat/injection/start", HTTP_POST, [this]() { handleUatInjectionStart(); });
    server.on("/api/v1/uat/injection/update", HTTP_POST, [this]() { handleUatInjectionUpdate(); });
    server.on("/api/v1/uat/injection/stop", HTTP_POST, [this]() { handleUatInjectionStop(); });

    server.on("/api/v1/status", HTTP_GET, [this]() { handleStatus(); });
    server.on("/api/v1/fusion", HTTP_GET, [this]() { handleFusion(); });
    server.on("/api/v1/sensors", HTTP_GET, [this]() { handleSensors(); });
    server.on("/api/v1/camera", HTTP_GET, [this]() { handleCamera(); });
    server.on("/api/v1/network", HTTP_GET, [this]() { handleNetwork(); });

    // Development aliases. The versioned endpoints above are the
    // contract the frontend should ultimately target.
    server.on("/status", HTTP_GET, [this]() { handleStatus(); });
    server.on("/fusion", HTTP_GET, [this]() { handleFusion(); });
    server.on("/sensors", HTTP_GET, [this]() { handleSensors(); });
    server.on("/camera", HTTP_GET, [this]() { handleCamera(); });
    server.on("/network", HTTP_GET, [this]() { handleNetwork(); });

    server.onNotFound([this]() { handleNotFound(); });
}

void SafeSeatApi::handleRoot()
{
    static const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SafeSeat Main Hub API</title>
<style>
body{font-family:Arial,sans-serif;max-width:760px;margin:40px auto;padding:0 18px;line-height:1.5}
code{background:#eee;padding:2px 5px;border-radius:4px}
a{display:block;margin:8px 0}
</style>
</head>
<body>
<h1>SafeSeat Main Hub</h1>
<p>Local telemetry API is running.</p>
<p>The Main Hub Fusion state is authoritative. Telemetry is read-only; /uat additionally provides an explicit research sensor-value injection console.</p>
<a href="/api/v1/status">/api/v1/status</a>
<a href="/api/v1/fusion">/api/v1/fusion</a>
<a href="/api/v1/sensors">/api/v1/sensors</a>
<a href="/api/v1/camera">/api/v1/camera</a>
<a href="/api/v1/network">/api/v1/network</a>
<a href="/health">/health</a>
<a href="/uat">/uat — research & validation console</a>
</body>
</html>
)rawliteral";

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/html", PAGE);
}

void SafeSeatApi::handleHealth()
{
    sendJson(200, buildHealthJson());
}

void SafeSeatApi::handleUat()
{
    // Research & validation console. Sensor-value injection is explicitly
    // labeled TEST MODE and never writes a Fusion result directly.
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", SAFESEAT_UAT_PAGE);
}

void SafeSeatApi::handleUatStimulusStatus()
{
    sendJson(
        200,
        F("{\"ok\":true,\"mode\":\"MAINTENANCE_ONLY\",\"hub_simulation_enabled\":false,\"fusion_authoritative\":true}")
    );
}

void SafeSeatApi::handleUatSimulateWarning()
{
    sendJson(
        410,
        F("{\"ok\":false,\"error\":\"direct_warning_simulation_disabled\",\"hint\":\"Direct Warning forcing is disabled. Use /uat sensor-value injection so the real model and Fusion path decides the state.\"}")
    );
}

void SafeSeatApi::handleUatClearSimulation()
{
    sendJson(
        410,
        F("{\"ok\":false,\"error\":\"direct_warning_simulation_disabled\",\"hint\":\"Direct Warning forcing is disabled. Stop a value-injection test with POST /api/v1/uat/injection/stop.\"}")
    );
}


void SafeSeatApi::applyUatRequestArgs()
{
    SafeSeatUatInjection &uat = SafeSeatUatInjection::instance();
    const SafeSeatUatInjectionState s = uat.getState();

    auto readBool = [this](const char *name, bool fallback) -> bool
    {
        if (!server.hasArg(name)) return fallback;
        String v = server.arg(name);
        v.toLowerCase();
        return v == "1" || v == "true" || v == "yes" || v == "on";
    };

    auto readFloat = [this](const char *name, float fallback, float minValue, float maxValue) -> float
    {
        if (!server.hasArg(name)) return fallback;
        const float value = server.arg(name).toFloat();
        if (!isfinite(value)) return fallback;
        return constrain(value, minValue, maxValue);
    };

    auto readInt = [this](const char *name, int fallback, int minValue, int maxValue) -> int
    {
        if (!server.hasArg(name)) return fallback;
        const int value = server.arg(name).toInt();
        return constrain(value, minValue, maxValue);
    };

    uat.setInjectFsr(readBool("use_fsr", s.injectFsr));
    uat.setInjectMlx(readBool("use_mlx", s.injectMlx));
    uat.setInjectMpu(readBool("use_mpu", s.injectMpu));
    uat.setInjectC1001(readBool("use_c1001", s.injectC1001));

    for (uint8_t i = 0; i < NUM_FSR; ++i)
    {
        const String name = String("fsr") + String(i + 1);
        uat.setFsrPressure(
            i,
            readFloat(name.c_str(), s.fsrPressure[i], 0.0f, 26000.0f)
        );
    }

    uat.setMlx(
        readFloat("mlx_ambient", s.mlxAmbientC, -40.0f, 85.0f),
        readFloat("mlx_object", s.mlxObjectC, -40.0f, 125.0f)
    );

    uat.setMpu(
        readFloat("mpu_ax", s.mpuAccelX, -16.0f, 16.0f),
        readFloat("mpu_ay", s.mpuAccelY, -16.0f, 16.0f),
        readFloat("mpu_az", s.mpuAccelZ, -16.0f, 16.0f),
        readFloat("mpu_gx", s.mpuGyroX, -2000.0f, 2000.0f),
        readFloat("mpu_gy", s.mpuGyroY, -2000.0f, 2000.0f),
        readFloat("mpu_gz", s.mpuGyroZ, -2000.0f, 2000.0f)
    );

    uat.setC1001(
        readBool("c_present", s.c1001Present),
        readInt("c_hr", s.c1001HeartRate, 1, 254),
        readInt("c_rr", s.c1001Respiration, 1, 254),
        readInt("c_motion", s.c1001Motion, -32768, 32767),
        readInt("c_move_range", s.c1001MoveRange, -32768, 32767)
    );

    if (server.hasArg("camera_mode"))
    {
        String mode = server.arg("camera_mode");
        mode.toLowerCase();
        if (mode == "upright")
            uat.setCameraMode(SafeSeatUatCameraMode::INJECT_UPRIGHT);
        else if (mode == "non_upright" || mode == "non-upright")
            uat.setCameraMode(SafeSeatUatCameraMode::INJECT_NON_UPRIGHT);
        else
            uat.setCameraMode(SafeSeatUatCameraMode::REAL_CAMERA);
    }
}

void SafeSeatApi::handleUatInjectionStatus()
{
    const SafeSeatUatInjectionState &s = SafeSeatUatInjection::instance().getState();
    String out;
    out.reserve(1800);
    out += F("{\"ok\":true,\"active\":");
    appendJsonBool(out, s.active);
    out += F(",\"session_id\":"); out += String(s.sessionId);
    out += F(",\"revision\":"); out += String(s.revision);
    out += F(",\"elapsed_ms\":"); out += String(s.active ? millis() - s.startedMillis : 0UL);
    out += F(",\"fusion_authoritative\":true,\"direct_warning_write\":false");
    out += F(",\"demo_policy\":{\"mlx_rapid_transition_bypass\":");
    appendJsonBool(out, s.active && s.injectMlx);
    out += F(",\"object_ta_fusion_gate\":false,\"baseline_and_ml_still_active\":true}");
    out += F(",\"sources\":{");
    out += F("\"fsr\":"); appendJsonBool(out, s.injectFsr);
    out += F(",\"mlx\":"); appendJsonBool(out, s.injectMlx);
    out += F(",\"mpu\":"); appendJsonBool(out, s.injectMpu);
    out += F(",\"c1001\":"); appendJsonBool(out, s.injectC1001);
    out += F("}");
    out += F(",\"fsr_pressure\":"); appendFloatArray(out, s.fsrPressure, NUM_FSR, 1);
    out += F(",\"mlx\":{\"ambient_c\":"); appendJsonFloat(out, s.mlxAmbientC, 2);
    out += F(",\"object_c\":"); appendJsonFloat(out, s.mlxObjectC, 2); out += F("}");
    out += F(",\"c1001\":{\"present\":"); appendJsonBool(out, s.c1001Present);
    out += F(",\"heart_rate\":"); out += String(s.c1001HeartRate);
    out += F(",\"respiration\":"); out += String(s.c1001Respiration);
    out += F(",\"motion\":"); out += String(s.c1001Motion);
    out += F(",\"move_range\":"); out += String(s.c1001MoveRange);
    out += F(",\"processing\":\"remote_node_real_ml\"}");
    out += F(",\"camera\":{\"mode\":");
    appendJsonString(out,
        s.cameraMode == SafeSeatUatCameraMode::INJECT_UPRIGHT ? "upright" :
        s.cameraMode == SafeSeatUatCameraMode::INJECT_NON_UPRIGHT ? "non_upright" : "real");
    out += F(",\"verification_only\":true,\"event_triggered\":true}");
    out += F("}");
    sendJson(200, out);
}

void SafeSeatApi::handleUatInjectionStart()
{
    applyUatRequestArgs();
    SafeSeatUatInjection::instance().start();
    handleUatInjectionStatus();
}

void SafeSeatApi::handleUatInjectionUpdate()
{
    applyUatRequestArgs();
    handleUatInjectionStatus();
}

void SafeSeatApi::handleUatInjectionStop()
{
    SafeSeatUatInjection::instance().stop();
    handleUatInjectionStatus();
}


void SafeSeatApi::handleStatus()
{
    sendJson(200, buildStatusJson());
}

void SafeSeatApi::handleFusion()
{
    sendJson(200, buildFusionJson());
}

void SafeSeatApi::handleSensors()
{
    sendJson(200, buildSensorsJson());
}

void SafeSeatApi::handleCamera()
{
    sendJson(200, buildCameraJson());
}

void SafeSeatApi::handleNetwork()
{
    sendJson(200, buildNetworkJson());
}

void SafeSeatApi::handleNotFound()
{
    String out;
    out.reserve(180);
    out += F("{\"error\":\"not_found\",\"path\":");
    appendJsonString(out, server.uri().c_str());
    out += F(",\"hint\":\"Use /api/v1/status or /health\"}");
    sendJson(404, out);
}

void SafeSeatApi::sendJson(
    int statusCode,
    const String &body
)
{
    // Useful for a future local browser/web frontend. Native mobile
    // apps do not require CORS, but allowing read-only GET access here
    // keeps the transport layer frontend-agnostic.
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("X-SafeSeat-API-Version", API_SCHEMA_VERSION);
    server.send(statusCode, "application/json", body);
}

String SafeSeatApi::buildHealthJson() const
{
    String out;
    out.reserve(320);

    const bool telemetryReady =
        telemetry != nullptr
        && telemetry->getSnapshot().ready;

    out += F("{\"ok\":true,\"service\":");
    appendJsonString(out, API_DEVICE_NAME);
    out += F(",\"api_version\":");
    appendJsonString(out, API_SCHEMA_VERSION);
    out += F(",\"uptime_ms\":");
    out += String(millis());
    out += F(",\"telemetry_ready\":");
    appendJsonBool(out, telemetryReady);
    out += F(",\"telemetry_read_only\":true,\"uat_injection_available\":true}");

    return out;
}

String SafeSeatApi::buildFusionJson() const
{
    String out;
    out.reserve(1800);

    if (telemetry == nullptr || !telemetry->getSnapshot().ready)
    {
        out = F("{\"schema_version\":\"1.0.0\",\"telemetry_ready\":false}");
        return out;
    }

    const SafeSeatTelemetrySnapshot &s = telemetry->getSnapshot();
    const FusionReading &f = s.fusion;

    out += F("{\"schema_version\":");
    appendJsonString(out, API_SCHEMA_VERSION);
    out += F(",\"telemetry_ready\":true");
    out += F(",\"timestamp_ms\":");
    out += String(s.capturedMillis);
    out += F(",\"uptime_ms\":");
    out += String(millis());
    out += F(",\"system\":{");
    out += F("\"fusion_authoritative\":true");
    out += F(",\"fusion_valid\":");
    appendJsonBool(out, f.valid);
    out += F(",\"fusion_state\":");
    appendJsonString(out, FusionEngine::getLevelText(f.level));
    out += F(",\"confidence\":");
    appendJsonFloat(out, f.confidence, 3);
    out += F(",\"emergency_active\":");
    appendJsonBool(out, f.level == FusionLevel::EMERGENCY);
    out += F(",\"camera_verification_requested\":");
    appendJsonBool(out, f.triggerCamera);
    out += F(",\"alert_requested\":");
    appendJsonBool(out, f.triggerAlert);
    out += F(",\"occupancy\":");
    appendJsonString(out, FusionEngine::getOccupancyText(f.occupancy));
    out += F(",\"motion_context\":");
    appendJsonString(out, FusionEngine::getMotionText(f.motion));
    out += F(",\"vitals_state\":");
    appendJsonString(out, FusionEngine::getVitalsText(f.vitals));
    out += F(",\"pressure_state\":");
    appendJsonString(out, FusionEngine::getPressureText(f.pressure));
    out += F(",\"temperature_state\":");
    appendJsonString(out, FusionEngine::getTemperatureText(f.temperature));
    out += F(",\"respiration_state\":");
    appendJsonString(out, FusionEngine::getRespirationText(f.respiration));
    out += F(",\"evidence\":{");
    out += F("\"valid_sensor_count\":");
    out += String(f.evidence.validSensorCount);
    out += F(",\"unavailable_sensor_count\":");
    out += String(f.evidence.unavailableSensorCount);
    out += F(",\"anomaly_evidence_count\":");
    out += String(f.evidence.anomalyEvidenceCount);
    out += F(",\"strong_anomaly_evidence_count\":");
    out += String(f.evidence.strongAnomalyEvidenceCount);
    out += F(",\"normal_evidence_count\":");
    out += String(f.evidence.normalEvidenceCount);
    out += F(",\"supporting_context_count\":");
    out += String(f.evidence.supportingContextCount);
    out += F(",\"motion_artifact_possible\":");
    appendJsonBool(out, f.evidence.motionArtifactPossible);
    out += F(",\"multi_sensor_agreement\":");
    appendJsonBool(out, f.evidence.multiSensorAgreement);
    out += F("}}}");

    return out;
}

String SafeSeatApi::buildStatusJson() const
{
    String out;
    out.reserve(8500);

    if (telemetry == nullptr || !telemetry->getSnapshot().ready)
    {
        out = F("{\"schema_version\":\"1.0.0\",\"telemetry_ready\":false}");
        return out;
    }

    const SafeSeatTelemetrySnapshot &s = telemetry->getSnapshot();
    const FusionInput &in = s.input;
    const FusionReading &f = s.fusion;

    out += F("{\"schema_version\":");
    appendJsonString(out, API_SCHEMA_VERSION);
    out += F(",\"device\":");
    appendJsonString(out, API_DEVICE_NAME);
    out += F(",\"telemetry_ready\":true");
    out += F(",\"timestamp_ms\":");
    out += String(s.capturedMillis);
    out += F(",\"uptime_ms\":");
    out += String(millis());

    // --------------------------------------------------------
    // System / Fusion
    // --------------------------------------------------------
    out += F(",\"system\":{");
    out += F("\"fusion_authoritative\":true");
    out += F(",\"fusion_valid\":");
    appendJsonBool(out, f.valid);
    out += F(",\"fusion_state\":");
    appendJsonString(out, FusionEngine::getLevelText(f.level));
    out += F(",\"confidence\":");
    appendJsonFloat(out, f.confidence, 3);
    out += F(",\"emergency_active\":");
    appendJsonBool(out, f.level == FusionLevel::EMERGENCY);
    out += F(",\"camera_verification_requested\":");
    appendJsonBool(out, f.triggerCamera);
    out += F(",\"alert_requested\":");
    appendJsonBool(out, f.triggerAlert);

    out += F(",\"occupancy\":");
    appendJsonString(out, FusionEngine::getOccupancyText(f.occupancy));
    out += F(",\"motion_context\":");
    appendJsonString(out, FusionEngine::getMotionText(f.motion));
    out += F(",\"vitals_state\":");
    appendJsonString(out, FusionEngine::getVitalsText(f.vitals));
    out += F(",\"pressure_state\":");
    appendJsonString(out, FusionEngine::getPressureText(f.pressure));
    out += F(",\"temperature_state\":");
    appendJsonString(out, FusionEngine::getTemperatureText(f.temperature));
    out += F(",\"respiration_state\":");
    appendJsonString(out, FusionEngine::getRespirationText(f.respiration));

    out += F(",\"evidence\":{");
    out += F("\"valid_sensor_count\":");
    out += String(f.evidence.validSensorCount);
    out += F(",\"unavailable_sensor_count\":");
    out += String(f.evidence.unavailableSensorCount);
    out += F(",\"anomaly_evidence_count\":");
    out += String(f.evidence.anomalyEvidenceCount);
    out += F(",\"strong_anomaly_evidence_count\":");
    out += String(f.evidence.strongAnomalyEvidenceCount);
    out += F(",\"normal_evidence_count\":");
    out += String(f.evidence.normalEvidenceCount);
    out += F(",\"supporting_context_count\":");
    out += String(f.evidence.supportingContextCount);
    out += F(",\"motion_artifact_possible\":");
    appendJsonBool(out, f.evidence.motionArtifactPossible);
    out += F(",\"multi_sensor_agreement\":");
    appendJsonBool(out, f.evidence.multiSensorAgreement);
    out += F("}}");

    // --------------------------------------------------------
    // Network
    // --------------------------------------------------------
    out += F(",\"network\":");
    out += buildNetworkJson();

    // --------------------------------------------------------
    // Sensors
    // --------------------------------------------------------
    out += F(",\"sensors\":");
    out += buildSensorsJson();

    // --------------------------------------------------------
    // Camera
    // --------------------------------------------------------
    out += F(",\"camera\":");
    out += buildCameraJson();

    out += F("}");

    (void)in; // in is used indirectly in nested builders through snapshot.
    return out;
}

String SafeSeatApi::buildNetworkJson() const
{
    String out;
    out.reserve(520);

    if (telemetry == nullptr || !telemetry->getSnapshot().ready)
    {
        out = F("{\"telemetry_ready\":false}");
        return out;
    }

    const SafeSeatTelemetrySnapshot &s = telemetry->getSnapshot();

    out += F("{\"ap_running\":");
    appendJsonBool(out, s.network.running);
    out += F(",\"ssid\":");
    appendJsonString(out, SAFESEAT_AP_SSID);
    out += F(",\"ip\":");
    appendJsonString(out, s.network.ipAddress.toString().c_str());
    out += F(",\"channel\":");
    out += String(s.network.channel);
    out += F(",\"connected_clients\":");
    out += String(s.network.connectedClients);
    out += F(",\"local_only\":true");
    out += F(",\"internet_required\":false");
    out += F(",\"esp_now_enabled\":true}");

    return out;
}

String SafeSeatApi::buildSensorsJson() const
{
    String out;
    out.reserve(6500);

    if (telemetry == nullptr || !telemetry->getSnapshot().ready)
    {
        out = F("{\"telemetry_ready\":false}");
        return out;
    }

    const SafeSeatTelemetrySnapshot &s = telemetry->getSnapshot();
    const FusionInput &in = s.input;

    // --------------------------------------------------------
    // C1001
    // --------------------------------------------------------
    out += F("{\"c1001\":{");
    out += F("\"health\":");
    appendJsonString(out, FusionEngine::getSensorHealthText(in.c1001.health));
    out += F(",\"connected\":");
    appendJsonBool(out, s.c1001Link.connected);
    out += F(",\"stale\":");
    appendJsonBool(out, s.c1001Link.stale);
    out += F(",\"packet_age_ms\":");
    out += String(s.c1001Link.packetAgeMillis);
    out += F(",\"packets_received\":");
    out += String(s.c1001Link.packetsReceived);
    out += F(",\"present\":");
    appendJsonBool(out, in.c1001.reading.present);
    out += F(",\"status\":");
    appendJsonString(out, c1001StatusText(in.c1001.reading.status));
    out += F(",\"trusted_vitals\":");
    appendJsonBool(out, in.c1001.reading.trustedVitalsAvailable);
    out += F(",\"heart_rate_bpm\":");
    appendJsonFloat(out, in.c1001.reading.filteredHeartRate, 1);
    out += F(",\"respiration_rate_bpm\":");
    appendJsonFloat(out, in.c1001.reading.filteredRespiration, 1);
    out += F(",\"motion\":");
    out += String(in.c1001.reading.motion);
    out += F(",\"move_range\":");
    out += String(in.c1001.reading.moveRange);
    out += F(",\"motion_artifact_active\":");
    appendJsonBool(out, in.c1001.reading.motionArtifactActive);
    out += F(",\"model\":");
    appendModelEvidence(out, in.c1001.model);
    out += F(",\"model_runtime\":{\"status\":");
    appendJsonString(out, c1001RemoteMLStatusShort(s.c1001Link.remoteMLStatus));
    out += F(",\"window_samples_collected\":"); out += String(s.c1001Link.remoteWindowSamplesCollected);
    out += F(",\"window_samples_required\":"); out += String(s.c1001Link.remoteWindowSamplesRequired);
    out += F(",\"samples_until_next_inference\":"); out += String(s.c1001Link.remoteSamplesUntilNextInference);
    out += F(",\"windows_evaluated\":"); out += String(s.c1001Link.remoteWindowsEvaluated);
    out += F("}}");

    // --------------------------------------------------------
    // MLX90614
    // --------------------------------------------------------
    out += F(",\"mlx90614\":{");
    out += F("\"health\":");
    appendJsonString(out, FusionEngine::getSensorHealthText(in.mlx.health));
    out += F(",\"connected\":");
    appendJsonBool(out, in.mlx.reading.connected);
    out += F(",\"valid\":");
    appendJsonBool(out, in.mlx.reading.valid);
    out += F(",\"target_visible\":");
    appendJsonBool(out, in.mlx.targetVisible);
    out += F(",\"fusion_contribution_suspended\":");
    appendJsonBool(out, in.mlx.contributionSuspended);
    out += F(",\"target_reacquiring\":");
    appendJsonBool(out, in.mlx.reacquiringTarget);
    out += F(",\"object_temperature_c\":");
    appendJsonFloat(out, in.mlx.reading.filteredObjectC, 2);
    out += F(",\"sensor_ta_c\":");
    appendJsonFloat(out, in.mlx.reading.filteredAmbientC, 2);
    out += F(",\"object_minus_ta_c\":");
    appendJsonFloat(out, in.mlx.reading.objectMinusAmbientC, 2);
    out += F(",\"context\":{");
    out += F("\"available\":");
    appendJsonBool(out, in.mlx.context.available);
    out += F(",\"thermal_target_qualified\":");
    appendJsonBool(out, in.mlx.context.thermalContrastQualified);
    out += F(",\"target_contrast_degraded\":");
    appendJsonBool(out, in.mlx.context.targetContrastDegraded);
    out += F(",\"low_thermal_contrast_info_only\":");
    appendJsonBool(out, in.mlx.context.lowThermalContrast);
    out += F(",\"low_contrast_samples\":");
    out += String(in.mlx.context.lowContrastSamples);
    out += F(",\"target_losses\":");
    out += String(in.mlx.context.targetLosses);
    out += F(",\"baseline_ready\":");
    appendJsonBool(out, in.mlx.context.baselineReady);
    out += F(",\"context_change\":");
    appendJsonBool(out, in.mlx.context.contextChange);
    out += F(",\"baseline_object_c\":");
    appendJsonFloat(out, in.mlx.context.baselineObjectC, 2);
    out += F(",\"deviation_from_baseline_c\":");
    appendJsonFloat(out, in.mlx.context.deviationFromBaselineC, 2);
    out += F("}");
    out += F(",\"native_mlx_model\":");
    appendModelEvidence(out, in.mlx.model);
    out += F(",\"model_runtime\":{\"baseline_blocks_collected\":"); out += String(s.mlxMl.baselineBlocksCollected);
    out += F(",\"baseline_blocks_required\":"); out += String(s.mlxMl.baselineBlocksRequired);
    out += F(",\"evaluated_blocks\":"); out += String(s.mlxMl.evaluatedBlocks);
    out += F(",\"anomaly_candidate_blocks\":"); out += String(s.mlxMl.anomalyCandidateBlocks);
    out += F(",\"geometry_degraded\":"); appendJsonBool(out, s.mlxMl.geometryDegraded);
    out += F(",\"reacquiring\":"); appendJsonBool(out, s.mlxMl.reacquiring);
    out += F("}");
    out += F(",\"native_mlx_model_fusion_role\":");
    appendJsonString(
        out,
        in.mlx.contributionSuspended
            ? "temporarily_suspended_target_unavailable"
            : "active_conservative_evidence"
    );
    out += F("}");

    // --------------------------------------------------------
    // FSR
    // --------------------------------------------------------
    out += F(",\"fsr\":{");
    out += F("\"health\":");
    appendJsonString(out, FusionEngine::getSensorHealthText(in.fsr.health));
    out += F(",\"connected\":");
    appendJsonBool(out, in.fsr.reading.connected);
    out += F(",\"calibrated\":");
    appendJsonBool(out, in.fsr.reading.baselineValid);
    out += F(",\"occupied\":");
    appendJsonBool(out, in.fsr.reading.occupiedByPressure);
    out += F(",\"back_contact\":");
    appendJsonBool(out, (in.fsr.reading.backrestTotal > 300.0f));
    out += F(",\"sampling_rate_hz\":");
    appendJsonFloat(out, in.fsr.reading.actualSamplingRateHz, 2);
    out += F(",\"backrest_total\":");
    appendJsonFloat(out, in.fsr.reading.backrestTotal, 1);
    out += F(",\"cushion_total\":");
    appendJsonFloat(out, in.fsr.reading.cushionTotal, 1);
    out += F(",\"whole_seat_total\":");
    appendJsonFloat(out, in.fsr.reading.wholeSeatTotal, 1);
    out += F(",\"pressure\":");
    appendFloatArray(out, in.fsr.reading.pressure, NUM_FSR, 1);
    out += F(",\"pressure_share\":");
    appendFloatArray(out, in.fsr.reading.modelShare, NUM_FSR, 5);
    out += F(",\"model\":");
    appendModelEvidence(out, in.fsr.model);
    out += F(",\"model_runtime\":{\"window_samples_collected\":"); out += String(s.fsrMl.windowSamplesCollected);
    out += F(",\"window_samples_required\":"); out += String(s.fsrMl.windowSamplesRequired);
    out += F(",\"samples_until_next_inference\":"); out += String(s.fsrMl.samplesUntilNextInference);
    out += F(",\"windows_evaluated\":"); out += String(s.fsrMl.windowsEvaluated);
    out += F("}}");

    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------
    out += F(",\"mpu6050\":{");
    out += F("\"health\":");
    appendJsonString(out, FusionEngine::getSensorHealthText(in.mpu.health));
    out += F(",\"connected\":");
    appendJsonBool(out, in.mpu.reading.connected);
    out += F(",\"valid\":");
    appendJsonBool(out, in.mpu.reading.valid);
    out += F(",\"sampling_rate_hz\":");
    appendJsonFloat(out, in.mpu.reading.actualSamplingRateHz, 2);
    out += F(",\"accel_magnitude_g\":");
    appendJsonFloat(out, in.mpu.reading.accelMagnitude, 4);
    out += F(",\"gyro_magnitude_dps\":");
    appendJsonFloat(out, in.mpu.reading.gyroMagnitude, 3);
    out += F(",\"dynamic_acceleration_g\":");
    appendJsonFloat(out, in.mpu.reading.dynamicAcceleration, 4);
    out += F(",\"road_motion_model\":");
    appendModelEvidence(out, in.mpu.model);
    out += F(",\"model_runtime\":{\"baseline_ready\":"); appendJsonBool(out, s.mpuMl.stationaryBaselineReady);
    out += F(",\"baseline_samples_collected\":"); out += String(s.mpuMl.baselineSamplesCollected);
    out += F(",\"window_samples_collected\":"); out += String(s.mpuMl.windowSamplesCollected);
    out += F(",\"window_samples_required\":"); out += String(s.mpuMl.windowSamplesRequired);
    out += F(",\"windows_evaluated\":"); out += String(s.mpuMl.windowsEvaluated);
    out += F("}");
    out += F(",\"model_interpretation\":\"road_domain_diagnostic_only_after_physical_motion\",\"fusion_role\":\"vehicle_motion_context_for_fsr_artifact_handling\"}");

    // Close the top-level sensors object opened before c1001.
    // Without this brace, /api/v1/sensors and /api/v1/status are invalid JSON.
    out += F("}");

    return out;
}

String SafeSeatApi::buildCameraJson() const
{
    String out;
    out.reserve(1000);

    if (telemetry == nullptr || !telemetry->getSnapshot().ready)
    {
        out = F("{\"telemetry_ready\":false}");
        return out;
    }

    const SafeSeatTelemetrySnapshot &s = telemetry->getSnapshot();
    const CameraFusionEvidence &e = s.input.camera;
    const CameraRemoteStatus &r = s.cameraLink;

    out += F("{\"available\":");
    appendJsonBool(out, e.available);
    out += F(",\"connected\":");
    appendJsonBool(out, e.connected);
    out += F(",\"transport_connected\":");
    appendJsonBool(out, r.connected);
    out += F(",\"stale\":");
    appendJsonBool(out, r.stale);
    out += F(",\"camera_ready\":");
    appendJsonBool(out, r.cameraReady);
    out += F(",\"model_ready\":");
    appendJsonBool(out, r.modelReady);
    out += F(",\"psram_ready\":");
    appendJsonBool(out, r.psramReady);
    out += F(",\"busy\":");
    appendJsonBool(out, r.busy);
    out += F(",\"session_active\":");
    appendJsonBool(out, r.sessionActive);
    out += F(",\"local_session_active\":");
    appendJsonBool(out, r.localOccupancySessionActive);
    out += F(",\"remote_session_id\":");
    out += String(r.remoteSessionId);
    out += F(",\"local_session_id\":");
    out += String(r.localSessionId);
    out += F(",\"baseline_ready\":");
    appendJsonBool(out, r.baselineReady);
    out += F(",\"calibrating\":");
    appendJsonBool(out, r.calibrating);
    out += F(",\"calibration_count\":");
    out += String(r.calibrationCount);
    out += F(",\"calibration_target\":");
    out += String(r.calibrationTarget);
    out += F(",\"packet_age_ms\":");
    out += String(r.packetAgeMillis);
    out += F(",\"status_packets_received\":");
    out += String(r.statusPacketsReceived);
    out += F(",\"result_packets_received\":");
    out += String(r.resultPacketsReceived);
    out += F(",\"verification_requested\":");
    appendJsonBool(out, s.fusion.triggerCamera);
    out += F(",\"request_active\":");
    appendJsonBool(out, r.requestActive);
    out += F(",\"active_request_id\":");
    out += String(r.activeRequestId);
    out += F(",\"result_valid\":");
    appendJsonBool(out, e.resultValid);
    out += F(",\"result_request_id\":");
    out += String(e.requestId);
    out += F(",\"posture\":");
    appendJsonString(
        out,
        cameraPostureText(
            static_cast<CameraPostureClass>(e.postureClass)
        )
    );
    out += F(",\"posture_normal\":");
    appendJsonBool(out, e.postureNormal);
    out += F(",\"posture_abnormal\":");
    appendJsonBool(out, e.postureAbnormal);
    out += F(",\"confidence\":");
    appendJsonFloat(out, e.confidence, 3);
    out += F(",\"verification_only\":true}");

    return out;
}

void SafeSeatApi::appendJsonBool(
    String &out,
    bool value
)
{
    out += value ? F("true") : F("false");
}

void SafeSeatApi::appendJsonFloat(
    String &out,
    float value,
    uint8_t decimals
)
{
    if (!isfinite(value))
    {
        out += F("null");
        return;
    }

    out += String(value, static_cast<unsigned int>(decimals));
}

void SafeSeatApi::appendJsonString(
    String &out,
    const char *value
)
{
    out += '"';

    if (value != nullptr)
    {
        for (const char *p = value; *p != '\0'; ++p)
        {
            switch (*p)
            {
                case '"': out += F("\\\""); break;
                case '\\': out += F("\\\\"); break;
                case '\n': out += F("\\n"); break;
                case '\r': out += F("\\r"); break;
                case '\t': out += F("\\t"); break;
                default: out += *p; break;
            }
        }
    }

    out += '"';
}

void SafeSeatApi::appendModelEvidence(
    String &out,
    const ModelEvidence &model
)
{
    out += F("{\"available\":");
    appendJsonBool(out, model.available);
    out += F(",\"valid\":");
    appendJsonBool(out, model.valid);
    out += F(",\"isolation_forest_anomaly\":");
    appendJsonBool(out, model.isolationForestAnomaly);
    out += F(",\"one_class_svm_anomaly\":");
    appendJsonBool(out, model.oneClassSVMAnomaly);
    out += F(",\"both_models_anomaly\":");
    appendJsonBool(out, model.bothModelsAnomaly);
    out += F(",\"either_model_anomaly\":");
    appendJsonBool(out, model.eitherModelAnomaly);
    out += F(",\"isolation_forest_score\":");
    appendJsonFloat(out, model.isolationForestScore, 6);
    out += F(",\"one_class_svm_score\":");
    appendJsonFloat(out, model.oneClassSVMScore, 6);
    out += F(",\"confidence\":");
    appendJsonFloat(out, model.confidence, 3);
    out += F("}");
}

void SafeSeatApi::appendFloatArray(
    String &out,
    const float *values,
    size_t count,
    uint8_t decimals
)
{
    out += '[';

    for (size_t i = 0; i < count; ++i)
    {
        if (i > 0)
        {
            out += ',';
        }

        appendJsonFloat(out, values[i], decimals);
    }

    out += ']';
}
