#include "SafeSeatApi.h"

#include <math.h>

#include "NetworkConfig.h"
#include "CameraProtocol.h"

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

    Serial.println("[API] SafeSeat local read-only API started.");
    Serial.println("[API] Base URL : http://192.168.4.1");
    Serial.println("[API] Status   : /api/v1/status");
    Serial.println("[API] Sensors  : /api/v1/sensors");
    Serial.println("[API] Camera   : /api/v1/camera");
    Serial.println("[API] Network  : /api/v1/network");
    Serial.println("[API] Health   : /health");

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

    server.on("/api/v1/status", HTTP_GET, [this]() { handleStatus(); });
    server.on("/api/v1/sensors", HTTP_GET, [this]() { handleSensors(); });
    server.on("/api/v1/camera", HTTP_GET, [this]() { handleCamera(); });
    server.on("/api/v1/network", HTTP_GET, [this]() { handleNetwork(); });

    // Development aliases. The versioned endpoints above are the
    // contract the frontend should ultimately target.
    server.on("/status", HTTP_GET, [this]() { handleStatus(); });
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
<p>Local read-only telemetry API is running.</p>
<p>The Main Hub Fusion state is authoritative; these endpoints only expose current state.</p>
<a href="/api/v1/status">/api/v1/status</a>
<a href="/api/v1/sensors">/api/v1/sensors</a>
<a href="/api/v1/camera">/api/v1/camera</a>
<a href="/api/v1/network">/api/v1/network</a>
<a href="/health">/health</a>
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

void SafeSeatApi::handleStatus()
{
    sendJson(200, buildStatusJson());
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
    out += F(",\"read_only\":true}");

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
    out += F("}");

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
    out += F(",\"baseline_ready\":");
    appendJsonBool(out, in.mlx.context.baselineReady);
    out += F(",\"context_change\":");
    appendJsonBool(out, in.mlx.context.contextChange);
    out += F(",\"baseline_object_c\":");
    appendJsonFloat(out, in.mlx.context.baselineObjectC, 2);
    out += F(",\"deviation_from_baseline_c\":");
    appendJsonFloat(out, in.mlx.context.deviationFromBaselineC, 2);
    out += F("}");
    out += F(",\"legacy_wesad_model\":");
    appendModelEvidence(out, in.mlx.model);
    out += F(",\"legacy_wesad_model_fusion_role\":\"diagnostic_only\"}");

    // --------------------------------------------------------
    // FSR
    // --------------------------------------------------------
    out += F(",\"fsr\":{");
    out += F("\"health\":");
    appendJsonString(out, FusionEngine::getSensorHealthText(in.fsr.health));
    out += F(",\"connected\":");
    appendJsonBool(out, in.fsr.reading.connected);
    out += F(",\"calibrated\":");
    appendJsonBool(out, in.fsr.reading.calibrated);
    out += F(",\"occupied\":");
    appendJsonBool(out, in.fsr.reading.occupied);
    out += F(",\"back_contact\":");
    appendJsonBool(out, in.fsr.reading.backContact);
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
    appendFloatArray(out, in.fsr.reading.pressureShare, NUM_FSR, 5);
    out += F(",\"model\":");
    appendModelEvidence(out, in.fsr.model);
    out += F("}");

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
    out += F(",\"fusion_role\":\"artifact_context\"}");

    // --------------------------------------------------------
    // Piezo - deterministic respiration support
    // --------------------------------------------------------
    out += F(",\"piezo\":{");
    out += F("\"available\":");
    appendJsonBool(out, in.piezo.available);
    out += F(",\"connected\":");
    appendJsonBool(out, in.piezo.connected);
    out += F(",\"valid\":");
    appendJsonBool(out, in.piezo.valid);
    out += F(",\"signal_usable\":");
    appendJsonBool(out, in.piezo.signalUsable);
    out += F(",\"breath_tracking_ready\":");
    appendJsonBool(out, in.piezo.breathTrackingReady);
    out += F(",\"breath_detected_recently\":");
    appendJsonBool(out, in.piezo.breathDetectedRecently);
    out += F(",\"packet_age_ms\":");
    out += String(s.piezoLink.packetAgeMillis);
    out += F(",\"packets_received\":");
    out += String(s.piezoLink.packetsReceived);
    out += F(",\"sampling_rate_hz\":");
    appendJsonFloat(out, s.piezoLink.remoteSamplingRateHz, 2);
    out += F(",\"respiration_bpm\":");
    appendJsonFloat(out, in.piezo.respirationBPM, 2);
    out += F(",\"respiration_wave\":");
    appendJsonFloat(out, in.piezo.respirationWave, 2);
    out += F(",\"total_breath_events\":");
    out += String(in.piezo.totalBreaths);
    out += F(",\"no_breath_duration_ms\":");
    out += String(in.piezo.noBreathDurationMs);
    out += F(",\"no_breath_support_active\":");
    appendJsonBool(out, in.piezo.noBreathTimerExceeded);
    out += F(",\"deployment\":\"deterministic_no_ml\"");
    out += F(",\"fusion_role\":\"secondary_respiration_corroboration\"}");

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
    out += F(",\"camera_ready\":");
    appendJsonBool(out, r.cameraReady);
    out += F(",\"model_ready\":");
    appendJsonBool(out, r.modelReady);
    out += F(",\"psram_ready\":");
    appendJsonBool(out, r.psramReady);
    out += F(",\"busy\":");
    appendJsonBool(out, r.busy);
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
